"""Offline replay CLI. Replay time is independent of elapsed wall time."""
import argparse
import ctypes
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import sys
import time

import numpy as np
from .backends import BackendUnavailable, MEMORY_CONFIG, PythonBackend, SubprocessBackend
from .generator import HZ, ReplayConfig, ground_truth, replay
from .metrics import (BBOX_PADDING_M, MATCH_WINDOW_S, MIN_IOU, SOURCE_DISTANCE_M,
                      Matcher, distribution, rss_summary)


def current_rss(pid):
    """Current resident bytes, not ru_maxrss's historical high-water mark."""
    if sys.platform.startswith("linux"):
        fields = Path(f"/proc/{pid}/statm").read_text().split()
        return int(fields[1]) * os.sysconf("SC_PAGE_SIZE")
    if sys.platform == "darwin":
        lib = ctypes.CDLL("/usr/lib/libproc.dylib", use_errno=True)
        lib.proc_pidinfo.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_uint64,
                                    ctypes.c_void_p, ctypes.c_int]
        # proc_taskinfo begins with virtual_size and resident_size (uint64).
        buffer = (ctypes.c_uint64 * 12)()
        count = lib.proc_pidinfo(pid, 4, 0, ctypes.byref(buffer), ctypes.sizeof(buffer))
        if count != ctypes.sizeof(buffer):
            raise OSError(ctypes.get_errno(), "proc_pidinfo(PROC_PIDTASKINFO) failed")
        return int(buffer[1])
    raise RuntimeError("current RSS sampling supports macOS and Linux")


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def markdown(report):
    lines = ["# Room memory benchmark", "", f"Status: **{report['status']}**", ""]
    if report["status"] != "complete":
        return "\n".join(lines + [report["reason"], ""])
    timing, accuracy, rss = report["tick_ms"], report["accuracy"], report["harness_rss"]
    lines += ["| Metric | Value |", "| --- | --- |",
              f"| Backend | {report['backend']} |",
              f"| Replay / wall seconds | {report['replay']['duration']} / {report['wall_s']:.3f} |",
              f"| Ticks / frames / points per frame | {report['ticks']} / {report['ticks'] * 2} / {report['replay']['points']} |"]
    for key, value in timing.items():
        lines.append(f"| Tick {key}, ms | {value:.3f} |")
    lines += [f"| Hits / eligible sensor targets | {accuracy['hits']} / {accuracy['expected']} |",
              f"| Hit rate | {accuracy['hit_rate']} |",
              f"| False alarms / hour | {accuracy['false_alarms_per_hour']} |",
              f"| False alarms / exposure seconds | {accuracy['false_alarms']} / {accuracy['exposure_s']} |",
              f"| Censored sensor targets | {accuracy['censored_targets']} |",
              f"| Harness RSS start / end / sampled peak, MiB | {rss['start_bytes']/2**20:.2f} / {rss['end_bytes']/2**20:.2f} / {rss['sampled_peak_bytes']/2**20:.2f} |",
              f"| Harness RSS growth, MiB | {rss['growth_bytes']/2**20:.2f} |",
              f"| Harness RSS slope, bytes/replay second | {rss['slope_bytes_per_replay_s']} |"]
    for key, value in accuracy['detection_latency_s'].items():
        lines.append(f"| Detection latency {key}, replay seconds | {value} |")
    child = report.get('backend_rss')
    if child is not None:
        lines += [f"| Child RSS start / end / sampled peak, MiB | {child['start_bytes']/2**20:.2f} / {child['end_bytes']/2**20:.2f} / {child['sampled_peak_bytes']/2**20:.2f} |",
                  f"| Child RSS growth, MiB | {child['growth_bytes']/2**20:.2f} |",
                  f"| Child RSS slope, bytes/replay second | {child['slope_bytes_per_replay_s']} |"]
    lines += ["",
              "Tick timings include both sensors and window closure/status when due; generation and RSS sampling are excluded.",
              "Replay is unpaced. Candidate hits are per sensor, with a 10-second deadline. See README for matching and caveats.",
              "RSS is sampled at replay boundaries; transient peaks between samples are not measured.", ""]
    return "\n".join(lines)


def run(args):
    config = ReplayConfig(args.duration, args.points, args.seed, args.noise_m)
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    for filename in ("ticks.jsonl", "detections.jsonl", "backend.stderr.log"):
        (output / filename).unlink(missing_ok=True)
    write_json(output / "rss_samples.json", [])
    report = {"schema": 1, "status": "running", "backend": args.backend, "replay": asdict(config),
              "memory_config": MEMORY_CONFIG, "created_utc": datetime.now(timezone.utc).isoformat(),
              "environment": {"platform": platform.platform(), "machine": platform.machine(),
                              "python": platform.python_version(), "numpy": np.__version__},
              "rss_sample_interval_s": args.rss_interval,
              "matching": {"window_s": MATCH_WINDOW_S, "bbox_padding_m": BBOX_PADDING_M,
                           "min_iou": MIN_IOU, "source_distance_m": SOURCE_DISTANCE_M,
                           "unit": "per-sensor candidate"}}
    truth = ground_truth(config)
    write_json(output / "ground_truth.json", truth)
    write_json(output / "report.json", report)
    (output / "report.md").write_text("# Room memory benchmark\n\nStatus: **running**\n")
    backend = None
    started = time.perf_counter()
    tick_ms, closing_ms, generation_ms, samples = [], [], [], []
    matcher = Matcher(truth, config.duration)
    completed = 0
    try:
        backend = (PythonBackend() if args.backend == "python" else
                   SubprocessBackend(args.binary, output / "backend.stderr.log", args.timeout))

        def sample(stamp):
            samples.append({"time_s": stamp, "wall_s": time.perf_counter() - started,
                            "harness_rss_bytes": current_rss(os.getpid()),
                            "backend_rss_bytes": current_rss(backend.pid) if backend.pid else None})

        sample(0.)
        iterator = iter(replay(config))
        next_progress = time.monotonic() + 30
        with (output / "detections.jsonl").open("w") as detections, (output / "ticks.jsonl").open("w") as ticks:
            for _ in range(config.duration * HZ):
                begin = time.perf_counter()
                tick = next(iterator)
                generation_ms.append((time.perf_counter() - begin) * 1000)
                begin = time.perf_counter()
                events = backend.tick(tick)
                elapsed = (time.perf_counter() - begin) * 1000
                tick_ms.append(elapsed)
                if tick.close_window:
                    closing_ms.append(elapsed)
                completed += 1
                ticks.write(json.dumps({"index": tick.index, "time_s": tick.time_s, "backend_ms": elapsed,
                                        "generation_ms": generation_ms[-1], "close_window": tick.close_window}) + "\n")
                for event in events:
                    result = matcher.observe(event)
                    detections.write(json.dumps({**event, **result}, allow_nan=False) + "\n")
                if completed % (args.rss_interval * HZ) == 0 or completed == config.duration * HZ:
                    sample(completed / HZ)
                    write_json(output / "rss_samples.json", samples)
                if time.monotonic() >= next_progress:
                    print(f"replayed {completed/HZ:.1f}/{config.duration}s; wall {time.perf_counter()-started:.1f}s", file=sys.stderr, flush=True)
                    next_progress = time.monotonic() + 30
        report.update(status="complete", ticks=completed, tick_ms=distribution(tick_ms),
                      closing_tick_ms=distribution(closing_ms), generation_ms=distribution(generation_ms),
                      backend_total_s=sum(tick_ms) / 1000,
                      ticks_over_100ms=sum(value > 100 for value in tick_ms),
                      accuracy=matcher.summary(), harness_rss=rss_summary(samples),
                      backend_rss=rss_summary(samples, "backend_rss_bytes"))
        backend.close()
        backend = None
    except BackendUnavailable as error:
        report.update(status="skipped", reason=str(error))
    except (Exception, KeyboardInterrupt) as error:
        report.update(status="failed", reason=f"{type(error).__name__}: {error}", ticks=completed)
    finally:
        if backend is not None:
            if isinstance(backend, SubprocessBackend):
                backend.abort()
            else:
                backend.close()
    report["wall_s"] = time.perf_counter() - started
    write_json(output / "rss_samples.json", samples)
    write_json(output / "report.json", report)
    (output / "report.md").write_text(markdown(report))
    print(f"{report['status']}: {output / 'report.json'}", flush=True)
    return 1 if report["status"] == "failed" else 0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return value


def positive_float(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=positive_int, default=3600, help="whole replay seconds (default: 3600)")
    parser.add_argument("--points", type=positive_int, default=80_000, help="points per sensor frame")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--noise-m", type=float, default=.01, help="range noise standard deviation, meters")
    parser.add_argument("--backend", choices=("python", "cpp"), default="python")
    parser.add_argument("--binary", type=Path, default=Path(__file__).resolve().parents[1] / "room_memory_cpp" / "build" / "room_memory_bench")
    parser.add_argument("--timeout", type=positive_float, default=120, help="subprocess deadline per exchange, wall seconds")
    parser.add_argument("--rss-interval", type=positive_int, default=10, help="RSS sampling interval, replay seconds")
    parser.add_argument("--output", type=Path, default=Path("room-memory-bench-results"))
    args = parser.parse_args(argv)
    if args.seed < 0 or not np.isfinite(args.noise_m) or args.noise_m < 0:
        parser.error("seed and noise-m must be nonnegative; noise-m must be finite")
    return run(args)
