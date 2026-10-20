# ROS 2 bag timing inspector

This Mac-side inspector reads MCAP with embedded ROS 2 message schemas. It never changes a timestamp or runs ROS/FAST-LIO. Its pure core consumes `(topic, receipt_ns, decoded_msg_like)` records and emits the `spatio.bagtiming/1` report.

From the repository root:

```sh
python3 -m spatio.bagtiming /path/to/bag --out /separate/report/directory
python3 -m spatio.bagtiming /path/to/bag_0.mcap --out /separate/slice --max-seconds 30
python3 -m spatio.bagtiming /path/to/bag --out /separate/report --topics /livox/lidar /livox/imu /Odometry --gap-factor 3.0
```

Outputs are `bagtiming.json` and `bagtiming.md`, including the compatibility table and definitions. Exit 0 means inspection completed, including reports containing errors/warnings. Exit 2 means unreadable/invalid input (or invalid CLI arguments). Exit 1 is an output-write failure. Output cannot be the MCAP input directory or a subdirectory of it. Use a distinct derived-output directory.

For the team recordings, every probe/slice/full pass must use the shared lock:

```sh
/usr/bin/lockf -k -t 1800 ~/Developer/lc-tasks/.spatio-bigio.lock nice -n 10 python3 -m spatio.bagtiming INPUT --out OUTPUT --max-seconds 30
```

Time one 30-second slice from each bag, sequentially. `analysis.elapsed_wall_s` records inspection time. Estimate full runtime from slice time × full duration / 30, label that an estimate, and include serialization/IO uncertainty. If above about 1,200 seconds, use `--point-timing-sampled-every K` (choose enough to reduce point decoding). A full run omits `--max-seconds`. Header and receipt statistics still include every selected message; unsampled scan bounds and coverage stay unknown. All real runs must remain under the shared lock; a lock failure does not authorize an unlocked run.

## Reader and dependencies

Uses the preinstalled Python 3.9-compatible `mcap`, `mcap_ros2` (distribution `mcap-ros2-support`), and numpy. Tested here with mcap 1.4.0 and mcap-ros2-support 0.5.7. No installations required. The first successful decode per topic/segment is recorded in `decoder_probes`. See `_out/SPATIO-x1-bagtiming.md` for measured recording results.

A minimal CDR1 CustomMsg fallback handles the exact copied message layout; it activates only if the embedded-schema decoder fails and the schema matches. Schema checks normalize declaration whitespace and supported type qualification, preserving exact names, types, arrays and order. The real Mid-360 embedded schema is retained in `tests/fixtures/`, including the aligned whitespace that caused the original rejection. Hand-built big/little endian buffers and independent mcap_ros2 decoding test the fallback. On unsampled lidar messages, an exact standard Header CDR1 parser avoids decoding the point array. Other unknown schema/encoding failures return exit 2; messages are never silently dropped. SQLite ROS bags and IDL-only schemas are unsupported.

IO streams in storage order, retaining timestamp arrays and per-scan summaries rather than decoded point clouds. Natural segment filename order preserves recording order across `bag_2`/`bag_10`; MCAP log-time sorting is deliberately disabled. Large MCAP chunk allocation is determined by the input file. No other `spatio.*` package is imported.

## Evidence contracts

PointCloud2 has no standard point-time unit or absolute/relative origin. Even a FLOAT32 `time` or epoch-looking FLOAT64 `timestamp` is **unknown** unless evidence establishes both. Raw fields, datatypes, counts, finite ranges and nonfinite counts are always reported. Missing timing never falls back to a header-derived scan interval.

Use optional `--evidence /path/to/evidence.json` only with actual producer/configuration evidence. Example for a **synthetic** fixture:

```json
{
  "timing_contracts": {
    "/points": {
      "field": "time",
      "unit": "s",
      "origin": "header",
      "evidence": "Synthetic producer explicitly writes relative seconds from header.stamp."
    }
  },
  "clocks": {
    "/points": {
      "receipt": {"domain": "postprocess-receipt", "evidence": "Synthetic replay log provenance."},
      "header": {"domain": "unknown", "evidence": "Synthetic fixture provides no physical clock provenance."},
      "points": {"domain": "unknown", "evidence": "Physical clock identity not established."}
    }
  }
}
```

Supported units: `ns`, `s`; origins: `header` (relative) or `absolute`. Clock domains: `sensor`, `host`, `postprocess-receipt`, `unknown`. Receipt/header/points have separate domain evidence. User-supplied evidence is retained verbatim; the inspector cannot authenticate it. It does not automatically parse YAML sidecars or infer clocks from file names. Missing provenance remains unknown.

Livox `timebase + offset_time` nanoseconds are established by the copied source, detailed in every report. The `.msg` files themselves establish names/types/relative offset but do not explicitly name units. Publisher clock provenance is still unknown: the copied driver chooses device timestamps for PTP/GPS and a host-clock fallback otherwise. `ptp4l running: 0` alone does not establish which branch generated a particular message.

Numeric IMU coverage and nearest-header offsets do not prove synchronization. The real copied JT128 adapter is `spark-src/hesai_ws/jt128_slam_adapter.py`; the `launchers/` copy is empty. It writes FLOAT32 `time` in seconds relative to the first kept point, clips negative offsets, and preserves the input header. It does not establish that header time equals first-point time. Without upstream evidence of that relationship, do not supply an `origin: header` contract: absolute bounds and IMU coverage remain unknown. Physical fit for FAST-LIO cannot be concluded while these unknowns remain. This tool evaluates neither IMU units/axes nor trajectories.

## Tests

```sh
cd ~/Developer/spatio-x1-bagtiming && nice -n 10 python3 -m unittest discover -s spatio/bagtiming/tests -t . -v
```

Only small synthetic MCAP files are generated, in temporary directories, using synthetic schemas and the checked-in recorded schema. No skips or external recordings required. See findings for exact test results and the Swift validation limitation.
