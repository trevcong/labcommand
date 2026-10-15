"""Explicit one-to-one, per-sensor candidate matching and metric arithmetic."""
import numpy as np
from .generator import SENSORS

MATCH_WINDOW_S = 10.
BBOX_PADDING_M = .1
MIN_IOU = .05
SOURCE_DISTANCE_M = .5


def distribution(values):
    if not len(values):
        return {key: None for key in ("p50", "p95", "p99", "max")}
    values = np.asarray(values, dtype=float)
    return dict(zip(("p50", "p95", "p99", "max"),
                    [float(x) for x in (*np.percentile(values, [50, 95, 99]), values.max())]))


def bbox_iou(a, b, padding=BBOX_PADDING_M):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    low_a, high_a = a[0] - padding, a[1] + padding
    low_b, high_b = b[0] - padding, b[1] + padding
    intersection = np.maximum(0, np.minimum(high_a, high_b) - np.maximum(low_a, low_b)).prod()
    union = (high_a - low_a).prod() + (high_b - low_b).prod() - intersection
    return float(intersection / union) if union else 0.


class Matcher:
    def __init__(self, truth, duration):
        self.events = truth["events"]
        self.duration = duration
        self.reference_s = truth["reference_build_s"]
        self.matched = {}
        self.seen = set()
        self.false_alarms = 0
        self.ignored = 0

    def observe(self, detection):
        """Return an auditable disposition; repeated lifecycle IDs are ignored."""
        identity = (detection["sensor"], detection["event_id"])
        if detection["state"] != "candidate" or identity in self.seen:
            self.ignored += 1
            return {"disposition": "ignored"}
        self.seen.add(identity)
        choices = []
        for truth in self.events:
            key = (truth["id"], detection["sensor"])
            latency = detection["time_s"] - truth["time_s"]
            if key in self.matched or truth["type"] != detection["type"]:
                continue
            if not 0 <= latency <= MATCH_WINDOW_S or detection["time_s"] > truth["end_s"]:
                continue
            score = bbox_iou(detection["bbox"], truth["bbox"])
            if score < MIN_IOU:
                continue
            if truth["type"] == "moved":
                source = detection.get("from_centroid")
                center = np.mean(truth["from_bbox"], axis=0)
                if source is None or np.linalg.norm(np.asarray(source) - center) > SOURCE_DISTANCE_M:
                    continue
            choices.append((-score, truth["time_s"], truth["id"], key, latency))
        if choices:
            _, _, truth_id, key, latency = min(choices)
            self.matched[key] = latency
            return {"disposition": "hit", "truth_id": truth_id, "latency_s": latency}
        self.false_alarms += 1
        return {"disposition": "false_alarm"}

    def summary(self):
        eligible = {e["id"] for e in self.events if e["time_s"] + MATCH_WINDOW_S <= self.duration}
        expected = len(eligible) * len(SENSORS)
        latency = [v for (event_id, _), v in self.matched.items() if event_id in eligible]
        exposure = max(0, self.duration - self.reference_s)
        per_sensor = {}
        for sensor in SENSORS:
            hits = sum(eid in eligible and s == sensor for eid, s in self.matched)
            per_sensor[sensor] = {"hits": hits, "expected": len(eligible),
                                  "hit_rate": hits / len(eligible) if eligible else None}
        return {"expected": expected, "hits": len(latency),
                "hit_rate": len(latency) / expected if expected else None,
                "censored_targets": (len(self.events) - len(eligible)) * len(SENSORS),
                "false_alarms": self.false_alarms,
                "false_alarms_per_hour": self.false_alarms * 3600 / exposure if exposure else None,
                "exposure_s": exposure, "detection_latency_s": distribution(latency),
                "ignored_transitions": self.ignored, "per_sensor": per_sensor}


def rss_summary(samples, field="harness_rss_bytes"):
    valid = [(s["time_s"], s[field]) for s in samples if s.get(field) is not None]
    if not valid:
        return None
    times, values = np.asarray(valid, dtype=float).T
    slope = float(np.polyfit(times - times[0], values, 1)[0]) if len(set(times)) > 1 else None
    return {"start_bytes": int(values[0]), "end_bytes": int(values[-1]),
            "growth_bytes": int(values[-1] - values[0]), "sampled_peak_bytes": int(values.max()),
            "slope_bytes_per_replay_s": slope}
