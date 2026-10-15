import pytest
from spark.room_memory_bench.generator import ReplayConfig, ground_truth
from spark.room_memory_bench.metrics import Matcher, bbox_iou, distribution, rss_summary


def detection(event, **changes):
    result = {'sensor': 'jt128', 'event_id': 1, 'time_s': event['time_s'] + 1,
              'type': event['type'], 'state': 'candidate', 'bbox': event['bbox']}
    if 'from_bbox' in event:
        result['from_centroid'] = [(a + b) / 2 for a, b in zip(*event['from_bbox'])]
    result.update(changes)
    return result


def test_math_percentiles_rates_latency_and_rss():
    assert distribution([1, 2, 3, 4, 5]) == pytest.approx({'p50': 3, 'p95': 4.8, 'p99': 4.96, 'max': 5})
    assert distribution([])['p50'] is None
    truth = ground_truth(ReplayConfig(duration=60))
    matcher = Matcher(truth, 60)
    event = truth['events'][0]
    assert matcher.observe(detection(event))['disposition'] == 'hit'
    assert matcher.observe(detection(event, sensor='mid360', time_s=13))['disposition'] == 'hit'
    matcher.observe(detection(event, event_id=2))  # duplicate spatial candidate is a false alarm
    result = matcher.summary()
    assert result['hits'] == 2 and result['expected'] == 6
    assert result['hit_rate'] == pytest.approx(1 / 3)
    assert result['false_alarms_per_hour'] == pytest.approx(3600 / 55)
    assert result['detection_latency_s']['p50'] == 2
    samples = [{'time_s': t, 'harness_rss_bytes': v} for t, v in [(0, 100), (10, 120), (20, 140)]]
    rss = rss_summary(samples)
    assert rss['growth_bytes'] == 40 and rss['sampled_peak_bytes'] == 140
    assert rss['slope_bytes_per_replay_s'] == pytest.approx(2)
    assert rss_summary([]) is None


def test_spatial_type_time_and_source_gates_and_lifecycle():
    truth = ground_truth(ReplayConfig(duration=60))
    event = truth['events'][0]
    for change in ({'time_s': 9.9}, {'time_s': 20.1}, {'type': 'disappeared'},
                   {'bbox': [[20, 20, 20], [21, 21, 21]]}):
        matcher = Matcher(truth, 60)
        assert matcher.observe(detection(event, **change))['disposition'] == 'false_alarm'
    matcher = Matcher(truth, 60)
    assert matcher.observe(detection(event, time_s=20))['disposition'] == 'hit'
    assert matcher.observe(detection(event))['disposition'] == 'ignored'
    assert matcher.observe(detection(event, state='expired', event_id=2))['disposition'] == 'ignored'
    moved = truth['events'][2]
    for source in (None, [50, 50, 50]):
        assert Matcher(truth, 60).observe(detection(moved, from_centroid=source))['disposition'] == 'false_alarm'
    assert Matcher(truth, 60).observe(detection(moved))['disposition'] == 'hit'


def test_censoring_empty_exposure_and_bbox_math():
    truth = ground_truth(ReplayConfig(duration=15))
    matcher = Matcher(truth, 15)
    matcher.observe(detection(truth['events'][0]))
    result = matcher.summary()
    assert result['expected'] == 0 and result['censored_targets'] == 2
    assert result['hit_rate'] is None and result['false_alarms'] == 0
    empty = Matcher(ground_truth(ReplayConfig(duration=1)), 1).summary()
    assert empty['false_alarms_per_hour'] is None
    assert bbox_iou([[0, 0, 0], [1, 1, 1]], [[0, 0, 0], [1, 1, 1]], padding=0) == 1
    assert bbox_iou([[0, 0, 0], [1, 1, 1]], [[.5, 0, 0], [1.5, 1, 1]], padding=0) == pytest.approx(1 / 3)


def test_one_detection_cannot_match_two_overlapping_targets():
    truth = ground_truth(ReplayConfig(duration=60))
    truth['events'].append({**truth['events'][0], 'id': 'overlap'})
    matcher = Matcher(truth, 60)
    matcher.observe(detection(truth['events'][0]))
    assert len(matcher.matched) == 1
