import numpy as np
import pytest
from spark.room_memory_bench.generator import BOXES, ReplayConfig, active_boxes, ground_truth, replay


def test_seeded_lazy_replay_is_deterministic_and_sensor_specific():
    config = ReplayConfig(duration=2, points=512)
    first, second = replay(config), replay(config)
    assert iter(first) is first
    ticks = list(first)
    for left, right in zip(ticks, second):
        assert left.index == right.index
        for sensor in left.clouds:
            assert left.clouds[sensor].shape == (512, 3)
            assert left.clouds[sensor].dtype == np.float32
            assert np.isfinite(left.clouds[sensor]).all()
            assert np.array_equal(left.clouds[sensor], right.clouds[sensor])
    assert len(ticks) == 20
    assert [t.index for t in ticks if t.close_window] == [9, 19]
    assert not np.array_equal(ticks[0].clouds['jt128'], ticks[0].clouds['mid360'])
    changed = next(replay(ReplayConfig(duration=2, points=512, seed=8)))
    assert not np.array_equal(ticks[0].clouds['jt128'], changed.clouds['jt128'])
    # Later frames neither alias nor mutate earlier frames.
    assert not np.shares_memory(ticks[0].clouds['jt128'], ticks[1].clouds['jt128'])


def test_default_hour_and_script_boundaries():
    config = ReplayConfig()
    assert config.duration * 10 == 36000 and config.points == 80000
    truth = ground_truth(config)
    assert len(truth['events']) == 180 and len(truth['restorations_s']) == 59
    assert truth['events'][-1]['time_s'] == 3580
    assert BOXES['appeared'] not in active_boxes(9.9)
    assert BOXES['appeared'] in active_boxes(10)
    assert BOXES['disappeared'] in active_boxes(24.9)
    assert BOXES['disappeared'] not in active_boxes(25)
    assert BOXES['destination'] in active_boxes(40)
    assert active_boxes(60) == active_boxes(0)
    assert len(ground_truth(ReplayConfig(duration=10))['events']) == 0


def test_rays_stop_on_first_box_and_room():
    tick = next(replay(ReplayConfig(duration=1, points=80000, noise_m=0)))
    for cloud in tick.clouds.values():
        # Every ideal return lies on a room or active-box face, inside that box's bounds.
        on_surface = np.zeros(len(cloud), dtype=bool)
        from spark.room_memory_bench.generator import ROOM
        for low, high in [ROOM] + active_boxes(0):
            low, high = np.asarray(low), np.asarray(high)
            inside = ((cloud >= low - 1e-6) & (cloud <= high + 1e-6)).all(axis=1)
            face = (np.isclose(cloud, low, atol=1e-6) | np.isclose(cloud, high, atol=1e-6)).any(axis=1)
            on_surface |= inside & face
        assert on_surface.all()
        # A room return's ray may not pass through a nearer object interior.
        for low, high in active_boxes(0):
            with np.errstate(divide='ignore', invalid='ignore'):
                a, b = np.asarray(low) / cloud, np.asarray(high) / cloud
            near, far = np.minimum(a, b).max(axis=1), np.maximum(a, b).min(axis=1)
            assert not ((near > 0) & (near < 1 - 1e-5) & (far > near + 1e-5)).any()


@pytest.mark.parametrize('kwargs', [{'duration': 0}, {'points': 0}, {'seed': -1}, {'noise_m': float('nan')}])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ReplayConfig(**kwargs)
