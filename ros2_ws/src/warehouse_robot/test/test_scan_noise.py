import math

import numpy as np

from warehouse_robot.scan_noise import ScanNoiseConfig, degrade


def sim_scan(dist):
    """Gazebo 처럼 360개 빔, 모든 방향 dist(m)."""
    return [dist] * 360, 0.0, 2 * math.pi / 360


def test_beam_count_varies_like_lds02():
    rng = np.random.default_rng(0)
    counts = set()
    for _ in range(50):
        ranges, amin, inc = sim_scan(1.0)
        r, a0, a_inc = degrade(ranges, amin, inc, 0.12, rng)
        counts.add(len(r))
        assert abs(len(r) * a_inc - 2 * math.pi) < 1e-9
        assert amin <= a0 < amin + a_inc
    assert min(counts) >= 238 and max(counts) <= 244 and len(counts) > 3


def test_noise_and_dropout_grow_with_distance():
    rng = np.random.default_rng(1)
    near, far = [], []
    for _ in range(40):
        near.append(degrade(*sim_scan(1.0), 0.12, rng)[0])
        far.append(degrade(*sim_scan(3.2), 0.12, rng)[0])
    near, far = np.concatenate(near), np.concatenate(far)
    drop_near, drop_far = np.isnan(near).mean(), np.isnan(far).mean()
    assert drop_near < 0.05 and drop_far > 0.15
    std_near = np.std(near[np.isfinite(near)])
    std_far = np.std(far[np.isfinite(far)])
    assert 0.01 < std_near < 0.04 and std_far > std_near


def test_no_return_is_nan_like_real_lds02():
    rng = np.random.default_rng(2)
    cfg = ScanNoiseConfig(drop_base=0.0, drop_far=0.0)
    r, _, _ = degrade(*sim_scan(5.0), 0.12, rng, cfg)      # max_range 3.5 보다 멂
    assert np.isnan(r).all()
    r, _, _ = degrade([math.inf] * 360, 0.0, 2 * math.pi / 360, 0.12, rng, cfg)   # Gazebo: 아무것도 안 맞음
    assert np.isnan(r).all()
