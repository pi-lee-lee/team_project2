import math

import pytest

from warehouse_robot.follower import PurePursuit


def simulate(follower, x, y, yaw, dt=0.05, steps=2000):
    """차동 구동 로봇 운동학으로 추종기를 돌려 최종 상태를 반환."""
    for _ in range(steps):
        v, w, state = follower.compute(x, y, yaw)
        if state in ('done', 'off_path'):
            return x, y, yaw, state
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
    return x, y, yaw, 'timeout'


def test_straight_line_reaches_goal_and_yaw():
    pts = [(i * 0.05, 0.0) for i in range(21)]  # 1m 직선
    x, y, yaw, state = simulate(PurePursuit(pts, goal_yaw=math.pi / 2), 0.0, 0.0, 0.0)
    assert state == 'done'
    assert math.hypot(x - 1.0, y) <= 0.08
    assert abs(math.atan2(math.sin(yaw - math.pi / 2), math.cos(yaw - math.pi / 2))) <= 0.1


def test_l_shaped_path():
    pts = [(i * 0.05, 0.0) for i in range(21)] + [(1.0, j * 0.05) for j in range(1, 21)]
    x, y, _, state = simulate(PurePursuit(pts, goal_yaw=math.pi / 2), 0.0, 0.0, 0.0)
    assert state == 'done'
    assert math.hypot(x - 1.0, y - 1.0) <= 0.08


def test_rotates_in_place_when_facing_backwards():
    pts = [(-i * 0.05, 0.0) for i in range(21)]
    f = PurePursuit(pts, goal_yaw=math.pi)
    v, w, state = f.compute(0.0, 0.0, 0.0)
    assert state == 'rotating' and v == 0.0 and w != 0.0
    assert simulate(f, 0.0, 0.0, 0.0)[3] == 'done'


def test_off_path_detected():
    pts = [(i * 0.05, 0.0) for i in range(21)]
    assert PurePursuit(pts, 0.0).compute(0.5, 1.0, 0.0)[2] == 'off_path'


def test_speed_limits():
    pts = [(i * 0.05, 0.0) for i in range(100)]
    f = PurePursuit(pts, 0.0, max_v=0.15, max_w=1.0)
    v, w, _ = f.compute(0.0, 0.05, 0.3)
    assert 0.0 < v <= 0.15 and abs(w) <= 1.0


def test_empty_path_rejected():
    with pytest.raises(ValueError):
        PurePursuit([], 0.0)
