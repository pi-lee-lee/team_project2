import math

import pytest

from warehouse_robot.follower import PurePursuit


def simulate(follower, x, y, yaw, dt=0.05, steps=2000):
    """차동 구동 로봇 운동학으로 추종기를 돌려 최종 상태를 반환."""
    for i in range(steps):
        v, w, state = follower.compute(x, y, yaw, now=i * dt)
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


def run_profile(f, dt=0.05, steps=2000):
    """직선 경로를 따라 (시각, 선속도) 기록과 마지막 상태."""
    x, y, yaw, log = 0.0, 0.0, 0.0, []
    for i in range(steps):
        v, w, state = f.compute(x, y, yaw, now=i * dt)
        log.append(v)
        if state in ('done', 'off_path'):
            return log, x, state
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
    return log, x, 'timeout'


def test_start_ramps_with_acceleration_limit():
    pts = [(i * 0.05, 0.0) for i in range(41)]  # 2m
    f = PurePursuit(pts, 0.0, max_v=0.26, max_acc=0.3)
    log, _, state = run_profile(f)
    assert state == 'done'
    assert log[0] <= 0.3 * 0.05 + 1e-9           # 첫 명령부터 최고 속도로 튀지 않음
    ups = [b - a for a, b in zip(log, log[1:]) if b > a]
    assert max(ups) <= 0.3 * 0.05 + 1e-9         # 가속은 한도 안
    assert max(log) == pytest.approx(0.26)       # 결국 최고 속도까지


def test_stop_profile_decelerates_smoothly_and_arrives():
    pts = [(i * 0.05, 0.0) for i in range(41)]
    f = PurePursuit(pts, 0.0, max_v=0.26, max_acc=0.3)
    log, x, state = run_profile(f)
    assert state == 'done' and abs(x - 2.0) <= 0.08
    downs = [a - b for a, b in zip(log, log[1:]) if b < a]
    assert max(downs[:-1]) <= 2 * 0.3 * 0.05 + 1e-9   # 마지막(허용 오차 안에서 0) 말고는 급정지 없음
    assert log[-2] <= 0.08                          # 목표 직전 속도는 느림


def test_set_speed_restarts_ramp_after_external_stop():
    pts = [(i * 0.05, 0.0) for i in range(100)]
    f = PurePursuit(pts, 0.0, max_v=0.26, max_acc=0.3)
    for i in range(40):
        v, _, _ = f.compute(0.0, 0.0, 0.0, now=i * 0.05)
    assert v == pytest.approx(0.26)
    f.set_speed(0.0, 0.0)                            # 앞이 막혀 브리지가 0 을 보냄
    v, _, _ = f.compute(0.0, 0.0, 0.0, now=2.05)
    assert v <= 0.3 * 0.05 + 1e-9


def test_long_gap_restarts_ramp():
    pts = [(i * 0.05, 0.0) for i in range(100)]
    f = PurePursuit(pts, 0.0, max_v=0.26, max_acc=0.3)
    for i in range(40):
        f.compute(0.0, 0.0, 0.0, now=i * 0.05)
    v, _, _ = f.compute(0.0, 0.0, 0.0, now=10.0)    # 물러나기 동작 뒤 다시 불림
    assert v <= 0.3 * 0.05 + 1e-9


def test_in_place_rotation_ramps_and_settles():
    pts = [(-i * 0.05, 0.0) for i in range(21)]
    f = PurePursuit(pts, goal_yaw=math.pi, max_w=1.82, max_acc_w=1.5)
    ws, yaw = [], 0.0
    for i in range(200):
        v, w, state = f.compute(0.0, 0.0, yaw, now=i * 0.05)
        if state != 'rotating':
            break
        ws.append(abs(w))
        yaw += w * 0.05
    assert ws[0] <= 0.15 + 1e-9                     # 최소 회전 속도부터
    ups = [b - a for a, b in zip(ws, ws[1:]) if b > a]
    assert max(ups) <= 1.5 * 0.05 + 1e-9
