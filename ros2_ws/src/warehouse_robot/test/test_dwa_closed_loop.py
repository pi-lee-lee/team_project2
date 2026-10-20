"""추적기 + DWA 폐루프: 가상 LiDAR(원기둥만) → tracker → DWAFollower → 로봇 이동.

sim 에서 가로지르는 원기둥에 부딪힌 원인을 가르기 위한 시험 (브리지/가제보 없이 같은 장면).
"""
import math

import numpy as np

from warehouse_robot.dwa import DWAConfig, DWAFollower
from warehouse_robot.tracker import MultiObjectTracker, detect_clusters

# Waffle Pi 외곽 (base_footprint 기준): 앞 0.069, 뒤 -0.197, 좌우 0.153
FOOT = (-0.197, 0.069, -0.153, 0.153)


def footprint_gap(x, y, yaw, cx, cy, r):
    """로봇 직사각형과 원 사이 거리 (음수 = 겹침)."""
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * (cx - x) - s * (cy - y), s * (cx - x) + c * (cy - y)
    qx = min(max(lx, FOOT[0]), FOOT[1])
    qy = min(max(ly, FOOT[2]), FOOT[3])
    return math.hypot(lx - qx, ly - qy) - r


def fake_scan(x, y, yaw, cx, cy, r, n=360, max_range=3.5):
    """원기둥 하나만 있는 공간에서 LiDAR(로봇 x-0.064) 점 (odom 프레임, 스캔 순서)."""
    sx, sy = x - 0.064 * math.cos(yaw), y - 0.064 * math.sin(yaw)
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False) + yaw:
        dx, dy = math.cos(a), math.sin(a)
        ox, oy = sx - cx, sy - cy
        b = ox * dx + oy * dy
        disc = b * b - (ox * ox + oy * oy - r * r)
        if disc < 0:
            continue
        t = -b - math.sqrt(disc)
        if 0.12 < t < max_range:
            pts.append((sx + t * dx, sy + t * dy))
    return np.array(pts).reshape(-1, 2), (sx, sy)


def run(mover_start_delay, perfect=False, cfg=None, steps=400, dt=0.05):
    """로봇: (0.2, 0) → (2.0, 0) 직선 경로. 원기둥: x=1.0 에서 y 0.38 → -1.0 을 0.25 m/s 로 한 번 지나감."""
    path = [[x, 0.0] for x in np.linspace(0.2, 2.0, 37)]
    f = DWAFollower(path, 0.0, cfg or DWAConfig(max_v=0.18, control_period=0.1))
    tracker = MultiObjectTracker()
    x, y, yaw, v, w = 0.2, 0.0, 0.0, 0.0, 0.0
    r = 0.12
    tracks, scan_pts = [], np.empty((0, 2))
    min_gap, cmd = math.inf, (0.0, 0.0, 'driving')
    for k in range(steps):
        t = k * dt
        my = 0.38 - 0.25 * max(0.0, t - mover_start_delay)
        my = max(my, -1.0)
        mvy = -0.25 if mover_start_delay < t and my > -1.0 else 0.0
        if k % 4 == 0:   # LiDAR 5 Hz → 추적기
            scan_pts, sensor = fake_scan(x, y, yaw, 1.0, my, r)
            tracks = tracker.update(detect_clusters(scan_pts, 0.15, 3, 0.8, 0.05, sensor), t)
        if k % 2 == 0:   # DWA 10 Hz (브리지와 같음)
            dyn = ([[1.0, my, 0.0, mvy, r]] if perfect
                   else [[tr.x[0], tr.x[1], tr.x[2], tr.x[3], tr.radius] for tr in tracks])
            cmd = f.compute((x, y, yaw), v, w, (0.0, 0.0, 0.0), [] if perfect else scan_pts, dyn)
        v, w, state = cmd
        if state == 'done':
            break
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        min_gap = min(min_gap, footprint_gap(x, y, yaw, 1.0, my, r))
    return min_gap, (x, y), state


def test_crossing_with_perfect_obstacle_info():
    for delay in (1.0, 2.0, 3.0):   # 4초 이상은 원기둥이 로봇이 이미 들어선 줄로 내려와 피할 수 없음
        gap, end, state = run(delay, perfect=True)
        assert gap > 0.0, f'delay {delay}: 겹침 {gap:.3f}'


def test_crossing_with_tracker():
    for delay in (1.0, 2.0, 3.0):   # 4초 이상은 원기둥이 로봇이 이미 들어선 줄로 내려와 피할 수 없음
        gap, end, state = run(delay)
        assert gap > 0.0, f'delay {delay}: 겹침 {gap:.3f}'


if __name__ == '__main__':
    for perfect in (True, False):
        for delay in (1.0, 2.0, 3.0, 4.0, 5.0):
            gap, end, state = run(delay, perfect=perfect)
            print(f'{"정답 정보" if perfect else "추적기   "} 원기둥 출발 {delay:.0f}초: 최소 여유 {gap:+.3f} m, 끝 {state} ({end[0]:.2f}, {end[1]:.2f})')


PROJECT = dict(max_v=0.18, control_period=0.1, robot_radius=0.17, footprint=(-0.197, 0.069, 0.153),
               allow_escape=True, safety_margin=0.08, dynamic_margin=0.25, predict_time=3.0)


def test_narrow_corridor_passable_with_rectangle():
    """폭 0.62m 통로 (벽이 중심에서 0.31m): 원 하나(반경 0.25 + 여유 0.08)면 앞으로 못 가고, 직사각형이면 지나간다."""
    walls = [[x, yy] for x in np.arange(-0.5, 2.5, 0.03) for yy in (-0.31, 0.31)]
    path = [[x, 0.0] for x in np.linspace(0.0, 2.0, 41)]

    def drive(cfg):
        f = DWAFollower(path, 0.0, cfg)
        x = y = yaw = v = w = 0.0
        for _ in range(300):
            v, w, state = f.compute((x, y, yaw), v, w, (0, 0, 0), walls, [])
            if state == 'done':
                return x
            yaw += w * 0.1
            x += v * math.cos(yaw) * 0.1
            y += v * math.sin(yaw) * 0.1
        return x

    one = dict(PROJECT, robot_radius=0.25, circle_offsets=(0.0,), footprint=(), allow_escape=False)
    assert drive(DWAConfig(**one)) < 0.3          # 거의 못 나아감
    assert drive(DWAConfig(**PROJECT)) > 1.85     # 끝까지 감


def test_crossing_with_project_params():
    for delay in (1.0, 2.0, 3.0):
        gap, end, state = run(delay, cfg=DWAConfig(**PROJECT))
        assert gap > 0.0, f'delay {delay}: 겹침 {gap:.3f}'


def test_escape_when_parked_next_to_wall():
    """sim 에서 갇힌 자세 재현: 로봇 뒤 벽이 뒷면에서 3.8cm. 원 두 개(뒤로 0.30 까지)는 '이미 닿음' 으로 갇히고,
    직사각형은 앞으로 출발."""
    wall = [[-0.235, yy] for yy in np.arange(-1.0, 1.0, 0.02)]      # 뒷면(-0.197)에서 3.8cm
    path = [[x, 0.0] for x in np.linspace(0.0, 1.5, 31)]
    circles = DWAConfig(**dict(PROJECT, footprint=(), circle_offsets=(-0.13, 0.0), allow_escape=False))
    v, w, state = DWAFollower(path, 0.0, circles).compute((0, 0, 0), 0.0, 0.0, (0, 0, 0), wall, [])
    assert state == 'blocked'
    v, w, state = DWAFollower(path, 0.0, DWAConfig(**PROJECT)).compute((0, 0, 0), 0.0, 0.0, (0, 0, 0), wall, [])
    assert state == 'driving' and v > 0.0
