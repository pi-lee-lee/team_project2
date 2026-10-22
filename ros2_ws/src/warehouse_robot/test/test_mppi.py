"""MPPI 폐루프 (ROS 없음): 경로 따라가기, 경로 옆 상자 비켜 가기, 가로지르는 물체 피하기."""
import math

import numpy as np

from warehouse_robot.dwa import DWAConfig
from warehouse_robot.mppi import MPPIFollower

CFG = dict(max_v=0.13, min_v=-0.05, max_w=1.82, max_acc_v=0.3, max_acc_w=1.5, robot_radius=0.17,
           footprint=(-0.197, 0.069, 0.153), allow_escape=True, safety_margin=0.08, predict_time=2.5,
           control_period=0.1, reverse_only_when_needed=True)


def rect_gap(x, y, yaw, px, py):
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * (px - x) - s * (py - y), s * (px - x) + c * (py - y)
    return math.hypot(max(-0.197 - lx, lx - 0.069, 0.0), max(abs(ly) - 0.153, 0.0))


def drive(path, walls, limit=60.0, dyn=lambda t: []):
    f = MPPIFollower(path, 0.0, DWAConfig(**CFG), lookahead=0.3)
    x = y = yaw = v = w = 0.0
    t, gap = 0.0, math.inf
    walls = np.asarray(walls, dtype=float).reshape(-1, 2)
    while t < limit:
        v, w, state = f.compute((x, y, yaw), v, w, (0, 0, 0), walls, dyn(t))
        if state == 'done':
            return True, t, gap
        yaw += w * 0.1
        x += v * math.cos(yaw) * 0.1
        y += v * math.sin(yaw) * 0.1
        t += 0.1
        for px, py in walls:
            gap = min(gap, rect_gap(x, y, yaw, px, py))
    return False, t, gap


def test_follows_straight_path():
    ok, t, _ = drive([[x, 0.0] for x in np.linspace(0, 1.5, 31)], [])
    assert ok and t < 20


def test_passes_box_beside_path():
    # 경로에서 0.2m 옆 상자: 차체(반폭 0.153) + 여유 0.08 이 걸리므로 조금 비켜서 지나감 (재계획 없이)
    box = [[x, y] for x in np.linspace(0.9, 1.1, 5) for y in np.linspace(0.2, 0.4, 5)]
    ok, _, gap = drive([[x, 0.0] for x in np.linspace(0, 2.0, 41)], box)
    assert ok and gap > 0.05


def test_avoids_crossing_object():
    # x=1.0 을 위에서 아래로 가로지르는 원기둥 (0.1 m/s): 기다리거나 비켜서 닿지 않고 도착
    def dyn(t):
        k = max(0.0, t - 1.5)
        return [[1.0, 0.6 - 0.1 * k, 0.0, -0.1 if t >= 1.5 else 0.0, 0.12]]
    f = MPPIFollower([[x, 0.0] for x in np.linspace(0, 2.0, 41)], 0.0, DWAConfig(**CFG), lookahead=0.3)
    x = y = yaw = v = w = 0.0
    t, gap, state = 0.0, math.inf, ''
    while t < 60 and state != 'done':
        v, w, state = f.compute((x, y, yaw), v, w, (0, 0, 0), [], dyn(t))
        yaw += w * 0.1
        x += v * math.cos(yaw) * 0.1
        y += v * math.sin(yaw) * 0.1
        t += 0.1
        ox, oy, _, _, r = dyn(t)[0]
        gap = min(gap, rect_gap(x, y, yaw, ox, oy) - r)
    assert state == 'done' and gap > 0.0
