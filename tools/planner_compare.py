"""hybrid 주행 (pure pursuit ↔ DWA) 비교 시뮬레이션 (ROS 없음, A* / DWA / pure pursuit 모듈만 사용).

    python3 tools/planner_compare.py                    # params.yaml 그대로 + 변형들 x 전체 시나리오
    python3 tools/planner_compare.py --only A,pred0 --scen box_center -v

브리지(bridge_node.py)의 주행 로직을 단순화해 흉내 낸다:
  - A* 는 지도(벽) + LiDAR 로 2.5초 넘게 같은 칸에 보인 '지도에 없는 점'(scan_persist, 20초 기억)으로 계획.
    그걸로 경로가 없으면 지도만으로 계획 (astar_planner_node 와 같음)
  - 평소 pure pursuit (앞쪽 ±25° 0.35m 정지 규칙). 로봇 앞 사각형(hybrid.box_*) 안에 지도에 없는 점이 있거나
    움직이는 물체가 hybrid.moving_predict 초 안에 사각형에 들어오면 DWA, 지나가면 pure pursuit (브리지 hybrid_switch 와 같음)
  - 진행 없음 15초 / 막힘 5초 → (DWA 면) 뒤로 15cm → 재계획, 재시도 2번 넘으면 실패
LiDAR 는 360개 빔(최대 3.5m, 0.2초마다)을 벽/상자/사람에 직접 쏴서 만든다. 상자는 지도에 없다 (출발 때 A* 가 모름).
움직이는 사람은 추적 결과 대신 정답 위치/속도를 준다 (dwa_tune 과 같음).
"""
import argparse
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'warehouse_robot'))
from scipy.spatial import cKDTree  # noqa: E402
from warehouse_robot.astar import GridPlanner, PlanningError, obstacle_distance  # noqa: E402
from warehouse_robot.dwa import DWAConfig, DWAFollower  # noqa: E402
from warehouse_robot.follower import PurePursuit  # noqa: E402

PARAMS = os.path.join(HERE, '..', 'ros2_ws', 'src', 'warehouse_robot', 'config', 'params.yaml')
DT = 0.1
RES = 0.05          # 지도 / A* 해상도
TRUTH = 0.025       # LiDAR 를 쏘는 실제 세계 해상도 (지도 칸의 1/2)
LIDAR_X = -0.064
BODY = (-0.197, 0.069, 0.153)
EXTRA = (-0.277, -0.197, 0.04)   # 뒤에 붙인 물건 (x_min, x_max, 반폭)

# 변형: dwa / astar / hybrid_cfg = params.yaml 값 덮어쓰기, pp_speed = pure pursuit 최고 속도
STRATEGIES = {
    'A': dict(desc='params.yaml 그대로 (기본)'),
    'pred0': dict(hybrid_cfg=dict(moving_predict=0.0), desc='움직이는 물체도 지금 위치만 (예측 없음)'),
    'pred1': dict(hybrid_cfg=dict(moving_predict=1.0), desc='예측 1초'),
    'pred3': dict(hybrid_cfg=dict(moving_predict=3.0), desc='예측 3초'),
    'front1': dict(hybrid_cfg=dict(box_front=1.0), desc='box_front 1.0'),
    'accw15': dict(dwa=dict(max_acc_w=1.5), desc='dwa.max_acc_w 1.5'),
    'old3': dict(hybrid_cfg=dict(box_front=1.0), dwa=dict(max_acc_w=1.5), pp_speed=0.26,
                 desc='box_front 1.0 + max_acc_w 1.5 + PP 0.26'),
}


def load_params():
    p = yaml.safe_load(open(PARAMS))
    a = p['astar_planner']['ros__parameters']
    b = p['warehouse_bridge']['ros__parameters']
    dwa = {k: tuple(v) if isinstance(v, list) else v for k, v in b['dwa'].items()}
    dwa['control_period'] = 0.1
    return a, b, dwa


# ------------------------------------------------------------------ 세계
def rect(x0, x1, y0, y1):
    return (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1))


def room_walls(x0, x1, y0, y1, t=0.1):
    """(x0..x1, y0..y1) 안쪽이 빈 방의 네 벽."""
    return [rect(x0 - t, x1 + t, y0 - t, y0), rect(x0 - t, x1 + t, y1, y1 + t),
            rect(x0 - t, x0, y0, y1), rect(x1, x1 + t, y0, y1)]


def scenarios():
    S = {}
    # 1. 폭 1.3m 통로 가운데 상자 0.3m (옆 틈 0.5m 씩)
    S['box_center'] = dict(walls=room_walls(-0.6, 4.6, -0.65, 0.65), boxes=[rect(1.85, 2.15, -0.15, 0.15)],
                           start=(0.0, 0.0, 0.0), goal=(4.0, 0.0))
    # 2. 폭 1.2m, 상자가 위로 치우침 (위 틈 0.35m 는 못 지나감, 아래 0.55m)
    S['box_offset'] = dict(walls=room_walls(-0.6, 4.6, -0.6, 0.6), boxes=[rect(1.85, 2.15, -0.05, 0.25)],
                           start=(0.0, 0.0, 0.0), goal=(4.0, 0.0))
    # 3. 넓은 방 가운데 큰 상자 0.6 x 0.8
    S['room_bigbox'] = dict(walls=room_walls(-0.6, 4.6, -1.5, 1.5), boxes=[rect(1.7, 2.3, -0.4, 0.4)],
                            start=(0.0, 0.0, 0.0), goal=(4.0, 0.0))
    # 4. ㄱ자 통로(폭 1.3m), 꺾인 직후 상자
    walls = [rect(-0.7, 3.75, -0.75, -0.65), rect(-0.7, 2.35, 0.65, 0.75), rect(-0.7, -0.6, -0.65, 0.65),
             rect(3.65, 3.75, -0.65, 4.1), rect(2.25, 2.35, 0.65, 4.1), rect(2.25, 3.75, 4.0, 4.1)]
    S['corner_box'] = dict(walls=walls, boxes=[rect(2.85, 3.15, 1.65, 1.95)], start=(0.0, 0.0, 0.0), goal=(3.0, 3.5))
    # 5. 두 갈래 길. 위(폭 1.0m) 가 A* 첫 경로인데 상자로 완전히 막힘 → 아래(0.9m)로 돌아가야 함
    walls = room_walls(-1.0, 5.0, -1.4, 1.5) + [rect(0.5, 3.5, -0.5, 0.5)]
    S['blocked_route'] = dict(walls=walls, boxes=[rect(1.8, 2.2, 0.5, 1.5)], start=(-0.5, 0.0, 0.0), goal=(4.5, 0.0))
    # 6. 폭 1.6m 통로, 사람이 x=2.0 에서 통로를 가로질러 왔다 갔다 (0.2 m/s, 끝에서 8초 쉼).
    #    쉬지 않고 왔다 갔다 하면 통로 전체를 5.5초마다 쓸어서 0.13 m/s 로는 안전하게 지나갈 틈이 아예 없음
    S['person_cross'] = dict(walls=room_walls(-0.6, 4.6, -0.8, 0.8), boxes=[], start=(0.0, 0.0, 0.0), goal=(4.0, 0.0),
                             movers=[dict(kind='pingpong', p0=(2.0, 0.55), p1=(2.0, -0.55), speed=0.2, r=0.2,
                                          pause=8.0)])
    # 7. 폭 1.6m 통로, 사람이 경로를 따라 마주 옴 (0.15 m/s)
    S['person_headon'] = dict(walls=room_walls(-0.6, 4.6, -0.8, 0.8), boxes=[], start=(0.0, 0.0, 0.0), goal=(4.0, 0.0),
                              movers=[dict(kind='line', p0=(4.3, 0.0), v=(-0.15, 0.0), r=0.2, until=40.0)])
    # 8. 사람이 걸어와 경로 한가운데 서 버림 (멈춘 뒤엔 상자와 같음)
    S['person_stops'] = dict(walls=room_walls(-0.6, 4.6, -0.8, 0.8), boxes=[], start=(0.0, 0.0, 0.0), goal=(4.0, 0.0),
                             movers=[dict(kind='line', p0=(2.2, 0.75), v=(0.0, -0.15), r=0.2, until=5.0)])
    # 9. 장애물 없는 S자 좁은 통로 (폭 1.0m) — 평소 경로 추종 / 벽 여유
    walls = [rect(-0.6, 2.5, -0.6, -0.5), rect(-0.6, 1.5, 0.5, 0.6), rect(-0.7, -0.6, -0.6, 0.6),
             rect(2.5, 2.6, -0.6, 3.1), rect(1.5, 1.6, 0.5, 2.0), rect(-0.5, 1.6, 2.0, 2.1),
             rect(-0.6, 2.6, 3.0, 3.1), rect(-0.7, -0.6, 2.0, 3.1)]
    S['s_corridor'] = dict(walls=walls, boxes=[], start=(0.0, 0.0, 0.0), goal=(0.0, 2.55))
    # --- 극한 시나리오 ---
    # 10. 폭 1.4m 통로에 상자 3개가 좌우 번갈아 (슬라럼)
    S['x_slalom'] = dict(walls=room_walls(-0.6, 4.6, -0.7, 0.7),
                         boxes=[rect(1.0, 1.3, -0.05, 0.3), rect(2.0, 2.3, -0.3, 0.05), rect(3.0, 3.3, -0.05, 0.3)],
                         start=(0.0, 0.0, 0.0), goal=(4.2, 0.0))
    # 11. 폭 1.0m 통로, 벽에 붙은 상자로 남은 틈 0.5m
    S['x_tight_gap'] = dict(walls=room_walls(-0.6, 4.6, -0.5, 0.5), boxes=[rect(1.8, 2.1, -0.5, 0.0)],
                            start=(0.0, 0.0, 0.0), goal=(4.0, 0.0))
    # 12. 방 가운데 ㄷ자 상자 함정 (입구가 로봇 쪽): 들어가면 뒤로 나와 돌아가야 함
    S['x_trap'] = dict(walls=room_walls(-0.6, 4.6, -1.5, 1.5),
                       boxes=[rect(2.2, 2.35, -0.6, 0.6), rect(1.4, 2.35, 0.45, 0.6), rect(1.4, 2.35, -0.6, -0.45)],
                       start=(0.0, 0.0, 0.0), goal=(4.0, 0.0))
    # 13. 폭 1.4m 통로, 로봇이 0.8m 앞까지 오면 경로 위에 물체가 갑자기 나타남 (반경 0.2)
    S['x_sudden'] = dict(walls=room_walls(-0.6, 4.6, -0.7, 0.7), boxes=[], start=(0.0, 0.0, 0.0), goal=(4.0, 0.0),
                         movers=[dict(kind='appear', p=(2.0, 0.0), r=0.2, near=0.8)])
    # 14. 방에서 사람 3명이 경로를 가로질러 왔다 갔다 + 상자 1개
    S['x_crowd'] = dict(walls=room_walls(-0.6, 4.6, -1.5, 1.5), boxes=[rect(2.45, 2.75, -0.15, 0.15)],
                        start=(0.0, 0.0, 0.0), goal=(4.0, 0.0),
                        movers=[dict(kind='pingpong', p0=(1.2, 1.2), p1=(1.2, -1.2), speed=0.25, r=0.2, pause=3.0),
                                dict(kind='pingpong', p0=(3.3, -1.2), p1=(3.3, 1.2), speed=0.2, r=0.2, pause=5.0),
                                dict(kind='line', p0=(4.3, 0.4), v=(-0.12, 0.0), r=0.2, until=60.0)])
    # 15. 폭 1.2m 통로, 마주 오는 사람 + 그 뒤 상자
    S['x_headon_box'] = dict(walls=room_walls(-0.6, 4.6, -0.6, 0.6), boxes=[rect(2.8, 3.1, -0.15, 0.15)],
                             start=(0.0, 0.0, 0.0), goal=(4.0, 0.0),
                             movers=[dict(kind='line', p0=(2.5, 0.0), v=(-0.15, 0.0), r=0.2, until=40.0)])
    S.update(real_map_scenarios())
    return S


REAL_MAP = '/home/abc/maps/map2_clean.yaml'


def real_map_scenarios(n=6, seed=3):
    """실물 지도(map2_clean) 위 무작위 출발/목표 + 경로 위에 상자 1~2개 (지도에 없음). 상자를 넣어도 A* 경로가 있는 것만."""
    if not os.path.exists(REAL_MAP):
        return {}
    from PIL import Image
    meta = yaml.safe_load(open(REAL_MAP))
    img = np.flipud(np.array(Image.open(os.path.join(os.path.dirname(REAL_MAP), meta['image']))))
    grid = img < 100        # 0 = 벽, 205 = 미탐색 (벽 바깥), 254 = 빈 칸
    unknown = (img > 100) & (img < 250)
    ox, oy = meta['origin'][:2]
    a = yaml.safe_load(open(PARAMS))['astar_planner']['ros__parameters']
    rng = np.random.default_rng(seed)
    occ = np.where(grid | unknown, 100, 0).astype(np.int16)
    base = GridPlanner(occ.ravel().tolist(), grid.shape[1], grid.shape[0], RES, ox, oy,
                       robot_radius=0.3, inflation_radius=0.55)
    free = np.argwhere(~base.lethal)
    out = {}
    tries = 0
    while len(out) < n and tries < 500:
        tries += 1
        (r0, c0), (r1, c1) = free[rng.integers(len(free), size=2)]
        start, goal = base.cell_to_world(r0, c0), base.cell_to_world(r1, c1)
        try:
            path = np.asarray(base.plan(start, goal))
        except PlanningError:
            continue
        L = np.hypot(*np.diff(path, axis=0).T).sum()
        if L < 3.0:
            continue
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
        boxes = []
        for frac in rng.uniform(0.3, 0.75, size=rng.integers(1, 3)):
            i = int(np.searchsorted(s, frac * L))
            bx, by = path[i] + rng.uniform(-0.12, 0.12, 2)
            half = rng.uniform(0.12, 0.18)
            boxes.append(rect(bx - half, bx + half, by - half, by + half))
        # 상자를 넣어도 갈 수 있어야 함 (반경 0.25: 차체 반폭 0.153 + DWA 여유 0.08 이 지나가는 틈), 출발/목표가 상자에 걸리면 안 됨
        g2 = occ.copy()
        for x0, x1, y0, y1 in boxes:
            c_0, c_1 = int((x0 - ox) / RES), int(math.ceil((x1 - ox) / RES))
            r_0, r_1 = int((y0 - oy) / RES), int(math.ceil((y1 - oy) / RES))
            g2[r_0:r_1, c_0:c_1] = 100
        chk = GridPlanner(g2.ravel().tolist(), grid.shape[1], grid.shape[0], RES, ox, oy,
                          robot_radius=0.25, inflation_radius=a['inflation_radius'])
        if not chk.is_free(*start) or not chk.is_free(*goal):
            continue
        try:
            chk.plan(start, goal)
        except PlanningError:
            continue
        yaw0 = math.atan2(path[min(5, len(path) - 1)][1] - start[1], path[min(5, len(path) - 1)][0] - start[0])
        out[f'map2_{len(out)}'] = dict(grid=(grid | unknown, ox, oy), walls=[], boxes=boxes,
                                       start=(start[0], start[1], yaw0), goal=goal)
    return out


class World:
    def __init__(self, sc):
        self.sc = sc
        if 'grid' in sc:
            # 실제 지도 (pgm) + 지도에 없는 상자
            grid, self.ox, self.oy = sc['grid']
            self.h, self.w = grid.shape
            self.map = grid.copy()
            self.truth = np.kron(grid, np.ones((2, 2), dtype=bool)) | \
                self._raster(sc['boxes'], TRUTH, self.w * 2, self.h * 2)
        else:
            rects = sc['walls'] + sc['boxes']
            xs = [r[0] for r in rects] + [r[1] for r in rects]
            ys = [r[2] for r in rects] + [r[3] for r in rects]
            self.ox, self.oy = min(xs) - 0.2, min(ys) - 0.2
            self.w = int(math.ceil((max(xs) + 0.2 - self.ox) / RES))
            self.h = int(math.ceil((max(ys) + 0.2 - self.oy) / RES))
            self.map = self._raster(sc['walls'], RES, self.w, self.h)          # A* / 지도 (상자 없음)
            self.truth = self._raster(rects, TRUTH, self.w * 2, self.h * 2)
        self.map_dist = obstacle_distance(self.map, RES, 0.5)
        occ = np.argwhere(self.truth)
        self.occ_pts = np.column_stack([self.ox + (occ[:, 1] + 0.5) * TRUTH, self.oy + (occ[:, 0] + 0.5) * TRUTH])
        self.occ_tree = cKDTree(self.occ_pts)
        self.movers = sc.get('movers', [])
        ang = np.radians(np.arange(360))
        self.ray_dir = np.column_stack([np.cos(ang), np.sin(ang)])
        self.ray_s = np.arange(0.02, 3.5, TRUTH / 1.5)
        self.robot_xy = sc['start'][:2]

    def _raster(self, rects, res, w, h):
        g = np.zeros((h, w), dtype=bool)
        for x0, x1, y0, y1 in rects:
            c0, c1 = int(math.floor((x0 - self.ox) / res)), int(math.ceil((x1 - self.ox) / res))
            r0, r1 = int(math.floor((y0 - self.oy) / res)), int(math.ceil((y1 - self.oy) / res))
            g[max(r0, 0):max(r1, 0), max(c0, 0):max(c1, 0)] = True
        return g

    def mover_state(self, t):
        """[[x, y, vx, vy, r], ...] (정답)."""
        out = []
        for m in self.movers:
            if m['kind'] == 'appear':
                # 로봇이 near 만큼 다가오면 그 자리에 갑자기 나타나는 물체 (사람이 상자를 내려놓음 등)
                if not m.get('_on') and math.hypot(self.robot_xy[0] - m['p'][0], self.robot_xy[1] - m['p'][1]) < m['near']:
                    m['_on'] = True
                if m.get('_on'):
                    out.append([m['p'][0], m['p'][1], 0.0, 0.0, m['r']])
                continue
            if m['kind'] == 'line':
                k = min(t, m['until'])
                moving = t < m['until']
                out.append([m['p0'][0] + m['v'][0] * k, m['p0'][1] + m['v'][1] * k,
                            m['v'][0] * moving, m['v'][1] * moving, m['r']])
            else:
                p0, p1 = np.array(m['p0']), np.array(m['p1'])
                L = np.hypot(*(p1 - p0))
                u = (p1 - p0) / L
                go, rest = L / m['speed'], m.get('pause', 0.0)
                k = t % (2 * (go + rest))
                if k < go:                          # p0 → p1
                    pos, vel = p0 + u * m['speed'] * k, u * m['speed']
                elif k < go + rest:                 # p1 에서 쉼
                    pos, vel = p1, u * 0.0
                elif k < 2 * go + rest:             # p1 → p0
                    pos, vel = p1 - u * m['speed'] * (k - go - rest), -u * m['speed']
                else:                               # p0 에서 쉼
                    pos, vel = p0, u * 0.0
                out.append([pos[0], pos[1], vel[0], vel[1], m['r']])
        return np.array(out, dtype=float).reshape(-1, 5)

    def scan(self, x, y, yaw, t):
        """LiDAR 점 (map 좌표, (N, 2)) 와 빔별 거리 / 각도 (로봇 기준)."""
        lx, ly = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        d = self.ray_dir @ np.array([[c, s], [-s, c]])      # 빔 방향 (map)
        px = lx + d[:, 0:1] * self.ray_s[None]
        py = ly + d[:, 1:2] * self.ray_s[None]
        col = np.floor((px - self.ox) / TRUTH).astype(int)
        row = np.floor((py - self.oy) / TRUTH).astype(int)
        th, tw = self.truth.shape
        ok = (row >= 0) & (row < th) & (col >= 0) & (col < tw)
        hit = np.zeros_like(ok)
        hit[ok] = self.truth[row[ok], col[ok]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        rng = np.where(first >= 0, self.ray_s[np.maximum(first, 0)], np.inf)
        for mx, my, _, _, r in self.mover_state(t):
            ox_, oy_ = mx - lx, my - ly
            b = d[:, 0] * ox_ + d[:, 1] * oy_
            disc = b ** 2 - (ox_ ** 2 + oy_ ** 2 - r ** 2)
            tt = b - np.sqrt(np.maximum(disc, 0.0))
            rng = np.where((disc > 0) & (tt > 0) & (tt < rng), tt, rng)
        fin = np.isfinite(rng)
        pts = np.column_stack([lx + d[fin, 0] * rng[fin], ly + d[fin, 1] * rng[fin]])
        return pts, rng, np.radians(np.arange(360))

    def unmapped(self, pts):
        """지도 벽에서 0.15m 넘게 떨어진 점 (= 지도에 없는 물체)."""
        col = np.floor((pts[:, 0] - self.ox) / RES).astype(int)
        row = np.floor((pts[:, 1] - self.oy) / RES).astype(int)
        ok = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        keep = np.zeros(len(pts), dtype=bool)
        keep[ok] = self.map_dist[row[ok], col[ok]] > 0.15
        return pts[keep]

    def body_gap(self, x, y, yaw, t):
        """차체(+뒤 부착물) 외곽과 실제 장애물 사이 최소 거리 [m] (음수 = 닿음)."""
        pts = []
        for x0, x1, hw in (BODY, EXTRA):
            for px in np.arange(x0, x1 + 1e-9, 0.02):
                pts += [(px, hw), (px, -hw)]
            for py in np.arange(-hw, hw + 1e-9, 0.02):
                pts += [(x0, py), (x1, py)]
        p = np.array(pts)
        c, s = math.cos(yaw), math.sin(yaw)
        wp = np.column_stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]])
        g = float(self.occ_tree.query(wp)[0].min()) - TRUTH / 2
        # 외곽점이 벽 안으로 완전히 들어가도 잡히게: 로봇 중심 칸이 벽이면 음수
        for mx, my, _, _, r in self.mover_state(t):
            g = min(g, float(np.hypot(wp[:, 0] - mx, wp[:, 1] - my).min()) - r)
        return g


# ------------------------------------------------------------------ 주행 (브리지 흉내)
class Run:
    def __init__(self, world, strat, params, verbose=False):
        self.wd, self.st, self.verbose = world, strat, verbose
        self.a, self.b, dwa = params
        self.a = {**self.a, **strat.get('astar', {})}
        self.cfg = DWAConfig(**{**dwa, **strat.get('dwa', {})})
        self.h = {**self.b['hybrid'], **strat.get('hybrid_cfg', {})}
        self.memory = {}          # 칸 → (처음 본 시각, 마지막 본 시각)
        self.log = []
        self.replans = 0
        self.switches = 0
        self.odo = 0.0
        self.box_streak = 0

    def say(self, t, msg):
        self.log.append(f'{t:6.1f}s {msg}')
        if self.verbose:
            print(self.log[-1])

    def remember(self, pts, t):
        """scan_persist 2.5초 / scan_memory 20초 흉내."""
        um = self.wd.unmapped(pts)
        cells = set(map(tuple, np.floor((um - [self.wd.ox, self.wd.oy]) / RES).astype(int)[:, ::-1]))
        for cell in cells:
            first, last = self.memory.get(cell, (t, t))
            if t - last > 0.6:
                first = t
            self.memory[cell] = (first, t)
        for cell in [k for k, (f, l) in self.memory.items() if t - l > 20.0]:
            del self.memory[cell]

    def planner(self, t, with_scan=True):
        grid = np.where(self.wd.map, 100, 0).astype(np.int16)
        if with_scan:
            for (r, c), (f, l) in self.memory.items():
                if l - f >= 2.5 and 0 <= r < self.wd.h and 0 <= c < self.wd.w:
                    grid[r, c] = 100
        return GridPlanner(grid.ravel().tolist(), self.wd.w, self.wd.h, RES, self.wd.ox, self.wd.oy,
                           robot_radius=self.a['robot_radius'], inflation_radius=self.a['inflation_radius'],
                           cost_weight=self.a['cost_weight'])

    def plan(self, x, y, t):
        """(경로, LiDAR 장애물 반영 여부) 또는 (None, None)."""
        goal = self.wd.sc['goal']
        try:
            return self.planner(t).plan((x, y), goal), True
        except PlanningError:
            pass
        try:
            return self.planner(t, with_scan=False).plan((x, y), goal), False
        except PlanningError:
            return None, None

    def make_follower(self, path):
        f = {'path': np.asarray(path, dtype=float)}
        f['local'] = DWAFollower(path, 0.0, self.cfg, goal_tolerance=self.b['dwa_follow']['goal_tolerance'],
                                 lookahead=self.b['dwa_follow']['lookahead'])
        f['local'].target_clear = self.b['dwa_follow']['target_clear']
        f['local'].target_max = self.b['dwa_follow']['target_max']
        f['pp'] = PurePursuit(path, 0.0, max_v=self.st.get('pp_speed', self.b['max_speed']), max_w=self.b['max_turn_speed'],
                              max_acc=self.b['max_accel'], max_acc_w=self.b['max_turn_accel'])
        f['use_local'] = False
        f['clear_since'] = None
        return f

    def update_box(self, pts, x, y, yaw):
        """브리지 update_box 와 같음: 스캔마다 로봇 앞(+뒤 조금) 사각형 안의 지도에 없는 점 → 연속 스캔 수."""
        h = self.h
        um = self.wd.unmapped(pts)
        c, s = math.cos(yaw), math.sin(yaw)
        lx = c * (um[:, 0] - x) + s * (um[:, 1] - y)
        ly = -s * (um[:, 0] - x) + c * (um[:, 1] - y)
        inside = (lx >= BODY[0] - h['box_back']) & (lx <= BODY[1] + h['box_front']) & (np.abs(ly) <= h['box_half_width'])
        self.box_streak = self.box_streak + 1 if np.count_nonzero(inside) >= h['box_min_points'] else 0

    def hybrid_trigger(self, x, y, yaw, v, t):
        """브리지 hybrid_obstacle 과 같음: 사각형 조건 또는 움직이는 물체가 moving_predict 초 안에 사각형과 겹침."""
        h = self.h
        if self.box_streak >= h['box_min_scans']:
            return True
        ts = np.arange(0.0, h['moving_predict'] + 1e-9, 0.1)
        c, s = math.cos(yaw), math.sin(yaw)
        bx0, bx1 = BODY[0] - h['box_back'], BODY[1] + h['box_front']
        for mx, my, vx, vy, r in self.wd.mover_state(t):
            if math.hypot(vx, vy) < self.b['dwa_follow']['moving_speed']:
                continue
            dx = mx + vx * ts - (x + v * c * ts)
            dy = my + vy * ts - (y + v * s * ts)
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            gap = np.hypot(np.maximum(np.maximum(bx0 - lx, lx - bx1), 0.0), np.maximum(np.abs(ly) - h['box_half_width'], 0.0))
            if np.any(gap <= r):
                return True
        return False

    def run(self, limit=150.0):
        wd = self.wd
        x, y, yaw = wd.sc['start']
        v = w = 0.0
        t = 0.0
        pts, rng, ang = wd.scan(x, y, yaw, t)
        self.remember(pts, t)
        path, _ = self.plan(x, y, t)
        if path is None:
            return self.result('A* 실패', t, math.inf, x, y)
        f = self.make_follower(path)
        retries = 0
        mark = (x, y, t)
        blocked_since = None
        backing = None
        min_gap = math.inf
        last_scan = 0.0
        while t < limit:
            if t - last_scan >= 0.2 - 1e-9:
                pts, rng, ang = wd.scan(x, y, yaw, t)
                self.remember(pts, t)
                self.update_box(pts, x, y, yaw)
                last_scan = t
            dyn = wd.mover_state(t)
            outcome = None
            if backing is not None:
                # 뒤로 15cm (start_recovery) 후 재계획
                v, w = -0.05, 0.0
                if math.hypot(x - backing[0], y - backing[1]) > 0.15 or t - backing[2] > 5.0:
                    backing = None
                    outcome = 'replan'
            else:
                trig = self.hybrid_trigger(x, y, yaw, v, t)
                if not f['use_local']:
                    if trig and not f['pp'].at_goal:
                        f['local'].progress = f['pp'].index
                        f['local'].turning = False
                        f['use_local'] = True
                        f['clear_since'] = None
                        self.switches += 1
                        self.say(t, '→ DWA')
                elif trig or f['local'].arrived:
                    f['clear_since'] = None
                else:
                    f['clear_since'] = f['clear_since'] or t
                    if t - f['clear_since'] >= self.h['release_time']:
                        rest = f['path'][f['local'].progress:f['local'].progress + 60]
                        if np.hypot(rest[:, 0] - x, rest[:, 1] - y).min() < self.h['rejoin_offset']:
                            f['pp'].index = f['local'].progress
                            f['pp'].set_speed(v, w)
                            f['use_local'] = False
                            self.say(t, '→ pure pursuit')
                if f['use_local']:
                    v, w, state = f['local'].compute((x, y, yaw), v, w, (0.0, 0.0, 0.0), pts, dyn,
                                                     moving_speed=self.b['dwa_follow']['moving_speed'],
                                                     unmapped_points=wd.unmapped(pts))
                else:
                    v, w, state = f['pp'].compute(x, y, yaw, now=t)
                if state == 'done':
                    return self.result('도착', t, min_gap, x, y)
                if state == 'off_path':
                    outcome = 'fail_or_retry'
                    self.say(t, '경로 이탈')
                elif state == 'blocked':
                    v = w = 0.0
                    blocked_since = blocked_since if blocked_since is not None else t
                    mark = (x, y, t)   # 기다리는 동안은 진행 없음으로 치지 않음 (브리지와 같음)
                    if t - blocked_since > self.b['blocked_timeout']:
                        outcome = 'recovery'
                        self.say(t, '막힘 5초')
                else:
                    blocked_since = None
                    if v > 0 and not f['use_local']:
                        front = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) <= math.radians(25)
                        if np.min(rng[front]) < self.b['stop_distance']:
                            v = w = 0.0
                            f['pp'].set_speed(0.0, 0.0)
                            blocked_since = t if blocked_since is None else blocked_since
                            mark = (x, y, t)
                            if t - (f.get('pp_block') or t) > self.b['blocked_timeout']:
                                outcome = 'fail_or_retry'
                                self.say(t, '앞 막힘 5초 (pure pursuit)')
                            f['pp_block'] = f.get('pp_block') or t
                        else:
                            f['pp_block'] = None
                if outcome is None:
                    if math.hypot(x - mark[0], y - mark[1]) > 0.05 or state == 'rotating':
                        mark = (x, y, t)
                    elif t - mark[2] > self.b['progress_timeout']:
                        outcome = 'recovery' if f['use_local'] else 'fail_or_retry'
                        self.say(t, '진행 없음 15초')
            if outcome == 'recovery':
                # 뒤가 비었으면 15cm 물러난 뒤 재계획
                back = np.abs(np.arctan2(np.sin(ang - math.pi), np.cos(ang - math.pi))) <= math.radians(30)
                if np.min(rng[back]) > 0.35:
                    backing = (x, y, t)
                    blocked_since = None
                    v = w = 0.0
                else:
                    outcome = 'replan'
            if outcome in ('replan', 'fail_or_retry'):
                retries += 1
                if retries > self.b['max_retries']:
                    return self.result('실패', t, min_gap, x, y)
                newp, scan_ok = self.plan(x, y, t)
                if newp is None:
                    return self.result('실패 (A*)', t, min_gap, x, y)
                self.replans += 1
                self.say(t, f'재시도 {retries}: 재계획 ({"LiDAR 장애물 포함" if scan_ok else "지도만"})')
                f = self.make_follower(newp)
                v = w = 0.0
                mark = (x, y, t)
                blocked_since = None
            yaw += w * DT
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            t += DT
            self.odo += abs(v) * DT
            wd.robot_xy = (x, y)
            min_gap = min(min_gap, wd.body_gap(x, y, yaw, t))
        return self.result('시간 초과', t, min_gap, x, y)

    def result(self, outcome, t, gap, x, y):
        return dict(outcome=outcome, time=round(t, 1), gap=round(gap, 3), pos=(round(x, 2), round(y, 2)),
                    speed=round(self.odo / max(t, 1e-6), 3),
                    replans=self.replans, switches=self.switches, log=self.log)


def job(args):
    key, name, verbose = args
    sc = scenarios()[name]
    t0 = time.time()
    r = Run(World(sc), STRATEGIES[key], load_params(), verbose).run()
    r['cpu'] = round(time.time() - t0, 1)
    return key, name, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default=','.join(STRATEGIES), help='쉼표로 구분 (예: A,H1)')
    ap.add_argument('--scen', default='')
    ap.add_argument('-v', action='store_true')
    ap.add_argument('--workers', type=int, default=10)
    args = ap.parse_args()
    names = args.scen.split(',') if args.scen else list(scenarios())
    keys = args.only.split(',')
    jobs = [(k, n, args.v) for k in keys for n in names]
    with ProcessPoolExecutor(args.workers) as ex:
        res = list(ex.map(job, jobs))
    table = {}
    for k, n, r in res:
        table.setdefault(k, {})[n] = r
    # 시나리오가 많아 표를 세로로: 행 = 시나리오, 열 = 방식. 칸 = ✓시간 또는 ✗결과, 충돌이면 '!'
    w = 13
    print(f"{'시나리오':16s}" + ''.join(f'{k:>{w}s}' for k in keys))
    for n in names:
        cells = []
        for k in keys:
            r = table[k][n]
            c = (f"{r['time']:.0f}s" if r['outcome'] == '도착' else '✗' + r['outcome'][:2]) + ('!' if r['gap'] < 0 else '')
            cells.append(f'{c:>{w}s}')
        print(f'{n:16s}' + ''.join(cells))
    print()
    for k in keys:
        rs = [table[k][n] for n in names]
        ok = sum(r['outcome'] == '도착' and r['gap'] >= 0 for r in rs)
        hit = sum(r['gap'] < 0 for r in rs)
        both = [n for n in names if all(table[q][n]['outcome'] == '도착' for q in keys)]
        tsum = sum(table[k][n]['time'] for n in both)
        gaps = [r['gap'] for r in rs if r['outcome'] == '도착' and r['gap'] >= 0]
        print(f"{k:4s} {STRATEGIES[k]['desc']:34s} 성공 {ok}/{len(names)}  충돌 {hit}  "
              f"공통 성공 {len(both)}개 시간합 {tsum:.0f}s  평균 속도 {np.mean([r['speed'] for r in rs]):.3f} m/s  "
              f"최소 여유 {min(gaps) if gaps else float('nan'):.3f} m  재계획 {sum(r['replans'] for r in rs)}")
    import json
    with open(os.path.join(HERE, '..', 'logs', 'planner_compare.json'), 'w') as fp:
        json.dump({k: {n: {kk: vv for kk, vv in r.items() if kk != 'log'} for n, r in t.items()} for k, t in table.items()},
                  fp, ensure_ascii=False, indent=1)
    if args.v or len(jobs) <= 3:
        for k, n, r in res:
            print(k, n, r['outcome'], r['pos'], r['cpu'], 's cpu')
            print('\n'.join(r['log'][-15:]))


if __name__ == '__main__':
    main()
