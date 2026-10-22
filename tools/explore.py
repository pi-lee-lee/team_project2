"""SLAM 모드 자동 탐색. 맵이 더 넓어지지 않을 때까지 미탐색 경계(프론티어)를 보러 계속 이동한다.

기본(--strategy sweep): 줄 단위로 훑기 (boustrophedon). 출발 방향으로 줄을 나눠 한 줄씩 왕복하며, 각 줄에서는
진행 방향 앞쪽의 프론티어만 차례로 간다 (Sweep 클래스 참고). 줄을 바꿀 때 말고는 되돌아가지 않는다.
--strategy heading: 아래 2번의 방향 유지 방식만 사용 (sweep 에서도 마지막 자투리 정리에 씀)

    source scripts/env.sh
    python3 tools/explore.py [--max-minutes 20] [--server ws://localhost:8000/ws/ui/]

1. /map 에서 프론티어(빈 칸 중 4방향 이웃에 미탐색이 있는 칸)를 찾고 이어진 것끼리 묶는다 (작은 묶음 제외)
   막힌 작은 미탐색 구멍(책상 아래, 앞판 뒤 등 POCKET_MIN 미만)에만 붙은 칸은 프론티어로 치지 않는다
   (가서 봐도 가려져서 안 보이는 곳이라 가장 가까운 목표로 뽑혀 시간만 쓰던 문제)
   아는 칸 사이의 가는 틈(LiDAR 광선 사이)의 미탐색도 본 것으로 친다
2. 목표 고르기: 묶음마다 중심에 가까운 안전한 빈 칸 중, 그 자리에서 센서 범위(SENSOR_RANGE)로 광선을 쏴서
   벽에 막히기 전까지 보이는 미탐색 넓이가 MIN_GAIN 이상인 첫 칸 (가려진 곳은 건너뜀)
   방향 유지(직진성): '탐색 진행 방향' 을 따로 기억하고(로봇의 순간 자세가 아님. DWA 는 피하거나 새 목표를 받으면
   제자리에서 돌기도 해서 순간 자세는 진행 방향과 다를 때가 많음) 진행 방향 ±60° 안에 후보가 하나라도 있으면
   그 안에서만 고른다. 없을 때만 ±120°, 그다음 전부로 넓힌다 → 앞에 갈 곳이 남아 있는 한 뒤로 가지 않는다.
   (예전: 거리 + 방향 벌점 + 크기를 한 점수로 비교 → 가깝거나 큰 뒤쪽 프론티어가 앞쪽 복도를 이겨 되돌아갔음)
   같은 단계 안에서는 점수 = 거리 + 방향 벌점 x 각도 - 크기 보너스
   (보일 넓이를 점수에 넣어 '가장 많이 보이는 곳' 에 서게 하면 복도로 나뉜 공간에서는 어디서나 비슷해서
   가까운 곳만 짧게 오가며 앞뒤로 돌았다 → 넓이는 걸러내기에만 씀)
3. 웹 "여기로 이동" 과 같은 요청(goto_point)을 서버로 보낸다. 멈추지 않고 이어 간다:
   - 목표 PREFETCH_DIST 앞까지 오면 다음 목표를 골라 바로 보낸다 (replace: 진행 중 이동을 새 목표로 바꿈, 멈추지 않음)
     단, 그렇게 한 번 떠난 목표가 다시 뽑히면 이번엔 끝까지 간다. 입구에 서야 보이는 곳(강단 입구 등)을
     0.5m 앞에서 두 입구 사이로 계속 바꿔 가며 왕복하던 문제. 도착해도 안 보이면 아래 규칙으로 제외
   - 가는 도중 목표 프론티어가 이미 보여서 사라지면 바로 다음 목표로
   - 실패한 곳은 다시 고르지 않는다
4. 프론티어가 남지 않으면 끝 (웹 "맵 저장" 으로 저장)
5. 금지 구역 (웹에서 그린 것, 서버 WebSocket 으로 받음): 구역 칸은 장애물로 보고, 로봇에서 A* 처럼(로봇 반경, 미탐색 통과)
   실제로 갈 수 있는 칸에서만 목표/줄 중심선 점을 고른다 → 구역 뒤나 구역으로 막힌 곳으로 목표를 보내지 않음.
   가는 중에 구역이 바뀌어 지금 목표가 구역 안/뒤가 되면 이동을 멈추고 다시 고른다
6. 실패: 실패한 곳 0.5m (같은 곳 두 번째 실패나 금지 구역 때문이면 1m) 안은 다시 고르지 않음 (줄 중심선 점 포함).
   연속 MAX_FAILS 번 실패하면 탐색을 멈춘다 (예전: 줄 중심선 점이 실패 목록을 안 봐서 같은 점을 0.5초마다 수백 번 보냄)

    --no-continuous : 목표마다 도착(정지)까지 기다림   --no-early-switch : 목표 프론티어가 보여도 계속 감
    --pocket-min 0 : 작은 구멍도 탐색
"""
import argparse
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import websocket
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from tf2_ros import Buffer, TransformListener

from warehouse_robot.astar import obstacle_distance
from warehouse_robot.plan_map import near_mask, zone_mask

MIN_CLUSTER = 8        # 이 칸 수(40cm) 미만 프론티어 묶음은 무시
CHUNK = 40             # 큰 프론티어 묶음은 이 칸 수(2m) 조각으로 나눠 각각 목표 후보로
CLEARANCE = 0.35       # 목표 지점은 벽(점유 칸)에서 이만큼 떨어져야 함 (m)
UNKNOWN_STEPS = (0.20, 0.10, 0.0)  # 미탐색 칸에서도 이만큼 떨어진 칸을 먼저 (아직 맵에 없는 벽 바로 옆을 고르면
                                   # 부딪혀 sim 에서 넘어졌음). 그 묶음에 칸이 없으면 다음 값으로 완화
SEARCH_RADIUS = 1.0    # 프론티어 묶음 중심에서 이 반경 안에서 목표 칸을 찾음 (m)
VIEW_SAMPLES = 30      # 묶음마다 보일 넓이를 확인할 후보 수 (중심에 가까운 순)
SENSOR_RANGE = 3.0     # 이득 계산에 쓰는 LiDAR 거리 (m). sim 3.5m, 실물 LDS-02 는 더 멀리 보지만 가장자리는 듬성듬성
RAYS = 64              # 이득 계산 광선 수
SIZE_WEIGHT = 0.02     # 프론티어 1칸 = 2cm 덜 가는 것과 같게 침 (큰 프론티어 우선)
MIN_GAIN = 0.5         # 그 자리에서 이보다 적게(m²) 보이면(가려진 곳) 후보에서 뺌
TURN_WEIGHT = 0.6      # 같은 방향 단계 안에서의 방향 벌점 (m/rad)
DIRECTION_TIERS = (60.0, 120.0, 180.0)   # 진행 방향 ±60° 안에 후보가 있으면 그 안에서만, 없으면 ±120°, 그다음 전부
LANE_RATIO = 0.65      # 줄 폭 = 이득 계산 거리(--sensor-range) x 이 값. 줄 가운데서 양옆 끝까지 센서가 확실히 보게
WAYPOINT_AHEAD = (1.0, 2.5)  # 줄 중심선 목표는 로봇보다 이만큼 앞 (m)
LANE_END_WAIT = 2.5    # 멈춘 뒤 이 시간(s) 동안 앞쪽 후보가 계속 없어야 줄을 바꿈 (SLAM 맵 갱신 2초보다 길게)
HEADING_BLEND = 0.3    # 앞쪽 단계에서 고르면 진행 방향을 새 목표 쪽으로 이만큼만 돌림 (좌우로 흔들리지 않게)
POCKET_MIN = 1.0       # 미탐색 영역이 이 넓이(m²)보다 작고 맵 가장자리에 닿지 않으면 막힌 구멍으로 보고 무시
MIN_TRAVEL = 0.7       # 이보다 가까운 목표는 고르지 않음 (PREFETCH_DIST 보다 커야 함)
PREFETCH_DIST = 0.5    # 목표까지 이만큼 남으면 다음 목표를 미리 보냄 (멈추지 않고 이어 가기)
VISITED_RADIUS = 0.0   # 로봇이 지나간 경로에서 이 거리 안의 미탐색은 본 것으로 침 (m). 기본 끔 (--visited-radius):
                       # 넓은 방에서는 LiDAR 가 끝까지 못 봐서(못 맞힌 광선은 빈 칸으로 안 칠함) 로봇 바로 옆에도
                       # 진짜 미탐색이 남는데, 이걸 지워 첫 방만 보고 '완료' 로 끝났음 (sim turtlebot3_house)
VISIT_STEP = 0.3       # 경로 기록 간격 (m)
GAP_CELLS = 2          # 아는 칸 사이 이 폭(칸) 이하의 미탐색 줄은 광선 틈으로 보고 메움
SLAM_JUMP = 0.5        # map->odom 이 한 번에 이보다 크게 바뀌면 경고 (m)
SEEN_RADIUS = 0.6      # 목표 프론티어 중심 이 반경 안에 프론티어가 MIN_CLUSTER 칸 미만이면 '이미 봄'
ROBOT_RADIUS = 0.28    # 갈 수 있는 칸 판정 (A* robot_radius 와 같게)
START_CLEAR = 0.35     # 로봇 주변은 갈 수 있는 것으로 (A* start_clear_radius 와 같게)
FAIL_RADIUS = (0.5, 1.0)  # 실패한 곳 제외 반경: 처음 / 같은 곳 두 번째 또는 금지 구역 때문
MAX_FAILS = 5          # 연속 이만큼 실패하면 탐색을 멈춤

ZONES = {'list': [], 'ver': 0}   # 서버에서 받은 금지 구역 (지금 맵), 바뀔 때마다 ver + 1
_finished = {}                   # 작업 번호 -> (성공 여부, 메시지). 다른 곳에서 읽어 버린 끝 상태도 놓치지 않게


def handle_msg(m):
    """서버 WebSocket 메시지 공통 처리: 금지 구역 갱신, 끝난 작업 기억."""
    if m.get('type') == 'zones' or (m.get('type') == 'snapshot' and 'zones' in m):
        zones = m.get('zones') or []
        if zones != ZONES['list']:
            ZONES['list'] = zones
            ZONES['ver'] += 1
            print(f'[explore] 금지 구역 {len(zones)}개 반영', flush=True)
    elif m.get('type') == 'state' and m.get('task') and m['task']['status'] != 'running':
        _finished[m['task']['id']] = (m['task']['status'] == 'done', (m.get('robot') or {}).get('message', ''))


def recv(ws):
    """메시지 하나 (handle_msg 를 거침). 시간 초과면 WebSocketTimeoutException."""
    m = json.loads(ws.recv())
    handle_msg(m)
    return m


def drain(ws):
    """쌓인 메시지를 읽어 금지 구역/작업 상태만 반영 (기다리지 않음)."""
    ws.settimeout(0.01)
    try:
        while True:
            recv(ws)
    except websocket.WebSocketTimeoutException:
        pass
    finally:
        ws.settimeout(0.5)


def zone_grid(msg):
    info = msg.info
    return zone_mask((info.height, info.width), info.resolution, info.origin.position.x, info.origin.position.y,
                     ZONES['list'])


def reachable(msg, robot_xy, zmask):
    """로봇에서 갈 수 있는 칸 (bool): 벽/금지 구역에서 ROBOT_RADIUS 떨어진 칸(미탐색 포함) 중 로봇과 이어진 것.
    A*(SLAM 모드: 미탐색 통과, 출발 주변 비움)와 같은 기준."""
    info = msg.info
    res = info.resolution
    raw = grid_of(msg)
    wall = raw >= 65
    passable = obstacle_distance(wall | zmask, res, ROBOT_RADIUS + 0.1) >= ROBOT_RADIUS
    near = near_mask(raw.shape, res, info.origin.position.x, info.origin.position.y, robot_xy, START_CLEAR)
    passable |= near & ~wall
    labels, _ = ndimage.label(passable, structure=np.ones((3, 3), bool))
    seeds = np.unique(labels[near & passable])
    seeds = seeds[seeds > 0]
    return np.isin(labels, seeds) if len(seeds) else np.zeros(raw.shape, bool)


def blocked_by(blacklist, xs, ys):
    """blacklist [(x, y, 반경)] 안에 드는 칸 (bool)."""
    out = np.zeros(np.shape(xs), bool)
    for bx, by, br in blacklist:
        out |= np.hypot(xs - bx, ys - by) < br
    return out


def grid_of(msg):
    info = msg.info
    return np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)


def open_unknown(grid, min_cells):
    """미탐색 칸 중 '넓은' 영역에 속한 칸 (bool). 넓음 = min_cells 이상이거나 맵 가장자리에 닿음(맵 밖으로 이어짐).
    min_cells <= 0 이면 미탐색 전부."""
    unknown = grid < 0
    if min_cells <= 0:
        return unknown
    labels, n = ndimage.label(unknown)
    if n == 0:
        return unknown
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    edge = np.zeros(n + 1, dtype=bool)
    edge[np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))] = True
    keep = (sizes >= min_cells) | edge
    keep[0] = False
    return keep[labels]


def effective_unknown(msg, pocket_min=POCKET_MIN, visited=()):
    """탐색 대상 미탐색 칸 (bool): 광선 틈은 메우고, 지나온 자리 근처와 막힌 작은 구멍은 뺀다."""
    info = msg.info
    res = info.resolution
    grid = grid_of(msg)
    known = grid >= 0
    if GAP_CELLS > 0:
        st = np.ones((3, 3), bool)
        filled = ndimage.binary_closing(known, structure=st, iterations=GAP_CELLS, border_value=0)
        grid = np.where(~known & filled, 0, grid)   # 메운 칸은 빈 칸처럼 (프론티어 판정용)
    unknown = open_unknown(grid, int(pocket_min / res ** 2))
    if len(visited) and VISITED_RADIUS > 0:
        seeds = np.zeros(grid.shape, bool)
        v = np.asarray(visited)
        c = ((v[:, 0] - info.origin.position.x) / res).astype(int)
        r = ((v[:, 1] - info.origin.position.y) / res).astype(int)
        ok = (r >= 0) & (r < info.height) & (c >= 0) & (c < info.width)
        seeds[r[ok], c[ok]] = True
        if seeds.any():
            unknown &= ndimage.distance_transform_edt(~seeds) * res > VISITED_RADIUS
    return grid, unknown


def frontier_mask(grid, unknown):
    """프론티어 칸 (bool): 빈 칸 중 4방향 이웃에 (넓은) 미탐색이 있는 칸."""
    near = np.zeros_like(unknown)
    near[1:, :] |= unknown[:-1, :]
    near[:-1, :] |= unknown[1:, :]
    near[:, 1:] |= unknown[:, :-1]
    near[:, :-1] |= unknown[:, 1:]
    return (grid == 0) & near


def frontiers(front):
    """프론티어 칸 → [ [(r, c), ...], ... ] 이어진 묶음 (큰 묶음은 CHUNK 칸씩 조각)."""
    h, w = front.shape
    seen = np.zeros_like(front)
    clusters = []
    for r0, c0 in zip(*np.nonzero(front)):
        if seen[r0, c0]:
            continue
        q, cells = deque([(r0, c0)]), []
        seen[r0, c0] = True
        while q:
            r, c = q.popleft()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and front[nr, nc] and not seen[nr, nc]:
                        seen[nr, nc] = True
                        q.append((nr, nc))
        if len(cells) >= MIN_CLUSTER:
            # BFS 순서는 공간적으로 이어져 있으므로 순서대로 잘라 조각을 만든다
            for i in range(0, len(cells), CHUNK):
                part = cells[i:i + CHUNK]
                if len(part) >= MIN_CLUSTER:
                    clusters.append(part)
    return clusters


def view_gain(rows, cols, occ, unknown, res, sensor_range=SENSOR_RANGE, rays=RAYS):
    """각 후보 칸 (rows[i], cols[i]) 에서 보이는 미탐색 넓이 (m²).
    광선마다 점유 칸에 막히기 전까지 지나는 미탐색 칸을, 그 거리의 부채꼴 넓이(r·dθ·res)로 더한다. 맵 밖은 미탐색."""
    n = int(sensor_range / res)
    ang = np.linspace(0, 2 * np.pi, rays, endpoint=False)
    k = np.arange(1, n + 1)
    dr = np.rint(np.sin(ang)[:, None] * k[None, :]).astype(int)        # (rays, n)
    dc = np.rint(np.cos(ang)[:, None] * k[None, :]).astype(int)
    rr = rows[:, None, None] + dr[None]                                 # (N, rays, n)
    cc = cols[:, None, None] + dc[None]
    h, w = occ.shape
    inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
    rcl, ccl = np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)
    hit = occ[rcl, ccl] & inside
    blocked = np.cumsum(hit, axis=2) > 0
    unk = (unknown[rcl, ccl] | ~inside) & ~blocked
    weight = k * res * (2 * np.pi / rays) * res                         # 거리 k 의 한 칸이 대표하는 넓이
    return (unk * weight[None, None, :]).sum(axis=(1, 2))


def frontier_left(msg, fx, fy, pocket_min=POCKET_MIN, visited=()):
    """(fx, fy) 주변 SEEN_RADIUS 안 프론티어 칸 수. 목표로 가는 도중 이미 보였는지 판단용."""
    info = msg.info
    res = info.resolution
    front = frontier_mask(*effective_unknown(msg, pocket_min, visited))
    c = int((fx - info.origin.position.x) / res)
    r = int((fy - info.origin.position.y) / res)
    k = int(SEEN_RADIUS / res) + 1
    rr, cc = np.nonzero(front[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1])
    rr = rr + max(0, r - k)
    cc = cc + max(0, c - k)
    return int(np.count_nonzero((rr - r) ** 2 + (cc - c) ** 2 <= k * k))


def pick_goal(msg, robot_xy, blacklist, heading=None, turn_weight=TURN_WEIGHT, pocket_min=POCKET_MIN,
              exclude=(), sensor_range=SENSOR_RANGE, visited=()):
    """다음 목표 (x, y, yaw, 보일 넓이 m², 프론티어 중심 x, y, 방향 단계) 또는 None (방향 유지 방식)."""
    return choose_by_heading(_candidates(msg, robot_xy, blacklist, pocket_min, exclude, sensor_range, visited),
                             robot_xy, heading, turn_weight)


def choose_by_heading(cands, robot_xy, heading=None, turn_weight=TURN_WEIGHT):
    """방향 단계(DIRECTION_TIERS) 순서로, 그 안에 후보가 있는 첫 단계에서 점수가 가장 좋은 후보.
    반환 (x, y, yaw, 보일 넓이, 프론티어 중심 x, y, 방향 단계) 또는 None."""
    for tier, limit in enumerate(DIRECTION_TIERS):
        best = None
        for gx, gy, bearing, gain, fx, fy, size in cands:
            travel = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
            cost = travel - SIZE_WEIGHT * size                 # 가깝고 큰 프론티어 우선
            if heading is not None:
                off = abs(math.atan2(math.sin(bearing - heading), math.cos(bearing - heading)))
                if math.degrees(off) > limit:
                    continue
                cost += turn_weight * off
            if best is None or cost < best[0]:
                best = (cost, gx, gy, bearing, gain, fx, fy, tier)
        if best is not None:
            return best[1:]
    return None


class Sweep:
    """줄 단위로 훑기 (boustrophedon, 잔디 깎기 방식). 프론티어를 '매번 가장 좋은 것' 으로 고르면
    좌석 줄이 양옆으로 반복되는 곳(강당)에서는 경계가 사방에 동시에 생겨 지그재그로 되돌아갔다.
    - 줄 방향(u)은 맵에서 찾은 벽의 주된 방향(wall_axis) 중 로봇 방향에 가장 가까운 쪽 (벽이 아직 없으면 로봇 방향),
      그 왼쪽 수직(v)으로 폭 lane_width (센서 거리 x LANE_RATIO) 의 줄을 나눈다
    - 지금 줄에서 진행 방향 앞쪽의 후보만 가까운 순으로 간다
    - 앞쪽 후보가 없으면 다음 줄로 넘어가고 진행 방향을 뒤집는다 (후보가 있는 가장 가까운 줄, 처음엔 후보가 많은 쪽)
    - 단, 줄 끝 판단은 서두르지 않는다: 이동 중이면 지금 목표까지 가고('wait'), 멈춘 뒤에도 맵이 한 번 더
      갱신될 때(LANE_END_WAIT) 까지 앞쪽 후보가 없어야 줄을 바꾼다. 앞쪽 후보가 잠깐 없는 건 대부분
      '조금 더 가면 LiDAR 가 새 경계를 찾는' 일시적인 상태라, 바로 줄을 바꾸면 되돌아갔다가 다시 돌아왔다"""

    def __init__(self, x0, y0, yaw0, lane_width):
        self.x0, self.y0 = x0, y0
        self.ux, self.uy = math.cos(yaw0), math.sin(yaw0)
        self.vx, self.vy = -math.sin(yaw0), math.cos(yaw0)
        self.w = lane_width
        self.lane = 0
        self.dir = 1          # 줄 안에서 가는 방향 (+u / -u)
        self.side = None      # 줄을 넘어가는 쪽 (+v / -v), 처음 넘어갈 때 후보가 많은 쪽으로
        self.end_since = None # 지금 줄에 앞쪽 후보가 없다고 처음 본 시각

    def uv(self, x, y):
        dx, dy = x - self.x0, y - self.y0
        return dx * self.ux + dy * self.uy, dx * self.vx + dy * self.vy

    def lane_of(self, x, y):
        return int(math.floor(self.uv(x, y)[1] / self.w + 0.5))

    def pick(self, cands, robot_xy, moving, now, waypoint=None):
        """반환 (후보, 설명) / (None, 'wait': 줄 끝인지 아직 모름, 조금 뒤 다시) / (None, ''): 후보 없음.
        waypoint(lane, dir): 줄 중심선 위 앞쪽의 갈 수 있는 점 (x, y) 또는 None.
        줄 앞쪽에 프론티어가 남아 있으면 프론티어 칸 대신 중심선 점으로 곧게 나아간다 (옆 입구마다 들어갔다
        나왔다 하던 문제: 입구 안쪽은 지나가면서 LiDAR 로 보면 됨). 중심선으로 더 못 가면 프론티어 칸으로."""
        if not cands:
            return None, ''
        ru, _ = self.uv(*robot_xy)
        lanes = {}
        for c in cands:
            lanes.setdefault(self.lane_of(c[0], c[1]), []).append(c)

        def ahead_in(lane, d):
            return [c for c in lanes.get(lane, []) if (self.uv(c[0], c[1])[0] - ru) * d > -0.3]

        ahead = ahead_in(self.lane, self.dir)
        if ahead:
            self.end_since = None
            arrow = "→" if self.dir > 0 else "←"
            far = max((self.uv(c[0], c[1])[0] - ru) * self.dir for c in ahead)
            wp = waypoint(self.lane, self.dir) if waypoint else None
            if wp is not None and (self.uv(*wp)[0] - ru) * self.dir <= far + 1.0:
                bearing = math.atan2(wp[1] - robot_xy[1], wp[0] - robot_xy[0])
                return (wp[0], wp[1], bearing, 0.0, None, None, 0), f'줄 {self.lane} {arrow} 중심선'
            c = min(ahead, key=lambda c: (self.uv(c[0], c[1])[0] - ru) * self.dir)
            return c, f'줄 {self.lane} {arrow}'
        if moving:
            return None, 'wait'
        if self.end_since is None:
            self.end_since = now
        if now - self.end_since < LANE_END_WAIT:
            return None, 'wait'
        # 줄 끝: 후보가 있는 가장 가까운 다음 줄 (같은 쪽 우선, 없으면 반대쪽)
        self.end_since = None
        if self.side is None:
            left = sum(len(v) for k, v in lanes.items() if k > self.lane)
            right = sum(len(v) for k, v in lanes.items() if k < self.lane)
            self.side = 1 if left >= right else -1
        nxt = [k for k in lanes if (k - self.lane) * self.side > 0]
        if not nxt:
            self.side = -self.side
            nxt = [k for k in lanes if (k - self.lane) * self.side > 0]
        if not nxt:   # 후보가 지금 줄 뒤쪽에만: 이 줄을 반대로
            self.dir = -self.dir
        else:
            self.lane = min(nxt, key=lambda k: abs(k - self.lane))
            self.dir = -self.dir
            if not ahead_in(self.lane, self.dir):   # 새 줄의 후보가 반대쪽에만 있으면 그쪽으로
                self.dir = -self.dir
        ahead = ahead_in(self.lane, self.dir)
        if not ahead:
            return None, ''
        c = min(ahead, key=lambda c: (self.uv(c[0], c[1])[0] - ru) * self.dir)
        return c, f'줄 {self.lane} {"→" if self.dir > 0 else "←"} (줄 바꿈)'


def wall_axis(msg, min_points=40):
    """맵의 벽(점유 칸)이 가로/세로 줄에 가장 잘 모이는 방향 (rad, 0~90°) 또는 None (벽이 너무 적음).
    tools/straighten_map.py 의 기울기 추정과 같은 방식 (회전시켜 투영한 히스토그램이 가장 뾰족한 각도)."""
    info = msg.info
    res = info.resolution
    rr, cc = np.nonzero(grid_of(msg) >= 65)
    if len(rr) < min_points:
        return None
    x, y = cc * res, rr * res
    best = (-1, 0.0)
    for deg in range(0, 90):
        t = math.radians(deg)
        c, s = math.cos(t), math.sin(t)
        u, v = c * x + s * y, -s * x + c * y
        hu = np.bincount(np.floor((u - u.min()) / res).astype(int))
        hv = np.bincount(np.floor((v - v.min()) / res).astype(int))
        score = int((hu.astype(np.int64) ** 2).sum() + (hv.astype(np.int64) ** 2).sum())
        if score > best[0]:
            best = (score, t)
    return best[1]


def lane_heading(msg, robot_yaw):
    """줄 방향: 벽의 주된 방향(4 방향) 중 로봇 방향에 가장 가까운 것. 벽을 못 찾으면 로봇 방향."""
    axis = wall_axis(msg)
    if axis is None:
        return robot_yaw
    options = [axis + k * math.pi / 2 for k in range(4)]
    return min(options, key=lambda a: abs(math.atan2(math.sin(a - robot_yaw), math.cos(a - robot_yaw))))


def lane_waypoint(msg, sweep, robot_xy, lane, d, allowed=None, blacklist=(), exclude=()):
    """줄 중심선(v = lane x 줄 폭) 근처에서 로봇보다 WAYPOINT_AHEAD 만큼 앞(진행 방향 d)의 안전한 빈 칸 (x, y) 또는 None.
    앞으로 많이, 중심선에 가까울수록 좋음. allowed: 갈 수 있는 칸(금지 구역 반영), blacklist/exclude: 실패한 곳/가는 곳."""
    info = msg.info
    res = info.resolution
    raw = grid_of(msg)
    dist = obstacle_distance(raw >= 65, res, CLEARANCE + 0.1)
    safe = (raw == 0) & (dist >= CLEARANCE)
    if allowed is not None:
        safe &= allowed
    rr, cc = np.nonzero(safe)
    if len(rr) == 0:
        return None
    xs = info.origin.position.x + (cc + 0.5) * res
    ys = info.origin.position.y + (rr + 0.5) * res
    dx, dy = xs - sweep.x0, ys - sweep.y0
    u = dx * sweep.ux + dy * sweep.uy
    v = dx * sweep.vx + dy * sweep.vy
    ru = sweep.uv(*robot_xy)[0]
    prog = (u - ru) * d
    off = np.abs(v - lane * sweep.w)
    ok = (prog >= WAYPOINT_AHEAD[0]) & (prog <= WAYPOINT_AHEAD[1]) & (off <= sweep.w / 2)
    ok &= ~blocked_by(blacklist, xs, ys)
    for ex, ey in exclude:
        ok &= np.hypot(xs - ex, ys - ey) >= 0.8
    if not ok.any():
        return None
    score = np.where(ok, prog - 1.5 * off, -np.inf)
    i = int(np.argmax(score))
    return float(xs[i]), float(ys[i])


def _candidates(msg, robot_xy, blacklist, pocket_min, exclude, sensor_range, visited, allowed=None, zmask=None):
    """프론티어 묶음마다 목표 후보 하나: [(x, y, 가는 방향, 보일 넓이 m², 프론티어 중심 x, y, 묶음 칸 수)].
    blacklist [(x, y, 반경)] / exclude: 그 반경 / 0.8m 안의 후보는 버림 (실패한 곳 / 지금 가고 있는 곳).
    allowed: 로봇에서 갈 수 있는 칸 (reachable), zmask: 금지 구역 칸 - 구역 안 프론티어와 목표는 버림.
    목표 칸은 벽에서 CLEARANCE, 미탐색에서 UNKNOWN_STEPS 순서로 (그 묶음에 칸이 있는 가장 엄격한 여유로) 떨어진 칸.
    여유 완화는 묶음마다 따로 한다. 맵 전체로 하면 트인 곳(뒤쪽)의 묶음에 엄격한 칸이 하나라도 있을 때
    좁은 복도(앞쪽) 묶음은 칸이 0개라 후보에서 빠져, 방향 규칙과 상관없이 뒤로만 갔다."""
    info = msg.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    raw = grid_of(msg)
    occ = raw >= 65
    grid, unknown = effective_unknown(msg, pocket_min, visited)
    dist = obstacle_distance(occ, res, CLEARANCE + 0.1)
    base = (raw == 0) & (dist >= CLEARANCE)      # 메운 틈 칸에는 서지 않음
    if allowed is not None:
        base &= allowed
    dist_unknown = obstacle_distance(raw < 0, res, max(UNKNOWN_STEPS) + 0.1)
    levels = [(base & (dist_unknown >= u), mt) for u in UNKNOWN_STEPS for mt in (MIN_TRAVEL, 0.5)]
    k = int(SEARCH_RADIUS / res)
    out = []
    front = frontier_mask(grid, unknown)
    if zmask is not None:
        front &= ~zmask
    for cells in frontiers(front):
        cr = int(np.mean([c[0] for c in cells]))
        cc = int(np.mean([c[1] for c in cells]))
        r0, r1 = max(0, cr - k), min(info.height, cr + k + 1)
        c0, c1 = max(0, cc - k), min(info.width, cc + k + 1)
        for safe, min_travel in levels:
            rr, ccs = np.nonzero(safe[r0:r1, c0:c1])
            rr, ccs = rr + r0, ccs + c0
            if len(rr) == 0:
                continue
            xs, ys = ox + (ccs + 0.5) * res, oy + (rr + 0.5) * res
            ok = np.hypot(xs - robot_xy[0], ys - robot_xy[1]) >= min_travel
            ok &= ~blocked_by(blacklist, xs, ys)
            for ex, ey in exclude:
                ok &= np.hypot(xs - ex, ys - ey) >= 0.8
            if ok.any():
                break
        else:
            continue
        # 프론티어 중심에 가까운 칸부터, 그 자리에서 실제로 보일 넓이가 있는 첫 칸 (가려진 곳은 건너뜀)
        order = np.flatnonzero(ok)[np.argsort(((rr - cr) ** 2 + (ccs - cc) ** 2)[ok])][:VIEW_SAMPLES]
        gains = view_gain(rr[order], ccs[order], occ, unknown, res, sensor_range)
        good = np.flatnonzero(gains >= MIN_GAIN)
        if len(good) == 0:
            continue
        j = order[good[0]]
        gx, gy = float(xs[j]), float(ys[j])
        # 도착 방향 = 가는 방향 (LiDAR 는 360° 라 어디를 보든 같음 → 도착해서 따로 돌지 않게)
        bearing = math.atan2(gy - robot_xy[1], gx - robot_xy[0])
        out.append((gx, gy, bearing, float(gains[good[0]]), ox + (cc + 0.5) * res, oy + (cr + 0.5) * res, len(cells)))
    return out


def wait_idle(ws, timeout=3.0):
    """취소 후 작업이 끝났다는 상태가 올 때까지 (다음 goto 가 '진행 중' 으로 거절되지 않게)."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            m = recv(ws)
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'state' and not (m.get('task') and m['task']['status'] == 'running'):
            return


_last = {'task': None}   # 마지막으로 보낸 이동 작업 번호 (이어 가기에서 이전 작업 상태와 구별)


def goto(ws, x, y, yaw, timeout, check=None, replace=False):
    """서버에 goto_point 를 보내고 작업이 끝나거나 check() 가 이유를 돌려줄 때까지 기다린다.
    반환: (성공 여부, 메시지). check(): 가는 도중 0.5초마다 불러서 문자열이면 멈추지 않고 (True, 그 문자열) 반환
    (이동은 계속 진행 중 → 다음 goto 를 replace=True 로 보내 이어 간다).
    replace: 진행 중인 이동을 이 목표로 바꿈 (로봇을 멈추지 않음)."""
    ws.send(json.dumps({'action': 'goto_point', 'x': x, 'y': y, 'yaw': yaw, 'replace': replace}))
    end = time.time() + timeout
    task_id = None
    next_check = time.time() + 0.5
    while time.time() < end:
        if check is not None and task_id is not None and time.time() >= next_check:
            next_check = time.time() + 0.5
            why = check()
            if why:
                return True, why
        try:
            m = recv(ws)
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'error':
            return None, m['message']   # 이동이 아니라 요청이 거절됨 (진행 중 작업 등)
        if m['type'] == 'state' and m.get('task'):
            t = m['task']
            if task_id is None and t['status'] == 'running' and t['id'] != _last['task']:
                task_id = t['id']
                _last['task'] = task_id
            if t['id'] == task_id and t['status'] != 'running':
                return t['status'] == 'done', m['robot'].get('message', '')
    ws.send(json.dumps({'action': 'cancel_task'}))
    wait_idle(ws)
    return False, f'{timeout:.0f}초 안에 도착하지 못함'


def main():
    global VISITED_RADIUS
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-minutes', type=float, default=20.0)
    ap.add_argument('--goal-timeout', type=float, default=90.0)
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--turn-weight', type=float, default=TURN_WEIGHT, help='같은 방향 단계 안의 방향 벌점 (m/rad)')
    ap.add_argument('--no-early-switch', action='store_true', help='목표 프론티어가 보여도 도착까지 감')
    ap.add_argument('--no-continuous', action='store_true', help='목표마다 도착(정지)까지 기다림')
    ap.add_argument('--pocket-min', type=float, default=POCKET_MIN,
                    help='이보다 작은(m²) 막힌 미탐색 구멍은 무시, 0 이면 전부 탐색')
    ap.add_argument('--strategy', choices=('sweep', 'heading'), default='sweep',
                    help='sweep: 줄 단위로 훑기(기본), heading: 프론티어 + 방향 유지')
    ap.add_argument('--lane-width', type=float, default=0.0,
                    help='줄 단위 훑기의 줄 폭 (m). 0 이면 --sensor-range x %.2f' % LANE_RATIO)
    ap.add_argument('--visited-radius', type=float, default=VISITED_RADIUS,
                    help='지나온 경로에서 이 거리(m) 안의 미탐색은 본 것으로 침 (0 = 끔)')
    ap.add_argument('--debug-dir', default='', help='고를 때마다 맵/후보를 npz 로 저장 (분석용)')
    ap.add_argument('--sensor-range', type=float, default=SENSOR_RANGE, help='이득 계산에 쓰는 LiDAR 거리 (m)')
    args = ap.parse_args()
    VISITED_RADIUS = args.visited_radius

    rclpy.init()
    node = Node('explore_tool', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=os.environ.get('WAREHOUSE_SIM_TIME') == 'true')])
    buf = Buffer()
    TransformListener(buf, node)
    latest = {}
    node.create_subscription(OccupancyGrid, 'map', lambda m: latest.__setitem__('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    ws = websocket.create_connection(args.server, timeout=0.5)
    snap = recv(ws)
    if snap.get('task') and snap['task']['status'] == 'running':
        print('[explore] 진행 중인 작업을 취소하고 시작합니다', flush=True)
        ws.send(json.dumps({'action': 'cancel_task'}))
        wait_idle(ws)

    trail = []   # 로봇이 지나온 자리 (VISIT_STEP 간격, map 좌표)
    last_m_o = []  # 직전 map->odom (SLAM 보정). 갑자기 크게 바뀌면 경고 (반복 구조에서 잘못된 루프 클로저 등)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            x, y, _ = robot_pose()
        except Exception:
            return
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= VISIT_STEP:
            trail.append((x, y))
        try:
            t = buf.lookup_transform('map', 'odom', rclpy.time.Time()).transform.translation
        except Exception:
            return
        if last_m_o and math.hypot(t.x - last_m_o[0], t.y - last_m_o[1]) > SLAM_JUMP:
            print(f'[explore] 경고: SLAM 위치 보정이 갑자기 {math.hypot(t.x - last_m_o[0], t.y - last_m_o[1]):.2f}m 바뀜 '
                  '(잘못된 루프 클로저일 수 있음, 맵 확인 필요)', flush=True)
        last_m_o[:] = [t.x, t.y]

    def robot_pose():
        tf = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        q = tf.rotation
        return tf.translation.x, tf.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    blacklist = []  # [(x, y, 반경)] 실패한 곳
    failed_at = []  # 실패한 목표 (같은 곳 두 번째 실패 판정)
    fails = 0       # 연속 실패 수
    reached = []    # 도착한 목표. 같은 곳에 두 번 도착해도 프론티어가 안 사라지면(그 자리에서 볼 수 없는 곳) 제외
    start = time.time()
    visited = ok_count = switched = chained = 0
    moving = False  # 이전 목표로 아직 움직이는 중 (다음 goto 는 replace 로 이어 감)
    exclude = []    # 미리 보낼 때: 지금 가고 있는 목표 근처는 다시 고르지 않음
    left_early = [] # 도착 전에 다음 목표로 넘어간 목표들. 다시 뽑히면 끝까지 감
    # SLAM 시작 직후 맵이 아주 작으면 고를 곳이 없어 바로 '완료' 로 끝나므로 맵이 조금 커질 때까지 기다림
    wait_end = time.time() + 10.0
    while time.time() < wait_end:
        spin(0.5)
        m = latest.get('map')
        if m is not None and np.count_nonzero(np.asarray(m.data) >= 0) * m.info.resolution ** 2 >= 3.0:
            break
    heading = None  # 탐색 진행 방향 (rad). 첫 목표 전에는 로봇 자세
    sweep = None    # 줄 단위 훑기 상태 (--strategy sweep)

    def choose(rx, ry, excl):
        """반환 (x, y, yaw, 보일 넓이, 프론티어 중심 x, y, 설명) 또는 None."""
        m = latest['map']
        zm = zone_grid(m)
        reach = reachable(m, (rx, ry), zm)
        cands = _candidates(m, (rx, ry), blacklist, args.pocket_min, excl, args.sensor_range, trail, reach, zm)
        if not cands:
            return None
        if sweep is not None:
            c, label = sweep.pick(cands, (rx, ry), moving, time.time(),
                                  lambda lane, d: lane_waypoint(m, sweep, (rx, ry), lane, d, reach, blacklist, excl))
            if label == 'wait':
                return 'wait'
            if c is not None:
                return c[0], c[1], c[2], c[3], c[4], c[5], label
        g = choose_by_heading(cands, (rx, ry), heading, args.turn_weight)
        if g is None:
            return None
        return g[:6] + ((('자투리 ' if sweep is not None else '') + ('앞', '옆', '뒤')[g[6]]),)
    def fail(gx, gy, message):
        """실패한 목표를 제외하고 연속 실패를 센다. 반환: 탐색을 멈춰야 하면 True."""
        nonlocal fails
        again = any(math.hypot(gx - px, gy - py) < FAIL_RADIUS[0] for px, py in failed_at)
        r = FAIL_RADIUS[1] if again or '금지 구역' in message else FAIL_RADIUS[0]
        failed_at.append((gx, gy))
        blacklist.append((gx, gy, r))
        fails += 1
        if fails >= MAX_FAILS:
            print(f'[explore] 연속 {fails}번 실패해 탐색을 멈춥니다 (마지막: {message})', flush=True)
            return True
        return False

    while time.time() - start < args.max_minutes * 60:
        spin(0.05 if moving else 0.3)
        drain(ws)   # 금지 구역 변경 반영
        if 'map' not in latest or not buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
            print('[explore] 맵/위치 대기 중', flush=True)
            spin(2.0)
            continue
        rx, ry, ryaw = robot_pose()
        if heading is None:
            heading = ryaw
            if args.strategy == 'sweep':
                lane_yaw = lane_heading(latest['map'], ryaw)
                lane_w = args.lane_width or args.sensor_range * LANE_RATIO
                sweep = Sweep(rx, ry, lane_yaw, lane_w)
                print(f'[explore] 줄 단위 훑기: 줄 방향 {math.degrees(lane_yaw):.0f}° (로봇 {math.degrees(ryaw):.0f}°), '
                      f'줄 폭 {lane_w:.1f}m', flush=True)
        goal = choose(rx, ry, exclude)
        if goal == 'wait' and not moving:
            spin(0.5)          # 줄 끝인지 맵 갱신을 기다려 다시 확인
            continue
        if goal == 'wait':
            goal = None        # 이동 중: 지금 목표까지 마저 감 (아래)
        if goal is None and moving:
            # 이어 갈 곳이 없으면 지금 목표까지 마저 간다
            exclude = []
            moving = False
            goto_wait(ws, args.goal_timeout)
            continue
        if goal is None:
            # SLAM 맵 갱신이 늦을 수 있으니 한 번 더 기다렸다가 확인
            spin(4.0)
            rx, ry, ryaw = robot_pose()
            goal = choose(rx, ry, ())
            while goal == 'wait':
                spin(0.5)
                goal = choose(rx, ry, ())
            if goal is None:
                print('[explore] 남은 프론티어 없음: 탐색 완료', flush=True)
                if args.debug_dir:
                    m = latest['map']
                    np.savez_compressed(os.path.join(args.debug_dir, 'final.npz'), grid=grid_of(m), res=m.info.resolution,
                                        origin=[m.info.origin.position.x, m.info.origin.position.y],
                                        robot=[rx, ry, ryaw], trail=np.array(trail),
                                        blacklist=np.array(blacklist).reshape(-1, 3))
                break
        gx, gy, gyaw, gain, fx, fy, label = goal
        tier = 0 if not label.endswith(('옆', '뒤')) else (1 if label.endswith('옆') else 2)
        visited += 1
        if args.debug_dir:   # 분석용: 고를 때의 맵, 로봇, 진행 방향, 후보 전부
            m = latest['map']
            np.savez_compressed(os.path.join(args.debug_dir, f'pick_{visited:03d}.npz'),
                                grid=grid_of(m), res=m.info.resolution,
                                origin=[m.info.origin.position.x, m.info.origin.position.y],
                                robot=[rx, ry, ryaw], heading=heading, goal=[gx, gy, tier], label=label,
                                trail=np.array(trail),
                                exclude=np.array(exclude).reshape(-1, 2), blacklist=np.array(blacklist).reshape(-1, 3))
        turn = math.degrees(abs(math.atan2(math.sin(gyaw - heading), math.cos(gyaw - heading))))
        print(f'[explore] #{visited} 목표 ({gx:.2f}, {gy:.2f}) 보일 넓이 {gain:.1f} m², 진행 방향과 {turn:.0f}° '
              f'[{label}]{" (이어 가기)" if moving else ""}', flush=True)
        # 진행 방향 갱신: 앞쪽에서 골랐으면 조금만, 옆/뒤로 넓혀서 골랐으면 새 방향으로
        if tier == 0:
            hx = (1 - HEADING_BLEND) * math.cos(heading) + HEADING_BLEND * math.cos(gyaw)
            hy = (1 - HEADING_BLEND) * math.sin(heading) + HEADING_BLEND * math.sin(gyaw)
            heading = math.atan2(hy, hx)
        else:
            heading = gyaw

        # 전에 도착 전에 떠난 곳이거나, 미리 보내기 거리 안팎의 가까운 목표면 끝까지 감
        # (줄 중심선 목표는 계속 앞으로 이어 가는 점이라 해당 없음)
        is_waypoint = fx is None
        commit = not is_waypoint and (any(math.hypot(gx - px, gy - py) < 0.8 for px, py in left_early)
                                      or math.hypot(gx - rx, gy - ry) < PREFETCH_DIST + 0.2)
        if commit and math.hypot(gx - rx, gy - ry) >= PREFETCH_DIST + 0.2:
            print('[explore]   전에 도착 전에 떠난 곳: 이번엔 끝까지 감', flush=True)

        zone_ver = ZONES['ver']

        def check():
            nonlocal zone_ver
            spin(0.05)
            if ZONES['ver'] != zone_ver:   # 가는 중에 금지 구역이 바뀜: 목표가 구역 안/뒤가 됐는지
                zone_ver = ZONES['ver']
                m = latest['map']
                x, y, _ = robot_pose()
                info = m.info
                c = int((gx - info.origin.position.x) / info.resolution)
                r = int((gy - info.origin.position.y) / info.resolution)
                inside = 0 <= r < info.height and 0 <= c < info.width
                if not inside or not reachable(m, (x, y), zone_grid(m))[r, c]:
                    return '금지 구역 변경'
            if not args.no_continuous and not commit:
                x, y, _ = robot_pose()
                if math.hypot(gx - x, gy - y) < PREFETCH_DIST:
                    return '근접'
            if (not args.no_early_switch and fx is not None
                    and frontier_left(latest['map'], fx, fy, args.pocket_min, trail) < MIN_CLUSTER):
                return '이미 봄'
            return None

        ok, message = goto(ws, gx, gy, gyaw, args.goal_timeout, check, replace=moving)
        moving = False
        exclude = []
        if ok and message == '금지 구역 변경':
            print('[explore]   금지 구역이 바뀌어 지금 목표로 갈 수 없음: 멈추고 다시 고름', flush=True)
            ws.send(json.dumps({'action': 'cancel_task'}))
            wait_idle(ws)
            blacklist.append((gx, gy, FAIL_RADIUS[1]))
            continue
        if ok:
            fails = 0
        if ok and message in ('근접', '이미 봄'):
            if message == '이미 봄':
                switched += 1
                print('[explore]   가는 도중 목표 프론티어가 보임: 다음 목표로', flush=True)
            else:
                chained += 1
                if not is_waypoint:
                    exclude = [(gx, gy)]
                    left_early.append((gx, gy))
            if args.no_continuous:
                ws.send(json.dumps({'action': 'cancel_task'}))
                wait_idle(ws)
            else:
                moving = True
            continue
        if ok:
            ok_count += 1
            if is_waypoint:
                continue
            if any(math.hypot(gx - px, gy - py) < 0.4 for px, py in reached):
                blacklist.append((gx, gy, FAIL_RADIUS[0]))
                print('[explore]   같은 곳에 다시 왔지만 프론티어가 남음: 이 근처는 제외', flush=True)
            reached.append((gx, gy))
        elif ok is None and ('금지 구역' in message or '같은 지점' in message):
            # 서버가 목표 자체를 거절 (금지 구역 안, 같은 곳 반복 실패): 그 목표는 실패로 침
            print(f'[explore]   요청 거절: {message}', flush=True)
            if fail(gx, gy, message):
                break
        elif ok is None:
            print(f'[explore]   요청 거절: {message} (잠시 뒤 다시 시도)', flush=True)
            spin(3.0)
        else:
            print(f'[explore]   실패: {message}', flush=True)
            if any(w in message for w in ('넘어', '직접 확인', '비상정지')):
                # 로봇을 사람이 봐야 하는 상태: 이후 목표가 모두 거절되므로 계속하면 전부 제외되고 '완료' 로 잘못 끝남
                print('[explore] 로봇 확인이 필요해 탐색을 멈춥니다 (상태 초기화 후 다시 시작)', flush=True)
                break
            if fail(gx, gy, message):
                break
    else:
        print(f'[explore] 시간 제한 {args.max_minutes:.0f}분 도달', flush=True)
    if moving:
        ws.send(json.dumps({'action': 'cancel_task'}))
        wait_idle(ws)
    m = latest.get('map')
    known = int(np.count_nonzero(np.array(m.data) >= 0)) * m.info.resolution ** 2 if m else 0
    print(f'[explore] 끝: 목표 {visited}개 (도착 {ok_count}, 이어 가기 {chained}, 도중 전환 {switched}), '
          f'탐색한 면적 약 {known:.1f} m², {(time.time() - start) / 60:.1f}분', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


def goto_wait(ws, timeout):
    """진행 중인 goto 의 결과만 기다림 (새 요청 없이). 반환: (성공 여부, 메시지)"""
    end = time.time() + timeout
    task_id = _last['task']
    if task_id in _finished:   # 다른 곳(drain)에서 이미 끝 상태를 읽었으면
        return _finished[task_id]
    while time.time() < end:
        try:
            m = recv(ws)
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'state' and m.get('task') and m['task']['id'] == task_id and m['task']['status'] != 'running':
            return m['task']['status'] == 'done', m['robot'].get('message', '')
        if m['type'] == 'state' and not m.get('task'):
            return True, ''
    return False, '시간 초과'


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  # 웹 "탐색 중지" (SIGINT)
        print('[explore] 중지됨', flush=True)
        raise SystemExit(130)
