"""SLAM 모드 자동 탐색 (프론티어 탐색). 맵이 더 넓어지지 않을 때까지 미탐색 경계로 계속 이동한다.

    source scripts/env.sh
    python3 tools/explore.py [--max-minutes 20] [--server ws://localhost:8000/ws/ui/]

1. /map 에서 프론티어(빈 칸 중 4방향 이웃에 미탐색이 있는 칸)를 찾고 이어진 것끼리 묶는다 (작은 묶음 제외)
2. 묶음마다 벽에서 30cm 이상 떨어진 가까운 빈 칸을 목표로 잡고 점수가 가장 좋은 것을 고른다
   점수 = 거리 + 방향 벌점 x (로봇이 보는 방향과 목표 방향의 각도) - 크기 보너스
   → 가던 방향을 유지한다 (복도 끝까지 간 뒤 돌아서기). 뒤로 돌아가는 목표는 가까워도 뒤로 미룬다
3. 웹 "여기로 이동" 과 같은 요청(goto_point)을 서버로 보내고 결과를 기다린다. 실패한 곳은 다시 고르지 않는다
   가는 도중 목표 프론티어가 이미 보여서 사라지면 도착을 기다리지 않고 취소 후 바로 다음 목표로 (시간 절약)
   막힌 작은 미탐색 구멍(책상 아래, 앞판 뒤, 물건 사이 등 POCKET_MIN 미만)에 붙은 프론티어는 고르지 않는다.
   가서 봐도 가려져서 안 보이는 곳이라 가장 가까운 목표로 뽑혀 시간만 쓰던 문제
4. 프론티어가 남지 않으면 끝 (웹 "맵 저장" 으로 저장)

    --turn-weight 0 --no-early-switch --pocket-min 0 : 예전 방식 (가깝고 큰 것 우선, 도착까지 기다림)
"""
import argparse
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from scipy import ndimage
import websocket
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from warehouse_robot.astar import obstacle_distance

MIN_CLUSTER = 8        # 이 칸 수(40cm) 미만 프론티어 묶음은 무시
CHUNK = 40             # 큰 프론티어 묶음은 이 칸 수(2m) 조각으로 나눠 각각 목표 후보로
CLEARANCE = 0.30       # 목표 지점은 벽(점유 칸)에서 이만큼 떨어져야 함 (m)
SEARCH_RADIUS = 1.0    # 프론티어 묶음 주변에서 목표를 찾는 반경 (m)
TURN_WEIGHT = 0.6      # 방향 벌점 (m/rad): 180° 뒤의 목표는 약 1.9m 더 먼 것으로 친다
POCKET_MIN = 1.0       # 미탐색 영역이 이 넓이(m²)보다 작고 맵 가장자리에 닿지 않으면 막힌 구멍으로 보고 무시
MIN_TRAVEL = 0.5       # 이보다 가까운 목표는 고르지 않음 (제자리 수준의 이동)
SEEN_RADIUS = 0.6      # 목표 프론티어 중심 이 반경 안에 프론티어가 MIN_CLUSTER 칸 미만이면 '이미 봄'


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


def frontiers(grid, pocket_cells=0):
    """grid: (h, w) int, -1 미탐색, 0~100. 반환: [ [(r, c), ...], ... ] 프론티어 묶음.
    pocket_cells > 0 이면 그보다 작은 막힌 미탐색 구멍에만 붙은 칸은 프론티어가 아님."""
    front = frontier_mask(grid, pocket_cells)
    h, w = grid.shape
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


def grid_of(msg):
    info = msg.info
    return np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)


def frontier_mask(grid, pocket_cells=0):
    """프론티어 칸 (bool): 빈 칸 중 4방향 이웃에 (넓은) 미탐색이 있는 칸."""
    free = grid == 0
    unknown = open_unknown(grid, pocket_cells)
    near = np.zeros_like(free)
    near[1:, :] |= unknown[:-1, :]
    near[:-1, :] |= unknown[1:, :]
    near[:, 1:] |= unknown[:, :-1]
    near[:, :-1] |= unknown[:, 1:]
    return free & near


def frontier_left(msg, fx, fy, pocket_min=POCKET_MIN):
    """(fx, fy) 주변 SEEN_RADIUS 안 프론티어 칸 수. 목표로 가는 도중 이미 보였는지 판단용."""
    info = msg.info
    res = info.resolution
    front = frontier_mask(grid_of(msg), int(pocket_min / res ** 2))
    c = int((fx - info.origin.position.x) / res)
    r = int((fy - info.origin.position.y) / res)
    k = int(SEEN_RADIUS / res) + 1
    rr, cc = np.nonzero(front[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1])
    rr = rr + max(0, r - k)
    cc = cc + max(0, c - k)
    return int(np.count_nonzero((rr - r) ** 2 + (cc - c) ** 2 <= k * k))


def pick_goal(msg, robot_xy, blacklist, robot_yaw=None, turn_weight=TURN_WEIGHT, pocket_min=POCKET_MIN):
    """다음 목표 (x, y, yaw, 묶음 크기, 프론티어 중심 x, y) 또는 None.
    robot_yaw 를 주면 그 방향에서 많이 벗어난 목표일수록 벌점 (가던 방향 유지)."""
    info = msg.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    grid = grid_of(msg)
    occ = grid >= 65
    # 벽에서만 떨어지면 된다 (미탐색 바로 옆도 허용). SLAM 은 LiDAR 가 아무것도 못 맞힌 방향(무한대)을
    # 빈 공간으로 칠하지 않아서, 넓은 곳에서는 아는 공간이 좁고 미탐색에서 떨어진 칸이 거의 없다
    dist = obstacle_distance(occ, res, CLEARANCE + 0.1)
    safe = (grid == 0) & (dist >= CLEARANCE)
    best = None
    k = int(SEARCH_RADIUS / res)
    for cells in frontiers(grid, int(pocket_min / res ** 2)):
        cr = int(np.mean([c[0] for c in cells]))
        cc = int(np.mean([c[1] for c in cells]))
        r0, r1 = max(0, cr - k), min(info.height, cr + k + 1)
        c0, c1 = max(0, cc - k), min(info.width, cc + k + 1)
        rr, ccs = np.nonzero(safe[r0:r1, c0:c1])
        if len(rr) == 0:
            continue
        # 프론티어 중심에 가까운 순서로, 실패했던 곳에서 0.5m 이상 떨어진 첫 칸
        xs = ox + (ccs + c0 + 0.5) * res
        ys = oy + (rr + r0 + 0.5) * res
        order = np.argsort((rr + r0 - cr) ** 2 + (ccs + c0 - cc) ** 2)
        pick = next((j for j in order[:200]
                     if all(math.hypot(xs[j] - bx, ys[j] - by) >= 0.5 for bx, by in blacklist)), None)
        if pick is None:
            continue
        gx, gy = float(xs[pick]), float(ys[pick])
        fx, fy = ox + (cc + 0.5) * res, oy + (cr + 0.5) * res
        travel = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if travel < MIN_TRAVEL:   # 이미 그 자리
            continue
        cost = travel - 0.02 * len(cells)          # 가깝고 큰 프론티어 우선
        bearing = math.atan2(gy - robot_xy[1], gx - robot_xy[0])
        if robot_yaw is not None:
            turn = abs(math.atan2(math.sin(bearing - robot_yaw), math.cos(bearing - robot_yaw)))
            cost += turn_weight * turn
        if best is None or cost < best[0]:
            # 도착 방향 = 가는 방향 (도착해서 따로 돌지 않게. 프론티어가 옆이면 프론티어 쪽)
            face = math.atan2(fy - gy, fx - gx)
            yaw = bearing if math.hypot(fx - gx, fy - gy) < 0.3 else face
            best = (cost, gx, gy, yaw, len(cells), fx, fy)
    return None if best is None else best[1:]


def wait_idle(ws, timeout=3.0):
    """취소 후 작업이 끝났다는 상태가 올 때까지 (다음 goto 가 '진행 중' 으로 거절되지 않게)."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            m = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'state' and not (m.get('task') and m['task']['status'] == 'running'):
            return


def goto(ws, x, y, yaw, timeout, seen=None):
    """서버에 goto_point 를 보내고 작업이 끝날 때까지 기다린다. 반환: (성공 여부, 메시지)
    seen(): 가는 도중 1초마다 불러서 True 면 (목표 프론티어를 이미 봄) 취소하고 (True, '이미 봄')."""
    ws.send(json.dumps({'action': 'goto_point', 'x': x, 'y': y, 'yaw': yaw}))
    end = time.time() + timeout
    task_id = None
    next_check = time.time() + 1.5
    while time.time() < end:
        if seen is not None and task_id is not None and time.time() >= next_check:
            next_check = time.time() + 1.0
            if seen():
                ws.send(json.dumps({'action': 'cancel_task'}))
                wait_idle(ws)
                return True, '이미 봄'
        try:
            m = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'error':
            return None, m['message']   # 이동이 아니라 요청이 거절됨 (진행 중 작업 등)
        if m['type'] == 'state' and m.get('task'):
            t = m['task']
            if task_id is None and t['status'] == 'running':
                task_id = t['id']
            if t['id'] == task_id and t['status'] != 'running':
                return t['status'] == 'done', m['robot'].get('message', '')
    ws.send(json.dumps({'action': 'cancel_task'}))
    return False, f'{timeout:.0f}초 안에 도착하지 못함'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-minutes', type=float, default=20.0)
    ap.add_argument('--goal-timeout', type=float, default=90.0)
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--turn-weight', type=float, default=TURN_WEIGHT, help='방향 벌점 (m/rad), 0 이면 방향 무시')
    ap.add_argument('--no-early-switch', action='store_true', help='목표 프론티어가 보여도 도착까지 기다림')
    ap.add_argument('--pocket-min', type=float, default=POCKET_MIN,
                    help='이보다 작은(m²) 막힌 미탐색 구멍은 무시, 0 이면 전부 탐색')
    args = ap.parse_args()

    rclpy.init()
    node = Node('explore_tool', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=__import__('os').environ.get('WAREHOUSE_SIM_TIME') == 'true')])
    buf = Buffer()
    TransformListener(buf, node)
    latest = {}
    node.create_subscription(OccupancyGrid, 'map', lambda m: latest.__setitem__('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    ws = websocket.create_connection(args.server, timeout=0.5)
    snap = json.loads(ws.recv())
    if snap.get('task') and snap['task']['status'] == 'running':
        print('[explore] 진행 중인 작업을 취소하고 시작합니다', flush=True)
        ws.send(json.dumps({'action': 'cancel_task'}))
        time.sleep(1.0)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    blacklist = []
    reached = []   # 도착한 목표. 같은 곳에 두 번 도착해도 프론티어가 안 사라지면(그 자리에서 볼 수 없는 곳) 제외
    start = time.time()
    visited = ok_count = 0
    def robot_pose():
        tf = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        q = tf.rotation
        return tf.translation.x, tf.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    switched = 0
    while time.time() - start < args.max_minutes * 60:
        spin(0.3)
        if 'map' not in latest or not buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
            print('[explore] 맵/위치 대기 중', flush=True)
            spin(2.0)
            continue
        rx, ry, ryaw = robot_pose()
        goal = pick_goal(latest['map'], (rx, ry), blacklist, ryaw, args.turn_weight, args.pocket_min)
        if goal is None:
            # SLAM 맵 갱신이 늦을 수 있으니 한 번 더 기다렸다가 확인
            spin(6.0)
            rx, ry, ryaw = robot_pose()
            goal = pick_goal(latest['map'], (rx, ry), blacklist, ryaw, args.turn_weight, args.pocket_min)
            if goal is None:
                print('[explore] 남은 프론티어 없음: 탐색 완료', flush=True)
                break
        gx, gy, gyaw, size, fx, fy = goal
        visited += 1
        turn = math.degrees(abs(math.atan2(math.sin(math.atan2(gy - ry, gx - rx) - ryaw),
                                           math.cos(math.atan2(gy - ry, gx - rx) - ryaw))))
        print(f'[explore] #{visited} 목표 ({gx:.2f}, {gy:.2f}) 프론티어 {size}칸, 방향 차이 {turn:.0f}°', flush=True)

        def seen():
            spin(0.05)
            return frontier_left(latest['map'], fx, fy, args.pocket_min) < MIN_CLUSTER

        ok, message = goto(ws, gx, gy, gyaw, args.goal_timeout, None if args.no_early_switch else seen)
        if ok and message == '이미 봄':
            switched += 1
            print('[explore]   가는 도중 목표 프론티어가 보임: 다음 목표로', flush=True)
            continue
        if ok:
            ok_count += 1
            if any(math.hypot(gx - rx, gy - ry) < 0.4 for rx, ry in reached):
                blacklist.append((gx, gy))
                print(f'[explore]   같은 곳에 다시 왔지만 프론티어가 남음: 이 근처는 제외', flush=True)
            reached.append((gx, gy))
        elif ok is None:
            print(f'[explore]   요청 거절: {message} (잠시 뒤 다시 시도)', flush=True)
            spin(3.0)
        else:
            blacklist.append((gx, gy))
            print(f'[explore]   실패: {message}', flush=True)
    else:
        print(f'[explore] 시간 제한 {args.max_minutes:.0f}분 도달', flush=True)
    m = latest.get('map')
    known = int(np.count_nonzero(np.array(m.data) >= 0)) * m.info.resolution ** 2 if m else 0
    print(f'[explore] 끝: 이동 {visited}회 (도착 {ok_count}, 도중 전환 {switched}), 탐색한 면적 약 {known:.1f} m², '
          f'{(time.time() - start) / 60:.1f}분', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  # 웹 "탐색 중지" (SIGINT)
        print('[explore] 중지됨', flush=True)
        raise SystemExit(130)
