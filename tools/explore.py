"""SLAM 모드 자동 탐색 (프론티어 탐색). 맵이 더 넓어지지 않을 때까지 미탐색 경계로 계속 이동한다.

    source scripts/env.sh
    python3 tools/explore.py [--max-minutes 20] [--server ws://localhost:8000/ws/ui/]

1. /map 에서 프론티어(빈 칸 중 4방향 이웃에 미탐색이 있는 칸)를 찾고 이어진 것끼리 묶는다 (작은 묶음 제외)
2. 묶음마다 벽에서 30cm 이상 떨어진 가까운 빈 칸을 목표로 잡고, 가깝고 큰 것부터 고른다
3. 웹 "여기로 이동" 과 같은 요청(goto_point)을 서버로 보내고 결과를 기다린다. 실패한 곳은 다시 고르지 않는다
4. 프론티어가 남지 않으면 끝 (웹 "맵 저장" 으로 저장)
"""
import argparse
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
import websocket
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from warehouse_robot.astar import obstacle_distance

MIN_CLUSTER = 5        # 이 칸 수(25cm) 미만 프론티어 묶음은 무시
CHUNK = 40             # 큰 프론티어 묶음은 이 칸 수(2m) 조각으로 나눠 각각 목표 후보로
CLEARANCE = 0.30       # 목표 지점은 벽(점유 칸)에서 이만큼 떨어져야 함 (m)
SEARCH_RADIUS = 1.0    # 프론티어 묶음 주변에서 목표를 찾는 반경 (m)


def frontiers(grid):
    """grid: (h, w) int, -1 미탐색, 0~100. 반환: [ [(r, c), ...], ... ] 프론티어 묶음."""
    free = grid == 0
    unknown = grid < 0
    near_unknown = np.zeros_like(free)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    front = free & near_unknown
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


def pick_goal(msg, robot_xy, blacklist):
    """다음 목표 (x, y, yaw, 묶음 크기) 또는 None."""
    info = msg.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
    occ = grid >= 65
    # 벽에서만 떨어지면 된다 (미탐색 바로 옆도 허용). SLAM 은 LiDAR 가 아무것도 못 맞힌 방향(무한대)을
    # 빈 공간으로 칠하지 않아서, 넓은 곳에서는 아는 공간이 좁고 미탐색에서 떨어진 칸이 거의 없다
    dist = obstacle_distance(occ, res, CLEARANCE + 0.1)
    safe = (grid == 0) & (dist >= CLEARANCE)
    best = None
    k = int(SEARCH_RADIUS / res)
    for cells in frontiers(grid):
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
        if travel < 0.3:   # 이미 그 자리
            continue
        cost = travel - 0.02 * len(cells)          # 가깝고 큰 프론티어 우선
        if best is None or cost < best[0]:
            best = (cost, gx, gy, math.atan2(fy - gy, fx - gx), len(cells))
    return None if best is None else best[1:]


def goto(ws, x, y, yaw, timeout):
    """서버에 goto_point 를 보내고 작업이 끝날 때까지 기다린다. 반환: (성공 여부, 메시지)"""
    ws.send(json.dumps({'action': 'goto_point', 'x': x, 'y': y, 'yaw': yaw}))
    end = time.time() + timeout
    task_id = None
    while time.time() < end:
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
    start = time.time()
    visited = ok_count = 0
    while time.time() - start < args.max_minutes * 60:
        spin(1.0)
        if 'map' not in latest or not buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
            print('[explore] 맵/위치 대기 중', flush=True)
            spin(2.0)
            continue
        t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
        goal = pick_goal(latest['map'], (t.x, t.y), blacklist)
        if goal is None:
            # SLAM 맵 갱신이 늦을 수 있으니 한 번 더 기다렸다가 확인
            spin(6.0)
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
            goal = pick_goal(latest['map'], (t.x, t.y), blacklist)
            if goal is None:
                print('[explore] 남은 프론티어 없음: 탐색 완료', flush=True)
                break
        gx, gy, gyaw, size = goal
        visited += 1
        print(f'[explore] #{visited} 목표 ({gx:.2f}, {gy:.2f}) 프론티어 {size}칸', flush=True)
        ok, message = goto(ws, gx, gy, gyaw, args.goal_timeout)
        if ok:
            ok_count += 1
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
    print(f'[explore] 끝: 이동 {visited}회 (성공 {ok_count}), 탐색한 면적 약 {known:.1f} m², '
          f'{(time.time() - start) / 60:.1f}분', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  # 웹 "탐색 중지" (SIGINT)
        print('[explore] 중지됨', flush=True)
        raise SystemExit(130)
