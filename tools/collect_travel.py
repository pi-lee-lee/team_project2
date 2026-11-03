"""Collect records for training the travel-time model: repeatedly drive to random free spots on the map (same request as the web "go here").

    source scripts/env.sh
    python3 tools/collect_travel.py [--count 30] [--min-dist 0.5] [--max-dist 2.5] [--seed 1]

For each move the server records the start pose / A* path length / amount of turning / elapsed time (planning.py).
Once enough records exist, the "travel-time model" on the web locations card switches to the learned values.
"""
import argparse
import json
import math
import random
import time

import numpy as np
import rclpy
import websocket
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from warehouse_robot.astar import obstacle_distance


def free_points(msg, clearance=0.35):
    info = msg.info
    grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
    dist = obstacle_distance((grid >= 65) | (grid < 0), info.resolution, clearance + 0.1)
    rr, cc = np.nonzero((grid == 0) & (dist >= clearance))
    return [(info.origin.position.x + (c + 0.5) * info.resolution,
             info.origin.position.y + (r + 0.5) * info.resolution) for r, c in zip(rr, cc)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=30)
    ap.add_argument('--min-dist', type=float, default=0.5)
    ap.add_argument('--max-dist', type=float, default=2.5)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    args = ap.parse_args()
    random.seed(args.seed)

    rclpy.init()
    node = Node('collect_travel')
    got = {}
    node.create_subscription(OccupancyGrid, 'map', lambda m: got.setdefault('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    while 'map' not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    points = free_points(got['map'])
    print(f'[collect] 후보 지점 {len(points)}개', flush=True)

    ws = websocket.create_connection(args.server, timeout=0.5)
    snap = json.loads(ws.recv())
    pose = snap.get('pose') or {'x': 0.0, 'y': 0.0}
    done = ok = 0
    while done < args.count:
        cands = [p for p in points if args.min_dist <= math.hypot(p[0] - pose['x'], p[1] - pose['y']) <= args.max_dist]
        if not cands:
            print('[collect] 조건에 맞는 지점이 없습니다', flush=True)
            break
        x, y = random.choice(cands)
        yaw = random.uniform(-math.pi, math.pi)
        ws.send(json.dumps({'action': 'goto_point', 'x': x, 'y': y, 'yaw': yaw}))
        t0, task_id, result = time.time(), None, None
        while time.time() - t0 < 120 and result is None:
            try:
                m = json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            if m['type'] == 'pose':
                pose = m
            elif m['type'] == 'error':
                result = 'error: ' + m['message']
            elif m['type'] == 'state' and m.get('task'):
                t = m['task']
                if task_id is None and t['status'] == 'running':
                    task_id = t['id']
                if t['id'] == task_id and t['status'] != 'running':
                    result = t['status']
        done += 1
        ok += result == 'done'
        print(f'[collect] {done}/{args.count} ({x:.2f}, {y:.2f}) {result} {time.time() - t0:.1f}초', flush=True)
        if result is None:
            ws.send(json.dumps({'action': 'cancel_task'}))
    print(f'[collect] 끝: {done}회 중 성공 {ok}회', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('[collect] 중지됨', flush=True)
