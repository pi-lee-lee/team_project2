"""LiDAR 스캔 한 장으로 기준 위치 주변을 전수 탐색해서 맵과 가장 잘 맞는 위치를 찾고,
웹 서버를 통해 초기 위치로 지정한다.

    source scripts/env.sh
    python3 tools/localize.py [X Y YAW_DEG] [--range 0.4] [--dry-run]

기준 위치를 생략하면 현재 추정 위치(map->base_footprint), 그것도 없으면 (0, 0, 0)을 쓴다.
"""
import argparse
import json
import math

import numpy as np
import rclpy
import websocket
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from warehouse_robot.astar import obstacle_distance
from warehouse_robot.astar_planner_node import yaw_from_quaternion


def score_points(dist, info, px, py):
    c = ((px - info.origin.position.x) / info.resolution).astype(int)
    r = ((py - info.origin.position.y) / info.resolution).astype(int)
    inside = (c >= 0) & (c < info.width) & (r >= 0) & (r < info.height)
    d = np.full(len(px), 0.5)
    d[inside] = dist[r[inside], c[inside]]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose', nargs='*', type=float)
    ap.add_argument('--range', type=float, default=0.4, help='위치 탐색 범위 (m)')
    ap.add_argument('--yaw-range', type=float, default=45.0, help='방향 탐색 범위 (도)')
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--dry-run', action='store_true', help='찾기만 하고 지정하지 않음')
    args = ap.parse_args()

    rclpy.init()
    node = Node('localize_tool')
    buf = Buffer()
    TransformListener(buf, node)
    got = {}
    node.create_subscription(LaserScan, 'scan', lambda m: got.setdefault('scan', m), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, 'map', lambda m: got.setdefault('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'scan' in got and 'map' in got and buf.can_transform('base_footprint', 'base_scan', rclpy.time.Time()):
            break
    else:
        raise SystemExit('scan / map / TF를 받지 못했습니다. bringup과 Nav2(map_server)가 실행 중인지 확인하세요.')

    if len(args.pose) == 3:
        x0, y0, yaw0 = args.pose[0], args.pose[1], math.radians(args.pose[2])
    elif buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
        t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        x0, y0, yaw0 = t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)
    else:
        x0, y0, yaw0 = 0.0, 0.0, 0.0

    scan, grid = got['scan'], got['map']
    info = grid.info
    off = buf.lookup_transform('base_footprint', 'base_scan', rclpy.time.Time()).transform.translation
    occ = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width) >= 65
    dist = np.minimum(obstacle_distance(occ, info.resolution, 1.0), 0.5)

    ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    rng = np.array(scan.ranges)
    ok = np.isfinite(rng) & (rng > scan.range_min) & (rng < 3.5)
    lx = off.x + rng[ok] * np.cos(ang[ok])
    ly = off.y + rng[ok] * np.sin(ang[ok])

    best = None
    for dyaw in np.radians(np.arange(-args.yaw_range, args.yaw_range + 0.1, 2.0)):
        yaw = yaw0 + dyaw
        cx = lx * math.cos(yaw) - ly * math.sin(yaw)
        cy = lx * math.sin(yaw) + ly * math.cos(yaw)
        for dx in np.arange(-args.range, args.range + 1e-6, 0.025):
            for dy in np.arange(-args.range, args.range + 1e-6, 0.025):
                d = score_points(dist, info, cx + x0 + dx, cy + y0 + dy)
                s = float(np.mean(d))
                if best is None or s < best[0]:
                    best = (s, x0 + dx, y0 + dy, yaw, float(np.mean(d < 0.1)), float(np.mean(d < 0.2)))

    _, x, y, yaw, p10, p20 = best
    print(f'가장 잘 맞는 위치: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°  '
          f'(스캔 점 중 벽과 10cm 이내 {p10 * 100:.0f}%, 20cm 이내 {p20 * 100:.0f}%)')
    if p20 < 0.8:
        print('경고: 맞는 정도가 낮습니다. 기준 위치나 탐색 범위를 바꿔 다시 시도하세요.')
    if not args.dry_run:
        ws = websocket.create_connection(args.server, timeout=2)
        ws.send(json.dumps({'action': 'set_initial_pose', 'x': x, 'y': y, 'yaw': yaw}))
        print('초기 위치 지정 요청을 보냈습니다.')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
