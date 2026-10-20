"""LiDAR 스캔 한 장으로 기준 위치 주변을 전수 탐색해서 맵과 가장 잘 맞는 위치를 찾고,
웹 서버를 통해 초기 위치로 지정한다.

    source scripts/env.sh
    python3 tools/localize.py [X Y YAW_DEG] [--range 0.4] [--dry-run]
    python3 tools/localize.py --global [--dry-run]    # 로봇 위치를 모를 때: 맵 전체에서 찾기

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


def search(lx, ly, dist, info, xs, ys, yaws):
    """후보 위치 (xs[i], ys[i]) 와 방향 yaws 의 모든 조합 점수. 반환: [(평균거리, x, y, yaw), ...]"""
    out = []
    for yaw in yaws:
        cx = lx * math.cos(yaw) - ly * math.sin(yaw)
        cy = lx * math.sin(yaw) + ly * math.cos(yaw)
        px = cx[None, :] + xs[:, None]          # (후보, 스캔 점)
        py = cy[None, :] + ys[:, None]
        d = score_points(dist, info, px.ravel(), py.ravel()).reshape(px.shape)
        mean = d.mean(axis=1)
        out.extend(zip(mean.tolist(), xs.tolist(), ys.tolist(), [yaw] * len(xs)))
    return out


def global_search(lx, ly, dist, occ_known_free, info):
    """맵 전체 탐색: 넓게(10cm, 5도) → 상위 후보 주변 정밀(2.5cm, 1도)."""
    res = info.resolution
    step = max(1, int(round(0.1 / res)))
    rr, cc = np.nonzero(occ_known_free[::step, ::step])
    xs = info.origin.position.x + (cc * step + 0.5) * res
    ys = info.origin.position.y + (rr * step + 0.5) * res
    coarse = search(lx, ly, dist, info, xs, ys, np.radians(np.arange(0, 360, 5)))
    coarse.sort()
    # 서로 30cm 이상 떨어진 상위 후보 5곳
    picks = []
    for c in coarse:
        if all(math.hypot(c[1] - p[1], c[2] - p[2]) > 0.3 for p in picks):
            picks.append(c)
        if len(picks) == 5:
            break
    refined = []
    for _, x0, y0, yaw0 in picks:
        g = np.arange(-0.15, 0.15 + 1e-6, 0.025)
        gx, gy = np.meshgrid(g, g)
        r = search(lx, ly, dist, info, x0 + gx.ravel(), y0 + gy.ravel(),
                   yaw0 + np.radians(np.arange(-6, 7, 1)))
        refined.append(min(r))
    refined.sort()
    return refined


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose', nargs='*', type=float)
    ap.add_argument('--range', type=float, default=0.4, help='위치 탐색 범위 (m)')
    ap.add_argument('--yaw-range', type=float, default=45.0, help='방향 탐색 범위 (도)')
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--dry-run', action='store_true', help='찾기만 하고 지정하지 않음')
    ap.add_argument('--global', dest='global_search', action='store_true',
                    help='맵 전체에서 찾기 (로봇 위치를 모를 때)')
    ap.add_argument('--fallback-global', action='store_true',
                    help='기준 위치 주변에서 잘 안 맞으면(20cm 이내 80% 미만) 맵 전체에서 다시 찾기')
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
    if args.global_search:
        grid_v = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
        known_free = (grid_v == 0) & (dist > 0.2)
        found = global_search(lx, ly, dist, known_free, info)
        if not found:
            raise SystemExit('맵에 빈 공간이 없습니다.')
        s0, x, y, yaw = found[0]
        d = score_points(dist, info, lx * math.cos(yaw) - ly * math.sin(yaw) + x,
                         lx * math.sin(yaw) + ly * math.cos(yaw) + y)
        best = (s0, x, y, yaw, float(np.mean(d < 0.1)), float(np.mean(d < 0.2)))
        # 멀리 떨어진 다른 후보가 거의 같은 점수면 (대칭인 방 등) 모호함
        for s1, x1, y1, _ in found[1:]:
            if math.hypot(x1 - x, y1 - y) > 0.5 and s1 < s0 * 1.15:
                print(f'경고: 비슷하게 맞는 다른 위치가 있습니다 ({x1:.2f}, {y1:.2f}). 결과를 지도에서 확인하세요.')
                break
    for dyaw in (np.radians(np.arange(-args.yaw_range, args.yaw_range + 0.1, 2.0))
                 if not args.global_search else []):
        yaw = yaw0 + dyaw
        cx = lx * math.cos(yaw) - ly * math.sin(yaw)
        cy = lx * math.sin(yaw) + ly * math.cos(yaw)
        for dx in np.arange(-args.range, args.range + 1e-6, 0.025):
            for dy in np.arange(-args.range, args.range + 1e-6, 0.025):
                d = score_points(dist, info, cx + x0 + dx, cy + y0 + dy)
                s = float(np.mean(d))
                if best is None or s < best[0]:
                    best = (s, x0 + dx, y0 + dy, yaw, float(np.mean(d < 0.1)), float(np.mean(d < 0.2)))

    if args.fallback_global and not args.global_search and best is not None and best[5] < 0.8:
        # 맵 좌표계가 예상과 다를 때 (예: Cartographer 맵은 원점이 로봇 출발 위치) 기준 위치가 틀린다
        print(f'기준 위치 주변이 잘 맞지 않아(20cm 이내 {best[5] * 100:.0f}%) 맵 전체에서 다시 찾습니다.')
        grid_v = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
        found = global_search(lx, ly, dist, (grid_v == 0) & (dist > 0.2), info)
        if found:
            s0, gx, gy, gyaw = found[0]
            d = score_points(dist, info, lx * math.cos(gyaw) - ly * math.sin(gyaw) + gx,
                             lx * math.sin(gyaw) + ly * math.cos(gyaw) + gy)
            if float(np.mean(d < 0.2)) > best[5]:
                best = (s0, gx, gy, gyaw, float(np.mean(d < 0.1)), float(np.mean(d < 0.2)))
    _, x, y, yaw, p10, p20 = best
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    print(f'가장 잘 맞는 위치: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°  '
          f'(스캔 점 중 벽과 10cm 이내 {p10 * 100:.0f}%, 20cm 이내 {p20 * 100:.0f}%)')
    print(f'RESULT {x:.3f} {y:.3f} {yaw:.4f} {p20:.3f}')  # 서버가 읽는 줄
    if p20 < 0.8:
        print('경고: 맞는 정도가 낮습니다. 기준 위치나 탐색 범위를 바꿔 다시 시도하세요.')
    if not args.dry_run:
        ws = websocket.create_connection(args.server, timeout=2)
        ws.send(json.dumps({'action': 'set_initial_pose', 'x': x, 'y': y, 'yaw': yaw, 'precise': True}))
        print('초기 위치 지정 요청을 보냈습니다.')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
