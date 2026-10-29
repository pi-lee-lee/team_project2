"""LiDAR 스캔 한 장으로 기준 위치 주변을 전수 탐색해서 맵과 가장 잘 맞는 위치를 찾고,
웹 서버를 통해 초기 위치로 지정한다.

    source scripts/env.sh
    python3 tools/localize.py [X Y YAW_DEG] [--range 0.4] [--dry-run]
    python3 tools/localize.py --global [--dry-run]    # 로봇 위치를 모를 때: 맵 전체에서 찾기

기준 위치를 생략하면 현재 추정 위치(map->base_footprint), 그것도 없으면 (0, 0, 0)을 쓴다.

--global (웹 "위치 자동 찾기"): 스캔 한 장으로는 좌석 줄처럼 반복되는 곳에서 옆 줄로 잘못 찾을 수 있어서
  1. 제자리에서 45° 씩 8번 돌며 멈춰서 스캔을 모음 (돌면서 찍으면 시각 차이로 점이 번짐)
  2. 오도메트리로 스캔들을 지금 로봇 기준으로 합쳐(5cm 칸으로 솎음) 맵 전체에서 찾음
  3. 비슷한 다른 후보가 있거나 잘 안 맞으면, 가장 트인 쪽으로 최대 0.8m 가서 다시 모아 찾음 (최대 2번)
  움직임은 브리지의 수동 조작 입력(cmd_vel_teleop)으로 → 브리지 안전장치(앞 0.25m / 뒤 0.3m)도 같이 적용.
  이 도구도 LiDAR 로 한 번 더 확인: 로봇 중심 0.30m 안에 물체가 있으면 돌지 않고, 앞 빈 틈 0.35m 미만이면 멈춤.
  --no-move: 예전처럼 움직이지 않고 스캔 한 장으로만
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
import websocket
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from warehouse_robot.astar import obstacle_distance
from warehouse_robot.astar_planner_node import yaw_from_quaternion


MAX_RANGE = 3.5        # m, 이보다 먼 점은 쓰지 않음 (LDS-02 를 넉넉히 믿는 거리, tools/scan_check.py 로 확인)
ROT_STEP = math.radians(45)
ROT_SPEED = 0.4        # rad/s
MOVE_SPEED = 0.1       # m/s
MOVE_MAX = 0.8         # m, 한 번에 가는 최대 거리
MAX_MOVES = 2
ROT_CLEAR = 0.33       # m, 로봇 중심에서 이 안에 물체가 있으면 돌지 않음 (뒤에 붙인 8cm 포함 반경 0.28 + 여유)
FRONT_CLEAR = 0.35     # m, 앞 빈 틈(로봇 앞면 ~ 물체)이 이보다 작아지면 멈춤
HALF_WIDTH = 0.22      # m, 앞으로 갈 때 확인하는 통로 반폭 (로봇 0.153 + 여유)
FRONT_X = 0.07         # m, base_footprint 에서 로봇 앞면까지
VOXEL = 0.05           # m, 합친 점을 솎는 칸


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


def global_search(lx, ly, dist, occ_known_free, info, coarse_points=400):
    """맵 전체 탐색: 넓게(10cm, 5도) → 상위 후보 주변 정밀(2.5cm, 1도).
    넓게 찾을 때는 점이 많으면(여러 스캔을 합친 경우) coarse_points 개만 골라 써서 빠르게, 정밀은 전부."""
    res = info.resolution
    if len(lx) > coarse_points:
        pick = np.linspace(0, len(lx) - 1, coarse_points).astype(int)
        cx_, cy_ = lx[pick], ly[pick]
    else:
        cx_, cy_ = lx, ly
    step = max(1, int(round(0.1 / res)))
    rr, cc = np.nonzero(occ_known_free[::step, ::step])
    xs = info.origin.position.x + (cc * step + 0.5) * res
    ys = info.origin.position.y + (rr * step + 0.5) * res
    coarse = search(cx_, cy_, dist, info, xs, ys, np.radians(np.arange(0, 360, 5)))
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


def scan_xy(scan, off):
    """스캔 → base_footprint 기준 점 (x, y). MAX_RANGE 안의 유효한 빔만."""
    ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    rng = np.array(scan.ranges)
    ok = np.isfinite(rng) & (rng > scan.range_min) & (rng < MAX_RANGE)
    return off.x + rng[ok] * np.cos(ang[ok]), off.y + rng[ok] * np.sin(ang[ok])


class Explorer:
    """위치를 찾으려고 제자리 회전/조금 이동하며 멈춘 스캔을 모은다. 움직임은 cmd_vel_teleop (브리지 안전장치 경유)."""

    def __init__(self, node, buf, off):
        self.node, self.buf, self.off = node, buf, off
        self.latest = None        # (받은 시각, 스캔)
        self.records = []         # [(xs, ys, 오도메트리 x, y, yaw)]
        self.pub = node.create_publisher(Twist, 'cmd_vel_teleop', 10)
        node.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'latest', (time.monotonic(), m)),
                                 qos_profile_sensor_data)

    def spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def odom(self):
        t = self.buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time()).transform
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    def points(self):
        """지금 스캔의 base_footprint 기준 점. 스캔이 1초 넘게 없으면 None (= 위험, 멈춤)."""
        if self.latest is None or time.monotonic() - self.latest[0] > 1.0:
            return None
        return scan_xy(self.latest[1], self.off)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            self.spin(0.05)

    def capture(self):
        """멈춘 뒤(0.5초) 새로 들어온 스캔 한 장을 오도메트리 위치와 함께 기록."""
        self.stop()
        self.spin(0.5)
        t0 = time.monotonic()
        while self.latest is None or self.latest[0] < t0:
            self.spin(0.05)
            if time.monotonic() - t0 > 2.0:
                return False
        xs, ys = scan_xy(self.latest[1], self.off)
        self.records.append((xs, ys, *self.odom()))
        return True

    def rotate_safe(self):
        p = self.points()
        return p is not None and (len(p[0]) == 0 or float(np.hypot(*p).min()) > ROT_CLEAR)

    def front_free(self):
        """앞 통로(반폭 HALF_WIDTH)의 빈 틈 (m, 로봇 앞면부터). 스캔이 없으면 0."""
        p = self.points()
        if p is None:
            return 0.0
        x, y = p
        sel = (x > 0) & (np.abs(y) < HALF_WIDTH)
        return float(x[sel].min() - FRONT_X) if sel.any() else MAX_RANGE

    def rotate(self, angle):
        """angle(rad) 만큼 제자리 회전. 반환: 다 돌았으면 True (가까운 물체/명령이 안 먹혀서 멈추면 False)."""
        if not self.rotate_safe():
            print(f'[위치 찾기] 로봇 주변 {ROT_CLEAR:.2f}m 안에 물체가 있어 돌지 않습니다', flush=True)
            return False
        done, last, moved_at = 0.0, self.odom()[2], time.monotonic()
        w = math.copysign(ROT_SPEED, angle)
        while done < abs(angle) - math.radians(2):
            if not self.rotate_safe():
                self.stop()
                print('[위치 찾기] 도는 중 가까운 물체: 멈춤', flush=True)
                return False
            self.send(0.0, w)
            self.spin(0.1)
            yaw = self.odom()[2]
            step = abs(math.atan2(math.sin(yaw - last), math.cos(yaw - last)))
            if step > 1e-3:
                moved_at = time.monotonic()
            done += step
            last = yaw
            if time.monotonic() - moved_at > 3.0:
                self.stop()
                print('[위치 찾기] 회전 명령이 먹히지 않음 (다른 작업/정지 중?): 움직이지 않고 찾습니다', flush=True)
                return False
        self.stop()
        return True

    def forward(self, dist):
        """앞으로 dist(m). 앞이 FRONT_CLEAR 보다 가까워지면 멈춤. 반환: 간 거리."""
        x0, y0, _ = self.odom()
        moved_at = time.monotonic()
        last = 0.0
        while True:
            x, y, _ = self.odom()
            gone = math.hypot(x - x0, y - y0)
            if gone >= dist:
                break
            if self.front_free() < FRONT_CLEAR:
                print(f'[위치 찾기] 앞 {FRONT_CLEAR:.2f}m 안에 물체: 멈춤 ({gone:.2f}m 이동)', flush=True)
                break
            if gone - last > 0.005:
                moved_at, last = time.monotonic(), gone
            if time.monotonic() - moved_at > 3.0:
                print('[위치 찾기] 이동 명령이 먹히지 않음: 멈춤', flush=True)
                break
            self.send(MOVE_SPEED, 0.0)
            self.spin(0.1)
        self.stop()
        x, y, _ = self.odom()
        return math.hypot(x - x0, y - y0)

    def open_heading(self):
        """가장 트인 방향 (로봇 기준 rad, 빈 틈 m). 15° 간격, 같으면 덜 도는 쪽. 없으면 None."""
        p = self.points()
        if p is None:
            return None
        best = None
        for deg in range(0, 360, 15):
            a = math.radians(deg if deg <= 180 else deg - 360)
            c, s_ = math.cos(-a), math.sin(-a)
            x = p[0] * c - p[1] * s_
            y = p[0] * s_ + p[1] * c
            sel = (x > 0) & (np.abs(y) < HALF_WIDTH)
            free = float(x[sel].min() - FRONT_X) if sel.any() else MAX_RANGE
            key = (round(free, 1), -abs(a))
            if best is None or key > best[0]:
                best = (key, a, free)
        return best[1:] if best else None

    def merged(self):
        """모은 스캔을 지금 로봇(base_footprint) 기준으로 합친 점 (VOXEL 칸으로 솎음)."""
        fx, fy, fyaw = self.odom()
        allx, ally = [], []
        for xs, ys, ox, oy, oyaw in self.records:
            # 스캔 때 로봇 기준 → odom → 지금 로봇 기준
            wx = ox + xs * math.cos(oyaw) - ys * math.sin(oyaw)
            wy = oy + xs * math.sin(oyaw) + ys * math.cos(oyaw)
            dx, dy = wx - fx, wy - fy
            allx.append(dx * math.cos(-fyaw) - dy * math.sin(-fyaw))
            ally.append(dx * math.sin(-fyaw) + dy * math.cos(-fyaw))
        x, y = np.concatenate(allx), np.concatenate(ally)
        _, keep = np.unique(np.floor(np.column_stack([x, y]) / VOXEL).astype(int), axis=0, return_index=True)
        return x[keep], y[keep]


def evaluate(lx, ly, dist, info):
    """맵 전체에서 찾기. 반환: (best=(평균, x, y, yaw, 10cm 비율, 20cm 비율), 비슷한 다른 후보 (x, y) 또는 None)."""
    grid_v = np.array(evaluate.grid.data, dtype=np.int16).reshape(info.height, info.width)
    found = global_search(lx, ly, dist, (grid_v == 0) & (dist > 0.2), info)
    if not found:
        raise SystemExit('맵에 빈 공간이 없습니다.')
    s0, x, y, yaw = found[0]
    d = score_points(dist, info, lx * math.cos(yaw) - ly * math.sin(yaw) + x,
                     lx * math.sin(yaw) + ly * math.cos(yaw) + y)
    other = next(((x1, y1) for s1, x1, y1, _ in found[1:] if math.hypot(x1 - x, y1 - y) > 0.5 and s1 < s0 * 1.15),
                 None)
    return (s0, x, y, yaw, float(np.mean(d < 0.1)), float(np.mean(d < 0.2))), other


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose', nargs='*', type=float)
    ap.add_argument('--range', type=float, default=0.4, help='위치 탐색 범위 (m)')
    ap.add_argument('--yaw-range', type=float, default=45.0, help='방향 탐색 범위 (도)')
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--dry-run', action='store_true', help='찾기만 하고 지정하지 않음')
    ap.add_argument('--global', dest='global_search', action='store_true',
                    help='맵 전체에서 찾기 (로봇 위치를 모를 때)')
    ap.add_argument('--no-move', action='store_true',
                    help='--global 에서 돌거나 움직이지 않고 지금 스캔 한 장으로만 찾기')
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

    lx, ly = scan_xy(scan, off)

    best = None
    evaluate.grid = grid
    if args.global_search and not args.no_move:
        ex = Explorer(node, buf, off)
        try:
            # 1. 제자리에서 45° 씩 돌며 멈춰서 모으기
            ex.capture()
            turned = 0
            while turned < 7 and ex.rotate(ROT_STEP):
                ex.capture()
                turned += 1
            print(f'[위치 찾기] 제자리 {turned * 45}° 돌며 스캔 {len(ex.records)}장', flush=True)
            if turned == 7:   # 315° 돌면 처음보다 오른쪽 45° 라 마저 돌아 처음 방향으로 (그 방향 스캔은 이미 있음)
                ex.rotate(ROT_STEP)
            lx, ly = ex.merged()
            best, other = evaluate(lx, ly, dist, info)
            # 2. 모호하거나 잘 안 맞으면 트인 쪽으로 조금 가서 더 모으기
            moves = 0
            while (other is not None or best[5] < 0.8) and moves < MAX_MOVES:
                h = ex.open_heading()
                if h is None or h[1] < FRONT_CLEAR + 0.2:
                    print('[위치 찾기] 갈 만큼 트인 곳이 없어 이동하지 않습니다', flush=True)
                    break
                why = f'비슷한 후보 ({other[0]:.2f}, {other[1]:.2f})' if other else f'20cm 일치 {best[5] * 100:.0f}%'
                print(f'[위치 찾기] {why}: {math.degrees(h[0]):.0f}° 쪽(빈 틈 {h[1]:.1f}m)으로 조금 이동', flush=True)
                if abs(h[0]) > math.radians(5) and not ex.rotate(h[0]):
                    break
                gone = ex.forward(min(MOVE_MAX, h[1] - FRONT_CLEAR))
                moves += 1
                ex.capture()
                for _ in range(3):   # 새 자리에서 90° 씩 둘러보기
                    if not ex.rotate(math.radians(90)):
                        break
                    ex.capture()
                print(f'[위치 찾기] {gone:.2f}m 이동, 스캔 {len(ex.records)}장', flush=True)
                lx, ly = ex.merged()
                best, other = evaluate(lx, ly, dist, info)
        finally:
            ex.stop()
        if other is not None:
            print(f'경고: 비슷하게 맞는 다른 위치가 있습니다 ({other[0]:.2f}, {other[1]:.2f}). 결과를 지도에서 확인하세요.')
    elif args.global_search:
        best, other = evaluate(lx, ly, dist, info)
        # 멀리 떨어진 다른 후보가 거의 같은 점수면 (대칭인 방 등) 모호함
        if other is not None:
            print(f'경고: 비슷하게 맞는 다른 위치가 있습니다 ({other[0]:.2f}, {other[1]:.2f}). 결과를 지도에서 확인하세요.')
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
