"""직접 구현한 A*를 ROS 2 서비스로 제공하는 노드.

구독:  map (nav_msgs/OccupancyGrid, transient local)
       scan_level (sensor_msgs/LaserScan, 브리지가 기울지 않았을 때만 발행) - 맵에 없는 장애물(새로 놓인 상자, 사람 등)을 잠시 기억해 경로 계획에 반영
       astar/obstacle (geometry_msgs/PointStamped) - 브리지가 부딪힘을 감지한 자리 (LiDAR 에 안 보이는 낮은 물체)
       astar/zones (std_msgs/String, JSON 목록, transient local) - 웹에서 그린 금지 구역 (브리지가 서버에서 받아 발행)
발행:  astar/remembered (geometry_msgs/PoseArray) - 지금 기억 중인 LiDAR 장애물 칸 (1초마다, 웹 표시용)
       astar/bumps (geometry_msgs/PoseArray) - 기억 중인 부딪힌 자리 중심
서비스: astar/get_plan (nav_msgs/srv/GetPlan)
발행:  astar/plan (nav_msgs/Path)  - RViz 확인용
       astar/plan_map (nav_msgs/OccupancyGrid, transient local) - 실제로 계획에 쓰는 격자 (가공 + 금지 구역, RViz 확인용)
       astar/fail_reason (std_msgs/String) - 경로 계획 실패 이유 (금지 구역 때문인지 등). 브리지가 실패 메시지에 붙임

기존 맵 모드에서는 맵을 plan_map.py 로 가공해서 쓴다 (미탐색 줄무늬 메움, 의자/책상 다리 점 부풀리기).
AMCL 은 원본 맵을 그대로 쓰므로 위치 추정에는 영향 없음.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .astar import GridPlanner, PlanningError, pad_grid, path_yaws
from .plan_map import INSIDE_MARGIN, near_mask, prepare, with_added, zone_contains, zone_mask


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('robot_radius', 0.22)       # Waffle Pi
        self.declare_parameter('inflation_radius', 0.55)
        self.declare_parameter('cost_weight', 3.0)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown', False)   # SLAM 모드에서는 true (미탐색 칸도 지나감)
        self.declare_parameter('unknown_cost', 2.0)
        self.declare_parameter('goal_tolerance', 0.3)
        # 맵에 없는 장애물 기억: LiDAR 로 본 점 중 맵의 벽에서 떨어진 것만, 이 시간 동안 장애물로 취급
        self.declare_parameter('scan_obstacles', True)
        self.declare_parameter('scan_memory', 20.0)      # s
        self.declare_parameter('scan_max_range', 5.0)    # m, 이보다 먼 점은 무시
        self.declare_parameter('scan_wall_margin', 0.15)  # m, 맵 벽에서 이 거리 안의 점은 벽으로 보고 무시
        # 같은 5cm 칸에서 이 시간 넘게 계속 보여야 기억한다. 지나가는 사람/물체(한 칸에 1~2초)는 기억하지 않음
        # (움직이는 원기둥이 지나간 궤적 전체가 727칸의 가짜 벽이 되어 통로가 막혔었음)
        self.declare_parameter('scan_persist', 2.5)       # s
        # 기억한 장애물 때문에 경로가 없을 때: 장애물은 그대로 두고 반경만 이만큼으로 줄여 다시 (차체 반폭 0.153 + Frenet 느릴 때
        # 여유 0.04). 그래도 없으면 최근 scan_recent 초 안에 다시 본 장애물만으로 (지나간 사람, 회전 중 번진 스캔 같은 틀린 기억을 뺌).
        # 그래도 없으면 실패 (예전: 기억을 통째로 빼고 맵만으로 → 소화기 4 cm 옆 경로를 내 Frenet 이 멈추고 재계획마다 반복, 실물 2761/2771)
        self.declare_parameter('fallback_radius', 0.20)  # m
        self.declare_parameter('scan_recent', 3.0)        # s
        self.declare_parameter('bump_memory', 120.0)     # s, 부딪힌 자리를 장애물로 기억하는 시간
        self.declare_parameter('bump_radius', 0.15)      # m, 부딪힌 자리 주변을 이 반경만큼 막음
        # 경로 계획용 맵 가공 (기존 맵 모드만, plan_map.py). 0 이면 끔
        self.declare_parameter('stripe_cells', 2)            # 빈 칸 사이 폭 2*N 칸 미만 미탐색 줄무늬 → 빈 칸
        self.declare_parameter('cluster_inflate', 0.1)       # m, 작은 덩어리(다리 점)를 부풀림 → 2*(반경+이 값) 미만 틈이 막힘
        self.declare_parameter('cluster_max_size', 0.4)      # m, 가로/세로가 이보다 긴 장애물 덩어리(벽)는 부풀리지 않음
        self.declare_parameter('start_clear_radius', 0.35)   # m, 출발 위치 주변은 가공/금지 구역에서 뺌 (빠져나올 수 있게)

        self.planner = None
        self.map_msg = None
        self.base_grid = None     # 계획용 기본 격자 (2차원, 줄무늬 메움)
        self.added = None         # 기본 격자에 장애물로 더할 칸 (부풀린 다리 점 | 금지 구역)
        self.cluster_added = None
        self.zones = []
        self.frame_id = 'map'
        self.scan_points = deque()   # (시각, xs, ys) map 좌표 - 오래 머문(정지한) 장애물만
        self.cell_seen = {}          # 5cm 칸 -> [처음 본 시각, 마지막 본 시각, 본 횟수]
        self.bump_points = deque()   # (시각, xs, ys) 부딪힌 자리 주변 점
        self.last_scan_time = 0.0
        self.last_remember_time = None   # 마지막으로 기억에 넣은 시각 (로그: 기억이 비어 있을 때 이유)
        self.last_clear_time = None      # 웹 '상태 초기화' 로 지운 시각

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        topic = self.get_parameter('map_topic').value
        self.create_subscription(OccupancyGrid, topic, self.on_map, map_qos)
        self.path_pub = self.create_publisher(Path, 'astar/plan', 1)
        self.create_service(GetPlan, 'astar/get_plan', self.on_get_plan)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)   # 구독 콜백이 리스너를 붙잡고 있어 참조를 따로 안 들어도 됨
        self.create_subscription(PointStamped, 'astar/obstacle', self.on_bump, 10)
        self.create_service(Empty, 'astar/clear_obstacles', self.on_clear)
        self.create_subscription(String, 'astar/zones', self.on_zones, map_qos)
        self.plan_map_pub = self.create_publisher(OccupancyGrid, 'astar/plan_map', map_qos)
        self.fail_pub = self.create_publisher(String, 'astar/fail_reason', 10)
        self.remembered_pub = self.create_publisher(PoseArray, 'astar/remembered', 1)
        self.bumps_pub = self.create_publisher(PoseArray, 'astar/bumps', 1)
        self.bump_centers = deque()   # (시각, x, y) 웹 표시용
        self.create_timer(1.0, self.publish_memory)
        if self.get_parameter('scan_obstacles').value:
            self.create_subscription(LaserScan, 'scan_level', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'A* 플래너 시작, 맵 대기 중 ({topic})')

    def make_planner(self, data, width=None, height=None, origin_x=None, origin_y=None, robot_radius=None):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        info = self.map_msg.info
        return GridPlanner(
            data, width or info.width, height or info.height, info.resolution,
            info.origin.position.x if origin_x is None else origin_x,
            info.origin.position.y if origin_y is None else origin_y,
            robot_radius=p('robot_radius') if robot_radius is None else robot_radius, inflation_radius=p('inflation_radius'),
            cost_weight=p('cost_weight'), occupied_threshold=p('occupied_threshold'),
            allow_unknown=p('allow_unknown'), unknown_cost=p('unknown_cost'))

    def on_map(self, msg):
        info = msg.info
        self.frame_id = msg.header.frame_id or 'map'
        self.map_msg = msg
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        stats = None
        if p('allow_unknown'):
            # SLAM 모드: 맵이 계속 자라는 중이라 가공하지 않는다 (미탐색 칸도 지나가므로 줄무늬도 문제없음)
            self.base_grid, self.cluster_added = grid, None
        else:
            self.base_grid, self.cluster_added, stats = prepare(
                grid, info.resolution, p('stripe_cells'), p('cluster_inflate'), p('cluster_max_size'),
                p('occupied_threshold'))
        self.rebuild()
        self.get_logger().info(f'맵 수신: {info.width}x{info.height}, {info.resolution:.3f} m/px'
                               + (', 계획용 가공: ' + ', '.join(f'{k} {v}칸' for k, v in stats.items()) if stats else ''))

    def on_zones(self, msg):
        try:
            zones = json.loads(msg.data)
        except ValueError:
            self.get_logger().error(f'금지 구역 메시지 오류: {msg.data[:100]}')
            return
        self.zones = zones if isinstance(zones, list) else []
        self.get_logger().info(f'금지 구역 {len(self.zones)}개 받음')
        if self.map_msg is not None:
            self.rebuild()

    def rebuild(self):
        """기본 격자 + 부풀린 다리 점 + 금지 구역으로 계획용 격자를 다시 만든다 (맵/금지 구역이 바뀔 때)."""
        info = self.map_msg.info
        ox, oy = info.origin.position.x, info.origin.position.y
        zones = zone_mask(self.base_grid.shape, info.resolution, ox, oy, self.zones)
        self.added = zones if self.cluster_added is None else (self.cluster_added | zones)
        grid = with_added(self.base_grid, self.added, info.resolution, ox, oy)
        self.planner = self.make_planner(grid.ravel())
        out = OccupancyGrid()
        out.header = self.map_msg.header
        out.info = info
        out.data = grid.astype(np.int8).ravel().tolist()
        self.plan_map_pub.publish(out)

    def on_scan(self, msg):
        """0.5초마다 LiDAR 점을 map 좌표로 바꿔, 맵의 벽이 아닌 점만 기억한다."""
        now = time.monotonic()
        # SLAM 모드(allow_unknown)는 SLAM 맵 자체가 몇 초마다 장애물을 반영하므로 기억하지 않는다.
        # (맵이 계속 자라는 중이라 벽 밖 점이 전부 '맵에 없는 장애물'로 쌓여 통로가 막혔음: 998칸)
        if self.get_parameter('allow_unknown').value:
            return
        if self.planner is None or now - self.last_scan_time < 0.5:
            return
        self.last_scan_time = now
        try:
            t = self.tf_buffer.lookup_transform(self.frame_id, msg.header.frame_id, Time())
        except TransformException:
            return
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < self.get_parameter('scan_max_range').value)
        if not ok.any():
            return
        tr, yaw = t.transform.translation, yaw_from_quaternion(t.transform.rotation)
        lx, ly = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        xs = tr.x + lx * math.cos(yaw) - ly * math.sin(yaw)
        ys = tr.y + lx * math.sin(yaw) + ly * math.cos(yaw)
        pl = self.planner
        rows = np.floor((ys - pl.origin_y) / pl.resolution).astype(int)
        cols = np.floor((xs - pl.origin_x) / pl.resolution).astype(int)
        inside = (rows >= 0) & (rows < pl.height) & (cols >= 0) & (cols < pl.width)
        keep = np.zeros(len(xs), dtype=bool)
        keep[inside] = pl.dist[rows[inside], cols[inside]] > self.get_parameter('scan_wall_margin').value
        self.remember_persistent(now, xs[keep], ys[keep])

    def remember_persistent(self, now, xs, ys):
        """이번 스캔의 맵에 없는 점 중, 같은 칸에서 scan_persist 초 넘게 계속 보인 것만 기억에 넣는다."""
        persist = self.get_parameter('scan_persist').value
        cells = set(map(tuple, np.floor(np.column_stack([xs, ys]) / 0.05).astype(int).tolist()))
        for c in cells:
            seen = self.cell_seen.get(c)
            if seen is None:
                self.cell_seen[c] = [now, now, 1]
            else:
                seen[1], seen[2] = now, seen[2] + 1
        # 1초 넘게 안 보인 칸은 처음부터 다시 센다 (지나간 물체)
        self.cell_seen = {c: s for c, s in self.cell_seen.items() if now - s[1] <= 1.0}
        steady = [c for c in cells if now - self.cell_seen[c][0] >= persist
                  and self.cell_seen[c][2] >= 0.6 * persist / 0.5]   # 스캔은 0.5초마다
        if steady:
            pts = (np.array(steady, dtype=float) + 0.5) * 0.05
            self.scan_points.append((now, pts[:, 0], pts[:, 1]))
            self.last_remember_time = now

    def on_clear(self, request, response):
        self.scan_points.clear()
        self.cell_seen.clear()
        self.bump_points.clear()
        self.bump_centers.clear()
        self.last_clear_time = time.monotonic()
        self.get_logger().info('기억한 장애물(LiDAR/부딪힌 자리)을 모두 지웠습니다')
        return response

    def publish_memory(self):
        """기억 중인 장애물을 발행 (브리지가 웹으로 보냄). LiDAR 점은 5cm 칸으로 묶는다."""
        now = time.monotonic()
        while self.scan_points and now - self.scan_points[0][0] > self.get_parameter('scan_memory').value:
            self.scan_points.popleft()
        while self.bump_centers and now - self.bump_centers[0][0] > self.get_parameter('bump_memory').value:
            self.bump_centers.popleft()
        cells = np.empty((0, 2))
        if self.scan_points:
            xs = np.concatenate([p[1] for p in self.scan_points])
            ys = np.concatenate([p[2] for p in self.scan_points])
            if len(xs):
                cells = np.unique(np.floor(np.column_stack([xs, ys]) / 0.05), axis=0) * 0.05 + 0.025
        for pub, pts in ((self.remembered_pub, cells), (self.bumps_pub, [(x, y) for _, x, y in self.bump_centers])):
            msg = PoseArray()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            for x, y in pts:
                p = Pose()
                p.position.x, p.position.y = float(x), float(y)
                p.orientation.w = 1.0
                msg.poses.append(p)
            pub.publish(msg)

    def on_bump(self, msg):
        r = self.get_parameter('bump_radius').value
        a = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        xs = np.concatenate([[msg.point.x], msg.point.x + r * np.cos(a), msg.point.x + r / 2 * np.cos(a)])
        ys = np.concatenate([[msg.point.y], msg.point.y + r * np.sin(a), msg.point.y + r / 2 * np.sin(a)])
        self.bump_points.append((time.monotonic(), xs, ys))
        self.bump_centers.append((time.monotonic(), msg.point.x, msg.point.y))
        self.get_logger().warn(f'부딪힌 자리 ({msg.point.x:.2f}, {msg.point.y:.2f}) 를 장애물로 기억')

    def planners_for(self, start, goal):
        """(맵 + 부딪힌 자리 플래너, with_scan(반경, since) → (여기에 기억 장애물까지 더한 플래너 또는 None, 더한 칸 수)).

        부딪힌 자리(LiDAR 에 안 보이는 낮은 물체)는 LiDAR 장애물 때문에 경로가 없어 다시 계획할 때도 빼지 않는다.
        SLAM 모드(allow_unknown)에서 출발/목표가 아직 맵 밖이면 맵을 미탐색 칸으로 넓힌다.
        """
        now = time.monotonic()
        memory = self.get_parameter('scan_memory').value
        while self.scan_points and now - self.scan_points[0][0] > memory:
            self.scan_points.popleft()
        while self.bump_points and now - self.bump_points[0][0] > self.get_parameter('bump_memory').value:
            self.bump_points.popleft()
        info = self.map_msg.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        # 출발 위치가 금지 구역 안이면 그 구역은 이번 계획에서 빼고, 구역 가장자리/부풀린 다리 점 근처면
        # 출발 주변(start_clear_radius)만 빼고 다시 만든다 (그 안에 서 있는 로봇이 빠져나올 수 있게)
        clear = self.get_parameter('start_clear_radius').value
        added = self.added
        start_zones = [z for z in self.zones if zone_contains(z, start, INSIDE_MARGIN)]
        if start_zones:
            added = zone_mask(self.base_grid.shape, res, ox, oy, [z for z in self.zones if z not in start_zones])
            if self.cluster_added is not None:
                added = added | self.cluster_added
            self.get_logger().info(f'출발 위치가 금지 구역 {len(start_zones)}개 안에 있어 이번 계획에서는 뺍니다')
        start_inside = bool(start_zones) or bool(added.any() and (added & near_mask(
            self.base_grid.shape, res, ox, oy, start, clear)).any())
        data = with_added(self.base_grid, added, res, ox, oy, start if start_inside else None, clear).ravel()
        w, h = info.width, info.height
        if self.get_parameter('allow_unknown').value:
            data, w, h, ox, oy = pad_grid(data, w, h, info.resolution, ox, oy, [start, goal])
        grid = np.asarray(data, dtype=np.int16).reshape(h, w).copy()

        def mark(points):
            if not points:
                return 0
            xs = np.concatenate([p[1] for p in points])
            ys = np.concatenate([p[2] for p in points])
            rows = np.floor((ys - oy) / info.resolution).astype(int)
            cols = np.floor((xs - ox) / info.resolution).astype(int)
            inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
            before = grid[rows[inside], cols[inside]].copy()
            grid[rows[inside], cols[inside]] = 100
            return int(np.count_nonzero(before != 100))

        plain = (self.make_planner(grid.ravel(), w, h, ox, oy)
                 if start_inside or (w, h) != (info.width, info.height) else self.planner)
        base = self.make_planner(grid.ravel(), w, h, ox, oy) if mark(list(self.bump_points)) else plain
        self.plain_planner = plain   # 부딪힌 자리를 빼고 맵만 (실패 메시지 구분용)
        no_scan = grid.copy()        # 맵 + 부딪힌 자리 (다시 계획할 때 기억 장애물을 골라 넣는 바탕)

        def with_scan(radius, since=None):
            """바탕 격자 + 기억 장애물 (since 이후에 넣은 것만) 플래너, 더한 칸 수. 더한 칸이 없으면 (None, 0)."""
            grid[:] = no_scan
            n = mark([q for q in self.scan_points if since is None or q[0] >= since])
            if not n:
                return None, 0
            self.last_grid = (grid.copy(), ox, oy, info.resolution)
            return self.make_planner(grid.ravel(), w, h, ox, oy, robot_radius=radius), n

        return base, with_scan

    def log_memory(self):
        """계획마다 기억 장애물 상태를 로그로 (비어 있으면 왜인지: 지움 / 만료 / 아직 없음)."""
        now = time.monotonic()
        if self.scan_points:
            cells = len(set().union(*(set(zip(np.floor(q[1] / 0.05).astype(int).tolist(),
                                              np.floor(q[2] / 0.05).astype(int).tolist())) for q in self.scan_points)))
            self.get_logger().info(f'기억 장애물 {cells}칸 (가장 오래된 것 {now - self.scan_points[0][0]:.0f}초 전, '
                                   f'가장 최근 {now - self.scan_points[-1][0]:.1f}초 전)')
        elif self.last_clear_time is not None and (self.last_remember_time is None
                                                   or self.last_clear_time >= self.last_remember_time):
            self.get_logger().info(f'기억 장애물 없음: {now - self.last_clear_time:.0f}초 전 상태 초기화로 지움')
        elif self.last_remember_time is not None:
            self.get_logger().info(f'기억 장애물 없음: 마지막으로 넣은 지 {now - self.last_remember_time:.0f}초 '
                                   f'(기억 {self.get_parameter("scan_memory").value:.0f}초 지나 만료)')
        else:
            self.get_logger().info('기억 장애물 없음: 아직 기억한 것 없음')

    def fail_reason(self, start, goal, tolerance, error):
        """경로가 없을 때 금지 구역 때문인지. 반환: 설명 문자열 또는 None (금지 구역과 상관없음)."""
        if not self.zones:
            return None
        if any(zone_contains(z, goal, 0.0) for z in self.zones):
            return '목표가 금지 구역 안에 있습니다'
        info = self.map_msg.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        clear = self.get_parameter('start_clear_radius').value
        added = self.cluster_added if self.cluster_added is not None else np.zeros(self.base_grid.shape, dtype=bool)
        data = with_added(self.base_grid, added, res, ox, oy, start, clear).ravel()
        w, h = info.width, info.height
        if self.get_parameter('allow_unknown').value:
            data, w, h, ox, oy = pad_grid(data, w, h, res, ox, oy, [start, goal])
        try:
            self.make_planner(data, w, h, ox, oy).plan(start, goal, goal_tolerance=tolerance)
        except PlanningError:
            return None
        return '금지 구역에 막혀 갈 수 있는 경로가 없습니다'

    def dump_grid(self, start, goal):
        """진단용: LiDAR 장애물 때문에 경로가 없었을 때의 격자를 logs/astar_fail.npz 로 남긴다."""
        home = os.environ.get('WAREHOUSE_HOME')
        if not home or not hasattr(self, 'last_grid'):
            return
        grid, ox, oy, res = self.last_grid
        try:
            np.savez_compressed(os.path.join(home, 'logs', 'astar_fail.npz'), grid=grid, origin=(ox, oy),
                                resolution=res, start=start, goal=goal)
        except OSError:
            pass

    def on_get_plan(self, request, response):
        response.plan.header.frame_id = self.frame_id
        response.plan.header.stamp = self.get_clock().now().to_msg()
        if self.planner is None:
            self.get_logger().warn('아직 맵을 받지 못했습니다.')
            return response

        s, g = request.start.pose, request.goal.pose
        tolerance = request.tolerance if request.tolerance > 0 else self.get_parameter('goal_tolerance').value
        start, goal = (s.position.x, s.position.y), (g.position.x, g.position.y)
        base, with_scan = self.planners_for(start, goal)
        self.log_memory()
        points = None
        radius, small = self.get_parameter('robot_radius').value, self.get_parameter('fallback_radius').value
        recent = time.monotonic() - self.get_parameter('scan_recent').value
        tries = [(radius, None, '기억 장애물'), (small, None, f'반경 {small:.2f} m 로 줄여 기억 장애물'),
                 (small, recent, f'반경 {small:.2f} m, 최근 {self.get_parameter("scan_recent").value:.0f}초에 다시 본 장애물')]
        tried = False
        for r, since, what in tries:
            planner, extra = with_scan(r, since)
            if planner is None:
                # 넣을 장애물이 없음 (처음부터 기억이 비었거나, 막던 것을 최근에 다시 못 봄) → 아래에서 맵(+부딪힌 자리)만으로
                if tried:
                    self.get_logger().warn(f'막던 기억 장애물을 최근 {self.get_parameter("scan_recent").value:.0f}초 안에 '
                                           '다시 보지 못함 (지나간 물체로 보고) → 맵만으로 계획')
                break
            tried = True
            try:
                points = planner.plan(start, goal, goal_tolerance=tolerance)
                self.get_logger().info(f'{what} {extra}칸을 피해서 계획')
                break
            except PlanningError as e:
                self.get_logger().warn(f'{what} {extra}칸 포함 경로 없음 ({e})')
                self.dump_grid(start, goal)
        if points is None and tried and planner is not None:
            # 맵만으로는 길이 있으면(= 기억 장애물이 막음) 그 경로를 쓰지 않는다: 막고 있는 물체를 지나가서 Frenet 이 못 감.
            # 맵만으로도 없으면 아래에서 원래 이유(금지 구역 등)로 실패
            try:
                base.plan(start, goal, goal_tolerance=tolerance)
                reason = '맵에 없는 장애물(LiDAR 로 본 물체)에 막혀 지나갈 틈이 없습니다'
                self.get_logger().warn(f'경로 계획 실패: {reason}')
                self.fail_pub.publish(String(data=reason))
                return response
            except PlanningError:
                pass
        if points is None:
            try:
                points = base.plan(start, goal, goal_tolerance=tolerance)
            except PlanningError as e:
                # 부딪혔던 자리를 지나는 경로는 쓰지 않는다 (맵만으로 다시 계획하면 같은 물체에 계속 부딪혔음).
                # 기억은 bump_memory 초 뒤 또는 웹 '상태 초기화'(astar/clear_obstacles)로 지워진다
                reason = self.fail_reason(start, goal, tolerance, e)
                if reason is None and base is not self.plain_planner:
                    reason = '부딪혔던 자리 때문일 수 있음'
                self.get_logger().warn(f'경로 계획 실패: {e}{f" ({reason})" if reason else ""}')
                self.fail_pub.publish(String(data=reason or str(e)))
                return response

        for (x, y), yaw in zip(points, path_yaws(points, yaw_from_quaternion(g.orientation))):
            pose = PoseStamped()
            pose.header = response.plan.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = quaternion_from_yaw(yaw)
            response.plan.poses.append(pose)

        self.path_pub.publish(response.plan)
        self.get_logger().info(f'경로 계획 완료: {len(points)}개 점')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
