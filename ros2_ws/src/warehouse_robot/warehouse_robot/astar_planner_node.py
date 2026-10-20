"""직접 구현한 A*를 ROS 2 서비스로 제공하는 노드.

구독:  map (nav_msgs/OccupancyGrid, transient local)
       scan_level (sensor_msgs/LaserScan, 브리지가 기울지 않았을 때만 발행) - 맵에 없는 장애물(새로 놓인 상자, 사람 등)을 잠시 기억해 경로 계획에 반영
       astar/obstacle (geometry_msgs/PointStamped) - 브리지가 부딪힘을 감지한 자리 (LiDAR 에 안 보이는 낮은 물체)
발행:  astar/remembered (geometry_msgs/PoseArray) - 지금 기억 중인 LiDAR 장애물 칸 (1초마다, 웹 표시용)
       astar/bumps (geometry_msgs/PoseArray) - 기억 중인 부딪힌 자리 중심
서비스: astar/get_plan (nav_msgs/srv/GetPlan)
발행:  astar/plan (nav_msgs/Path)  - RViz 확인용
"""
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
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .astar import GridPlanner, PlanningError, pad_grid, path_yaws


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
        self.declare_parameter('scan_max_range', 2.0)    # m, 이보다 먼 점은 무시
        self.declare_parameter('scan_wall_margin', 0.15)  # m, 맵 벽에서 이 거리 안의 점은 벽으로 보고 무시
        self.declare_parameter('bump_memory', 120.0)     # s, 부딪힌 자리를 장애물로 기억하는 시간
        self.declare_parameter('bump_radius', 0.15)      # m, 부딪힌 자리 주변을 이 반경만큼 막음

        self.planner = None
        self.map_msg = None
        self.frame_id = 'map'
        self.scan_points = deque()   # (시각, xs, ys) map 좌표
        self.bump_points = deque()   # (시각, xs, ys) 부딪힌 자리 주변 점
        self.last_scan_time = 0.0

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        topic = self.get_parameter('map_topic').value
        self.create_subscription(OccupancyGrid, topic, self.on_map, map_qos)
        self.path_pub = self.create_publisher(Path, 'astar/plan', 1)
        self.create_service(GetPlan, 'astar/get_plan', self.on_get_plan)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointStamped, 'astar/obstacle', self.on_bump, 10)
        self.create_service(Empty, 'astar/clear_obstacles', self.on_clear)
        self.remembered_pub = self.create_publisher(PoseArray, 'astar/remembered', 1)
        self.bumps_pub = self.create_publisher(PoseArray, 'astar/bumps', 1)
        self.bump_centers = deque()   # (시각, x, y) 웹 표시용
        self.create_timer(1.0, self.publish_memory)
        if self.get_parameter('scan_obstacles').value:
            self.create_subscription(LaserScan, 'scan_level', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'A* 플래너 시작, 맵 대기 중 ({topic})')

    def make_planner(self, data, width=None, height=None, origin_x=None, origin_y=None):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        info = self.map_msg.info
        return GridPlanner(
            data, width or info.width, height or info.height, info.resolution,
            info.origin.position.x if origin_x is None else origin_x,
            info.origin.position.y if origin_y is None else origin_y,
            robot_radius=p('robot_radius'), inflation_radius=p('inflation_radius'),
            cost_weight=p('cost_weight'), occupied_threshold=p('occupied_threshold'),
            allow_unknown=p('allow_unknown'), unknown_cost=p('unknown_cost'))

    def on_map(self, msg):
        info = msg.info
        self.frame_id = msg.header.frame_id or 'map'
        self.map_msg = msg
        self.planner = self.make_planner(msg.data)
        self.get_logger().info(f'맵 수신: {info.width}x{info.height}, {info.resolution:.3f} m/px')

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
            self.scan_points.append((now, np.empty(0), np.empty(0)))
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
        self.scan_points.append((now, xs[keep], ys[keep]))

    def on_clear(self, request, response):
        self.scan_points.clear()
        self.bump_points.clear()
        self.bump_centers.clear()
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
        """(맵 + 부딪힌 자리 플래너, 여기에 LiDAR 장애물까지 더한 플래너 또는 None, LiDAR 로 더한 칸 수).

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
        data, w, h, ox, oy = self.map_msg.data, info.width, info.height, self.planner.origin_x, self.planner.origin_y
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

        plain = (self.make_planner(grid.ravel(), w, h, ox, oy) if (w, h) != (info.width, info.height)
                 else self.planner)
        base = self.make_planner(grid.ravel(), w, h, ox, oy) if mark(list(self.bump_points)) else plain
        self.plain_planner = plain   # 부딪힌 자리를 빼고 맵만 (실패 메시지 구분용)
        cells = mark(list(self.scan_points))
        if not cells:
            return base, None, 0
        self.last_grid = (grid.copy(), ox, oy, info.resolution)
        return base, self.make_planner(grid.ravel(), w, h, ox, oy), cells

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
        base, with_scan, extra = self.planners_for(start, goal)
        points = None
        if with_scan is not None:
            try:
                points = with_scan.plan(start, goal, goal_tolerance=tolerance)
                self.get_logger().info(f'맵에 없는 장애물 {extra}칸을 피해서 계획')
            except PlanningError as e:
                # 기억한 장애물 때문에 막히면(사람이 지나간 자리 등) 맵만으로 다시 계획
                self.get_logger().warn(f'맵에 없는 장애물 {extra}칸 포함 경로 없음 ({e}), 맵만으로 다시 계획')
                self.dump_grid(start, goal)
        if points is None:
            try:
                points = base.plan(start, goal, goal_tolerance=tolerance)
            except PlanningError as e:
                # 부딪혔던 자리를 지나는 경로는 쓰지 않는다 (맵만으로 다시 계획하면 같은 물체에 계속 부딪혔음).
                # 기억은 bump_memory 초 뒤 또는 웹 '상태 초기화'(astar/clear_obstacles)로 지워진다
                self.get_logger().warn(f'경로 계획 실패: {e}'
                                       f'{" (부딪혔던 자리 때문일 수 있음)" if base is not self.plain_planner else ""}')
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
