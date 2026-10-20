"""직접 구현한 A*를 ROS 2 서비스로 제공하는 노드.

구독:  map (nav_msgs/OccupancyGrid, transient local)
       scan (sensor_msgs/LaserScan) - 맵에 없는 장애물(새로 놓인 상자, 사람 등)을 잠시 기억해 경로 계획에 반영
서비스: astar/get_plan (nav_msgs/srv/GetPlan)
발행:  astar/plan (nav_msgs/Path)  - RViz 확인용
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
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

        self.planner = None
        self.map_msg = None
        self.frame_id = 'map'
        self.scan_points = deque()   # (시각, xs, ys) map 좌표
        self.last_scan_time = 0.0

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        topic = self.get_parameter('map_topic').value
        self.create_subscription(OccupancyGrid, topic, self.on_map, map_qos)
        self.path_pub = self.create_publisher(Path, 'astar/plan', 1)
        self.create_service(GetPlan, 'astar/get_plan', self.on_get_plan)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.get_parameter('scan_obstacles').value:
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
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

    def planners_for(self, start, goal):
        """(맵만 쓰는 플래너, 기억한 장애물까지 더한 플래너 또는 None, 더한 칸 수).

        SLAM 모드(allow_unknown)에서 출발/목표가 아직 맵 밖이면 맵을 미탐색 칸으로 넓힌다.
        """
        now = time.monotonic()
        memory = self.get_parameter('scan_memory').value
        while self.scan_points and now - self.scan_points[0][0] > memory:
            self.scan_points.popleft()
        info = self.map_msg.info
        base = self.planner
        data, w, h, ox, oy = self.map_msg.data, info.width, info.height, base.origin_x, base.origin_y
        if self.get_parameter('allow_unknown').value:
            data, w, h, ox, oy = pad_grid(data, w, h, info.resolution, ox, oy, [start, goal])
            if w != info.width or h != info.height:
                base = self.make_planner(data, w, h, ox, oy)
        if not self.scan_points:
            return base, None, 0
        xs = np.concatenate([p[1] for p in self.scan_points])
        ys = np.concatenate([p[2] for p in self.scan_points])
        rows = np.floor((ys - oy) / info.resolution).astype(int)
        cols = np.floor((xs - ox) / info.resolution).astype(int)
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        grid = np.asarray(data, dtype=np.int16).reshape(h, w).copy()
        before = grid.copy()
        grid[rows[inside], cols[inside]] = 100
        cells = int(np.count_nonzero(grid != before))
        if not cells:
            return base, None, 0
        return base, self.make_planner(grid.ravel(), w, h, ox, oy), cells

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
        if points is None:
            try:
                points = base.plan(start, goal, goal_tolerance=tolerance)
            except PlanningError as e:
                self.get_logger().warn(f'경로 계획 실패: {e}')
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
