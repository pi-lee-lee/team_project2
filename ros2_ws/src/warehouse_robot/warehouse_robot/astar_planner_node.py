"""직접 구현한 A*를 ROS 2 서비스로 제공하는 노드.

구독:  map (nav_msgs/OccupancyGrid, transient local)
서비스: astar/get_plan (nav_msgs/srv/GetPlan)
발행:  astar/plan (nav_msgs/Path)  - RViz 확인용
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .astar import GridPlanner, PlanningError, path_yaws


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

        self.planner = None
        self.frame_id = 'map'

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        topic = self.get_parameter('map_topic').value
        self.create_subscription(OccupancyGrid, topic, self.on_map, map_qos)
        self.path_pub = self.create_publisher(Path, 'astar/plan', 1)
        self.create_service(GetPlan, 'astar/get_plan', self.on_get_plan)
        self.get_logger().info(f'A* 플래너 시작, 맵 대기 중 ({topic})')

    def on_map(self, msg):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        info = msg.info
        self.frame_id = msg.header.frame_id or 'map'
        self.planner = GridPlanner(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            robot_radius=p('robot_radius'), inflation_radius=p('inflation_radius'),
            cost_weight=p('cost_weight'), occupied_threshold=p('occupied_threshold'),
            allow_unknown=p('allow_unknown'), unknown_cost=p('unknown_cost'))
        self.get_logger().info(f'맵 수신: {info.width}x{info.height}, {info.resolution:.3f} m/px')

    def on_get_plan(self, request, response):
        response.plan.header.frame_id = self.frame_id
        response.plan.header.stamp = self.get_clock().now().to_msg()
        if self.planner is None:
            self.get_logger().warn('아직 맵을 받지 못했습니다.')
            return response

        s, g = request.start.pose, request.goal.pose
        tolerance = request.tolerance if request.tolerance > 0 else self.get_parameter('goal_tolerance').value
        try:
            points = self.planner.plan((s.position.x, s.position.y), (g.position.x, g.position.y),
                                       goal_tolerance=tolerance)
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
