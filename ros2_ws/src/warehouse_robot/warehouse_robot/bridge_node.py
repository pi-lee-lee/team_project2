"""Django 서버(WebSocket)와 ROS 2 / Nav2를 잇는 브리지 노드.

서버 → 로봇: goto, cancel, stop, initial_pose
로봇 → 서버: hello, map, pose, battery, path, nav_status

이동 방식
  planner=astar : astar_planner 노드로 경로 계획 후
      controller=builtin : 이 노드의 pure pursuit 추종기로 cmd_vel 직접 발행 (기본)
      controller=nav2    : Nav2 controller_server(FollowPath)로 추종
  planner=nav2  : Nav2 NavigateToPose (Nav2 기본 플래너 사용, 비교/예비용)

builtin 추종기 안전장치: 위치 TF가 끊기면 정지, 앞쪽 장애물이 있으면 정지 후 대기,
경로 이탈/진행 없음/장애물로 오래 막힘 → 정지 후 재계획 또는 실패.
"""
import base64
import json
import math
import os
import queue
import subprocess
import threading
import time
import zlib

import numpy as np
import rclpy
import websocket
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .astar_planner_node import quaternion_from_yaw, yaw_from_quaternion
from .follower import PurePursuit


# LiPo 셀 전압 → 남은 용량(%) (일반적인 방전 곡선)
LIPO_CURVE = [(3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25), (3.77, 30),
              (3.79, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55), (3.87, 60), (3.91, 65),
              (3.95, 70), (3.98, 75), (4.02, 80), (4.08, 85), (4.11, 90), (4.15, 95), (4.20, 100)]


def lipo_capacity(cell_voltage):
    """셀 전압을 LIPO_CURVE로 선형 보간한 남은 용량(0~100%)."""
    if cell_voltage <= LIPO_CURVE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(LIPO_CURVE, LIPO_CURVE[1:]):
        if cell_voltage <= v1:
            return p0 + (p1 - p0) * (cell_voltage - v0) / (v1 - v0)
    return 100.0


def battery_percent(voltage, cells=3, empty_voltage=11.0):
    """팩 전압 → 표시용 %. empty_voltage 를 0% 로 본다 (TurtleBot3 는 11V 아래에서 충전 경고).
    그 위는 LiPo 방전 곡선 모양을 유지한 채 0~100% 로 다시 맞춘다."""
    lo = lipo_capacity(empty_voltage / cells)
    cap = lipo_capacity(voltage / cells)
    return max(0.0, min(100.0, (cap - lo) / (100.0 - lo) * 100.0))


class ServerConnection:
    """별도 스레드에서 WebSocket 연결을 유지한다. 받은 메시지는 inbox 큐에 넣는다."""

    def __init__(self, url, logger):
        self.url = url
        self.logger = logger
        self.inbox = queue.Queue()
        self.ws = None
        self.connected = False
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            self.ws = websocket.WebSocketApp(
                self.url,
                on_open=self._on_open,
                on_message=lambda ws, data: self.inbox.put(json.loads(data)),
                on_close=self._on_close,
                on_error=lambda ws, err: self.logger.warn(f'서버 연결 오류: {err}'))
            self.ws.run_forever(ping_interval=10, ping_timeout=5)
            self.connected = False
            if self.running:
                time.sleep(2.0)

    def _on_open(self, ws):
        self.connected = True
        self.logger.info(f'서버 연결됨: {self.url}')
        self.inbox.put({'type': '_connected'})

    def _on_close(self, ws, code, reason):
        if self.connected:
            self.logger.warn('서버 연결 끊김, 재연결 시도')
        self.connected = False

    def send(self, msg):
        if not self.connected:
            return
        try:
            with self.lock:
                self.ws.send(json.dumps(msg))
        except Exception as e:  # 연결이 끊기는 중이면 다음 재연결 때 다시 보냄
            self.logger.debug(f'전송 실패: {e}')

    def close(self):
        self.running = False
        if self.ws:
            self.ws.close()


class BridgeNode(Node):
    def __init__(self):
        super().__init__('warehouse_bridge')
        self.declare_parameter('server_url', 'ws://localhost:8000/ws/robot/tb3/')
        self.declare_parameter('map_name', 'default')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('planner', 'astar')  # astar | nav2
        self.declare_parameter('pose_rate', 5.0)
        self.declare_parameter('max_retries', 2)   # 이동 실패 시 재계획 횟수
        self.declare_parameter('arrive_tolerance', 0.3)  # 도착으로 인정할 목표와의 거리 (m)
        # FollowPath에 넘길 경로 좌표계. odom이면 controller_server가 map->odom TF 없이 주행한다
        self.declare_parameter('follow_frame', 'odom')
        # 경로 추종기 (controller=builtin)
        self.declare_parameter('controller', 'builtin')  # builtin | nav2
        self.declare_parameter('max_speed', 0.15)          # m/s
        self.declare_parameter('max_turn_speed', 1.0)      # rad/s
        self.declare_parameter('stop_distance', 0.35)      # 앞쪽 이 거리 안에 물체가 있으면 정지 (m)
        self.declare_parameter('blocked_timeout', 10.0)    # 이 시간 넘게 막히면 재계획/실패 (s)
        self.declare_parameter('progress_timeout', 15.0)   # 이 시간 동안 5cm도 못 가면 실패 (s)
        self.declare_parameter('tf_timeout', 0.5)          # 위치 TF가 이보다 오래되면 즉시 정지 (s)
        self.declare_parameter('rotate_timeout', 15.0)     # 제자리 회전이 이보다 길면 정지 후 재계획/실패 (s)
        # 부딪힘 감지 (LiDAR 보다 낮은 장애물 등): 기울기가 이 각도를 넘거나, 앞으로 가는데 이 시간 동안
        # 3cm 도 못 가면 멈추고 뒤로 물러난 뒤, 부딪힌 자리를 플래너에 장애물로 알리고 재계획
        self.declare_parameter('bump_tilt_deg', 6.0)
        self.declare_parameter('stall_timeout', 2.5)       # s
        self.declare_parameter('backoff_distance', 0.15)   # m
        self.declare_parameter('battery_cells', 3)         # LiPo 셀 수 (TurtleBot3: 3셀 11.1V)
        self.declare_parameter('battery_empty_v', 11.0)    # 0% 로 볼 전압 (TurtleBot3 충전 경고 전압)
        self.declare_parameter('mode', 'map')              # map: 기존 맵+AMCL / slam: 맵 만들기

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.global_frame = p('global_frame')
        self.base_frame = p('base_frame')
        self.planner_mode = p('planner')
        self.max_retries = p('max_retries')
        self.arrive_tolerance = p('arrive_tolerance')
        self.follow_frame = p('follow_frame')
        self.controller = p('controller')
        self.max_speed = p('max_speed')
        self.max_turn_speed = p('max_turn_speed')
        self.stop_distance = p('stop_distance')
        self.blocked_timeout = p('blocked_timeout')
        self.progress_timeout = p('progress_timeout')
        self.tf_timeout = p('tf_timeout')
        self.battery_cells = p('battery_cells')
        self.battery_empty_v = p('battery_empty_v')
        self.rotate_timeout = p('rotate_timeout')
        self.bump_tilt = math.radians(p('bump_tilt_deg'))
        self.stall_timeout = p('stall_timeout')
        self.backoff_distance = p('backoff_distance')
        self.mode = p('mode')
        # 이 브리지 실행 번호. 서버는 SLAM 실행이 바뀐 것을 알고 이전 SLAM 맵/위치(다른 좌표계)를 지운다
        self.session = f'{int(time.time())}-{os.getpid()}'
        self.map_dirty = False      # SLAM 모드: 서버로 보낼 새 맵이 있음
        self.last_map_sent = 0.0
        self.script_running = None  # 실행 중인 scripts/*.sh 이름

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_msg = None
        self.create_subscription(OccupancyGrid, p('map_topic'), self.on_map, map_qos)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self.on_nav2_plan, 1)  # nav2 모드 경로 표시용
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.tilt = 0.0           # IMU 로 계산한 기울기 (rad)
        # 평소 기울기 (IMU 기본 오프셋, 기운 바닥): 천천히 따라가는 평균. tilt_rel = 평소보다 더 기운 정도
        self.tilt_base = None
        self.tilt_rel = 0.0
        self.last_imu_time = None
        self.scan_gated_since = None  # 기울어서 AMCL/SLAM 에 스캔을 안 넘기기 시작한 시각
        self.lift_dir = 0.0           # 들린 쪽 방향 (로봇 기준 rad, 0 = 앞). 장애물이 있는 쪽
        self.tipped_since = None  # 넘어진 상태가 시작된 시각
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')  # AMCL
        self.nomotion_left = 0
        self.create_timer(0.3, self.nomotion_step)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.bump_pub = self.create_publisher(PointStamped, 'astar/obstacle', 10)  # 부딪힌 자리 → A* 플래너
        self.clear_obstacles_client = self.create_client(Empty, 'astar/clear_obstacles')
        self.level_scan_pub = self.create_publisher(LaserScan, 'scan_level', qos_profile_sensor_data)

        self.plan_client = self.create_client(GetPlan, 'astar/get_plan')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 현재 목표. generation은 취소된 목표의 늦은 콜백을 무시하기 위한 번호
        self.goal = None
        self.goal_handle = None
        self.generation = 0
        self.last_battery_sent = 0.0
        self.battery_history = []  # (시각, 전압)
        self.stop_count = 0
        self.follower = None  # 내장 추종기 상태 (controller=builtin)
        self.last_goal_id = None

        self.server = ServerConnection(p('server_url'), self.get_logger())
        self.create_timer(0.05, self.process_inbox)
        self.create_timer(1.0 / p('pose_rate'), self.send_pose)
        self.create_timer(0.1, self.publish_stop)
        self.create_timer(0.05, self.control_step)
        self.create_timer(1.0, self.send_health)
        self.create_timer(1.0, self.flush_map)
        self.get_logger().info(f'브리지 시작 (planner={self.planner_mode}, controller={self.controller})')

    # ---------------------------------------------------------- 서버 메시지 처리
    def process_inbox(self):
        while True:
            try:
                msg = self.server.inbox.get_nowait()
            except queue.Empty:
                return
            kind = msg.get('type')
            try:
                if kind == '_connected':
                    self.server.send({'type': 'hello', 'mode': self.mode, 'session': self.session,
                                      'active_goal_id': self.goal['id'] if self.goal else None})
                    self.send_map()
                elif kind == 'goto':
                    self.start_goal(msg['goal_id'], float(msg['x']), float(msg['y']), float(msg['yaw']))
                elif kind == 'cancel':
                    self.cancel_goal()
                elif kind == 'stop':
                    self.cancel_goal()
                    self.stop_count = 10  # 1초 동안 정지 명령 발행
                elif kind == 'save_map':
                    self.save_map(str(msg['name']))
                elif kind == 'reset':
                    self.reset_robot()
                elif kind == '_relocalized':
                    self.on_relocalized(msg)
                elif kind == 'initial_pose':
                    self.publish_initial_pose(float(msg['x']), float(msg['y']), float(msg['yaw']),
                                              bool(msg.get('precise')))
                else:
                    self.get_logger().warn(f'알 수 없는 메시지: {kind}')
            except (KeyError, ValueError) as e:
                self.get_logger().error(f'잘못된 메시지 {msg}: {e}')

    # ---------------------------------------------------------- 로봇 상태 → 서버
    def on_map(self, msg):
        self.map_msg = msg
        if self.mode == 'slam':
            # SLAM 은 1~5초마다 맵을 갱신하므로 서버에는 3초에 한 번만 보낸다 (flush_map)
            self.map_dirty = True
            return
        self.get_logger().info(f'맵 수신 {msg.info.width}x{msg.info.height}')
        self.send_map()

    def flush_map(self):
        if self.map_dirty and time.monotonic() - self.last_map_sent >= 3.0:
            self.map_dirty = False
            self.last_map_sent = time.monotonic()
            self.send_map()

    def run_script(self, script_name, args, reply_type, tag, env_extra=None, timeout=180):
        """scripts/<script_name> 을 별도 스레드로 실행하고, [tag] 로 시작하는 출력 마지막 3줄을 서버에 알린다.
        한 번에 하나만 실행한다."""
        if self.script_running:
            self.server.send({'type': reply_type, 'ok': False, 'message': f'{self.script_running} 이(가) 실행 중입니다.'})
            return
        script = os.path.join(os.environ.get('WAREHOUSE_HOME', ''), 'scripts', script_name)
        if not os.path.isfile(script):
            self.server.send({'type': reply_type, 'ok': False, 'message': f'스크립트 없음: {script}'})
            return
        self.script_running = script_name
        env = os.environ.copy()
        env.update(env_extra or {})

        def run():
            try:
                r = subprocess.run([script, *args], capture_output=True, text=True, timeout=timeout, env=env)
                lines = [l for l in (r.stdout + r.stderr).splitlines() if l.startswith(f'[{tag}]')]
                self.server.send({'type': reply_type, 'ok': r.returncode == 0,
                                  'message': ' / '.join(l.replace(f'[{tag}] ', '') for l in lines[-3:])})
            except Exception as e:  # 시간 초과 등
                self.server.send({'type': reply_type, 'ok': False, 'message': str(e)})
            finally:
                self.script_running = None
        threading.Thread(target=run, daemon=True).start()
        self.get_logger().info(f'{script_name} 실행: {args}')

    def save_map(self, name):
        """웹의 '맵 저장': scripts/save_map.sh"""
        if self.mode != 'slam':
            self.server.send({'type': 'map_saved', 'ok': False, 'message': 'SLAM 모드에서만 맵을 저장할 수 있습니다.'})
            return
        env = {}
        pose = self.current_pose()
        if pose is not None:  # 로봇이 있는 공간을 기준으로 맵을 정리하도록
            env = {'SEED_X': f'{pose[0]:.3f}', 'SEED_Y': f'{pose[1]:.3f}'}
        self.run_script('save_map.sh', [name], 'map_saved', 'save_map', env)

    def reset_robot(self):
        """웹의 '상태 초기화': 즉시 멈추고 목표를 취소한 뒤 scripts/reset_robot.sh
        (시뮬레이션이면 Gazebo 월드 초기화로 로봇을 출발 위치에 세움)."""
        self.cancel_goal()
        self.stop_count = 10
        self.tipped_since = None
        if self.clear_obstacles_client.service_is_ready():
            self.clear_obstacles_client.call_async(Empty.Request())  # 부딪힌 자리 기억도 지움
        self.run_script('reset_robot.sh', [], 'reset_done', 'reset', timeout=60)

    def send_map(self):
        m = self.map_msg
        if m is None or m.info.width == 0 or m.info.height == 0:
            return  # SLAM 시작 직후 빈 맵은 보내지 않는다 (웹은 맵 없이 로봇 주변 격자를 보여줌)
        raw = np.asarray(m.data, dtype=np.int8).tobytes()
        self.server.send({
            'type': 'map',
            'name': self.get_parameter('map_name').value,
            'resolution': m.info.resolution,
            'origin': [m.info.origin.position.x, m.info.origin.position.y],
            'width': m.info.width,
            'height': m.info.height,
            'data': base64.b64encode(zlib.compress(raw)).decode('ascii'),
        })

    def current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)

    def send_pose(self):
        # 위치 TF가 없거나 오래됐으면(로봇/AMCL이 멈춤) 화면에서 지우도록 알린다
        pose = self.pose_with_age()
        if pose is not None and pose[3] <= 1.0:
            self.server.send({'type': 'pose', 'x': pose[0], 'y': pose[1], 'yaw': pose[2]})
        else:
            self.server.send({'type': 'pose_lost'})

    def tf_age(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        return (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9

    def send_health(self):
        """로봇(Pi)이 살아 있는지(LiDAR, 오도메트리), 위치를 아는지 1초마다 서버에 알린다."""
        now = self.get_clock().now()
        scan_age = None if self.scan is None else \
            (now - Time.from_msg(self.scan.header.stamp)).nanoseconds / 1e9
        odom_age = self.tf_age('odom', self.base_frame)
        map_age = self.tf_age(self.global_frame, self.base_frame)
        scan_ok = scan_age is not None and scan_age < 2.0
        odom_ok = odom_age is not None and odom_age < 2.0
        self.server.send({
            'type': 'health',
            'mode': self.mode,
            'robot_ok': scan_ok and odom_ok,
            'scan_ok': scan_ok,
            'odom_ok': odom_ok,
            'localized': odom_ok and map_age is not None and map_age < 2.0,
            'tipped': self.tipped,
            'tilt_deg': round(math.degrees(self.tilt)),
        })

    def on_battery(self, msg):
        if not math.isnan(msg.voltage) and msg.voltage > 0:
            self.battery_history.append((time.monotonic(), msg.voltage))
        now = time.monotonic()
        self.battery_history = [(t, v) for t, v in self.battery_history if now - t <= 30.0]
        if now - self.last_battery_sent < 5.0 or not self.battery_history:
            return
        self.last_battery_sent = now
        # 주행 중 전압이 출렁이므로 최근 30초 중앙값 사용
        voltage = float(np.median([v for _, v in self.battery_history]))
        self.server.send({'type': 'battery', 'voltage': voltage,
                          'percentage': battery_percent(voltage, self.battery_cells, self.battery_empty_v)})

    def send_path(self, poses):
        pts = [[p.pose.position.x, p.pose.position.y] for p in poses]
        if len(pts) > 2:
            pts = pts[:-1:3] + [pts[-1]]  # 전송량 줄이기
        self.server.send({'type': 'path', 'points': pts})

    def on_nav2_plan(self, msg):
        if self.planner_mode == 'nav2' and self.goal:
            self.send_path(msg.poses)

    def report(self, status, message=''):
        """현재 목표의 결과를 서버에 알리고 목표를 비운다."""
        if self.goal is None:
            return
        self.get_logger().info(f'목표 {self.goal["id"]}: {status} {message}')
        self.server.send({'type': 'nav_status', 'goal_id': self.goal['id'],
                          'status': status, 'message': message})
        if status != 'navigating':
            self.goal = None
            self.goal_handle = None
            self.send_path([])

    # ---------------------------------------------------------- 이동
    def start_goal(self, goal_id, x, y, yaw):
        self.cancel_goal(report=False)
        if self.tipped:
            self.last_goal_id = goal_id
            self.server.send({'type': 'nav_status', 'goal_id': goal_id, 'status': 'failed',
                              'message': '로봇이 넘어져 있습니다. 세운 뒤 상태 초기화를 하세요.'})
            return
        if self.tilt_rel > self.bump_tilt:
            # 장애물에 걸쳐 기운 채로 출발하면 더 올라타기만 한다 (sim 에서 43도까지 기움)
            self.last_goal_id = goal_id
            self.server.send({'type': 'nav_status', 'goal_id': goal_id, 'status': 'failed',
                              'message': f'차체가 {math.degrees(self.tilt_rel):.0f}도 기운 채 장애물에 걸려 있습니다. '
                                         '로봇을 직접 확인하세요.'})
            return
        self.goal = {'id': goal_id, 'x': x, 'y': y, 'yaw': yaw, 'retries': 0}
        self.last_goal_id = goal_id
        self.get_logger().info(f'목표 {goal_id}: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        if self.planner_mode == 'nav2':
            self.send_navigate_to_pose()
        else:
            self.request_plan()

    def cancel_goal(self, report=True):
        self.generation += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.follower is not None:
            self.follower = None
            self.stop_count = 5
        if report:
            self.report('canceled')
        self.goal = None
        self.goal_handle = None

    def make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quaternion_from_yaw(yaw)
        return ps

    # astar 모드 ---------------------------------------------------
    def request_plan(self):
        gen = self.generation
        pose = self.current_pose()
        if pose is None:
            self.report('failed', '로봇 위치를 알 수 없습니다. 초기 위치를 지정하세요.')
            return
        if not self.plan_client.service_is_ready():
            self.report('failed', 'A* 플래너 노드가 실행 중이 아닙니다.')
            return
        req = GetPlan.Request()
        req.start = self.make_pose(*pose)
        req.goal = self.make_pose(self.goal['x'], self.goal['y'], self.goal['yaw'])
        future = self.plan_client.call_async(req)
        future.add_done_callback(lambda f: self.on_plan(f, gen))

    def on_plan(self, future, gen):
        if gen != self.generation or self.goal is None:
            return
        plan = future.result().plan if future.result() else None
        if plan is None or not plan.poses:
            self.report('failed', '경로를 찾을 수 없습니다.')
            return
        if self.controller == 'builtin':
            start = self.current_pose()
            if start is None:
                self.report('failed', '로봇 위치를 알 수 없습니다. 초기 위치를 지정하세요.')
                return
            self.send_path(plan.poses)
            self.stop_count = 0  # 이전 목표의 정지 명령이 새 주행에 섞이지 않게
            now = time.monotonic()
            self.follower = {
                'gen': gen,
                'pp': PurePursuit([(ps.pose.position.x, ps.pose.position.y) for ps in plan.poses],
                                  self.goal['yaw'], max_v=self.max_speed, max_w=self.max_turn_speed),
                'blocked_since': None,
                'mark_pos': start[:2],
                'mark_time': now,
            }
            return
        if not self.follow_client.server_is_ready():
            self.report('failed', 'Nav2 controller_server가 준비되지 않았습니다.')
            return
        self.send_path(plan.poses)
        goal = FollowPath.Goal()
        goal.path = self.path_in_frame(plan, self.follow_frame)
        if goal.path is None:
            self.report('failed', f'{self.global_frame} -> {self.follow_frame} 변환을 알 수 없습니다.')
            return
        self.follow_client.send_goal_async(goal).add_done_callback(
            lambda f: self.on_goal_accepted(f, gen))

    def path_in_frame(self, plan, frame):
        """경로를 frame 좌표로 변환 (2D). 같은 frame이면 그대로."""
        if not frame or frame == plan.header.frame_id:
            return plan
        try:
            t = self.tf_buffer.lookup_transform(frame, plan.header.frame_id, Time()).transform
        except TransformException:
            return None
        tyaw = yaw_from_quaternion(t.rotation)
        c, s = math.cos(tyaw), math.sin(tyaw)
        out = Path()
        out.header.frame_id = frame
        out.header.stamp = self.get_clock().now().to_msg()
        for p in plan.poses:
            x, y = p.pose.position.x, p.pose.position.y
            ps = self.make_pose(t.translation.x + c * x - s * y, t.translation.y + s * x + c * y,
                                yaw_from_quaternion(p.pose.orientation) + tyaw)
            ps.header.frame_id = frame
            out.poses.append(ps)
        return out

    # nav2 모드 ----------------------------------------------------
    def send_navigate_to_pose(self):
        gen = self.generation
        if not self.nav_client.server_is_ready():
            self.report('failed', 'Nav2 bt_navigator가 준비되지 않았습니다.')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(self.goal['x'], self.goal['y'], self.goal['yaw'])
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda f: self.on_goal_accepted(f, gen))

    # 공통 ---------------------------------------------------------
    def on_goal_accepted(self, future, gen):
        handle = future.result()
        if gen != self.generation:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.report('failed', 'Nav2가 목표를 거부했습니다.')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, gen))

    def on_result(self, future, gen):
        if gen != self.generation or self.goal is None:
            return
        status = future.result().status
        reason = '이동 중 막혔거나 목표에 도달하지 못했습니다.'
        if status == GoalStatus.STATUS_SUCCEEDED:
            # controller_server는 TF 변환에 실패해도 "Reached the goal"을 돌려주는 경우가 있어 직접 확인
            error = self.goal_error()
            if error is not None and error <= self.arrive_tolerance:
                self.report('succeeded')
                return
            reason = ('Nav2는 도착이라고 했지만 로봇 위치를 확인할 수 없습니다.' if error is None else
                      f'Nav2는 도착이라고 했지만 목표와 {error:.2f} m 떨어져 있습니다.')
            self.get_logger().warn(reason)
        if status == GoalStatus.STATUS_CANCELED:
            self.report('canceled')
        else:
            self.fail_or_retry(reason)

    def fail_or_retry(self, reason):
        """재시도 횟수가 남았으면 현재 위치에서 다시 계획, 아니면 실패 보고."""
        if self.goal['retries'] < self.max_retries:
            self.goal['retries'] += 1
            self.get_logger().warn(f'{reason} 재시도 {self.goal["retries"]}/{self.max_retries}')
            self.generation += 1
            self.goal_handle = None
            self.follower = None
            if self.planner_mode == 'nav2':
                self.send_navigate_to_pose()
            else:
                self.request_plan()
        else:
            self.report('failed', reason)

    # 내장 추종기 (controller=builtin) ------------------------------
    def on_scan(self, msg):
        self.scan = msg
        # AMCL/SLAM/A* 기억에는 평소보다 기울지 않았을 때의 스캔만 넘긴다 (scan_level).
        # 부딪혀 기울어지면 LiDAR 가 바닥을 벽처럼 보고, 그 스캔으로 AMCL 위치가 수십 cm 튄다.
        # 단, 기운 채로 3초가 지나면(장애물에 걸쳐 멈춤 등) 다시 넘긴다 - 안 넘기면 위치 추정이 멈춘다
        now = time.monotonic()
        if self.tilt_rel < math.radians(3):
            self.scan_gated_since = None
        elif self.scan_gated_since is None:
            self.scan_gated_since = now
        if self.scan_gated_since is None or now - self.scan_gated_since > 3.0:
            self.level_scan_pub.publish(msg)

    def on_imu(self, msg):
        """기울기가 35도를 넘은 채 0.5초 지나면 넘어진 것으로 보고 정지."""
        q = msg.orientation
        # 로봇 z축이 수직에서 얼마나 기울었는지
        cos_tilt = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.tilt = math.acos(max(-1.0, min(1.0, cos_tilt)))
        now = time.monotonic()
        if self.tilt_base is None:
            self.tilt_base = self.tilt
        self.tilt_rel = max(0.0, self.tilt - self.tilt_base)
        # 위쪽(중력 반대) 방향을 로봇 좌표로 본 것. 앞이 들리면 로봇 x 축이 위를 향하므로 up_x > 0
        # → 수평 성분의 방향이 곧 들린 쪽
        up_x = 2.0 * (q.x * q.z - q.w * q.y)
        up_y = 2.0 * (q.y * q.z + q.w * q.x)
        if math.hypot(up_x, up_y) > 1e-3:
            self.lift_dir = math.atan2(up_y, up_x)
        if self.tilt_rel < math.radians(2) and self.last_imu_time is not None:
            # 시간 상수 10초로 평소 기울기를 따라간다 (장애물에 올라탄 동안은 따라가지 않음)
            a = min(1.0, (now - self.last_imu_time) / 10.0)
            self.tilt_base += a * (self.tilt - self.tilt_base)
        self.last_imu_time = now
        if self.tilt > math.radians(35):
            if self.tipped_since is None:
                self.tipped_since = now
            elif now - self.tipped_since > 0.5 and (self.goal is not None or self.follower is not None):
                self.get_logger().error(f'넘어짐 감지 (기울기 {math.degrees(self.tilt):.0f}도), 정지')
                self.cancel_goal(report=False)
                self.stop_count = 10
                self.server.send({'type': 'nav_status', 'goal_id': self.last_goal_id, 'status': 'failed',
                                  'message': '로봇이 넘어졌습니다. 세운 뒤 상태 초기화를 하세요.'})
        else:
            self.tipped_since = None

    @property
    def tipped(self):
        return self.tipped_since is not None and time.monotonic() - self.tipped_since > 0.5

    def front_clearance(self, half_angle=math.radians(25), center=0.0):
        """center(앞=0, 뒤=pi) 방향 ±half_angle 안에서 가장 가까운 물체까지 거리 (LiDAR 기준).
        스캔이 없거나 오래되면 0 (= 막힘)."""
        s = self.scan
        if s is None:
            return 0.0
        age = (self.get_clock().now() - Time.from_msg(s.header.stamp)).nanoseconds / 1e9
        if age > 1.0:
            return 0.0
        r = np.asarray(s.ranges)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        ang = np.arctan2(np.sin(ang - center), np.cos(ang - center))
        ok = np.isfinite(r) & (r > s.range_min) & (np.abs(ang) <= half_angle)
        return float(r[ok].min()) if ok.any() else math.inf

    def pose_with_age(self):
        """(x, y, yaw, TF 나이[s]) 또는 None."""
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation), age

    def stop_following(self):
        self.follower = None
        self.stop_count = 5
        self.cmd_vel_pub.publish(Twist())

    def control_step(self):
        f = self.follower
        if f is None:
            return
        if f['gen'] != self.generation or self.goal is None:
            self.stop_following()
            return
        now = time.monotonic()
        if 'backoff' in f:
            self.backoff_step(f, now)
            return
        pose = self.pose_with_age()
        # map->base TF 스탬프는 odom 쪽(Pi) 스탬프를 따른다. AMCL은 미래 스탬프라 나이가 음수일 수 있음
        if pose is None or pose[3] > self.tf_timeout:
            if pose is not None and self.tilt_rel > math.radians(3):
                # 기울어진 동안은 AMCL 에 스캔을 안 넘기므로 map TF 가 멈춘다 → 넘어짐이 아니라 부딪힘으로 처리
                self.start_backoff(f, *pose[:3], now,
                                   f'차체가 {math.degrees(self.tilt_rel):.0f}도 기울었습니다 (낮은 장애물에 올라탐).')
                return
            self.stop_following()
            self.report('failed', '로봇 위치 정보가 갱신되지 않아 정지했습니다.')
            return
        x, y, yaw, _ = pose
        if self.tilt_rel < math.radians(2):
            f['level_pose'] = (x, y, yaw)   # 부딪힌 자리는 기울기 시작 전 위치/방향으로 계산 (밀리면서 돌아가므로)
        v, w, state = f['pp'].compute(x, y, yaw)

        if state == 'done':
            self.stop_following()
            self.report('succeeded')
            return
        if state == 'off_path':
            self.stop_following()
            self.fail_or_retry('경로에서 벗어났습니다.')
            return

        # 기울었으면(낮은 장애물에 올라탐) 앞이 막힌 것(LiDAR 가 바닥을 봄)보다 먼저 처리한다.
        # 기울어진 채로 기다리면 기울어진 스캔 때문에 AMCL 위치가 크게 틀어진다
        if self.tilt_rel > self.bump_tilt:
            self.start_backoff(f, x, y, yaw, now,
                               f'차체가 {math.degrees(self.tilt_rel):.0f}도 기울었습니다 (낮은 장애물에 올라탐).')
            return
        if v > 0.0:
            clearance = self.front_clearance()
            if clearance < self.stop_distance and self.tilt_rel > math.radians(3):
                # 장애물 모서리에 걸쳐 조금 기운 상태: LiDAR 가 아래를 보고 바닥/상자 윗면을 장애물로 본다.
                # 기다려도 풀리지 않으므로(SLAM 탐색 시험에서 계속 '앞이 막힘') 부딪힘으로 처리해 물러난다
                self.start_backoff(f, x, y, yaw, now,
                                   f'차체가 {math.degrees(self.tilt_rel):.0f}도 기운 채 앞이 막혔습니다 (장애물에 걸침).')
                return
            if clearance < self.stop_distance:
                if f['blocked_since'] is None:
                    f['blocked_since'] = now
                    self.get_logger().warn(f'앞쪽 {clearance:.2f} m 장애물, 정지 후 대기')
                if now - f['blocked_since'] > self.blocked_timeout:
                    self.stop_following()
                    self.fail_or_retry('앞이 막혀 진행할 수 없습니다.')
                    return
                f['mark_time'] = now  # 기다리는 동안은 진행 없음으로 치지 않음
                f.pop('drive_since', None)
                self.cmd_vel_pub.publish(Twist())
                return
            f['blocked_since'] = None

        stall = self.detect_stall(f, x, y, v, state, now)
        if stall:
            self.start_backoff(f, x, y, yaw, now, stall)
            return

        # 제자리 회전 중이 아닌데 오래 진행이 없으면 실패
        # 제자리 회전이 끝나지 않는 경우(방향 맞추기 실패, 미끄러짐 등) 차단
        if state == 'rotating':
            f.setdefault('rotate_since', now)
            if now - f['rotate_since'] > self.rotate_timeout:
                self.stop_following()
                self.fail_or_retry(f'{self.rotate_timeout:.0f}초 넘게 회전만 하고 있습니다.')
                return
        else:
            f.pop('rotate_since', None)
        if math.hypot(x - f['mark_pos'][0], y - f['mark_pos'][1]) > 0.05 or state == 'rotating':
            f['mark_pos'], f['mark_time'] = (x, y), now
        elif now - f['mark_time'] > self.progress_timeout:
            self.stop_following()
            self.fail_or_retry('진행이 없습니다.')
            return

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_vel_pub.publish(cmd)

    def start_backoff(self, f, x, y, yaw, now, reason):
        """x, y, yaw: 부딪힌 때의 map 좌표 (부딪힌 자리 계산용). 물러난 거리는 odom 으로 잰다
        (기울어진 동안은 map TF 가 갱신되지 않으므로)."""
        self.get_logger().warn(f'부딪힘 감지: {reason} 뒤로 물러납니다')
        x, y, yaw = f.get('level_pose', (x, y, yaw))
        # 기울어서 감지했으면 들린 쪽이 장애물. 앞이 들렸으면 뒤로, 뒤가 들렸으면(회전하다 뒤 모서리가 올라탐) 앞으로
        # 빠져나온다. 앞으로 가다 막혀서(정지) 감지했으면 앞쪽이 장애물
        tilted = self.tilt_rel > math.radians(3)
        side = self.lift_dir if tilted else 0.0
        f['backoff'] = {'x': x, 'y': y, 'yaw': yaw, 'side': side,
                        'speed': -0.05 if math.cos(side) >= 0 else 0.05, 'until': now + 5.0, 'reason': f'부딪힘 감지: {reason}',
                        'odom': self.odom_xy()}
        self.cmd_vel_pub.publish(Twist())

    def odom_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', self.base_frame, Time()).transform.translation
            return t.x, t.y
        except TransformException:
            return None

    def detect_stall(self, f, x, y, v, state, now):
        """앞으로 가라고 하는데 못 가고 있으면 이유 문자열, 아니면 None."""
        if v >= 0.05 and state != 'rotating':
            t0, x0, y0 = f.setdefault('drive_since', (now, x, y))
            if math.hypot(x - x0, y - y0) > 0.03:
                f['drive_since'] = (now, x, y)
            elif now - t0 > self.stall_timeout:
                return f'{self.stall_timeout:.1f}초 동안 앞으로 가지 못했습니다 (부딪힘/끼임).'
        else:
            f.pop('drive_since', None)
        return None

    def backoff_step(self, f, now):
        """부딪힌 뒤 천천히 빠져나온다 (보통 뒤로). 다 물러나면 부딪힌 자리를 플래너에 알리고 재계획."""
        b = f['backoff']
        # LiDAR 는 로봇 뒤 끝에서 0.13 m 앞에 있으므로 뒤쪽 0.3 m 안에 물체가 있으면 더 물러나지 않음 (앞으로 나올 때는 0.25 m)
        if b['speed'] < 0:
            blocked = self.front_clearance(math.radians(30), center=math.pi) < 0.3
        else:
            blocked = self.front_clearance(math.radians(30)) < 0.25
        cur, start = self.odom_xy(), b['odom']
        moved = math.hypot(cur[0] - start[0], cur[1] - start[1]) if cur and start else 0.0
        if moved >= self.backoff_distance or now > b['until'] or blocked:
            self.stop_following()
            # 로봇 앞 끝(base 에서 약 0.07 m) 앞쪽이 부딪힌 물체 (플래너가 bump_radius 반경을 막음)
            pt = PointStamped()
            pt.header.frame_id = self.global_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = b['x'] + 0.25 * math.cos(b['yaw'] + b['side'])
            pt.point.y = b['y'] + 0.25 * math.sin(b['yaw'] + b['side'])
            self.bump_pub.publish(pt)
            self.goal['bumps'] = self.goal.get('bumps', 0) + 1
            if self.goal['bumps'] >= 2:
                # 같은 이동에서 두 번 부딪히면 더 시도하지 않는다. LiDAR 에 안 보이는 물체를 자동으로 계속 피하려 하면
                # 계속 다시 부딪히고 올라타서(sim 에서 충돌 20회 이상, 43도까지 기움) 사람이 보는 게 낫다
                self.report('failed', f'{b["reason"]} 두 번 부딪혀 멈췄습니다. LiDAR 에 안 보이는 낮은 장애물이 '
                                      '있을 수 있으니 직접 확인하세요.')
                return
            self.relocalize_then_retry(b['reason'])
            return
        cmd = Twist()
        cmd.linear.x = b['speed']
        self.cmd_vel_pub.publish(cmd)

    def relocalize_then_retry(self, reason):
        """맵 모드: 부딪힌 뒤 스캔 매칭(tools/localize.py)으로 위치를 다시 맞추고 재계획한다.
        부딪혀 기울어진 동안의 스캔(바닥을 봄) 때문에 AMCL 위치가 수십 cm 틀어지기 때문."""
        pose = self.current_pose()
        tool = os.path.join(os.environ.get('WAREHOUSE_HOME', ''), 'tools', 'localize.py')
        gen = self.generation
        if self.mode != 'map' or pose is None or not os.path.isfile(tool) or self.goal is None:
            # 플래너가 부딪힌 자리(astar/obstacle)를 받은 뒤에 재계획하도록 0.5초 늦춘다
            # (바로 재계획하면 기억하기 전 경로로 같은 자리에 또 부딪혔다)
            def later():
                time.sleep(0.5)
                self.server.inbox.put({'type': '_relocalized', 'gen': gen, 'result': None, 'reason': reason})
            threading.Thread(target=later, daemon=True).start()
            return
        self.get_logger().info('부딪힌 뒤 위치 다시 맞추는 중 (스캔 매칭)')
        cmd = ['python3', tool, f'{pose[0]:.3f}', f'{pose[1]:.3f}', f'{math.degrees(pose[2]):.1f}',
               '--range', '0.5', '--yaw-range', '30', '--dry-run']

        def run():
            result = None
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                for line in r.stdout.splitlines():
                    if line.startswith('RESULT'):
                        result = [float(v) for v in line.split()[1:5]]
            except Exception as e:  # 시간 초과 등
                self.get_logger().warn(f'위치 다시 맞추기 실패: {e}')
            self.server.inbox.put({'type': '_relocalized', 'gen': gen, 'result': result, 'reason': reason})
        threading.Thread(target=run, daemon=True).start()

    def on_relocalized(self, msg):
        if msg['gen'] != self.generation or self.goal is None:
            return  # 그 사이 취소/새 목표
        r = msg['result']
        if r and r[3] >= 0.8:   # 스캔 점 80% 이상이 벽과 20cm 이내로 맞을 때만 믿는다
            self.publish_initial_pose(r[0], r[1], r[2], precise=True)
        elif self.mode == 'map':
            self.get_logger().warn(f'스캔 매칭 결과가 좋지 않아 AMCL 위치를 그대로 씁니다 ({r})')
        self.fail_or_retry(msg['reason'])

    def goal_error(self):
        """현재 위치와 목표 사이 거리(m). 위치를 모르면 None."""
        pose = self.current_pose()
        if pose is None or self.goal is None:
            return None
        return math.hypot(pose[0] - self.goal['x'], pose[1] - self.goal['y'])

    # ---------------------------------------------------------- 기타
    def publish_initial_pose(self, x, y, yaw, precise=False):
        """precise: 스캔 매칭으로 찾은 위치 → 표준편차 5cm/3°, 아니면(웹에서 찍은 위치) 20cm/15°.
        예전 값(50cm)은 입자가 너무 퍼져서 출발 후 몇 초 동안 위치가 20cm 넘게 틀렸다."""
        msg = PoseWithCovarianceStamped()
        msg.header = self.make_pose(x, y, yaw).header
        msg.pose.pose = self.make_pose(x, y, yaw).pose
        sxy, syaw = (0.05, math.radians(3)) if precise else (0.2, math.radians(15))
        msg.pose.covariance[0] = sxy ** 2
        msg.pose.covariance[7] = sxy ** 2
        msg.pose.covariance[35] = syaw ** 2
        self.initial_pose_pub.publish(msg)
        # 멈춘 채로도 스캔으로 몇 번 보정하게 한다 (AMCL 은 움직여야만 갱신하므로)
        self.nomotion_left = 10
        self.get_logger().info(f'초기 위치 지정 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)'
                               f'{" - 스캔 매칭 결과" if precise else ""}')

    def nomotion_step(self):
        if self.nomotion_left <= 0 or not self.nomotion_client.service_is_ready():
            return
        self.nomotion_left -= 1
        self.nomotion_client.call_async(Empty.Request())

    def publish_stop(self):
        if self.stop_count > 0:
            self.stop_count -= 1
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
