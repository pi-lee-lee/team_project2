"""Django 서버(WebSocket)와 ROS 2 를 잇는 브리지 노드.

서버 → 로봇: goto, cancel, stop, initial_pose
로봇 → 서버: hello, map, pose, battery, path, nav_status

이동 방식: astar_planner 노드로 경로 계획 후 hybrid 추종
  평소 pure pursuit, 로봇 앞 사각형(hybrid.box_*) 안에 지도에 없는 물체가 보이거나 움직이는 물체가
  hybrid.moving_predict 초 안에 사각형에 들어올 것 같으면 DWA 로 바꿔 피하고, 지나가면 pure pursuit 로 복귀.

안전장치: 위치 TF가 끊기면 정지, (pure pursuit) 앞쪽 장애물이 있으면 정지 후 대기,
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
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .astar import obstacle_distance
from .cmd_mux import CmdMux
from .astar_planner_node import quaternion_from_yaw, yaw_from_quaternion
from .dwa import DWAConfig, DWAFollower, transform_points
from .follower import PurePursuit
from .rl_follower import MLPPolicy, RLFollower
from .scan_noise import ScanNoiseConfig, degrade
from .tracker import MultiObjectTracker, detect_clusters


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


NO_RETURN_MARGIN = 0.05  # m. inf 빔을 range_max - 이 값으로 바꿈 (slam_toolbox max_laser_range 는 이보다 작아야 함)


SCAN_SEND_INTERVAL = 0.15  # s, 웹에 실시간 LiDAR 를 보내는 최소 간격 (화면 표시용). LiDAR 5Hz 면 스캔마다 (0.25 면 한 번 건너 2.5Hz 였음)
MAP_MIN_INTERVAL = 0.5  # s, SLAM 맵을 서버로 보내는 최소 간격 (갱신되면 바로 보내되 몰려 오면 이 간격으로)
SLAM_BEAMS = 240  # slam_toolbox 에 넘기는 스캔의 빔 개수 (1.5° 간격 고정). 실물 LDS-02 가 238~244 개라 빈 칸이 거의 없게


def to_slam_scan(msg):
    """slam_toolbox 용 스캔으로 바꾼다.
    1) 실물 LDS-02 는 빔 개수가 스캔마다 238~244 개로 바뀌는데, slam_toolbox 는 처음 등록한 개수와 다른 스캔을 전부 버린다
       ("contains 243 range readings, expected 242") → 지도가 첫 몇 m² 에서 멈추고 로봇은 지도에 없는 벽 앞에서 막혔음.
       → 0~360° 를 1.5° 칸 240 개로 다시 나눠 항상 같은 개수로 넘긴다 (같은 칸에 빔이 여럿이면 가장 가까운 값).
    2) Karto 는 range_max 이상(inf 포함)인 빔을 통째로 버린다 → 3.5 m 안에 아무것도 없는 넓은 공간은 로봇이 가도 빈 칸으로
       칠해지지 않아 미탐색으로 남는다. inf 를 range_max 바로 아래로 바꾸면 Karto 는 max_laser_range 까지만 빈 칸으로 칠하고
       끝점은 장애물로 찍지 않는다. AMCL/A* 기억은 이 값을 장애물로 보므로 scan_level(원본)을 그대로 쓴다.
    실물 LDS 는 반사 없음을 0 또는 NaN 으로 내보내는데, 0 은 너무 가까움과 구별이 안 되므로 바꾸지 않는다 (inf 만)."""
    r = np.asarray(msg.ranges, dtype=np.float32).copy()
    r[np.isposinf(r)] = msg.range_max - NO_RETURN_MARGIN
    inc = 2.0 * math.pi / SLAM_BEAMS
    ang = (msg.angle_min + np.arange(len(r)) * msg.angle_increment) % (2.0 * math.pi)
    idx = np.round(ang / inc).astype(int) % SLAM_BEAMS
    out_r = np.full(SLAM_BEAMS, np.nan, dtype=np.float32)
    valid = np.isfinite(r)
    # 칸마다 가장 가까운 값 (정렬 후 뒤에서부터 쓰면 마지막에 쓴 값 = 가장 작은 값)
    order = np.argsort(-np.where(valid, r, -1.0))
    order = order[valid[order]]
    out_r[idx[order]] = r[order]
    out = LaserScan()
    out.header = msg.header
    out.angle_min, out.angle_increment = 0.0, inc
    out.angle_max = (SLAM_BEAMS - 1) * inc
    out.time_increment = msg.time_increment * len(r) / SLAM_BEAMS
    out.scan_time = msg.scan_time
    out.range_min, out.range_max = msg.range_min, msg.range_max
    out.ranges = out_r.tolist()
    return out


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


class _AutoCmd:
    """자동 이동 명령을 선택기에 넣는 창구. 예전 코드의 self.cmd_vel_pub.publish(Twist) 를 그대로 쓰게 한다."""

    def __init__(self, mux):
        self.mux = mux

    def publish(self, msg):
        self.mux.auto(msg.linear.x, msg.angular.z, time.monotonic())


class BridgeNode(Node):
    def __init__(self):
        super().__init__('warehouse_bridge')
        self.declare_parameter('server_url', 'ws://localhost:8000/ws/robot/tb3/')
        self.declare_parameter('map_name', 'default')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_rate', 5.0)
        self.declare_parameter('max_retries', 2)   # 이동 실패 시 재계획 횟수
        # 경로 추종기 (pure pursuit)
        self.declare_parameter('max_speed', 0.1)           # m/s
        self.declare_parameter('max_turn_speed', 0.6)      # rad/s
        self.declare_parameter('max_accel', 0.3)           # m/s², pure pursuit 출발 가속/정지 감속
        self.declare_parameter('max_turn_accel', 1.5)      # rad/s², pure pursuit 제자리 회전
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
        # 시뮬레이션 LiDAR 를 실물 LDS-02 처럼 (빔 개수 238~244, 거리 흔들림, 먼 곳/약한 반사 값 빠짐). scan_noise.py
        # run.sh --scan-noise 로 켬. 실물 로봇(use_sim_time false)에서는 켜도 무시
        sn = lambda n, d: self.declare_parameter(f'sim_scan.{n}', d).value  # noqa: E731
        self.scan_noise = None
        if sn('enabled', False):
            if self.get_parameter('use_sim_time').value:
                self.scan_noise = ScanNoiseConfig(**{k: type(v)(sn(k, v)) for k, v in vars(ScanNoiseConfig()).items()})
                self.scan_noise_rng = np.random.default_rng()
                self.get_logger().info(f'시뮬레이션 LiDAR 를 실물처럼 바꿈: {self.scan_noise}')
            else:
                self.get_logger().warn('sim_scan.enabled 는 시뮬레이션에서만 씁니다 (실물이라 무시)')
        self.declare_parameter('battery_cells', 3)         # LiPo 셀 수 (TurtleBot3: 3셀 11.1V)
        self.declare_parameter('battery_empty_v', 11.0)    # 0% 로 볼 전압 (TurtleBot3 충전 경고 전압)
        self.declare_parameter('mode', 'map')              # map: 기존 맵+AMCL / slam: 맵 만들기

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.global_frame = p('global_frame')
        self.base_frame = p('base_frame')
        self.max_retries = p('max_retries')
        self.max_speed = p('max_speed')
        self.max_turn_speed = p('max_turn_speed')
        self.max_accel = p('max_accel')
        self.max_turn_accel = p('max_turn_accel')
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
        self.session = self.run_session()
        self.map_dirty = False      # SLAM 모드: 서버로 보낼 새 맵이 있음
        self.last_map_sent = 0.0
        self.script_running = None  # 실행 중인 scripts/*.sh 이름

        self.tf_buffer = Buffer()
        # TF 는 별도 노드 + 스레드에서 받는다. 이 노드는 IMU(~200Hz)·odom·scan·타이머를 한 스레드에서 처리하는데
        # 실행기는 한 바퀴에 구독마다 메시지 하나씩만 꺼내므로, 부하가 높으면 /tf(~100Hz) 가 큐에 쌓여
        # 0.5초 넘게 밀리고 '위치 TF 끊김' 으로 멈췄다 (바깥 노드에서 잰 TF 는 정상이었음)
        # use_global_arguments=False: launch 의 '__node:=warehouse_bridge' 가 이 노드에도 적용되어
        # 노드 목록에 /warehouse_bridge 가 두 개로 보였음 (브리지가 둘 떠 있는 것처럼)
        # 로봇 여러 대(네임스페이스 있음)면 TF 도 그 로봇 것(/<ns>/tf)만 받는다. 이 노드는 launch 인자를 안 받으므로 직접 넘김
        ns = self.get_namespace()
        tf_args = [] if ns == '/' else ['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static']
        self.tf_node = rclpy.create_node('warehouse_bridge_tf', namespace=ns, use_global_arguments=False, cli_args=tf_args,
                                         parameter_overrides=[rclpy.parameter.Parameter(
                                             'use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_msg = None
        self.create_subscription(OccupancyGrid, p('map_topic'), self.on_map, map_qos)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, qos_profile_sensor_data)
        self.scan = None
        self.last_scan_sent = 0.0   # 웹 실시간 LiDAR (send_scan)
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
        # 카메라 물건(LED) 인식 결과: 인식 노드(cargo_detector 또는 직접 만든 YOLO 노드)가 보이는 라벨을 발행.
        # '' = 아무것도 없음. 흔들림을 막기 위해 cargo_stable_sec 동안 같을 때만 서버에 알린다
        self.declare_parameter('cargo_stable_sec', 1.0)
        self.cargo_raw = None       # (라벨, 처음 본 시각)
        self.cargo_sent = None      # 서버에 마지막으로 알린 라벨 (None = 인식 노드 없음)
        self.cargo_resend = False   # 서버에 다시 연결되면 지금 라벨을 다시 알림
        self.create_subscription(String, 'cargo/detected', self.on_cargo, 10)
        self.create_timer(0.2, self.flush_cargo)
        # YOLO 부품 검출기 (yolo/yolo_training/infer_ros_switch.py): 웹 버튼으로 enabled(추론+검출 영상 발행)를 켜고 끄고,
        # 클래스별 개수 글자(/yolo/counts, 약 1초마다)를 웹에 띄운다. 검출기 노드가 없으면 '없음'으로 표시
        # 상대 이름: 네임스페이스 없이 띄우면 /component_detector_switch, /yolo/counts (로봇 여러 대면 /<ns>/...)
        self.declare_parameter('yolo_node', 'component_detector_switch')
        self.declare_parameter('yolo_counts_topic', 'yolo/counts')
        yn = self.get_parameter('yolo_node').value.rstrip('/')
        self.yolo_get = self.create_client(GetParameters, f'{yn}/get_parameters')
        self.yolo_set = self.create_client(SetParameters, f'{yn}/set_parameters')
        self.yolo_enabled = None     # None = 모름 (검출기 없음)
        self.yolo_counts = ('', None)   # (글자, 받은 시각)
        self.yolo_pending = None     # 답을 기다리는 enabled 조회 (future, 보낸 시각)
        self.create_subscription(String, self.get_parameter('yolo_counts_topic').value, self.on_yolo_counts, 10)
        self.create_timer(1.0, self.poll_yolo)

        # 움직이는 장애물 추적 (tb3_nav 의 obstacle_tracker 를 브리지 안으로): 스캔 → odom 점 → 지도 벽 근처 제외
        # → 덩어리 → 칼만필터로 위치/속도. DWA 가 미래 위치를 피하는 데 쓰고, 웹 지도에 표시한다
        tp = lambda n, d: self.declare_parameter(f'tracking.{n}', d).value  # noqa: E731
        self.tracking = tp('enabled', True)
        self.track_cfg = dict(max_range=tp('max_range', 5.0), static_match_dist=tp('static_match_dist', 0.15),
                              cluster_gap=tp('cluster_gap', 0.15), min_points=tp('min_points', 3),
                              max_diameter=tp('max_diameter', 0.8), min_radius=tp('min_radius', 0.05))
        self.tracker = MultiObjectTracker(gate=tp('gate', 0.5), min_hits=tp('min_hits', 3),
                                          max_misses=tp('max_misses', 5), accel_noise=tp('accel_noise', 1.0),
                                          meas_noise=tp('meas_noise', 0.05), max_speed=tp('max_speed', 1.5))
        self.tracks = []                   # 확정된 track (odom)
        self.scan_odom = np.empty((0, 2))  # 최근 스캔 점 (odom) - DWA 정적 장애물
        self.scan_unmapped = np.empty((0, 2))
        self.map_dist = None               # 지도 벽까지 거리 (벽 근처 점 = 정적) (dist, msg)
        self.last_obstacles_sent = 0.0
        self.last_dwa_sent = 0.0
        # DWA (hybrid 에서 장애물 피할 때). 기본값은 tb3_nav 값에서 속도/외곽(직사각형)/여유를 이 프로젝트에 맞춤 (params.yaml 과 같음)
        defaults = DWAConfig(max_v=0.15, max_w=1.0, max_acc_v=0.3, max_acc_w=1.5, robot_radius=0.17, footprint=(-0.197, 0.069, 0.153), allow_escape=True,
                             safety_margin=0.08, dynamic_margin=0.15, dynamic_growth=0.03, predict_time=2.5,
                             w_collision=2.0, reverse_only_when_needed=True)
        # controller: hybrid(기본, pure pursuit ↔ DWA) | rl (강화학습 정책 rl_follower.py, 안전 필터 포함. 전역 경로는 A* 그대로)
        self.declare_parameter('controller', 'hybrid')
        self.declare_parameter('rl.model', os.path.expanduser('~/webserver/config/rl_policy.npz'))
        self.declare_parameter('rl.goal_tolerance', 0.15)
        self.controller = self.get_parameter('controller').value
        self.rl_policy = MLPPolicy(self.get_parameter('rl.model').value) if self.controller == 'rl' else None
        if self.rl_policy is not None:
            self.get_logger().info(f"controller=rl: {self.get_parameter('rl.model').value}")
        self.dwa_cfg = DWAConfig(**{
            k: (tuple(float(x) for x in self.declare_parameter(f'dwa.{k}', [float(x) for x in v]).value)
                if isinstance(v, tuple) else type(v)(self.declare_parameter(f'dwa.{k}', v).value))
            for k, v in vars(defaults).items()})
        self.dwa_cfg.control_period = 0.1    # DWA 계산 주기 (dwa_compute)
        # 차체에 붙인 물건 (base_footprint 기준 사각형 x_min, x_max, y_min, y_max 를 4개씩 이어 붙인 목록).
        # 기본: 뒤쪽 가운데 8cm x 8cm (차체 뒤 끝 x=-0.197 에서 8cm 더 나옴). 제자리 회전/후진 여유와 웹 로봇 모양에 반영.
        # DWA 충돌 검사(dwa.footprint 직사각형)에는 아직 안 들어감
        ex = [float(v) for v in self.declare_parameter('footprint_extra', [-0.277, -0.197, -0.04, 0.04]).value]
        self.footprint_extra = [tuple(ex[i:i + 4]) for i in range(0, len(ex) - 3, 4)]
        self.declare_parameter('dwa_follow.lookahead', 0.5)
        self.declare_parameter('dwa_follow.goal_tolerance', 0.1)
        # 앞쪽 경로가 지도에 없는 장애물에서 이 거리 안을 지나면 목표점을 그 너머 경로점으로 (m, 0 이면 끔), target_max 까지
        self.declare_parameter('dwa_follow.target_clear', 0.233)
        self.declare_parameter('dwa_follow.target_max', 2.0)
        self.declare_parameter('dwa_follow.moving_speed', 0.08)  # 이보다 빠른 track 만 움직이는 장애물로 (멈춘 물체 속도 노이즈 ~0.06, 느린 사람 0.1)
        # hybrid: 평소 pure pursuit, 장애물이 가까우면 DWA
        self.declare_parameter('hybrid.release_time', 2.0)      # 장애물이 이 시간 동안 안 보이면 pure pursuit 로 (s)
        self.declare_parameter('hybrid.rejoin_offset', 0.25)    # 경로에서 이 거리 안으로 돌아왔을 때만 pure pursuit 로 (m)
        # 전환 조건: 로봇 앞(+뒤 조금) 사각형 안에 지도에 없는 스캔 점이 있거나 (box_*),
        # 움직이는 track 이 moving_predict 초 안에 사각형에 들어올 것 같으면 DWA
        self.declare_parameter('hybrid.box_front', 1.0)         # 차체 앞 끝(x=0.069)에서 앞으로 (m)
        self.declare_parameter('hybrid.box_back', 0.2)          # 차체 뒤 끝(x=-0.197)에서 뒤로 (m)
        self.declare_parameter('hybrid.box_half_width', 0.25)   # 좌우 반폭 (m): 차체 반폭 0.153 + 여유
        self.declare_parameter('hybrid.box_min_points', 3)      # 사각형 안 점이 이만큼 이상이어야 (AMCL 오차로 벽 점 몇 개가 새는 것 무시)
        self.declare_parameter('hybrid.box_min_scans', 2)       # 이만큼 연속 스캔에서 보여야 전환 (깜빡임 방지)
        # 움직이는 track: 지금 ~ 이 시간 뒤까지 속도로 예측한 위치가 사각형과 겹치면 전환 (s).
        # 로봇도 지금 속도로 직진한다고 보고 사각형을 같이 옮긴다. 0 이면 지금 위치만 (= 정적인 물체와 같음)
        self.declare_parameter('hybrid.moving_predict', 2.0)
        self.box_streak = 0      # 사각형 안에 점이 있던 연속 스캔 수
        self.box_count = 0       # 마지막 스캔에서 사각형 안 점 수
        self.box_near = math.inf # 그중 차체에서 가장 가까운 거리 (m, 로그용)
        self.last_box_sent = 0.0
        self.box_pts = np.empty((0, 2))      # 사각형 안의 지도에 없는 점 (odom) - 웹 표시
        self.unmapped_near = np.empty((0, 2))  # 사각형 밖, 로봇 2m 안의 지도에 없는 점 (odom) - 웹 표시
        self.hybrid_info = {}                # 지금 전환 상태 (hybrid_switch 가 채움) - 웹 표시
        self.odom_vw = (0.0, 0.0)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        # A* 가 기억 중인 장애물 (맵에 없는 LiDAR 장애물, 부딪힌 자리) → 웹 지도 표시
        self.memory = {'remembered': [], 'bumps': []}
        self.create_subscription(PoseArray, 'astar/remembered', lambda m: self.on_memory('remembered', m), 1)
        self.create_subscription(PoseArray, 'astar/bumps', lambda m: self.on_memory('bumps', m), 1)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')  # AMCL
        self.nomotion_left = 0
        self.create_timer(0.3, self.nomotion_step)
        # /cmd_vel 은 브리지만 내보낸다. 자동 이동은 self.cmd_vel_pub.publish(...) 로 선택기(CmdMux)에 넣고,
        # 수동 조작(웹 teleop, 외부 키보드 cmd_vel_teleop)·정지와 우선순위를 가려 20Hz 로 내보낸다
        self._cmd_out = self.create_publisher(Twist, 'cmd_vel', 1)
        self.mux = CmdMux()
        self.cmd_vel_pub = _AutoCmd(self.mux)
        self.create_subscription(Twist, 'cmd_vel_teleop', self.on_external_teleop, 10)
        self.create_timer(0.05, self.mux_step)
        self.bump_pub = self.create_publisher(PointStamped, 'astar/obstacle', 10)  # 부딪힌 자리 → A* 플래너
        self.clear_obstacles_client = self.create_client(Empty, 'astar/clear_obstacles')
        # 웹에서 그린 금지 구역 → A* 플래너 (JSON 목록). 플래너가 나중에 떠도 받도록 transient local
        self.zones_pub = self.create_publisher(String, 'astar/zones', map_qos)
        self.zones_json = None
        self.zone_replan_at = None   # 금지 구역이 바뀌어 경로를 다시 계산할 시각
        # A* 가 경로를 못 찾은 이유 (금지 구역 때문 등). 서비스 응답과 따로 오므로 받은 시각과 같이 둔다
        self.plan_fail = (0.0, '')
        self.create_subscription(String, 'astar/fail_reason',
                                 lambda m: setattr(self, 'plan_fail', (time.monotonic(), m.data)), 10)
        self.level_scan_pub = self.create_publisher(LaserScan, 'scan_level', qos_profile_sensor_data)
        # SLAM 전용: 아무것도 안 맞은 빔(inf)을 range_max 바로 아래 값으로 바꿔서 발행 (to_slam_scan 참고)
        self.slam_scan_pub = self.create_publisher(LaserScan, 'scan_slam', qos_profile_sensor_data)

        self.plan_client = self.create_client(GetPlan, 'astar/get_plan')

        # 현재 목표. generation은 취소된 목표의 늦은 콜백을 무시하기 위한 번호
        self.goal = None
        self.generation = 0
        self.last_battery_sent = 0.0
        self.battery_history = []  # (시각, 전압)
        self.stop_count = 0
        self.follower = None  # 추종기 상태 (pp / dwa, hybrid 전환)
        self.last_goal_id = None

        self.server = ServerConnection(p('server_url'), self.get_logger())
        self.create_timer(0.05, self.process_inbox)
        self.create_timer(1.0 / p('pose_rate'), self.send_pose)
        self.create_timer(0.1, self.publish_stop)
        # 웹 수동 조작 (teleop): 누르고 있는 동안 0.1초마다 명령이 온다. 0.4초 넘게 안 오면 멈춤
        self.teleop = None   # (v, w, 받은 시각)
        # (수동 조작 목표는 mux_step 에서 선택기로 넣음)
        self.create_timer(0.05, self.control_step)
        self.create_timer(1.0, self.send_health)
        self.create_timer(0.1, self.flush_map)
        # 실행 중 파라미터 변경 (ros2 param set, 웹 '주행 파라미터' 카드). 메모리에서만 바뀌고 재시작하면 params.yaml 값.
        # 시작 값을 기억해 웹에서 바뀐 것을 표시/되돌림. declare 가 모두 끝난 뒤에 등록 (declare 도 콜백을 부름)
        self.param_start = {n: self.get_parameter(n).value for n in self.tunable_params()}
        self.params_dirty = False
        self.add_on_set_parameters_callback(self.on_set_params)
        self.create_timer(0.2, self.flush_params)
        self.get_logger().info('브리지 시작 (A* + hybrid: pure pursuit / 장애물 근처 DWA)')

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
                    self.cargo_resend = self.cargo_sent is not None
                    self.server.send({'type': 'hello', 'mode': self.mode, 'session': self.session,
                                      'active_goal_id': self.goal['id'] if self.goal else None})
                    self.send_map()
                    self.params_dirty = True
                elif kind == 'goto':
                    self.start_goal(msg['goal_id'], float(msg['x']), float(msg['y']), float(msg['yaw']))
                elif kind == 'cancel':
                    self.cancel_goal()
                elif kind == 'stop':
                    self.teleop = None
                    self.cancel_goal()
                    self.stop_count = 10  # 1초 동안 정지 명령 발행
                elif kind == 'save_map':
                    self.save_map(str(msg['name']))
                elif kind == 'reset':
                    self.reset_robot()
                elif kind == '_relocalized':
                    self.on_relocalized(msg)
                elif kind == 'teleop':
                    self.on_teleop(float(msg.get('v', 0.0)), float(msg.get('w', 0.0)))
                elif kind == 'zones':
                    self.on_zones(msg.get('zones') or [])
                elif kind == 'param_set':
                    self.set_param_from_web(str(msg.get('name', '')), msg.get('value'))
                elif kind == 'yolo_set':
                    self.set_yolo(msg)
                elif kind == 'initial_pose':
                    self.publish_initial_pose(float(msg['x']), float(msg['y']), float(msg['yaw']),
                                              bool(msg.get('precise')))
                else:
                    self.get_logger().warn(f'알 수 없는 메시지: {kind}')
            except (KeyError, ValueError) as e:
                self.get_logger().error(f'잘못된 메시지 {msg}: {e}')

    def on_zones(self, zones):
        data = json.dumps(zones, sort_keys=True)
        if data == self.zones_json:
            return   # SLAM 맵 갱신마다 같은 목록이 다시 오므로 바뀔 때만 발행
        self.zones_json = data
        self.zones_pub.publish(String(data=data))
        self.get_logger().info(f'금지 구역 {len(zones)}개 → A* 플래너')
        # 이동 중이면 새 구역으로 경로를 다시 계산한다 (예전에는 출발 때 계산한 경로를 그대로 따라가 구역을 뚫고 지나감).
        # 플래너가 구역을 먼저 받도록 0.3초 뒤에 (control_step 에서)
        if self.goal is not None:
            self.zone_replan_at = time.monotonic() + 0.3

    # ---------------------------------------------------------- 로봇 상태 → 서버
    def on_map(self, msg):
        self.map_msg = msg
        self.update_map_dist(msg)
        if self.mode == 'slam':
            # SLAM 이 맵을 갱신하면(2초마다) 바로 보낸다. 짧은 간격으로 몰려 오면 MAP_MIN_INTERVAL 마다 최신 것만 (flush_map)
            # (예전: 3초에 한 번만 보내서 웹 지도가 최대 3초 늦었음)
            self.map_dirty = True
            self.flush_map()
            return
        self.get_logger().info(f'맵 수신 {msg.info.width}x{msg.info.height}')
        self.send_map()

    def flush_map(self):
        if self.map_dirty and time.monotonic() - self.last_map_sent >= MAP_MIN_INTERVAL:
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
            'footprint': [[round(v, 3) for v in r] for r in self.body_rects()],   # 웹 로봇 모양
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

    def report(self, status, message=''):
        """현재 목표의 결과를 서버에 알리고 목표를 비운다."""
        if self.goal is None:
            return
        self.get_logger().info(f'목표 {self.goal["id"]}: {status} {message}')
        self.server.send({'type': 'nav_status', 'goal_id': self.goal['id'],
                          'status': status, 'message': message})
        if status != 'navigating':
            self.goal = None
            self.send_path([])

    # ---------------------------------------------------------- 이동
    def start_goal(self, goal_id, x, y, yaw):
        self.cancel_goal(report=False)
        self.teleop = None   # 목표가 오면 수동 조작은 끝
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
        self.request_plan()

    def cancel_goal(self, report=True):
        self.generation += 1
        if self.follower is not None:
            self.follower = None
            self.stop_count = 5
        if report:
            self.report('canceled')
        self.goal = None

    def make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quaternion_from_yaw(yaw)
        return ps

    # 경로 계획 (A*) ------------------------------------------------
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
            # 실패 이유(astar/fail_reason)는 토픽으로 따로 오므로 잠깐 기다렸다가 붙인다
            asked = time.monotonic()

            def report_fail():
                self.destroy_timer(timer)   # 한 번만
                if gen != self.generation or self.goal is None:
                    return
                t, why = self.plan_fail
                self.report('failed', f'경로를 찾을 수 없습니다{f": {why}" if why and t >= asked - 1.0 else "."}')
            timer = self.create_timer(0.3, report_fail)
            return
        start = self.current_pose()
        if start is None:
            self.report('failed', '로봇 위치를 알 수 없습니다. 초기 위치를 지정하세요.')
            return
        self.send_path(plan.poses)
        self.stop_count = 0  # 이전 목표의 정지 명령이 새 주행에 섞이지 않게
        points = [(ps.pose.position.x, ps.pose.position.y) for ps in plan.poses]
        # pp / dwa 는 같은 경로 점 목록을 쓰므로 진행 index 를 서로 넘겨줄 수 있다 (hybrid 전환)
        dwa = DWAFollower(points, self.goal['yaw'], self.dwa_cfg,
                          goal_tolerance=self.get_parameter('dwa_follow.goal_tolerance').value,
                          lookahead=self.get_parameter('dwa_follow.lookahead').value)
        dwa.target_clear = self.get_parameter('dwa_follow.target_clear').value
        dwa.target_max = self.get_parameter('dwa_follow.target_max').value
        if self.rl_policy is not None:   # RL 이 DWA 자리에 들어가 처음부터 끝까지 주행 (막힘/복구 처리는 DWA 와 같음)
            dwa = RLFollower(points, self.goal['yaw'], self.rl_policy,
                             goal_tolerance=self.get_parameter('rl.goal_tolerance').value)
        self.follower = {
            'gen': gen,
            'blocked_since': None,
            'mark_pos': start[:2],
            'mark_time': time.monotonic(),
            'dwa': dwa,
            'pp': PurePursuit(points, self.goal['yaw'], max_v=self.max_speed, max_w=self.max_turn_speed,
                              max_acc=self.max_accel, max_acc_w=self.max_turn_accel),
            'use_dwa': self.rl_policy is not None,
            'clear_since': None,
        }

    def fail_or_retry(self, reason):
        """재시도 횟수가 남았으면 현재 위치에서 다시 계획, 아니면 실패 보고."""
        if self.goal['retries'] < self.max_retries:
            self.goal['retries'] += 1
            self.get_logger().warn(f'{reason} 재시도 {self.goal["retries"]}/{self.max_retries}')
            self.generation += 1
            self.follower = None
            self.request_plan()
        else:
            self.report('failed', reason)

    # 카메라 / 파라미터 ---------------------------------------------
    def on_cargo(self, msg):
        label = msg.data.strip()
        if self.cargo_raw is None or self.cargo_raw[0] != label:
            self.cargo_raw = (label, time.monotonic())

    # YOLO 검출기 ------------------------------------------------
    def on_yolo_counts(self, msg):
        self.yolo_counts = (msg.data.strip(), time.monotonic())
        self.send_yolo()

    def poll_yolo(self):
        """1초마다 검출기의 enabled 를 읽어 (노드가 있는지도 같이) 서버에 알린다."""
        if not self.yolo_get.service_is_ready():
            self.yolo_enabled = None
            self.send_yolo()
            return
        if self.yolo_pending is not None:
            fut, sent = self.yolo_pending
            if time.monotonic() - sent < 3.0:
                return
            # 답이 안 온 요청(통신 끊김, 검출기 바쁨)은 버리고 다시 묻는다. 안 그러면 영영 다시 묻지 않음
            self.yolo_get.remove_pending_request(fut)
        fut = self.yolo_get.call_async(GetParameters.Request(names=['enabled']))
        self.yolo_pending = (fut, time.monotonic())

        def done(f):
            if self.yolo_pending is None or self.yolo_pending[0] is not f:
                return   # 이미 버린 요청의 늦은 답
            self.yolo_pending = None
            r = f.result()
            if r is not None and r.values and r.values[0].type == ParameterType.PARAMETER_BOOL:
                self.yolo_enabled = r.values[0].bool_value
            self.send_yolo()
        fut.add_done_callback(done)

    # 실행 중에 바꾸면 바로 쓰이는 파라미터 → 브리지 속성 (나머지 tunable 은 쓸 때마다 get_parameter 로 읽음)
    LIVE_ATTRS = {'max_speed': 'max_speed', 'max_turn_speed': 'max_turn_speed', 'max_accel': 'max_accel',
                  'max_turn_accel': 'max_turn_accel', 'stop_distance': 'stop_distance',
                  'blocked_timeout': 'blocked_timeout', 'progress_timeout': 'progress_timeout',
                  'tf_timeout': 'tf_timeout', 'rotate_timeout': 'rotate_timeout', 'stall_timeout': 'stall_timeout',
                  'backoff_distance': 'backoff_distance',
                  'max_retries': 'max_retries', 'bump_tilt_deg': 'bump_tilt'}

    def tunable_params(self):
        """실행 중에 바꿀 수 있는 파라미터 이름 (웹 카드 순서). dwa.* 중 목록 값(footprint 등)은 ros2 param set 으로만."""
        names = list(self.LIVE_ATTRS)
        names += [f'dwa.{k}' for k, v in vars(self.dwa_cfg).items()
                  if k != 'control_period' and not isinstance(v, tuple)]
        names += ['dwa_follow.lookahead', 'dwa_follow.goal_tolerance', 'dwa_follow.moving_speed',
                  'dwa_follow.target_clear', 'dwa_follow.target_max']
        names += [f'hybrid.{k}' for k in ('box_front', 'box_back', 'box_half_width', 'box_min_points', 'box_min_scans',
                                          'moving_predict', 'release_time', 'rejoin_offset')]
        return names

    def on_set_params(self, params):
        """파라미터가 바뀌기 직전에 불림: 실행 중에 못 바꾸는 것은 거절하고, 나머지는 지금 주행에도 바로 반영."""
        for prm in params:
            n = prm.name
            if not (n in self.param_start or (n.startswith('dwa.') and n != 'dwa.control_period')):
                return SetParametersResult(successful=False, reason=f'{n} 은(는) 실행 중에 못 바꿉니다 (재시작 필요)')
            if n in ('dwa.v_samples', 'dwa.w_samples') and prm.value < 2:
                return SetParametersResult(successful=False, reason=f'{n} 은(는) 2 이상이어야 합니다')
        f = self.follower or {}
        for prm in params:
            n, v = prm.name, prm.value
            if n in self.LIVE_ATTRS:
                setattr(self, self.LIVE_ATTRS[n], math.radians(v) if n == 'bump_tilt_deg' else v)
            elif n.startswith('dwa.'):
                # DWAFollower 들이 같은 DWAConfig 객체를 쓰므로 주행 중인 DWA 에도 바로 적용
                k = n[4:]
                setattr(self.dwa_cfg, k, tuple(float(x) for x in v) if isinstance(getattr(self.dwa_cfg, k), tuple) else v)
            elif n in ('dwa_follow.lookahead', 'dwa_follow.goal_tolerance', 'dwa_follow.target_clear',
                       'dwa_follow.target_max') and 'dwa' in f:
                setattr(f['dwa'], n.split('.')[1], v)
            self.get_logger().info(f'파라미터 변경: {n} = {v}')
        if 'pp' in f:
            pp = f['pp']
            pp.max_v, pp.max_w, pp.max_acc, pp.max_acc_w = (self.max_speed, self.max_turn_speed,
                                                            self.max_accel, self.max_turn_accel)
        self.params_dirty = True
        return SetParametersResult(successful=True)

    def set_param_from_web(self, name, value):
        """웹에서 온 값: 선언된 타입으로 맞춰서 (JSON 은 2.0 을 2 로 보냄) set_parameters → 결과를 웹에 알림."""
        try:
            cur = self.get_parameter(name)
            t = cur.type_
            if t == Parameter.Type.DOUBLE:
                value = float(value)
            elif t == Parameter.Type.INTEGER:
                value = int(value)
            elif t == Parameter.Type.BOOL:
                value = bool(value)
            else:
                raise ValueError('숫자/참거짓 파라미터만 웹에서 바꿀 수 있습니다')
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('숫자가 아닙니다')
            r = self.set_parameters([Parameter(name, value=value)])[0]
            ok, reason = r.successful, r.reason
        except Exception as e:  # 선언 안 된 이름, 숫자 아님 등
            ok, reason = False, str(e) or type(e).__name__
        self.server.send({'type': 'param_result', 'name': name, 'ok': ok, 'reason': reason})

    def flush_params(self):
        """바뀐 파라미터 목록을 서버(웹 카드)로. ros2 param set 으로 바꾼 것도 여기로 들어옴."""
        if not self.params_dirty:
            return
        self.params_dirty = False
        self.server.send({'type': 'params', 'items': [
            {'name': n, 'value': self.get_parameter(n).value, 'start': s} for n, s in self.param_start.items()]})

    def set_yolo(self, msg):
        """웹 버튼: {'enabled': bool} 또는 {'capture': True} (검출 장면 한 장 저장)."""
        if not self.yolo_set.service_is_ready():
            self.get_logger().warn('YOLO 검출기 노드가 없습니다 (scripts/start_yolo.sh 로 실행)')
            self.yolo_enabled = None
            self.send_yolo()
            return
        params = []
        if 'enabled' in msg:
            params.append(ParamMsg(name='enabled', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=bool(msg['enabled']))))
        if msg.get('capture'):
            params.append(ParamMsg(name='capture_once', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=True)))
        if not params:
            return
        fut = self.yolo_set.call_async(SetParameters.Request(parameters=params))

        def done(f):
            r = f.result()
            ok = r is not None and all(x.successful for x in r.results)
            if ok and 'enabled' in msg:
                self.yolo_enabled = bool(msg['enabled'])
                if not self.yolo_enabled:
                    self.yolo_counts = ('', None)   # 끄면 이전 개수는 지움
            self.get_logger().info(f'YOLO 설정 {[p.name for p in params]} → {"성공" if ok else "실패"}')
            self.send_yolo()
        fut.add_done_callback(done)

    def send_yolo(self):
        text, at = self.yolo_counts
        age = None if at is None else round(time.monotonic() - at, 1)
        state = (self.yolo_enabled, text if age is not None and age < 3.0 else '')
        if state == getattr(self, 'yolo_sent', None) and age is not None and age < 3.0 and \
                time.monotonic() - getattr(self, 'yolo_sent_at', 0.0) < 0.5:
            return
        self.yolo_sent, self.yolo_sent_at = state, time.monotonic()
        self.server.send({'type': 'yolo', 'available': self.yolo_enabled is not None, 'enabled': self.yolo_enabled,
                          'counts': state[1], 'age': age})

    def flush_cargo(self):
        if self.cargo_raw is None:
            return
        label, since = self.cargo_raw
        if (label != self.cargo_sent or self.cargo_resend) and \
                time.monotonic() - since >= self.get_parameter('cargo_stable_sec').value:
            self.cargo_sent = label
            self.cargo_resend = False
            self.server.send({'type': 'cargo', 'label': label})
            self.get_logger().info(f'카메라 인식: {label or "없음"}')

    def on_scan(self, msg):
        if self.scan_noise is not None:
            r, amin, inc = degrade(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                   self.scan_noise_rng, self.scan_noise)
            noisy = LaserScan()
            noisy.header = msg.header
            noisy.angle_min, noisy.angle_increment = amin, inc
            noisy.angle_max = amin + (len(r) - 1) * inc
            noisy.time_increment = msg.time_increment * len(msg.ranges) / max(len(r), 1)
            noisy.scan_time, noisy.range_min, noisy.range_max = msg.scan_time, msg.range_min, msg.range_max
            noisy.ranges = r.astype(np.float32).tolist()
            msg = noisy
        self.scan = msg
        self.send_scan(msg)
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
            self.slam_scan_pub.publish(to_slam_scan(msg))
            self.update_tracks(msg)   # 기울어진 스캔은 바닥을 보므로 추적/DWA 에도 쓰지 않음

    def send_scan(self, msg):
        """웹 화면용 실시간 LiDAR: 빔 순서대로 지도 좌표 점 (응답 없는 빔은 None). 웹이 이웃 점끼리 선으로 잇는다.
        SCAN_SEND_INTERVAL 보다 자주는 안 보냄. 지도/경로 계획에는 쓰지 않음."""
        now = time.monotonic()
        if now - self.last_scan_sent < SCAN_SEND_INTERVAL or not self.server.connected:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, msg.header.frame_id, Time())
        except TransformException:
            return
        self.last_scan_sent = now
        r = np.asarray(msg.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= msg.range_min) & (r < msg.range_max)
        tx, ty = t.transform.translation.x, t.transform.translation.y
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment + yaw_from_quaternion(t.transform.rotation)
        rr = np.where(ok, r, 0.0)
        xs, ys = np.round(tx + rr * np.cos(ang), 3), np.round(ty + rr * np.sin(ang), 3)
        self.server.send({'type': 'scan', 'origin': [round(tx, 3), round(ty, 3)], 'inc': msg.angle_increment,
                          'points': [[float(x), float(y)] if k else None for x, y, k in zip(xs, ys, ok)]})

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
        if self.zone_replan_at is not None and time.monotonic() >= self.zone_replan_at:
            self.zone_replan_at = None
            if self.goal is not None:
                self.get_logger().info('금지 구역이 바뀌어 지금 위치에서 경로를 다시 계산합니다')
                self.generation += 1      # 지금 따르던 경로/계획 응답은 버림
                self.stop_following()
                self.request_plan()
                return
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
        if 'recovery' in f:
            self.recovery_step(f, now)
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
            why = 'TF 없음' if pose is None else f'TF 나이 {pose[3]:.2f}초'
            self.get_logger().warn(f'위치 TF 끊김: {why}, 기울기 {math.degrees(self.tilt_rel):.1f}°')
            self.report('failed', f'로봇 위치 정보가 갱신되지 않아 정지했습니다 ({why}).')
            return
        x, y, yaw, _ = pose
        if self.tilt_rel < math.radians(2):
            f['level_pose'] = (x, y, yaw)   # 부딪힌 자리는 기울기 시작 전 위치/방향으로 계산 (밀리면서 돌아가므로)
        if self.rl_policy is None:
            self.hybrid_switch(f, x, y, now)
        if f['use_dwa']:
            v, w, state = self.dwa_compute(f['dwa'])
        else:
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
        if state == 'blocked':
            # DWA: 안전한 궤적이 없음 (사람이 막고 있음 등) → 기다리다 오래 막히면 장애물을 피하는 경로로 재계획
            if f['blocked_since'] is None:
                f['blocked_since'] = now
                self.get_logger().warn('DWA: 안전한 궤적 없음, 정지 후 대기')
            if now - f['blocked_since'] > self.blocked_timeout:
                self.start_recovery(f, now, '앞이 막혀 진행할 수 없습니다.')
                return
            f['mark_time'] = now
            f.pop('drive_since', None)
            self.cmd_vel_pub.publish(Twist())
            return
        if v > 0.0 and not f['use_dwa']:   # DWA 는 장애물(움직이는 것 포함)을 스스로 피하므로 앞쪽 정지 규칙을 쓰지 않음
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
                f['pp'].set_speed(0.0, 0.0)   # 다시 갈 때 0 에서 가속
                self.cmd_vel_pub.publish(Twist())
                return
            f['blocked_since'] = None
        if state != 'blocked':
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
            if f['use_dwa']:
                self.start_recovery(f, now, '진행이 없습니다.')
                return
            self.stop_following()
            self.fail_or_retry('진행이 없습니다.')
            return

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_vel_pub.publish(cmd)

    # hybrid: 평소 pure pursuit, 장애물이 가까우면 DWA ---------------
    # DWA 만 쓰면 실물에서 벽 옆/좁은 곳에서 '안전한 궤적 없음', '진행 없음' 이 잦았다 → 지도에 있는 벽은
    # A* 경로 + pure pursuit 로 충분하므로, 지도에 없는 물체(추적 track: 상자, 사람)가 경로 가까이 올 때만 DWA
    def update_box(self, pts):
        """스캔마다: 로봇 앞(+뒤 조금) 사각형 안의 지도에 없는 점 수 → 연속 스캔 수."""
        pose = self.xyyaw('odom', self.base_frame)
        if pose is None or not len(pts):
            self.box_streak, self.box_count, self.box_near = 0, 0, math.inf
            self.box_pts = self.unmapped_near = np.empty((0, 2))
            self.send_box()
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        lx = c * (pts[:, 0] - x) + s * (pts[:, 1] - y)
        ly = -s * (pts[:, 0] - x) + c * (pts[:, 1] - y)
        x0, x1, _ = self.dwa_cfg.footprint if self.dwa_cfg.footprint else (-0.197, 0.069, 0.153)
        p = lambda n: self.get_parameter(f'hybrid.{n}').value
        inside = (lx >= x0 - p('box_back')) & (lx <= x1 + p('box_front')) & (np.abs(ly) <= p('box_half_width'))
        self.box_count = int(np.count_nonzero(inside))
        self.box_pts = pts[inside]
        self.unmapped_near = pts[~inside & (np.hypot(lx, ly) < 2.0)]
        if self.box_count >= p('box_min_points'):
            self.box_streak += 1
            gx = np.maximum(np.maximum(x0 - lx[inside], lx[inside] - x1), 0.0)
            self.box_near = float(np.min(np.hypot(gx, np.maximum(np.abs(ly[inside]) - 0.153, 0.0))))
        else:
            self.box_streak, self.box_near = 0, math.inf
        self.send_box()

    def send_box(self):
        """hybrid 전환 조건을 웹에 (0.2초마다, hybrid 일 때만, map 좌표):
        사각형 네 꼭짓점 + 안의 점(실제로 센 점) / 밖의 지도에 없는 점, 움직이는 물체 전환 반경, 지금 상태(이유, 복귀 대기)."""
        now = time.monotonic()
        if now - self.last_box_sent < 0.2:
            return
        pose = self.xyyaw(self.global_frame, self.base_frame)
        m_o = self.xyyaw(self.global_frame, 'odom')
        if pose is None or m_o is None:
            return
        self.last_box_sent = now
        p = lambda n: self.get_parameter(f'hybrid.{n}').value
        x0, x1, _ = self.dwa_cfg.footprint if self.dwa_cfg.footprint else (-0.197, 0.069, 0.153)
        bx0, bx1, hw = x0 - p('box_back'), x1 + p('box_front'), p('box_half_width')
        r3 = lambda a: [[round(float(u), 3), round(float(v), 3)] for u, v in a]
        corners = transform_points([[bx0, -hw], [bx1, -hw], [bx1, hw], [bx0, hw]], *pose)
        info = self.hybrid_info if self.follower else {}
        # 움직이는 track 의 예측 선 (지금 → moving_predict 초 뒤, map) 과 사각형에 들어오는 시각
        movers = []
        odom_pose = self.xyyaw('odom', self.base_frame)
        horizon = p('moving_predict')
        for t in self.tracks:
            if math.hypot(t.x[2], t.x[3]) < self.get_parameter('dwa_follow.moving_speed').value:
                continue
            a, b = transform_points([t.x[:2], [t.x[0] + t.x[2] * horizon, t.x[1] + t.x[3] * horizon]], *m_o)
            movers.append({'from': r3([a])[0], 'to': r3([b])[0], 'r': round(float(t.radius), 3),
                           'enter': self.moving_into_box(t, odom_pose)})
        self.server.send({
            'type': 'hybrid_box', 'corners': r3(corners),
            'points': r3(transform_points(self.box_pts[:80], *m_o)) if len(self.box_pts) else [],
            'others': r3(transform_points(self.unmapped_near[:150], *m_o)) if len(self.unmapped_near) else [],
            'count': self.box_count, 'min_points': p('box_min_points'),
            'active': self.box_streak >= p('box_min_scans'),
            'movers': movers, 'moving_predict': horizon,
            'driving': bool(info), 'dwa': bool(info.get('use_dwa')), 'why': info.get('why'),
            'clear_for': info.get('clear_for'), 'release_time': p('release_time'),
            'offset': info.get('offset'), 'rejoin_offset': p('rejoin_offset')})

    def box_bounds(self):
        """hybrid 사각형 (로봇 좌표 x_min, x_max, 반폭)."""
        x0, x1, _ = self.dwa_cfg.footprint if self.dwa_cfg.footprint else (-0.197, 0.069, 0.153)
        p = lambda n: self.get_parameter(f'hybrid.{n}').value
        return x0 - p('box_back'), x1 + p('box_front'), p('box_half_width')

    def moving_into_box(self, t, pose=None):
        """움직이는 track t (odom) 가 지금 ~ moving_predict 초 뒤 사이에 사각형과 겹치는 첫 시각 (s), 없으면 None.
        물체는 지금 속도로, 로봇은 지금 선속도로 직진한다고 본다 (사각형도 로봇과 같이 움직임)."""
        pose = pose or self.xyyaw('odom', self.base_frame)
        if pose is None:
            return None
        x, y, yaw = pose
        v = self.odom_vw[0]
        ts = np.arange(0.0, self.get_parameter('hybrid.moving_predict').value + 1e-9, 0.1)
        c, s = math.cos(yaw), math.sin(yaw)
        dx = t.x[0] + t.x[2] * ts - (x + v * c * ts)
        dy = t.x[1] + t.x[3] * ts - (y + v * s * ts)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        bx0, bx1, hw = self.box_bounds()
        gap = np.hypot(np.maximum(np.maximum(bx0 - lx, lx - bx1), 0.0), np.maximum(np.abs(ly) - hw, 0.0))
        hit = gap <= t.radius
        return float(ts[int(np.argmax(hit))]) if hit.any() else None

    def hybrid_obstacle(self, f, x, y):
        """DWA 로 피해야 할 장애물이 있으면 설명 글자, 없으면 None.
        사각형 안에 지도에 없는 점 (정적이든 동적이든), 또는 사각형에 곧 들어올 움직이는 track.
        멀어지거나 나란히 가는 사람은 무시하고, 옆/뒤에서 다가오는 사람은 미리 잡는다."""
        if self.box_streak >= self.get_parameter('hybrid.box_min_scans').value:
            return f'앞쪽 장애물 {self.box_near:.2f} m (점 {self.box_count}개)'
        if not self.tracks:
            return None
        pose = self.xyyaw('odom', self.base_frame)
        if pose is None:
            return None
        moving_v = self.get_parameter('dwa_follow.moving_speed').value
        for t in self.tracks:
            if math.hypot(t.x[2], t.x[3]) < moving_v:
                continue
            when = self.moving_into_box(t, pose)
            if when is not None:
                d = math.hypot(t.x[0] - pose[0], t.x[1] - pose[1]) - t.radius
                return f'움직이는 장애물 {when:.1f}초 뒤 사각형 ({d:.2f} m)'
        return None

    def hybrid_switch(self, f, x, y, now):
        why = self.hybrid_obstacle(f, x, y)
        pp, dwa = f['pp'], f['dwa']
        self.hybrid_info = {'use_dwa': f['use_dwa'], 'why': why}   # 웹 표시용 (send_box)
        if not f['use_dwa']:
            if why and not pp.at_goal:
                dwa.progress, dwa.turning, dwa.sidestep, dwa.last_cmd = pp.index, False, None, None
                f['use_dwa'], f['clear_since'] = True, None
                self.hybrid_info['use_dwa'] = True
                self.get_logger().info(f'{why} → DWA 로 피함')
            return
        # 경로 가까이 돌아왔을 때만 (멀면 pure pursuit 가 off_path 로 실패하므로 DWA 가 경로로 데려올 때까지)
        rest = np.asarray(pp.points[dwa.progress:dwa.progress + 60], dtype=float)
        offset = float(np.min(np.hypot(rest[:, 0] - x, rest[:, 1] - y)))
        self.hybrid_info['offset'] = round(offset, 3)
        if why or dwa.arrived:
            f['clear_since'] = None
            return
        f['clear_since'] = f['clear_since'] or now
        self.hybrid_info['clear_for'] = round(now - f['clear_since'], 1)
        if now - f['clear_since'] < self.get_parameter('hybrid.release_time').value:
            return
        if offset < self.get_parameter('hybrid.rejoin_offset').value:
            pp.index = dwa.progress
            pp.set_speed(*self.odom_vw)   # DWA 로 달리던 속도에서 이어서 (0 으로 뚝 떨어지지 않게)
            f['use_dwa'] = False
            self.hybrid_info = {'use_dwa': False, 'why': None}
            self.get_logger().info(f'장애물 지나감 (경로에서 {offset:.2f} m) → pure pursuit')

    # 움직이는 장애물 추적 + DWA -----------------------------------
    def on_odom(self, msg):
        self.odom_vw = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def xyyaw(self, target, source, stamp=None):
        try:
            t = self.tf_buffer.lookup_transform(target, source, stamp if stamp is not None else Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)

    def update_tracks(self, scan):
        """스캔 → odom 점 (DWA 정적 장애물) + 움직이는 장애물 추적."""
        c = self.track_cfg
        sensor = self.xyyaw('odom', scan.header.frame_id, Time.from_msg(scan.header.stamp)) \
            or self.xyyaw('odom', scan.header.frame_id)
        if sensor is None:
            return
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= min(scan.range_max, c['max_range']))
        pts = transform_points(np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]), *sensor)
        self.scan_odom = pts
        # 지도에 없는 물체의 스캔 점 (odom): DWA 목표점을 장애물 너머로 옮길 때, hybrid 앞 사각형 전환. 지도와 못 맞추면 비움
        self.scan_unmapped = pts[:0]
        if not self.tracking:
            return
        # 지도 벽 근처 점은 정적이므로 추적하지 않는다 (판별에만 map 을 쓰므로 AMCL 이 몇 cm 틀려도 속도에는 영향 없음).
        # 맵 밖/벽 너머/미탐색 칸의 점도 뺀다: 지도의 빈 칸에 있는 점만 추적 (SLAM 도 같음).
        # SLAM 중에는 아직 지도에 안 들어간 벽이 미탐색 칸에 있어 전부 '장애물' 로 잡혔고(실물 350건 중 345건),
        # 그 속도 노이즈(중앙값 0.06 m/s)가 움직이는 물체 기준을 넘었음. 미탐색 칸의 물체도 DWA 는 스캔 점으로 피한다
        if self.map_dist is None:
            pts = pts[:0]   # 지도가 아직 없으면 추적하지 않음
        if self.map_dist is not None and len(pts):
            m_o = self.xyyaw(self.global_frame, 'odom')
            if m_o is not None:
                dist, info, grid = self.map_dist
                pm = transform_points(pts, *m_o)
                col = np.floor((pm[:, 0] - info.origin.position.x) / info.resolution).astype(int)
                row = np.floor((pm[:, 1] - info.origin.position.y) / info.resolution).astype(int)
                inside = (row >= 0) & (row < info.height) & (col >= 0) & (col < info.width)
                keep = np.zeros(len(pts), dtype=bool)
                cell = grid[row[inside], col[inside]]
                free = (cell >= 0) & (cell < 65)
                keep[inside] = free & (dist[row[inside], col[inside]] > c['static_match_dist'])
                pts = pts[keep]
                self.scan_unmapped = pts
        self.update_box(self.scan_unmapped)
        dets = detect_clusters(pts, c['cluster_gap'], c['min_points'], c['max_diameter'], c['min_radius'],
                               sensor[:2])
        self.tracks = self.tracker.update(dets, Time.from_msg(scan.header.stamp).nanoseconds * 1e-9)
        self.send_obstacles()

    def on_memory(self, kind, msg):
        pts = [[round(p.position.x, 3), round(p.position.y, 3)] for p in msg.poses]
        if pts == self.memory[kind]:
            return
        self.memory[kind] = pts
        self.server.send({'type': 'memory', **self.memory})

    def update_map_dist(self, msg):
        if msg.info.width == 0 or not self.tracking:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        # 미탐색 칸은 벽이 아니다 → 그 안의 점은 움직이는 장애물 후보로 남긴다
        self.map_dist = (obstacle_distance(grid >= 65, msg.info.resolution, 0.5), msg.info, grid)

    def send_obstacles(self):
        """추적 중인 장애물을 웹에 (map 좌표, 0.25초마다)."""
        now = time.monotonic()
        if now - self.last_obstacles_sent < 0.25:
            return
        self.last_obstacles_sent = now
        m_o = self.xyyaw(self.global_frame, 'odom')
        if m_o is None:
            return
        c, s = math.cos(m_o[2]), math.sin(m_o[2])
        out = []
        moving = self.get_parameter('dwa_follow.moving_speed').value
        for t in self.tracks:
            # 추적은 지도의 빈 칸에서만 하므로 멈춘 track 은 '지도에 없는 멈춘 물체' (상자, 서 있는 사람 등)
            (x, y), = transform_points([t.x[:2]], *m_o)
            out.append({'id': t.id, 'x': round(float(x), 3), 'y': round(float(y), 3),
                        'moving': bool(math.hypot(t.x[2], t.x[3]) >= moving),
                        'vx': round(float(c * t.x[2] - s * t.x[3]), 3), 'vy': round(float(s * t.x[2] + c * t.x[3]), 3),
                        'r': round(float(t.radius), 3)})
        self.server.send({'type': 'obstacles', 'obstacles': out})

    def dwa_compute(self, follower):
        # DWA 는 0.1초마다만 계산하고 그 사이에는 같은 명령 (브리지 한 스레드에서 TF 수신 등과 같이 돌기 때문)
        now = time.monotonic()
        last = getattr(follower, 'last_cmd', None)
        if last is not None and now - last[0] < 0.1:
            return last[1]
        pose = self.xyyaw('odom', self.base_frame)
        map_to_odom = self.xyyaw('odom', self.global_frame)
        if pose is None or map_to_odom is None:
            return 0.0, 0.0, 'blocked'
        dyn = np.array([[t.x[0], t.x[1], t.x[2], t.x[3], t.radius] for t in self.tracks]).reshape(-1, 5)
        result = follower.compute(pose, *self.odom_vw, map_to_odom, self.scan_odom, dyn,
                                  unmapped_points=self.scan_unmapped,
                                  moving_speed=self.get_parameter('dwa_follow.moving_speed').value)
        follower.last_cmd = (now, result)
        self.send_dwa(follower, result)
        return result

    def send_dwa(self, follower, result):
        """DWA 가 고른 궤적(best)을 웹에 (map 좌표, 0.2초마다). 제자리 회전/막힘이면 빈 궤적."""
        now = time.monotonic()
        if now - self.last_dwa_sent < 0.2:
            return
        self.last_dwa_sent = now
        traj = follower.last_trajectory
        m_o = self.xyyaw(self.global_frame, 'odom')
        pts = []
        if traj is not None and m_o is not None:
            pts = [[round(float(px), 3), round(float(py), 3)] for px, py in transform_points(np.asarray(traj)[:, :2], *m_o)]
        v, w, state = result
        self.server.send({'type': 'dwa', 'traj': pts, 'v': round(float(v), 3), 'w': round(float(w), 3), 'state': state})

    # DWA 막힘 대처 (단순 fallback) --------------------------------
    # ① DWA 가 막히면 blocked_timeout(5초) 동안 기다림 (움직이는 장애물이 지나가길)
    # ② 그래도 막히거나 진행이 없으면: 뒤가 비었으면 15cm 물러나고, 뒤도 막혔으면 가장 트인 쪽으로 제자리 회전
    # ③ A* 로 다시 계획 (fail_or_retry, 최대 max_retries 번)  ④ 그래도 안 되면 실패 보고
    def start_recovery(self, f, now, reason):
        odom = self.xyyaw('odom', self.base_frame)
        if odom is None:
            self.stop_following()
            self.fail_or_retry(reason)
            return
        if self.front_clearance(math.radians(30), center=math.pi) > 0.35:
            rec = {'mode': 'back', 'speed': -0.05}
            what = '뒤로 15cm 물러난 뒤'
        elif not self.rotation_clear(odom):
            # 벽에 바짝 붙어 돌 공간이 없음: 돌면 차체 모서리가 벽에 걸려 올라타고 넘어졌다 (sim 강당)
            if self.front_clearance(math.radians(30)) > 0.35:
                rec = {'mode': 'forward', 'speed': 0.05}
                what = '돌 공간이 없어 앞으로 15cm 나온 뒤'
            else:
                self.stop_following()
                self.fail_or_retry(f'{reason} 앞뒤가 막혀 돌 공간도 없습니다.')
                return
        else:
            # 가장 트인 방향 (30° 칸마다 가장 가까운 물체 거리 중 가장 먼 쪽)
            best = max(range(-150, 181, 30), key=lambda d: self.front_clearance(math.radians(15), center=math.radians(d)))
            turn = math.radians(max(-90, min(90, best)) if abs(best) <= 90 else math.copysign(90, best))
            rec = {'mode': 'turn', 'target': turn}
            what = f'트인 쪽({best}°)으로 돌아선 뒤'
        rec.update(start=odom, until=now + 5.0, reason=reason)
        f['recovery'] = rec
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().warn(f'{reason} {what} 다시 계획합니다')

    def rotation_clear(self, odom):
        """제자리에서 돌아도 차체가 닿지 않는지: 회전 중심에서 차체 끝까지 반경(외곽 모서리) + 여유 안에 스캔 점이 없음."""
        pts = self.scan_odom
        if not len(pts):
            return True
        radius = self.body_radius() + 0.05
        return float(np.min(np.hypot(pts[:, 0] - odom[0], pts[:, 1] - odom[1]))) > radius

    def body_rects(self):
        """차체 + 붙인 물건 사각형 목록 (x_min, x_max, y_min, y_max), base_footprint 기준."""
        x0, x1, hw = self.dwa_cfg.footprint if self.dwa_cfg.footprint else (-0.197, 0.069, 0.153)
        return [(x0, x1, -hw, hw)] + self.footprint_extra

    def body_radius(self):
        """회전 중심(base_footprint)에서 가장 먼 차체 끝까지 거리 = 제자리에서 돌 때 쓸고 지나가는 반경."""
        return max(math.hypot(x, y) for a, b, c, d in self.body_rects() for x in (a, b) for y in (c, d))

    def rear_extent(self):
        """차체(붙인 물건 포함) 뒤 끝 x (음수)."""
        return min(r[0] for r in self.body_rects())

    def recovery_step(self, f, now):
        r = f['recovery']
        odom = self.xyyaw('odom', self.base_frame)
        done = odom is None or now > r['until']
        cmd = Twist()
        if not done and r['mode'] in ('back', 'forward'):
            blocked = (self.front_clearance(math.radians(30), center=math.pi) < 0.3 if r['mode'] == 'back'
                       else self.front_clearance(math.radians(30)) < 0.3)
            done = math.hypot(odom[0] - r['start'][0], odom[1] - r['start'][1]) >= 0.15 or blocked
            cmd.linear.x = r['speed']
        elif not done:
            turned = math.atan2(math.sin(odom[2] - r['start'][2]), math.cos(odom[2] - r['start'][2]))
            done = abs(turned) >= abs(r['target']) - 0.05
            cmd.angular.z = math.copysign(0.6, r['target'])
        if done:
            self.stop_following()
            self.fail_or_retry(r['reason'])
            return
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
        # LiDAR 는 차체 뒤 끝에서 0.13 m 앞에 있으므로 뒤쪽 0.3 m 안에 물체가 있으면 더 물러나지 않음 (앞으로 나올 때는 0.25 m).
        # 뒤에 붙인 물건이 더 나온 만큼 늘림 (기본 8cm → 0.38 m)
        if b['speed'] < 0:
            blocked = self.front_clearance(math.radians(30), center=math.pi) < 0.3 + max(0.0, -0.197 - self.rear_extent())
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

    @staticmethod
    def run_session():
        """SLAM 실행 번호 = run.sh 가 시작한 시각 (logs/run_mode.env 첫 줄). 브리지만 다시 떠도 같은 값이라
        서버가 SLAM 지도/위치를 지우지 않는다 (예전에는 브리지 실행마다 달라서 브리지 재시작만으로 위치가 지워졌음)."""
        path = os.path.join(os.environ.get('WAREHOUSE_HOME', ''), 'logs', 'run_mode.env')
        try:
            with open(path, encoding='utf-8') as f:
                first = f.readline().strip()
            if first.startswith('#') and '(' in first:
                return 'run-' + first[first.rindex('(') + 1:].rstrip(')')
        except OSError:
            pass
        return f'{int(time.time())}-{os.getpid()}'

    def on_teleop(self, v, w):
        """웹 수동 조작 (서버가 작업 중에는 막는다)."""
        if self.goal is not None or self.follower is not None:
            return
        v = max(-0.1, min(0.2, v))
        w = max(-1.2, min(1.2, w))
        self.teleop = (v, w, time.monotonic())

    def on_external_teleop(self, msg):
        """외부 키보드 조작 (ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop).
        키보드 프로그램은 키를 안 눌러도 0 을 계속 보내므로, 0 이 아닌 명령이 올 때만 수동 조작으로 본다
        (0 은 이미 수동 조작 중일 때만 = 멈춤). 자동 이동 중에 움직이면 자동 이동을 멈춘다 (수동이 우선)."""
        v, w = msg.linear.x, msg.angular.z
        moving = abs(v) > 1e-3 or abs(w) > 1e-3
        if not moving and self.teleop is None:
            return
        if moving and (self.goal is not None or self.follower is not None):
            self.get_logger().warn('외부 수동 조작(cmd_vel_teleop)이 들어와 자동 이동을 멈춥니다')
            self.cancel_goal(report=False)
            self.server.send({'type': 'nav_status', 'goal_id': self.last_goal_id, 'status': 'failed',
                              'message': '수동 조작(키보드)으로 중단했습니다.'})
        self.on_teleop(v, w)

    def teleop_step(self, now):
        """수동 조작 목표를 선택기에 넣는다 (안전 확인 후). 0.4초 넘게 명령이 없으면 선택기가 부드럽게 멈춘다."""
        t = self.teleop
        if t is None:
            return
        v, w, at = t
        if now - at > 0.4 or self.goal is not None or self.tipped:
            self.teleop = None
            return
        # 앞뒤에 물체가 가까우면 그쪽으로는 가지 않는다 (회전은 허용). LiDAR 기준 앞 0.25m / 뒤 0.3m
        if v > 0 and self.front_clearance(math.radians(30)) < 0.25:
            v = 0.0
        if v < 0 and self.front_clearance(math.radians(30), center=math.pi) < 0.3:
            v = 0.0
        self.mux.manual(v, w, now)

    def mux_step(self):
        now = time.monotonic()
        self.teleop_step(now)
        out = self.mux.step(now)
        if out is None:
            return
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = out
        self._cmd_out.publish(cmd)

    def publish_stop(self):
        if self.stop_count > 0:
            self.stop_count -= 1
            self.mux.stop(time.monotonic(), hold=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.close()
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
