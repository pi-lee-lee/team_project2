"""Bridge node connecting the Django server (WebSocket) and ROS 2.

Server → robot: goto, cancel, stop, initial_pose
Robot → server: hello, map, pose, battery, path, nav_status

Driving: plan a path with the astar_planner node, then follow it with the Frenet local planner (frenet.py).
  Using the A* path as the reference line, every 0.1 s pick the safe, lowest-cost trajectory among 'lateral offset x speed'
  candidates and execute its beginning.
  Blocked by static obstacles only → A* replan after 1 s; otherwise wait, then back off / replan.

Safety: stop when the pose TF is lost, no forward motion within estop_distance ahead, back off after a bump (tilt/stuck),
no progress / blocked too long → back off and replan, or fail.
"""
import base64
import gzip
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
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .astar import obstacle_distance
from .cmd_mux import CmdMux
from .astar_planner_node import quaternion_from_yaw, yaw_from_quaternion
from .frenet import FrenetConfig, FrenetFollower, transform_points
from .tracker import MultiObjectTracker, detect_clusters


# LiPo cell voltage → remaining capacity (%) (typical discharge curve)
LIPO_CURVE = [(3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25), (3.77, 30),
              (3.79, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55), (3.87, 60), (3.91, 65),
              (3.95, 70), (3.98, 75), (4.02, 80), (4.08, 85), (4.11, 90), (4.15, 95), (4.20, 100)]


def lipo_capacity(cell_voltage):
    """Remaining capacity (0-100%) by linear interpolation of the cell voltage on LIPO_CURVE."""
    if cell_voltage <= LIPO_CURVE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(LIPO_CURVE, LIPO_CURVE[1:]):
        if cell_voltage <= v1:
            return p0 + (p1 - p0) * (cell_voltage - v0) / (v1 - v0)
    return 100.0


def battery_percent(voltage, cells=3, empty_voltage=11.0):
    """Pack voltage → % for display. empty_voltage counts as 0% (TurtleBot3 warns to charge below 11 V).
    Above that, rescale to 0-100% keeping the shape of the LiPo discharge curve."""
    lo = lipo_capacity(empty_voltage / cells)
    cap = lipo_capacity(voltage / cells)
    return max(0.0, min(100.0, (cap - lo) / (100.0 - lo) * 100.0))


NO_RETURN_MARGIN = 0.05  # m. inf beams are replaced with range_max minus this (slam_toolbox max_laser_range must be smaller than that)


SCAN_SEND_INTERVAL = 0.15  # s, minimum interval for sending the live LiDAR to the web (display only). With a 5 Hz LiDAR this sends every scan
MAP_MIN_INTERVAL = 0.5  # s, minimum interval for sending the SLAM map to the server (sent as soon as updated, throttled to this if updates bunch up)
SLAM_BEAMS = 240  # beam count of scans passed to slam_toolbox (fixed 1.5° spacing). Close to the real LDS-02's beam count so few bins are empty


def _json_default(o):
    """For drive records: numpy values to JSON (floats rounded to 4 decimals)."""
    if isinstance(o, np.ndarray):
        return np.round(o, 4).tolist() if o.dtype.kind == 'f' else o.tolist()
    if isinstance(o, np.floating):
        return round(float(o), 4)
    if isinstance(o, (np.integer, np.bool_)):
        return o.item()
    raise TypeError(f'{type(o).__name__} 은(는) 기록할 수 없습니다')


def to_slam_scan(msg):
    """Convert a scan for slam_toolbox.
    1) The real LDS-02's beam count varies from scan to scan, but slam_toolbox drops every scan whose count differs from the
       first one it registered, so the map would stop growing.
       → Re-bin 0-360° into 240 bins of 1.5° so the count is always the same (nearest value when several beams fall in a bin).
    2) Karto drops beams at or beyond range_max (incl. inf) → open areas with nothing within range are never painted free
       and stay unexplored. Replacing inf with a value just below range_max makes Karto paint free space up to
       max_laser_range without marking the endpoint as an obstacle. AMCL/A* memory would treat that value as an obstacle,
       so they use scan_level (the original) as is.
    The real LDS reports no-return as 0 or NaN; 0 can't be told apart from 'too close', so it is left unchanged (inf only)."""
    r = np.asarray(msg.ranges, dtype=np.float32).copy()
    r[np.isposinf(r)] = msg.range_max - NO_RETURN_MARGIN
    inc = 2.0 * math.pi / SLAM_BEAMS
    ang = (msg.angle_min + np.arange(len(r)) * msg.angle_increment) % (2.0 * math.pi)
    idx = np.round(ang / inc).astype(int) % SLAM_BEAMS
    out_r = np.full(SLAM_BEAMS, np.nan, dtype=np.float32)
    valid = np.isfinite(r)
    # Nearest value per bin (sorted, written from the back, so the last write = the smallest value)
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
    """Keeps the WebSocket connection alive in a separate thread. Received messages go into the inbox queue."""

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
        except Exception as e:  # if the connection is closing, it is resent on the next reconnect
            self.logger.debug(f'전송 실패: {e}')

    def close(self):
        self.running = False
        if self.ws:
            self.ws.close()


class _AutoCmd:
    """Entry point that feeds automatic motion commands into the selector. Lets code keep calling self.cmd_vel_pub.publish(Twist)."""

    def __init__(self, mux):
        self.mux = mux
        self.last = None   # (v, w, time) last automatic command issued (for drive records)

    def publish(self, msg):
        now = time.monotonic()
        self.last = (msg.linear.x, msg.angular.z, now)
        self.mux.auto(msg.linear.x, msg.angular.z, now)


class BridgeNode(Node):
    def __init__(self):
        super().__init__('warehouse_bridge')
        self.declare_parameter('server_url', 'ws://localhost:8000/ws/robot/tb3/')
        self.declare_parameter('map_name', 'default')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_rate', 5.0)
        self.declare_parameter('max_retries', 2)   # number of replans when a move fails
        self.declare_parameter('stop_distance', 0.35)      # if the front ±25° within this distance is blocked while the body is tilted, treat it as 'hung up' and back off (m, from LiDAR)
        self.declare_parameter('estop_distance', 0.2)      # no forward motion if anything is within this distance in the front ±25° (emergency stop, m, from LiDAR)
        self.declare_parameter('blocked_timeout', 10.0)    # replan/fail if blocked longer than this (s)
        self.declare_parameter('progress_timeout', 15.0)   # fail if not even 5 cm of progress in this time (s)
        self.declare_parameter('tf_timeout', 0.5)          # stop immediately if the pose (odometry) is older than this (s)
        self.declare_parameter('pose_lost_timeout', 3.0)   # fail the goal if the pose keeps not arriving for longer than this (until then, stand and wait, then continue, s)
        self.declare_parameter('rotate_timeout', 15.0)     # stop and replan/fail if turning in place takes longer than this (s)
        # Bump detection (obstacles lower than the LiDAR, etc.): if the tilt exceeds this angle, or the robot moves forward but
        # makes less than 3 cm in this time, stop and back off, then report the bump spot to the planner as an obstacle and replan
        self.declare_parameter('bump_tilt_deg', 6.0)
        self.declare_parameter('stall_timeout', 2.5)       # s
        self.declare_parameter('backoff_distance', 0.15)   # m
        self.declare_parameter('battery_cells', 3)         # LiPo cell count (TurtleBot3: 3 cells, 11.1 V)
        self.declare_parameter('battery_empty_v', 11.0)    # voltage counted as 0% (TurtleBot3 low-battery warning voltage)
        self.declare_parameter('mode', 'map')              # map: saved map + AMCL / slam: building a map

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.global_frame = p('global_frame')
        self.base_frame = p('base_frame')
        self.max_retries = p('max_retries')
        self.stop_distance = p('stop_distance')
        self.estop_distance = p('estop_distance')
        self.blocked_timeout = p('blocked_timeout')
        self.progress_timeout = p('progress_timeout')
        self.tf_timeout = p('tf_timeout')
        self.pose_lost_timeout = p('pose_lost_timeout')
        self.battery_cells = p('battery_cells')
        self.battery_empty_v = p('battery_empty_v')
        self.rotate_timeout = p('rotate_timeout')
        self.bump_tilt = math.radians(p('bump_tilt_deg'))
        self.stall_timeout = p('stall_timeout')
        self.backoff_distance = p('backoff_distance')
        self.mode = p('mode')
        # Run ID of this bridge. The server uses it to notice a new SLAM run and clear the old SLAM map/pose (different frame)
        self.session = self.run_session()
        self.map_dirty = False      # SLAM mode: a new map is pending for the server
        self.last_map_sent = 0.0
        self.script_running = None  # name of the scripts/*.sh currently running

        self.tf_buffer = Buffer()
        # TF is received in a separate node + thread. This node handles IMU (~200 Hz), odom, scan and timers in one thread, and
        # the executor takes only one message per subscription per spin, so under load /tf (~100 Hz) would queue up, lag
        # beyond the timeout and trigger a 'pose TF lost' stop.
        # use_global_arguments=False: otherwise the launch '__node:=warehouse_bridge' applies to this node too and
        # /warehouse_bridge shows up twice in the node list (as if two bridges were running)
        # With multiple robots (namespaced), only that robot's TF (/<ns>/tf) is received. This node gets no launch args, so pass it explicitly
        ns = self.get_namespace()
        tf_args = [] if ns == '/' else ['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static']
        self.tf_node = rclpy.create_node('warehouse_bridge_tf', namespace=ns, use_global_arguments=False, cli_args=tf_args,
                                         parameter_overrides=[rclpy.parameter.Parameter(
                                             'use_sim_time', value=self.get_parameter('use_sim_time').value)])
        TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)   # the subscription callback holds the listener

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_msg = None
        self.create_subscription(OccupancyGrid, p('map_topic'), self.on_map, map_qos)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, qos_profile_sensor_data)
        self.scan = None
        self.last_scan_sent = 0.0   # web live LiDAR (send_scan)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.tilt = 0.0           # tilt computed from the IMU (rad)
        # Usual tilt (IMU offset, sloped floor): a slowly following average. tilt_rel = how much more tilted than usual
        self.tilt_base = None
        self.tilt_rel = 0.0
        self.last_imu_time = None
        self.scan_gated_since = None  # time we started withholding scans from AMCL/SLAM because of tilt
        self.lift_dir = 0.0           # direction of the lifted side (robot frame, rad, 0 = front). The side where the obstacle is
        self.tipped_since = None  # time the tipped-over state started
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        # Camera object (LED) detection: a detector node (cargo_detector or a custom YOLO node) publishes the visible label.
        # '' = nothing. To avoid flicker, the server is notified only when it stays the same for cargo_stable_sec
        self.declare_parameter('cargo_stable_sec', 1.0)
        self.cargo_raw = None       # (label, first seen time)
        self.cargo_sent = None      # label last reported to the server (None = no detector node)
        self.cargo_resend = False   # re-report the current label after reconnecting to the server
        self.create_subscription(String, 'cargo/detected', self.on_cargo, 10)
        self.create_timer(0.2, self.flush_cargo)
        # YOLO component detector (yolo/yolo_training/infer_ros_switch.py): a web button toggles enabled (inference + publishing
        # the detection image), and the per-class count text (/yolo/counts, about every 1 s) is shown on the web. Shown as 'none' if no detector node
        # Relative names: without a namespace /component_detector_switch, /yolo/counts (with multiple robots /<ns>/...)
        self.declare_parameter('yolo_node', 'component_detector_switch')
        self.declare_parameter('yolo_counts_topic', 'yolo/counts')
        yn = self.get_parameter('yolo_node').value.rstrip('/')
        self.yolo_get = self.create_client(GetParameters, f'{yn}/get_parameters')
        self.yolo_set = self.create_client(SetParameters, f'{yn}/set_parameters')
        self.yolo_enabled = None     # None = unknown (no detector)
        self.yolo_counts = ('', None)   # (text, time received)
        self.yolo_pending = None     # pending enabled query awaiting a reply (future, time sent)
        self.create_subscription(String, self.get_parameter('yolo_counts_topic').value, self.on_yolo_counts, 10)
        self.create_timer(1.0, self.poll_yolo)

        # Moving obstacle tracking (tb3_nav's obstacle_tracker moved into the bridge): scan → odom points → drop points near map
        # walls → clusters → Kalman filter for position/velocity. Frenet uses it to avoid future positions; shown on the web map
        tp = lambda n, d: self.declare_parameter(f'tracking.{n}', d).value  # noqa: E731
        self.tracking = tp('enabled', True)
        self.track_cfg = dict(max_range=tp('max_range', 5.0), static_match_dist=tp('static_match_dist', 0.15),
                              cluster_gap=tp('cluster_gap', 0.15), min_points=tp('min_points', 3),
                              max_diameter=tp('max_diameter', 0.8), min_radius=tp('min_radius', 0.05))
        self.tracker = MultiObjectTracker(gate=tp('gate', 0.5), min_hits=tp('min_hits', 3),
                                          max_misses=tp('max_misses', 5), accel_noise=tp('accel_noise', 1.0),
                                          meas_noise=tp('meas_noise', 0.05), max_speed=tp('max_speed', 1.5))
        # Only tracks faster than this are predicted/shown as moving objects (above the speed noise of stopped objects, below a slow walker)
        self.declare_parameter('tracking.moving_speed', 0.08)
        self.tracks = []                   # confirmed tracks (odom)
        self.scan_odom = np.empty((0, 2))  # recent scan points (odom) - Frenet static obstacles
        self.scan_unmapped = np.empty((0, 2))   # those not in the map (for records)
        self.map_dist = None               # distance to map walls (points near walls = static) (dist, msg)
        self.last_obstacles_sent = 0.0
        self.last_plan_sent = 0.0
        # Body rectangle (relative to base_footprint: x_min, x_max, half-width): Waffle Pi outline
        fp = tuple(float(v) for v in self.declare_parameter('footprint', [-0.197, 0.069, 0.153]).value)
        # Objects attached to the body (rectangles relative to base_footprint, x_min, x_max, y_min, y_max concatenated in groups of 4).
        # Default: 8 cm x 8 cm at the rear center (sticks out 8 cm past the body's rear end x=-0.197). Used for Frenet collision checks, back-off/in-place turn clearance, robot shape on the web
        ex = [float(v) for v in self.declare_parameter('footprint_extra', [-0.277, -0.197, -0.04, 0.04]).value]
        self.footprint_extra = [tuple(ex[i:i + 4]) for i in range(0, len(ex) - 3, 4)]
        # Frenet local planner settings (frenet.*: all but list values can be changed while running)
        fdef = FrenetConfig(footprint=fp, footprint_extra=tuple(v for r in self.footprint_extra for v in r))
        self.frenet_cfg = FrenetConfig(**{
            k: (tuple(float(x) for x in self.declare_parameter(f'frenet.{k}', [float(x) for x in v]).value)
                if isinstance(v, tuple) else type(v)(self.declare_parameter(f'frenet.{k}', v).value))
            for k, v in vars(fdef).items()})
        if self.mode == 'slam':   # turn slowly while building a map (fast turns warp the scan and worsen SLAM matching)
            self.frenet_cfg.rotate_speed = min(self.frenet_cfg.rotate_speed, 0.6)
        self.odom_vw = (0.0, 0.0)
        # Robot pose (odom frame) comes from /odom messages instead of /tf: /tf is reliable (retransmitting), so on Wi-Fi one lost
        # packet stalls the following data until retransmission, while /odom (best effort) recovers much sooner
        self.odom_msg = None   # (x, y, yaw, stamp Time)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        # For records: wheel encoder velocities (rad/s) and IMU angular velocity, to tell whether the wheels turn as commanded but
        # the body turns less (slipping/dragging) or the wheels themselves turn less (motor/firmware)
        self.wheel_vel = None    # {joint name: rad/s}
        self.imu_wz = None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, qos_profile_sensor_data)
        # Obstacles remembered by A* (LiDAR obstacles not in the map, bump spots) → web map display
        self.memory = {'remembered': [], 'bumps': []}
        self.create_subscription(PoseArray, 'astar/remembered', lambda m: self.on_memory('remembered', m), 1)
        self.create_subscription(PoseArray, 'astar/bumps', lambda m: self.on_memory('bumps', m), 1)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')  # AMCL
        self.nomotion_left = 0
        self.create_timer(0.3, self.nomotion_step)
        # Only the bridge publishes /cmd_vel. Automatic motion goes into the selector (CmdMux) via self.cmd_vel_pub.publish(...),
        # which arbitrates with manual control (web teleop, external keyboard cmd_vel_teleop) and stop, publishing at 20 Hz.
        # On the real robot it publishes to cmd_vel_pc and the Pi's watchdog (scripts/pi_cmd_watchdog.py) forwards it to cmd_vel:
        # if no command arrives for 0.5 s, the robot side stops (so a Wi-Fi drop can't leave it driving on its last command).
        # In simulation start_sim.sh runs the same watchdog, so the path is the same
        self.cmd_topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel_pc').value
        self._cmd_out = self.create_publisher(Twist, self.cmd_topic, 1)
        self.watchdog_warned = 0.0
        self.mux = CmdMux()
        self.cmd_vel_pub = _AutoCmd(self.mux)
        self.create_subscription(Twist, 'cmd_vel_teleop', self.on_external_teleop, 10)
        self.create_timer(0.05, self.mux_step)
        self.bump_pub = self.create_publisher(PointStamped, 'astar/obstacle', 10)  # bump spots → A* planner
        self.clear_obstacles_client = self.create_client(Empty, 'astar/clear_obstacles')
        # Keep-out zones drawn on the web → A* planner (JSON list). Transient local so a planner started later still gets them
        self.zones_pub = self.create_publisher(String, 'astar/zones', map_qos)
        self.zones_json = None
        self.zone_replan_at = None   # time to recompute the path after the keep-out zones changed
        # Why A* found no path (keep-out zones, etc.). It arrives separately from the service reply, so keep the time received with it
        self.plan_fail = (0.0, '')
        self.create_subscription(String, 'astar/fail_reason',
                                 lambda m: setattr(self, 'plan_fail', (time.monotonic(), m.data)), 10)
        self.level_scan_pub = self.create_publisher(LaserScan, 'scan_level', qos_profile_sensor_data)
        # SLAM only: publish with no-return beams (inf) replaced by a value just below range_max (see to_slam_scan)
        self.slam_scan_pub = self.create_publisher(LaserScan, 'scan_slam', qos_profile_sensor_data)

        self.plan_client = self.create_client(GetPlan, 'astar/get_plan')

        # Current goal. generation is a counter for ignoring late callbacks of cancelled goals
        self.goal = None
        self.generation = 0
        self.last_battery_sent = 0.0
        self.battery_history = []  # (time, voltage)
        self.stop_count = 0
        self.follower = None  # following state {'frenet': FrenetFollower, blocked/progress/recovery state}
        self.last_goal_id = None

        self.server = ServerConnection(p('server_url'), self.get_logger())
        self.create_timer(0.05, self.process_inbox)
        self.create_timer(1.0 / p('pose_rate'), self.send_pose)
        self.create_timer(0.1, self.publish_stop)
        # Web manual control (teleop): commands arrive every 0.1 s while held. Stop if none for over 0.4 s
        self.teleop = None   # (v, w, time received)
        # (manual control targets are fed into the selector in mux_step)
        self.create_timer(0.05, self.control_step)
        self.create_timer(1.0, self.send_health)
        self.create_timer(0.1, self.flush_map)
        # Runtime parameter changes (ros2 param set, web 'driving parameters' card). Memory only; on restart the params.yaml values apply.
        # Start values are kept so the web can show/revert changes. Registered after all declares (declare also calls the callback)
        self.param_start = {n: self.get_parameter(n).value for n in self.tunable_params()}
        self.params_dirty = False
        # Drive records (analyzed with tools/drive_replay.py): each bridge start gets a session folder logs/records/<start time>_frenet/,
        # inside it one goal = <time>_<goal>/record.jsonl.gz (analysis plots go in the same folder).
        # Every 0.1 s pose/velocity/command/tracks/scan points; in cycles where Frenet was computed, also its inputs and per-candidate verdicts/costs
        self.record_enabled = self.declare_parameter('record.enabled', True).value
        # When a goal ends, a summary (summary.txt) and key-moment plots (plot_*.png) are written into its folder (tools/drive_replay.py, low priority)
        self.record_plots = self.declare_parameter('record.plots', True).value
        self.record_dir = os.path.join(os.environ.get('WAREHOUSE_HOME') or os.getcwd(), 'logs', 'records')
        self.record_session = None   # this bridge session's folder (created on the first record)
        self.rec = None        # open record {'file', 't0', 'last', 'flushed', 'path'}
        self.rec_raw = None    # what the Frenet follower output this cycle (v, w, state), before emergency stop etc. altered it
        self.rec_frenet = None  # this cycle's Frenet inputs/results (filled by frenet_compute)
        self.add_on_set_parameters_callback(self.on_set_params)
        self.create_timer(0.2, self.flush_params)
        self.get_logger().info('브리지 시작 (A* + Frenet 로컬 플래너)')

    # ---------------------------------------------------------- server message handling
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
                    self.stop_count = 10  # publish stop for 1 s
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
            return   # the same list comes again with every SLAM map update, so publish only on change
        self.zones_json = data
        self.zones_pub.publish(String(data=data))
        self.get_logger().info(f'금지 구역 {len(zones)}개 → A* 플래너')
        # While moving, recompute the path for the new zones (otherwise the path planned at the start would cut through the zone).
        # Done 0.3 s later (in control_step) so the planner gets the zones first
        if self.goal is not None:
            self.zone_replan_at = time.monotonic() + 0.3

    # ---------------------------------------------------------- robot state → server
    def on_map(self, msg):
        self.map_msg = msg
        self.update_map_dist(msg)
        if self.mode == 'slam':
            # Send as soon as SLAM updates the map. If updates bunch up, send only the latest every MAP_MIN_INTERVAL (flush_map)
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
        """Run scripts/<script_name> in a separate thread and report the last 3 output lines starting with [tag] to the server.
        Only one runs at a time."""
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
            except Exception as e:  # timeout etc.
                self.server.send({'type': reply_type, 'ok': False, 'message': str(e)})
            finally:
                self.script_running = None
        threading.Thread(target=run, daemon=True).start()
        self.get_logger().info(f'{script_name} 실행: {args}')

    def save_map(self, name):
        """Web 'save map': scripts/save_map.sh"""
        if self.mode != 'slam':
            self.server.send({'type': 'map_saved', 'ok': False, 'message': 'SLAM 모드에서만 맵을 저장할 수 있습니다.'})
            return
        env = {}
        pose = self.current_pose()
        if pose is not None:  # so the map is cleaned up relative to the space the robot is in
            env = {'SEED_X': f'{pose[0]:.3f}', 'SEED_Y': f'{pose[1]:.3f}'}
        self.run_script('save_map.sh', [name], 'map_saved', 'save_map', env)

    def reset_robot(self):
        """Web 'reset state': stop immediately, cancel the goal, then scripts/reset_robot.sh
        (in simulation, the Gazebo world reset puts the robot back at its start)."""
        self.cancel_goal()
        self.stop_count = 10
        self.tipped_since = None
        if self.clear_obstacles_client.service_is_ready():
            self.clear_obstacles_client.call_async(Empty.Request())  # also clears the remembered bump spots
        self.run_script('reset_robot.sh', [], 'reset_done', 'reset', timeout=60)

    def send_map(self):
        m = self.map_msg
        if m is None or m.info.width == 0 or m.info.height == 0:
            return  # don't send the empty map right after SLAM starts (the web shows a grid around the robot without a map)
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
        # If the pose TF is missing or stale (robot/AMCL stopped), tell the UI to remove it from the screen
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
        """Report every 1 s whether the robot (Pi) is alive (LiDAR, odometry) and whether its pose is known."""
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
            'footprint': [[round(v, 3) for v in r] for r in self.body_rects()],   # robot shape on the web
        })

    def on_battery(self, msg):
        if not math.isnan(msg.voltage) and msg.voltage > 0:
            self.battery_history.append((time.monotonic(), msg.voltage))
        now = time.monotonic()
        self.battery_history = [(t, v) for t, v in self.battery_history if now - t <= 30.0]
        if now - self.last_battery_sent < 5.0 or not self.battery_history:
            return
        self.last_battery_sent = now
        # voltage fluctuates while driving, so use the median of the last 30 s
        voltage = float(np.median([v for _, v in self.battery_history]))
        self.server.send({'type': 'battery', 'voltage': voltage,
                          'percentage': battery_percent(voltage, self.battery_cells, self.battery_empty_v)})

    def send_path(self, poses):
        pts = [[p.pose.position.x, p.pose.position.y] for p in poses]
        if len(pts) > 2:
            pts = pts[:-1:3] + [pts[-1]]  # reduce the amount sent
        self.server.send({'type': 'path', 'points': pts})

    def report(self, status, message=''):
        """Report the current goal's result to the server and clear the goal."""
        if self.goal is None:
            return
        self.get_logger().info(f'목표 {self.goal["id"]}: {status} {message}')
        self.server.send({'type': 'nav_status', 'goal_id': self.goal['id'],
                          'status': status, 'message': message})
        if status != 'navigating':
            self.record_close(status, message)
            self.goal = None
            self.send_path([])

    # ---------------------------------------------------------- motion
    def start_goal(self, goal_id, x, y, yaw):
        self.cancel_goal(report=False)
        self.teleop = None   # a new goal ends manual control
        if self.tipped:
            self.last_goal_id = goal_id
            self.server.send({'type': 'nav_status', 'goal_id': goal_id, 'status': 'failed',
                              'message': '로봇이 넘어져 있습니다. 세운 뒤 상태 초기화를 하세요.'})
            return
        if self.tilt_rel > self.bump_tilt:
            # Starting while tilted on an obstacle would only climb further onto it
            self.last_goal_id = goal_id
            self.server.send({'type': 'nav_status', 'goal_id': goal_id, 'status': 'failed',
                              'message': f'차체가 {math.degrees(self.tilt_rel):.0f}도 기운 채 장애물에 걸려 있습니다. '
                                         '로봇을 직접 확인하세요.'})
            return
        self.goal = {'id': goal_id, 'x': x, 'y': y, 'yaw': yaw, 'retries': 0}
        self.last_goal_id = goal_id
        self.get_logger().info(f'목표 {goal_id}: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        self.record_open()
        self.request_plan()

    def cancel_goal(self, report=True):
        self.generation += 1
        if self.follower is not None:
            self.follower = None
            self.stop_count = 5
        if report:
            self.report('canceled')
        self.record_close('canceled')
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

    # Path planning (A*) ------------------------------------------------
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
            # The failure reason (astar/fail_reason) arrives on a separate topic, so wait briefly before attaching it
            asked = time.monotonic()

            def report_fail():
                self.destroy_timer(timer)   # once only
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
        self.stop_count = 0  # so the previous goal's stop commands don't mix into the new drive
        points = [(ps.pose.position.x, ps.pose.position.y) for ps in plan.poses]
        self.record_write({'type': 'plan', 'retries': self.goal['retries'], 'start': start, 'path': points})
        try:
            try:
                fr = FrenetFollower(points, self.goal['yaw'], self.frenet_cfg)
            except ValueError:
                # Only one path point (already within the goal cell): a 5 cm path toward the goal → treated as arrived, only align heading
                sx, sy = points[0]
                yaw = self.goal['yaw']
                fr = FrenetFollower([(sx, sy), (sx + 0.05 * math.cos(yaw), sy + 0.05 * math.sin(yaw))], yaw, self.frenet_cfg)
        except Exception as e:   # don't let the bridge die on a path that can't form a reference line (if it dies, the robot goes 'offline')
            self.get_logger().error(f'경로로 기준선을 못 만듦 ({len(points)}개 점): {e!r}')
            self.record_event(f'기준선 만들기 실패: {e!r}')
            self.report('failed', '경로를 따라갈 기준선을 만들지 못했습니다.')
            return
        self.follower = {'gen': gen, 'blocked_since': None, 'mark_pos': start[:2], 'mark_time': time.monotonic(),
                         'frenet': fr}

    def fail_or_retry(self, reason):
        """If retries remain, replan from the current pose; otherwise report failure."""
        if self.goal['retries'] < self.max_retries:
            self.goal['retries'] += 1
            self.get_logger().warn(f'{reason} 재시도 {self.goal["retries"]}/{self.max_retries}')
            self.record_event(f'{reason} 재시도 {self.goal["retries"]}/{self.max_retries}')
            self.generation += 1
            self.follower = None
            self.request_plan()
        else:
            self.report('failed', reason)

    # Camera / parameters ---------------------------------------------
    def on_cargo(self, msg):
        label = msg.data.strip()
        if self.cargo_raw is None or self.cargo_raw[0] != label:
            self.cargo_raw = (label, time.monotonic())

    # YOLO detector ------------------------------------------------
    def on_yolo_counts(self, msg):
        self.yolo_counts = (msg.data.strip(), time.monotonic())
        self.send_yolo()

    def poll_yolo(self):
        """Every 1 s read the detector's enabled (which also tells whether the node exists) and report it to the server."""
        if not self.yolo_get.service_is_ready():
            self.yolo_enabled = None
            self.send_yolo()
            return
        if self.yolo_pending is not None:
            fut, sent = self.yolo_pending
            if time.monotonic() - sent < 3.0:
                return
            # Drop requests that got no reply (link down, detector busy) and ask again; otherwise it would never ask again
            self.yolo_get.remove_pending_request(fut)
        fut = self.yolo_get.call_async(GetParameters.Request(names=['enabled']))
        self.yolo_pending = (fut, time.monotonic())

        def done(f):
            if self.yolo_pending is None or self.yolo_pending[0] is not f:
                return   # late reply to an already dropped request
            self.yolo_pending = None
            r = f.result()
            if r is not None and r.values and r.values[0].type == ParameterType.PARAMETER_BOOL:
                self.yolo_enabled = r.values[0].bool_value
            self.send_yolo()
        fut.add_done_callback(done)

    # Parameters that take effect immediately when changed at runtime → bridge attributes (other tunables are read with get_parameter on each use)
    LIVE_ATTRS = {'stop_distance': 'stop_distance', 'estop_distance': 'estop_distance',
                  'blocked_timeout': 'blocked_timeout', 'progress_timeout': 'progress_timeout',
                  'tf_timeout': 'tf_timeout', 'pose_lost_timeout': 'pose_lost_timeout', 'rotate_timeout': 'rotate_timeout', 'stall_timeout': 'stall_timeout',
                  'backoff_distance': 'backoff_distance',
                  'max_retries': 'max_retries', 'bump_tilt_deg': 'bump_tilt'}

    def tunable_params(self):
        """Names of parameters changeable at runtime (web card order). frenet.* list values (lateral_dists, etc.) need a restart."""
        names = [f'frenet.{k}' for k, v in vars(self.frenet_cfg).items() if not isinstance(v, tuple)]
        names += ['tracking.moving_speed']
        names += list(self.LIVE_ATTRS)
        return names

    def on_set_params(self, params):
        """Called just before parameters change: reject those that can't change at runtime, apply the rest to the current drive immediately."""
        for prm in params:
            n = prm.name
            if n not in self.param_start:
                return SetParametersResult(successful=False, reason=f'{n} 은(는) 실행 중에 못 바꿉니다 (재시작 필요)')
            if n in ('frenet.n_speed',) and prm.value < 0:
                return SetParametersResult(successful=False, reason=f'{n} 은(는) 0 이상이어야 합니다')
        for prm in params:
            n, v = prm.name, prm.value
            if n in self.LIVE_ATTRS:
                setattr(self, self.LIVE_ATTRS[n], math.radians(v) if n == 'bump_tilt_deg' else v)
            elif n.startswith('frenet.'):
                # FrenetFollowers share the same FrenetConfig object, so it applies mid-drive (reference spacing/smoothing from the next goal)
                setattr(self.frenet_cfg, n[7:], v)
            self.get_logger().info(f'파라미터 변경: {n} = {v}')
            self.record_event(f'파라미터 변경: {n} = {v}', param=n, value=v)
        self.params_dirty = True
        return SetParametersResult(successful=True)

    def set_param_from_web(self, name, value):
        """Value from the web: cast to the declared type (JSON sends 2.0 as 2), set_parameters → report the result to the web."""
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
        except Exception as e:  # undeclared name, not a number, etc.
            ok, reason = False, str(e) or type(e).__name__
        self.server.send({'type': 'param_result', 'name': name, 'ok': ok, 'reason': reason})

    def flush_params(self):
        """Send the changed parameter list to the server (web card). Changes made with ros2 param set also arrive here."""
        if not self.params_dirty:
            return
        self.params_dirty = False
        self.server.send({'type': 'params', 'items': [
            {'name': n, 'value': self.get_parameter(n).value, 'start': s} for n, s in self.param_start.items()]})

    def set_yolo(self, msg):
        """Web button: {'enabled': bool} or {'capture': True} (save one detection frame).
        The Pi camera stays on and only the detector is toggled: while the detector is off it doesn't subscribe to the image,
        so nothing comes over Wi-Fi. (Toggling the camera itself is too slow to start to be used per stop.)"""
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
                    self.yolo_counts = ('', None)   # clear the previous counts when turning off
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
        self.scan = msg
        self.send_scan(msg)
        # AMCL/SLAM/A* memory only get scans taken while not more tilted than usual (scan_level).
        # When a bump tilts the robot, the LiDAR sees the floor as a wall and those scans make the AMCL pose jump by tens of cm.
        # But after 3 s of being tilted (stopped hung up on an obstacle, etc.) scans pass again - otherwise localization would stop
        now = time.monotonic()
        if self.tilt_rel < math.radians(3):
            self.scan_gated_since = None
        elif self.scan_gated_since is None:
            self.scan_gated_since = now
        if self.scan_gated_since is None or now - self.scan_gated_since > 3.0:
            self.level_scan_pub.publish(msg)
            self.slam_scan_pub.publish(to_slam_scan(msg))
            self.update_tracks(msg)   # tilted scans see the floor, so they aren't used for tracking/Frenet either

    def send_scan(self, msg):
        """Live LiDAR for the web: map-frame points in beam order (None for beams with no return). The web connects neighbouring points.
        Not sent more often than SCAN_SEND_INTERVAL. Not used for the map or planning."""
        now = time.monotonic()
        if now - self.last_scan_sent < SCAN_SEND_INTERVAL or not self.server.connected:
            return
        # Place the scan on the map with the pose at the time it was taken. Using the latest pose (Time()) would draw it rotated
        # further while turning, so the LiDAR lines wouldn't line up with the map on the web. If TF for that time isn't there yet, use the latest
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
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
        """If the tilt stays above 35 degrees for 0.5 s, treat the robot as tipped over and stop."""
        self.imu_wz = round(float(msg.angular_velocity.z), 4)
        q = msg.orientation
        # How far the robot z axis is tilted from vertical
        cos_tilt = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.tilt = math.acos(max(-1.0, min(1.0, cos_tilt)))
        now = time.monotonic()
        if self.tilt_base is None:
            self.tilt_base = self.tilt
        self.tilt_rel = max(0.0, self.tilt - self.tilt_base)
        # Up direction (against gravity) in robot coordinates. If the front is lifted, the robot x axis points up, so up_x > 0
        # → the direction of the horizontal component is the lifted side
        up_x = 2.0 * (q.x * q.z - q.w * q.y)
        up_y = 2.0 * (q.y * q.z + q.w * q.x)
        if math.hypot(up_x, up_y) > 1e-3:
            self.lift_dir = math.atan2(up_y, up_x)
        if self.tilt_rel < math.radians(2) and self.last_imu_time is not None:
            # Follow the usual tilt with a 10 s time constant (not while climbing an obstacle)
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
        """Distance (from the LiDAR) to the nearest object within ±half_angle of direction center (front=0, back=pi).
        0 (= blocked) if there is no scan or it is stale."""
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

    def odom_pose(self):
        """Robot pose in the odom frame (x, y, yaw, age [s]) or None. From /odom messages (falls back to TF if none)."""
        now = self.get_clock().now()
        if self.odom_msg is not None:
            x, y, yaw, stamp = self.odom_msg
            return x, y, yaw, (now - stamp).nanoseconds / 1e9
        try:
            t = self.tf_buffer.lookup_transform('odom', self.base_frame, Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation), (now - Time.from_msg(t.header.stamp)).nanoseconds / 1e9

    def pose_with_age(self):
        """Pose in the map (global_frame): (x, y, yaw, age [s]) or None. The age is that of the odometry (Pi):
        map→odom (AMCL/SLAM) uses the latest TF (AMCL stamps 1 s ahead, so its age is not meaningful)."""
        o = self.odom_pose()
        if o is None:
            return None
        m = self.xyyaw(self.global_frame, 'odom')
        if m is None:
            return None
        c, sn = math.cos(m[2]), math.sin(m[2])
        return (m[0] + c * o[0] - sn * o[1], m[1] + sn * o[0] + c * o[1],
                math.atan2(math.sin(m[2] + o[2]), math.cos(m[2] + o[2])), o[3])

    def stop_following(self):
        self.follower = None
        self.stop_count = 5
        self.cmd_vel_pub.publish(Twist())

    def control_step(self):
        self.rec_raw = None
        self.drive_step()
        self.record_step()

    def drive_step(self):
        if self.zone_replan_at is not None and time.monotonic() >= self.zone_replan_at:
            self.zone_replan_at = None
            if self.goal is not None:
                self.get_logger().info('금지 구역이 바뀌어 지금 위치에서 경로를 다시 계산합니다')
                self.generation += 1      # drop the path/plan replies being followed
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
        # Pose age = odometry (Pi → Wi-Fi). If late, stop right away but keep the goal; when the pose returns, continue from there.
        # Wi-Fi delays are usually short, so failing the goal on a single late pose would make a person resend it.
        # Fail only if it keeps not arriving beyond pose_lost_timeout. If PC→Pi commands also stop while standing, the Pi watchdog stops the robot
        if pose is None or pose[3] > self.tf_timeout:
            why = '위치 없음' if pose is None else f'위치 나이 {pose[3]:.2f}초'
            if 'pose_wait' not in f:
                f['pose_wait'] = now
                self.get_logger().warn(f'위치 정보 지연: {why}, 서서 기다림 (기울기 {math.degrees(self.tilt_rel):.1f}°)')
            self.cmd_vel_pub.publish(Twist())
            self.rec_raw = (0.0, 0.0, 'pose_wait')
            if now - f['pose_wait'] > self.pose_lost_timeout:
                self.stop_following()
                self.report('failed', f'로봇 위치 정보가 {self.pose_lost_timeout:.0f}초 넘게 갱신되지 않아 정지했습니다 ({why}).')
            return
        if 'pose_wait' in f:
            self.get_logger().info(f'위치 정보 다시 받음 ({now - f.pop("pose_wait"):.1f}초 기다림), 이어서 감')
        x, y, yaw, _ = pose
        if self.tilt_rel < math.radians(2):
            f['level_pose'] = (x, y, yaw)   # compute bump spots from the pose/heading before tilting started (the robot turns as it gets pushed)
        v, w, state = self.frenet_compute(f['frenet'])
        self.rec_raw = (v, w, state)

        if state == 'done':
            self.stop_following()
            self.report('succeeded')
            return

        # If tilted (climbed a low obstacle), handle that before 'front blocked' (the LiDAR is seeing the floor).
        # Waiting while tilted lets the tilted scans throw the AMCL pose far off
        if self.tilt_rel > self.bump_tilt:
            self.start_backoff(f, x, y, yaw, now,
                               f'차체가 {math.degrees(self.tilt_rel):.0f}도 기울었습니다 (낮은 장애물에 올라탐).')
            return
        if state == 'blocked':
            # No safe candidate (a person blocking, etc.) → wait; if blocked too long, back off and replan around the obstacle
            if f['blocked_since'] is None:
                f['blocked_since'] = now
                self.get_logger().warn('Frenet: 안전한 궤적 없음, 정지 후 대기')
                self.record_event('Frenet: 안전한 궤적 없음, 정지 후 대기')
            # Frenet blocked by static obstacles only (the A* path still runs through a narrow gap beside an object it didn't know): waiting
            # won't help, so replan with A* after 1 s (A* remembers seen obstacles and avoids them), which is much faster than wait + back off + replan.
            # Up to 4 times per goal, after that wait/back off
            if (f['frenet'].static_blocked and now - f['blocked_since'] > 1.0
                    and self.goal.get('reroutes', 0) < 4):
                self.goal['reroutes'] = self.goal.get('reroutes', 0) + 1
                self.get_logger().info(f'Frenet: 정적 장애물로 막힘 → A* 재계획 ({self.goal["reroutes"]}/4)')
                self.record_event(f'Frenet: 정적 장애물로 막힘 → A* 재계획 ({self.goal["reroutes"]}/4)')
                self.generation += 1
                self.stop_following()
                self.request_plan()
                return
            if now - f['blocked_since'] > self.blocked_timeout:
                self.start_recovery(f, now, '앞이 막혀 진행할 수 없습니다.')
                return
            f['mark_time'] = now
            f.pop('drive_since', None)
            self.cmd_vel_pub.publish(Twist())
            return
        if v > 0.0:
            # Frenet avoids obstacles. Only emergency stop and 'hung up' are checked here
            clearance = self.front_clearance()
            if clearance < self.stop_distance and self.tilt_rel > math.radians(3):
                # Slightly tilted, hung on an obstacle edge: the LiDAR looks down and sees the floor/box top as an obstacle.
                # Waiting doesn't resolve it, so treat it as a bump and back off
                self.start_backoff(f, x, y, yaw, now,
                                   f'차체가 {math.degrees(self.tilt_rel):.0f}도 기운 채 앞이 막혔습니다 (장애물에 걸침).')
                return
            if clearance < self.estop_distance:
                # Emergency stop: no forward motion (turning/reversing allowed). Frenet keeps safety_margin, so this is normally
                # not reached → objects that suddenly appear, localization error, etc.
                if not f.get('estop'):
                    self.get_logger().warn(f'비상 정지: 앞쪽 {clearance:.2f} m')
                    self.record_event(f'비상 정지: 앞쪽 {clearance:.2f} m')
                f['estop'] = True
                v = 0.0
            else:
                f['estop'] = False
        if state != 'blocked':
            f['blocked_since'] = None

        stall = self.detect_stall(f, x, y, v, state, now)
        if stall:
            self.start_backoff(f, x, y, yaw, now, stall)
            return

        # Fail if there is no progress for long while not turning in place
        # Cut off in-place turns that never finish (heading alignment failing, slipping, etc.)
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
            self.start_recovery(f, now, '진행이 없습니다.')
            return

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_vel_pub.publish(cmd)

    # Drive records (analyzed with tools/drive_replay.py) ----------------------
    def record_open(self):
        """Open a new record file when a goal arrives (retries/replans use the same file)."""
        self.record_close('replaced')
        if not self.record_enabled:
            return
        try:
            if self.record_session is None:
                self.record_session = os.path.join(self.record_dir, f'{time.strftime("%Y%m%d_%H%M%S")}_frenet')
            folder = os.path.join(self.record_session, f'{time.strftime("%H%M%S")}_{self.goal["id"]}')
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, 'record.jsonl.gz')
            fh = gzip.open(path, 'wt', encoding='utf-8')
        except OSError as e:
            self.get_logger().warn(f'주행 기록 파일을 못 열었습니다: {e}')
            return
        now = time.monotonic()
        self.rec = {'file': fh, 't0': now, 'last': 0.0, 'flushed': now, 'path': path}
        params = {n: self.get_parameter(n).value for n in self.tunable_params()}
        params.update({f'frenet.{k}': v for k, v in vars(self.frenet_cfg).items()})   # all of them, including list values (for re-computation)
        self.record_write({'type': 'start', 'wall': time.strftime('%Y-%m-%d %H:%M:%S'), 'goal': self.goal,
                           'map': self.get_parameter('map_name').value, 'global_frame': self.global_frame,
                           'base_frame': self.base_frame, 'footprint_extra': self.footprint_extra,
                           'tracking': self.track_cfg, 'params': params})
        self.get_logger().info(f'주행 기록: {path}')

    def record_close(self, status, message=''):
        r = self.rec
        if r is None:
            return
        self.record_write({'type': 'end', 'status': status, 'message': message})
        self.rec = None
        try:
            r['file'].close()
        except OSError:
            pass
        if self.record_plots:
            self.record_summarize(os.path.dirname(r['path']))

    def record_summarize(self, folder):
        """Write a summary text (summary.txt) and key-moment plots (plot_*.png) into the finished goal's folder in a separate process.
        Runs at nice 19 without waiting so it doesn't disturb driving (the bridge). If it fails, the record (record.jsonl.gz) is unaffected."""
        tool = os.path.join(os.environ.get('WAREHOUSE_HOME', ''), 'tools', 'drive_replay.py')
        if not os.path.isfile(tool):
            return
        try:
            out = open(os.path.join(folder, 'summary.txt'), 'w', encoding='utf-8')
            subprocess.Popen(['nice', '-n', '19', 'python3', tool, folder], stdout=out, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True,
                             env={**os.environ, 'PYTHONWARNINGS': 'ignore'})   # so library warnings don't clutter the summary
            out.close()   # the child process writes through its own copy
        except OSError as e:
            self.get_logger().warn(f'주행 기록 요약/그림 실패: {e}')

    def record_write(self, obj):
        r = self.rec
        if r is None:
            return
        obj['t'] = round(time.monotonic() - r['t0'], 3)
        try:
            r['file'].write(json.dumps(obj, default=_json_default, ensure_ascii=False, separators=(',', ':')) + '\n')
            now = time.monotonic()
            if now - r['flushed'] > 1.0:   # flush every 1 s so it can be read while driving
                r['file'].flush()
                r['flushed'] = now
        except (OSError, ValueError, TypeError) as e:
            self.get_logger().warn(f'주행 기록 중단: {e}')
            self.rec = None

    def record_event(self, text, **extra):
        self.record_write({'type': 'event', 'text': text, **extra})

    def record_step(self):
        """Every control cycle (0.1 s apart, always in cycles where Frenet was recomputed): pose/velocity/command/tracks/scan.
        raw is what the Frenet follower output, cmd is what was actually sent after emergency stop etc."""
        r = self.rec
        if r is None:
            return
        now = time.monotonic()
        fr, self.rec_frenet = self.rec_frenet, None
        if fr is None and now - r['last'] < 0.1:
            return
        r['last'] = now
        f = self.follower or {}
        mode = None
        if f:
            mode = 'backoff' if 'backoff' in f else 'recovery' if 'recovery' in f else 'frenet'
        cmd = self.cmd_vel_pub.last
        pm = self.pose_with_age()
        out = {'type': 'step', 'mode': mode,
               'pose': pm[:3] if pm else None, 'pose_age': round(pm[3], 3) if pm else None,
               'odom': self.odom_xyyaw(), 'vw': self.odom_vw,
               'raw': self.rec_raw, 'cmd': cmd[:2] if cmd and now - cmd[2] < 0.06 else None,
               'estop': bool(f.get('estop')),
               'blocked_for': round(now - f['blocked_since'], 2) if f.get('blocked_since') else None,
               'front': self.front_clearance(), 'tilt': self.tilt_rel,
               'wheels': self.wheel_vel, 'imu_wz': self.imu_wz,
               'tracks': [[t.id, *t.x, t.radius, t.hits, t.misses] for t in self.tracks]}
        if fr is not None:
            out['frenet'] = fr      # scan points are inside the Frenet inputs
        else:
            out['map_to_odom'] = self.xyyaw('odom', self.global_frame)
            out['scan'], out['unmapped'] = self.scan_odom, self.scan_unmapped
        self.record_write(out)

    def on_odom(self, msg):
        self.odom_vw = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        q = msg.pose.pose
        self.odom_msg = (q.position.x, q.position.y, yaw_from_quaternion(q.orientation),
                         Time.from_msg(msg.header.stamp))

    def on_joint_states(self, msg):
        if msg.velocity:
            self.wheel_vel = {n: round(float(v), 4) for n, v in zip(msg.name, msg.velocity)}

    def xyyaw(self, target, source, stamp=None):
        try:
            t = self.tf_buffer.lookup_transform(target, source, stamp if stamp is not None else Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)

    def update_tracks(self, scan):
        """Scan → odom points (Frenet static obstacles) + moving obstacle tracking."""
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
        # Scan points of objects not in the map (odom, for records). Empty if it can't be matched to the map
        self.scan_unmapped = pts[:0]
        if not self.tracking:
            return
        # Points near map walls are static and not tracked (the map is used only for this check, so a few cm of AMCL error doesn't affect velocities).
        # Points outside the map / beyond walls / in unexplored cells are dropped too: only points in free map cells are tracked (same in SLAM).
        # During SLAM, walls not yet in the map lie in unexplored cells and would all be caught as 'obstacles',
        # with velocity noise above the moving-object threshold. Frenet still avoids objects in unexplored cells via the scan points
        if self.map_dist is None:
            pts = pts[:0]   # no tracking until a map exists
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
        # Unexplored cells are not walls → points in them stay as moving-obstacle candidates
        self.map_dist = (obstacle_distance(grid >= 65, msg.info.resolution, 0.5), msg.info, grid)

    def send_obstacles(self):
        """Send tracked obstacles to the web (map coords, every 0.25 s)."""
        now = time.monotonic()
        if now - self.last_obstacles_sent < 0.25:
            return
        self.last_obstacles_sent = now
        m_o = self.xyyaw(self.global_frame, 'odom')
        if m_o is None:
            return
        c, s = math.cos(m_o[2]), math.sin(m_o[2])
        out = []
        moving = self.get_parameter('tracking.moving_speed').value
        for t in self.tracks:
            # Tracking only runs in free map cells, so a stopped track is a 'stopped object not in the map' (box, standing person, ...)
            (x, y), = transform_points([t.x[:2]], *m_o)
            out.append({'id': t.id, 'x': round(float(x), 3), 'y': round(float(y), 3),
                        'moving': bool(math.hypot(t.x[2], t.x[3]) >= moving),
                        'vx': round(float(c * t.x[2] - s * t.x[3]), 3), 'vy': round(float(s * t.x[2] + c * t.x[3]), 3),
                        'r': round(float(t.radius), 3)})
        self.server.send({'type': 'obstacles', 'obstacles': out})

    def frenet_compute(self, follower):
        """One Frenet follower cycle. Computed only every 0.1 s, same command in between (it runs in the bridge's single thread alongside TF reception etc.)."""
        now = time.monotonic()
        last = getattr(follower, 'last_cmd', None)
        if last is not None and now - last[0] < 0.1:
            return last[1]
        pose = self.odom_xyyaw()
        map_to_odom = self.xyyaw('odom', self.global_frame)
        if pose is None or map_to_odom is None:
            return 0.0, 0.0, 'blocked'
        dyn = np.array([[t.x[0], t.x[1], t.x[2], t.x[3], t.radius] for t in self.tracks]).reshape(-1, 5)
        moving_speed = self.get_parameter('tracking.moving_speed').value
        # For records: follower state before computing (so re-computing with the same inputs gives the same result)
        pre = {'progress': follower.progress, 'offset': follower.offset, 'turning': follower.turning,
               'arrived': follower.arrived, 'prev_shape': follower.prev_shape}
        result = follower.compute(pose, *self.odom_vw, map_to_odom, self.scan_odom, dyn, moving_speed=moving_speed)
        follower.last_cmd = (now, result)
        if self.rec is not None:
            self.rec_frenet = {'pose': pose, 'vw': self.odom_vw, 'map_to_odom': map_to_odom, 'dyn': dyn,
                               'moving_speed': moving_speed, 'pre': pre, 'scan': self.scan_odom,
                               'result': result, 'debug': follower.last_debug, 'eval': follower.planner.last_eval,
                               'traj': follower.last_trajectory}
        self.send_local_plan(follower, result)
        return result

    def send_local_plan(self, follower, result):
        """Send the trajectory Frenet chose to the web (map coords, every 0.2 s). Empty when turning in place/blocked."""
        now = time.monotonic()
        if now - self.last_plan_sent < 0.2:
            return
        self.last_plan_sent = now
        traj = follower.last_trajectory
        m_o = self.xyyaw(self.global_frame, 'odom')
        pts = []
        if traj is not None and m_o is not None:
            pts = [[round(float(px), 3), round(float(py), 3)] for px, py in transform_points(np.asarray(traj)[:, :2], *m_o)]
        v, w, state = result
        self.server.send({'type': 'local_plan', 'traj': pts, 'v': round(float(v), 3), 'w': round(float(w), 3),
                          'state': state})

    # Blocked handling --------------------------------------------------
    # ① When blocked, wait blocked_timeout (for moving obstacles to pass). If blocked only statically, replan with A* after 1 s (drive_step)
    # ② Still blocked or no progress: back off 15 cm if the rear is clear, otherwise turn in place toward the most open side
    # ③ Replan with A* (fail_or_retry, up to max_retries)  ④ If that fails too, report failure
    def start_recovery(self, f, now, reason):
        odom = self.odom_xyyaw()
        if odom is None:
            self.stop_following()
            self.fail_or_retry(reason)
            return
        if self.front_clearance(math.radians(30), center=math.pi) > 0.35:
            rec = {'mode': 'back', 'speed': -0.05}
            what = '뒤로 15cm 물러난 뒤'
        elif not self.rotation_clear(odom):
            # Pressed against a wall with no room to turn: turning would catch a body corner on the wall, climb it and tip over
            if self.front_clearance(math.radians(30)) > 0.35:
                rec = {'mode': 'forward', 'speed': 0.05}
                what = '돌 공간이 없어 앞으로 15cm 나온 뒤'
            else:
                self.stop_following()
                self.fail_or_retry(f'{reason} 앞뒤가 막혀 돌 공간도 없습니다.')
                return
        else:
            # Most open direction (farthest of the nearest-object distances per 30° sector)
            best = max(range(-150, 181, 30), key=lambda d: self.front_clearance(math.radians(15), center=math.radians(d)))
            turn = math.radians(max(-90, min(90, best)) if abs(best) <= 90 else math.copysign(90, best))
            rec = {'mode': 'turn', 'target': turn}
            what = f'트인 쪽({best}°)으로 돌아선 뒤'
        rec.update(start=odom, until=now + 5.0, reason=reason)
        f['recovery'] = rec
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().warn(f'{reason} {what} 다시 계획합니다')
        self.record_event(f'{reason} {what} 다시 계획합니다')

    def rotation_clear(self, odom):
        """Whether turning in place is clear of the body: no scan points within the radius from the rotation center to the body's outer corner + margin."""
        pts = self.scan_odom
        if not len(pts):
            return True
        radius = self.body_radius() + 0.05
        return float(np.min(np.hypot(pts[:, 0] - odom[0], pts[:, 1] - odom[1]))) > radius

    def body_rects(self):
        """List of body + attached object rectangles (x_min, x_max, y_min, y_max), relative to base_footprint."""
        x0, x1, hw = self.frenet_cfg.footprint
        return [(x0, x1, -hw, hw)] + self.footprint_extra

    def body_radius(self):
        """Distance from the rotation center (base_footprint) to the farthest body point = radius swept when turning in place."""
        return max(math.hypot(x, y) for a, b, c, d in self.body_rects() for x in (a, b) for y in (c, d))

    def rear_extent(self):
        """Rear end x of the body (incl. attached objects) (negative)."""
        return min(r[0] for r in self.body_rects())

    def recovery_step(self, f, now):
        r = f['recovery']
        odom = self.odom_xyyaw()
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
        """x, y, yaw: map pose at the time of the bump (for computing the bump spot). The backed-off distance is measured with odom
        (the map TF isn't updated while tilted)."""
        self.get_logger().warn(f'부딪힘 감지: {reason} 뒤로 물러납니다')
        self.record_event(f'부딪힘 감지: {reason} 뒤로 물러납니다')
        x, y, yaw = f.get('level_pose', (x, y, yaw))
        # If detected by tilt, the lifted side is the obstacle. Front lifted → get out backward; rear lifted (rear corner climbed while
        # turning) → forward. If detected by being stuck while moving forward, the obstacle is in front
        tilted = self.tilt_rel > math.radians(3)
        side = self.lift_dir if tilted else 0.0
        f['backoff'] = {'x': x, 'y': y, 'yaw': yaw, 'side': side,
                        'speed': -0.05 if math.cos(side) >= 0 else 0.05, 'until': now + 5.0, 'reason': f'부딪힘 감지: {reason}',
                        'odom': self.odom_xy()}
        self.cmd_vel_pub.publish(Twist())

    def odom_xy(self):
        o = self.odom_pose()
        return None if o is None else (o[0], o[1])

    def odom_xyyaw(self):
        o = self.odom_pose()
        return None if o is None else o[:3]

    def detect_stall(self, f, x, y, v, state, now):
        """Reason string if commanded forward but not moving, otherwise None."""
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
        """Slowly get out after a bump (usually backward). Once backed off, report the bump spot to the planner and replan."""
        b = f['backoff']
        # The LiDAR is 0.13 m ahead of the body's rear end, so stop backing off if anything is within 0.3 m behind (0.25 m when going forward).
        # Increased by how far attached objects stick out at the rear (default 8 cm → 0.38 m)
        if b['speed'] < 0:
            blocked = self.front_clearance(math.radians(30), center=math.pi) < 0.3 + max(0.0, -0.197 - self.rear_extent())
        else:
            blocked = self.front_clearance(math.radians(30)) < 0.25
        cur, start = self.odom_xy(), b['odom']
        moved = math.hypot(cur[0] - start[0], cur[1] - start[1]) if cur and start else 0.0
        if moved >= self.backoff_distance or now > b['until'] or blocked:
            self.stop_following()
            # The bumped object is ahead of the robot's front end (about 0.07 m from base) (the planner blocks a bump_radius around it)
            pt = PointStamped()
            pt.header.frame_id = self.global_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = b['x'] + 0.25 * math.cos(b['yaw'] + b['side'])
            pt.point.y = b['y'] + 0.25 * math.sin(b['yaw'] + b['side'])
            self.bump_pub.publish(pt)
            self.goal['bumps'] = self.goal.get('bumps', 0) + 1
            if self.goal['bumps'] >= 2:
                # After two bumps in the same move, stop trying. Automatically trying to dodge an object the LiDAR can't see
                # keeps bumping and climbing onto it, so it's better for a person to look
                self.report('failed', f'{b["reason"]} 두 번 부딪혀 멈췄습니다. LiDAR 에 안 보이는 낮은 장애물이 '
                                      '있을 수 있으니 직접 확인하세요.')
                return
            self.relocalize_then_retry(b['reason'])
            return
        cmd = Twist()
        cmd.linear.x = b['speed']
        self.cmd_vel_pub.publish(cmd)

    def relocalize_then_retry(self, reason):
        """Map mode: after a bump, re-localize with scan matching (tools/localize.py), then replan.
        The scans taken while tilted (seeing the floor) throw the AMCL pose off by tens of cm."""
        pose = self.current_pose()
        tool = os.path.join(os.environ.get('WAREHOUSE_HOME', ''), 'tools', 'localize.py')
        gen = self.generation
        if self.mode != 'map' or pose is None or not os.path.isfile(tool) or self.goal is None:
            # Delay the replan by 0.5 s so the planner has received the bump spot (astar/obstacle) first
            # (replanning immediately would use the path from before it was remembered and hit the same spot again)
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
            except Exception as e:  # timeout etc.
                self.get_logger().warn(f'위치 다시 맞추기 실패: {e}')
            self.server.inbox.put({'type': '_relocalized', 'gen': gen, 'result': result, 'reason': reason})
        threading.Thread(target=run, daemon=True).start()

    def on_relocalized(self, msg):
        if msg['gen'] != self.generation or self.goal is None:
            return  # cancelled / new goal in the meantime
        r = msg['result']
        if r and r[3] >= 0.8:   # trust it only if at least 80% of scan points match walls within 20 cm
            self.publish_initial_pose(r[0], r[1], r[2], precise=True)
        elif self.mode == 'map':
            self.get_logger().warn(f'스캔 매칭 결과가 좋지 않아 AMCL 위치를 그대로 씁니다 ({r})')
        self.fail_or_retry(msg['reason'])

    # ---------------------------------------------------------- misc
    def publish_initial_pose(self, x, y, yaw, precise=False):
        """precise: pose found by scan matching → std dev 5 cm/3°, otherwise (pose clicked on the web) 20 cm/15°.
        A larger spread scatters the particles so much that the pose is off for a while after starting."""
        msg = PoseWithCovarianceStamped()
        msg.header = self.make_pose(x, y, yaw).header
        msg.pose.pose = self.make_pose(x, y, yaw).pose
        sxy, syaw = (0.05, math.radians(3)) if precise else (0.2, math.radians(15))
        msg.pose.covariance[0] = sxy ** 2
        msg.pose.covariance[7] = sxy ** 2
        msg.pose.covariance[35] = syaw ** 2
        self.initial_pose_pub.publish(msg)
        # Let AMCL correct with scans a few times even while stopped (AMCL only updates when moving)
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
        """SLAM run ID = the time run.sh started (first line of logs/run_mode.env). It stays the same when only the bridge restarts,
        so the server doesn't clear the SLAM map/pose."""
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
        """Web manual control (the server blocks it while a task is running)."""
        if self.goal is not None or self.follower is not None:
            return
        v = max(-0.1, min(0.2, v))
        w = max(-1.2, min(1.2, w))
        self.teleop = (v, w, time.monotonic())

    def on_external_teleop(self, msg):
        """External keyboard control (ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop).
        The keyboard program keeps sending 0 even with no key pressed, so only non-zero commands count as manual control
        (0 only while already in manual control = stop). Moving during automatic motion stops the automatic motion (manual has priority)."""
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
        """Feed the manual control target into the selector (after safety checks). If no command for over 0.4 s, the selector stops smoothly."""
        t = self.teleop
        if t is None:
            return
        v, w, at = t
        if now - at > 0.4 or self.goal is not None or self.tipped:
            self.teleop = None
            return
        # Don't move toward close objects in front/behind (turning allowed). From LiDAR: 0.25 m front / 0.3 m back
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
        if self.cmd_topic != 'cmd_vel' and (out[0] or out[1]) and now - self.watchdog_warned > 10.0 \
                and self._cmd_out.get_subscription_count() == 0:
            self.watchdog_warned = now
            self.get_logger().warn(f'{self.cmd_topic} 를 받는 Pi 명령 감시 노드가 없어 로봇이 움직이지 않습니다 '
                                   '(실물 run.sh 가 Pi 에 띄움: ~/warehouse_cmd_watchdog.py)')

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
