"""Node that exposes the hand-written A* as a ROS 2 service.

Subscribes: map (nav_msgs/OccupancyGrid, transient local)
       scan_level (sensor_msgs/LaserScan, published by the bridge only while not tilted) - briefly remembers obstacles not in the map (newly placed boxes, people, ...) and plans around them
       astar/obstacle (geometry_msgs/PointStamped) - spots where the bridge detected a bump (low objects the LiDAR can't see)
       astar/zones (std_msgs/String, JSON list, transient local) - keep-out zones drawn on the web (the bridge receives them from the server and publishes)
Publishes: astar/remembered (geometry_msgs/PoseArray) - LiDAR obstacle cells currently remembered (every 1 s, for web display)
       astar/bumps (geometry_msgs/PoseArray) - centers of remembered bump spots
Service: astar/get_plan (nav_msgs/srv/GetPlan)
Publishes: astar/plan (nav_msgs/Path)  - for checking in RViz
       astar/plan_map (nav_msgs/OccupancyGrid, transient local) - the grid actually used for planning (processed + keep-out zones, for RViz)
       astar/fail_reason (std_msgs/String) - why planning failed (e.g. keep-out zones). The bridge appends it to the failure message

In saved-map mode the map is processed with plan_map.py before planning (fills unexplored stripes, inflates chair/desk leg dots).
AMCL uses the original map, so localization is not affected.
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
        self.declare_parameter('allow_unknown', False)   # true in SLAM mode (unexplored cells are passable)
        self.declare_parameter('unknown_cost', 2.0)
        self.declare_parameter('goal_tolerance', 0.3)
        # Remember obstacles not in the map: LiDAR points away from the map's walls are treated as obstacles for this long
        self.declare_parameter('scan_obstacles', True)
        self.declare_parameter('scan_memory', 20.0)      # s
        self.declare_parameter('scan_max_range', 5.0)    # m, points farther than this are ignored
        self.declare_parameter('scan_wall_margin', 0.15)  # m, points within this distance of a map wall are treated as the wall and ignored
        # Only remembered after being seen continuously in the same 5 cm cell for longer than this. Passing people/objects
        # (1-2 s per cell) are not remembered, otherwise a moving object's whole trail becomes a fake wall that blocks passages
        self.declare_parameter('scan_persist', 2.5)       # s
        # When remembered obstacles leave no path: keep the obstacles but retry with the radius reduced to this (body half-width
        # + Frenet's slow margin). If still none, use only obstacles seen again within the last scan_recent s (drops wrong memories
        # such as people who walked past or scans smeared while turning). If still none, fail. Planning on the map alone would give
        # a path through the gap the obstacle blocks, which the Frenet follower then refuses on every replan.
        self.declare_parameter('fallback_radius', 0.20)  # m
        self.declare_parameter('scan_recent', 3.0)        # s
        self.declare_parameter('bump_memory', 120.0)     # s, how long a bump spot is remembered as an obstacle
        self.declare_parameter('bump_radius', 0.15)      # m, radius blocked around a bump spot
        # Map processing for planning (saved-map mode only, plan_map.py). 0 disables
        self.declare_parameter('stripe_cells', 2)            # unexplored stripes narrower than 2*N cells between free cells → free
        self.declare_parameter('cluster_inflate', 0.1)       # m, inflate small clusters (leg dots) → gaps narrower than 2*(radius + this) are closed
        self.declare_parameter('cluster_max_size', 0.4)      # m, obstacle clusters longer than this in x/y (walls) are not inflated
        self.declare_parameter('start_clear_radius', 0.35)   # m, area around the start is excluded from processing/keep-out zones (so the robot can get out)

        self.planner = None
        self.map_msg = None
        self.base_grid = None     # base grid for planning (2D, stripes filled)
        self.added = None         # cells added to the base grid as obstacles (inflated leg dots | keep-out zones)
        self.cluster_added = None
        self.zones = []
        self.frame_id = 'map'
        self.scan_points = deque()   # (time, xs, ys) map coords - only obstacles that stayed (stationary)
        self.cell_seen = {}          # 5 cm cell -> [first seen, last seen, times seen]
        self.bump_points = deque()   # (time, xs, ys) points around bump spots
        self.last_scan_time = 0.0
        self.last_remember_time = None   # last time something was added to memory (logged to explain an empty memory)
        self.last_clear_time = None      # time it was cleared by the web 'reset state'

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        topic = self.get_parameter('map_topic').value
        self.create_subscription(OccupancyGrid, topic, self.on_map, map_qos)
        self.path_pub = self.create_publisher(Path, 'astar/plan', 1)
        self.create_service(GetPlan, 'astar/get_plan', self.on_get_plan)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)   # the subscription callback holds the listener, so no separate reference is needed
        self.create_subscription(PointStamped, 'astar/obstacle', self.on_bump, 10)
        self.create_service(Empty, 'astar/clear_obstacles', self.on_clear)
        self.create_subscription(String, 'astar/zones', self.on_zones, map_qos)
        self.plan_map_pub = self.create_publisher(OccupancyGrid, 'astar/plan_map', map_qos)
        self.fail_pub = self.create_publisher(String, 'astar/fail_reason', 10)
        self.remembered_pub = self.create_publisher(PoseArray, 'astar/remembered', 1)
        self.bumps_pub = self.create_publisher(PoseArray, 'astar/bumps', 1)
        self.bump_centers = deque()   # (time, x, y) for web display
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
            # SLAM mode: the map is still growing, so it is not processed (unexplored cells are passable, so stripes don't matter)
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
        """Rebuild the planning grid from base grid + inflated leg dots + keep-out zones (when the map/zones change)."""
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
        """Every 0.5 s convert LiDAR points to map coords and remember only those that are not map walls."""
        now = time.monotonic()
        # SLAM mode (allow_unknown) doesn't remember: the SLAM map itself reflects obstacles every few seconds, and since the
        # map is still growing, every point beyond the walls would pile up as an 'obstacle not in the map' and block passages.
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
        """Of this scan's points not in the map, remember only those seen continuously in the same cell for more than scan_persist s."""
        persist = self.get_parameter('scan_persist').value
        cells = set(map(tuple, np.floor(np.column_stack([xs, ys]) / 0.05).astype(int).tolist()))
        for c in cells:
            seen = self.cell_seen.get(c)
            if seen is None:
                self.cell_seen[c] = [now, now, 1]
            else:
                seen[1], seen[2] = now, seen[2] + 1
        # Cells not seen for over 1 s start counting again (object passed by)
        self.cell_seen = {c: s for c, s in self.cell_seen.items() if now - s[1] <= 1.0}
        steady = [c for c in cells if now - self.cell_seen[c][0] >= persist
                  and self.cell_seen[c][2] >= 0.6 * persist / 0.5]   # scans come every 0.5 s
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
        """Publish remembered obstacles (the bridge sends them to the web). LiDAR points are grouped into 5 cm cells."""
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
        """(map + bump-spot planner, with_scan(radius, since) → (planner with remembered obstacles added or None, cells added)).

        Bump spots (low objects the LiDAR can't see) are kept even when replanning because LiDAR obstacles left no path.
        In SLAM mode (allow_unknown), if the start/goal is still outside the map, the map is extended with unknown cells.
        """
        now = time.monotonic()
        memory = self.get_parameter('scan_memory').value
        while self.scan_points and now - self.scan_points[0][0] > memory:
            self.scan_points.popleft()
        while self.bump_points and now - self.bump_points[0][0] > self.get_parameter('bump_memory').value:
            self.bump_points.popleft()
        info = self.map_msg.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        # If the start is inside a keep-out zone, that zone is left out of this plan; if it is near a zone edge or an
        # inflated leg dot, only the area around the start (start_clear_radius) is cleared (so a robot standing there can get out)
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
        self.plain_planner = plain   # map only, without bump spots (to tell failure messages apart)
        no_scan = grid.copy()        # map + bump spots (base onto which remembered obstacles are added when replanning)

        def with_scan(radius, since=None):
            """Planner with base grid + remembered obstacles (only those added after since), and number of cells added. (None, 0) if none added."""
            grid[:] = no_scan
            n = mark([q for q in self.scan_points if since is None or q[0] >= since])
            if not n:
                return None, 0
            self.last_grid = (grid.copy(), ox, oy, info.resolution)
            return self.make_planner(grid.ravel(), w, h, ox, oy, robot_radius=radius), n

        return base, with_scan

    def log_memory(self):
        """Log the remembered-obstacle state for each plan (if empty, why: cleared / expired / none yet)."""
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
        """Whether keep-out zones are why there is no path. Returns a description string or None (unrelated to zones)."""
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
        """Diagnostics: save the grid to logs/astar_fail.npz when LiDAR obstacles left no path."""
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
                # No obstacles to add (memory empty from the start, or the blocking ones weren't seen recently) → plan below on map (+ bump spots) only
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
            # If the map alone has a path (= remembered obstacles block it), don't use it: it passes through the blocking object and Frenet can't follow it.
            # If the map alone has none either, fail below with the original reason (keep-out zones, etc.)
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
                # Don't use a path through a former bump spot (planning on the map alone would hit the same object again).
                # The memory is cleared after bump_memory s or by the web 'reset state' (astar/clear_obstacles)
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
