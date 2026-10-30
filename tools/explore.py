"""Automatic exploration in SLAM mode. Keeps moving to look at unexplored boundaries (frontiers) until the map stops growing.

Default (--strategy sweep): lane-by-lane sweep (boustrophedon). The area is split into lanes along the starting direction and
covered back and forth one lane at a time; within a lane only frontiers ahead in the travel direction are visited in order
(see the Sweep class). The robot only turns back when changing lanes.
--strategy heading: use only the heading-keeping method of step 2 below (sweep also uses it to clean up leftovers at the end)

    source scripts/env.sh
    python3 tools/explore.py [--max-minutes 20] [--server ws://localhost:8000/ws/ui/]

1. Find frontiers on /map (free cells with an unexplored 4-neighbour) and group connected ones (small groups excluded).
   Cells that only touch small enclosed unexplored holes (under desks, behind panels, smaller than POCKET_MIN) are not frontiers
   (they stay occluded even when the robot goes there, so picking them as the nearest goal only wastes time).
   Unexplored thin gaps between known cells (between LiDAR rays) count as seen.
2. Goal selection: for each group, the first safe free cell near its centre from which rays cast up to the sensor range
   (SENSOR_RANGE) see at least MIN_GAIN of unexplored area before hitting a wall (occluded spots are skipped).
   Heading keeping (going straight): an 'exploration heading' is remembered separately (not the robot's instantaneous pose:
   the local planner may swerve or turn in place on a new goal, so the instantaneous pose often differs from the travel
   direction). If any candidate lies within +-60 deg of it, choose only among those; otherwise widen to +-120 deg, then
   everything -> no going back while somewhere ahead remains.
   (A single score of distance + heading penalty + size would let a near or large frontier behind beat the corridor ahead.)
   Within a tier, score = distance + heading penalty x angle - size bonus.
   (Visible area is only used as a filter: scoring by it to stand at 'the most visible spot' makes every place look alike
   in spaces split by corridors, so the robot shuttles between nearby spots.)
3. Sends the same request as the web "Go here" button (goto_point) to the server, chaining goals without stopping:
   - Within PREFETCH_DIST of the goal, the next goal is picked and sent right away (replace: switches the ongoing move to
     the new goal without stopping).
     A goal left this way is driven to the end if picked again, so a spot only visible from an entrance is not alternated
     between two entrances from 0.5 m away. If it is still not visible on arrival, it is excluded by the rule below.
   - If the target frontier becomes visible (disappears) on the way, go to the next goal at once
   - Failed places are not picked again
4. Ends when no frontier is left (save with the web "Save map")
5. No-go zones (drawn on the web, received over the server WebSocket): zone cells are treated as obstacles, and goals /
   lane-centreline points are picked only from cells reachable from the robot the way A* sees it (robot radius, unknown
   passable) -> no goals behind a zone or in places blocked by one.
   If zones change during a move and the current goal ends up inside/behind one, the move stops and a new goal is picked.
6. Failure: nothing within 0.5 m of a failed spot is picked again (1 m for a second failure at the same spot or a failure
   due to a no-go zone), lane-centreline points included.
   Exploration stops after MAX_FAILS consecutive failures.

    --no-continuous : wait for arrival (stop) at every goal   --no-early-switch : keep going even if the target frontier is seen
    --pocket-min 0 : explore small holes too
"""
import argparse
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import websocket
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from tf2_ros import Buffer, TransformListener

from warehouse_robot.astar import obstacle_distance
from warehouse_robot.plan_map import near_mask, zone_mask

MIN_CLUSTER = 8        # ignore frontier groups smaller than this many cells (40 cm)
CHUNK = 40             # split large frontier groups into pieces of this many cells (2 m), each a goal candidate
CLEARANCE = 0.35       # goal cells must be at least this far from walls (occupied cells) (m)
UNKNOWN_STEPS = (0.20, 0.10, 0.0)  # prefer cells this far from unknown cells too (a cell right next to a wall not yet
                                   # mapped risks a collision). Relaxed to the next value if the group has no such cell
SEARCH_RADIUS = 1.0    # search for a goal cell within this radius of the frontier group centre (m)
VIEW_SAMPLES = 30      # candidates per group whose visible area is checked (closest to the centre first)
SENSOR_RANGE = 5.0     # LiDAR range used for the gain (m). Same as the range SLAM actually fills the map to = slam_toolbox max_laser_range (5.0)
                       # (the LDS-02 measures up to 8-10 m but SLAM only uses 5 m; a smaller value ignores unknown area within SLAM's range and narrows the lanes)
RAYS = 64              # number of rays for the gain
SIZE_WEIGHT = 0.02     # one frontier cell counts as 2 cm less travel (prefer large frontiers)
MIN_GAIN = 0.5         # drop candidates that see less than this (m^2) from their spot (occluded)
TURN_WEIGHT = 0.6      # heading penalty within a direction tier (m/rad)
DIRECTION_TIERS = (60.0, 120.0, 180.0)   # only candidates within +-60 deg of the heading if any, else +-120 deg, then all
LANE_RATIO = 0.65      # lane width = gain range (--sensor-range) x this, so the sensor reliably sees both lane edges from the centre
WAYPOINT_AHEAD = (1.0, 2.5)  # lane-centreline goals are this far ahead of the robot (m)
LANE_END_WAIT = 2.5    # change lanes only after no candidate ahead for this long (s) after stopping (longer than the 2 s SLAM map update)
HEADING_BLEND = 0.3    # when picked from the forward tier, turn the heading only this much toward the new goal (avoids zig-zag)
POCKET_MIN = 1.0       # unknown regions smaller than this (m^2) not touching the map edge are enclosed holes and ignored
MIN_TRAVEL = 0.7       # goals closer than this are not picked (must be larger than PREFETCH_DIST)
PREFETCH_DIST = 0.5    # send the next goal early when this close to the goal (chain without stopping)
VISITED_RADIUS = 0.0   # unknown within this distance of the robot's trail counts as seen (m). Off by default (--visited-radius):
                       # in large rooms the LiDAR can't reach the far walls (rays with no return don't mark free cells), so real
                       # unknown area remains even next to the robot; clearing it can end exploration after the first room
VISIT_STEP = 0.3       # trail recording interval (m)
GAP_CELLS = 2          # unknown strips up to this width (cells) between known cells are ray gaps and get filled
SLAM_JUMP = 0.5        # warn when map->odom jumps by more than this at once (m)
SEEN_RADIUS = 0.6      # 'already seen' when fewer than MIN_CLUSTER frontier cells remain within this radius of the target frontier centre
ROBOT_RADIUS = 0.28    # for reachable cells (same as A* robot_radius)
START_CLEAR = 0.35     # area around the robot counts as passable (same as A* start_clear_radius)
FAIL_RADIUS = (0.5, 1.0)  # exclusion radius around failed spots: first failure / second failure at the same spot or due to a no-go zone
MAX_FAILS = 5          # stop exploring after this many consecutive failures

ZONES = {'list': [], 'ver': 0}   # no-go zones from the server (current map), ver + 1 on every change
_finished = {}                   # task id -> (success, message), so an end state read elsewhere is not missed


def handle_msg(m):
    """Common handling of server WebSocket messages: update no-go zones, remember finished tasks."""
    if m.get('type') == 'zones' or (m.get('type') == 'snapshot' and 'zones' in m):
        zones = m.get('zones') or []
        if zones != ZONES['list']:
            ZONES['list'] = zones
            ZONES['ver'] += 1
            print(f'[explore] 금지 구역 {len(zones)}개 반영', flush=True)
    elif m.get('type') == 'state' and m.get('task') and m['task']['status'] != 'running':
        _finished[m['task']['id']] = (m['task']['status'] == 'done', (m.get('robot') or {}).get('message', ''))


def recv(ws):
    """One message (passed through handle_msg). Raises WebSocketTimeoutException on timeout."""
    m = json.loads(ws.recv())
    handle_msg(m)
    return m


def drain(ws):
    """Read queued messages, applying only no-go zone / task state (does not wait)."""
    ws.settimeout(0.01)
    try:
        while True:
            recv(ws)
    except websocket.WebSocketTimeoutException:
        pass
    finally:
        ws.settimeout(0.5)


def zone_grid(msg):
    info = msg.info
    return zone_mask((info.height, info.width), info.resolution, info.origin.position.x, info.origin.position.y,
                     ZONES['list'])


def reachable(msg, robot_xy, zmask):
    """Cells reachable from the robot (bool): cells (unknown included) at least ROBOT_RADIUS from walls/no-go zones that
    connect to the robot. Same criteria as A* (SLAM mode: unknown passable, area around the start cleared)."""
    info = msg.info
    res = info.resolution
    raw = grid_of(msg)
    wall = raw >= 65
    passable = obstacle_distance(wall | zmask, res, ROBOT_RADIUS + 0.1) >= ROBOT_RADIUS
    near = near_mask(raw.shape, res, info.origin.position.x, info.origin.position.y, robot_xy, START_CLEAR)
    passable |= near & ~wall
    labels, _ = ndimage.label(passable, structure=np.ones((3, 3), bool))
    seeds = np.unique(labels[near & passable])
    seeds = seeds[seeds > 0]
    return np.isin(labels, seeds) if len(seeds) else np.zeros(raw.shape, bool)


def blocked_by(blacklist, xs, ys):
    """Cells inside any blacklist entry [(x, y, radius)] (bool)."""
    out = np.zeros(np.shape(xs), bool)
    for bx, by, br in blacklist:
        out |= np.hypot(xs - bx, ys - by) < br
    return out


def grid_of(msg):
    info = msg.info
    return np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)


def open_unknown(grid, min_cells):
    """Unknown cells belonging to a 'large' region (bool). Large = at least min_cells or touching the map edge (continues
    outside the map). min_cells <= 0 means all unknown cells."""
    unknown = grid < 0
    if min_cells <= 0:
        return unknown
    labels, n = ndimage.label(unknown)
    if n == 0:
        return unknown
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    edge = np.zeros(n + 1, dtype=bool)
    edge[np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))] = True
    keep = (sizes >= min_cells) | edge
    keep[0] = False
    return keep[labels]


def effective_unknown(msg, pocket_min=POCKET_MIN, visited=()):
    """Unknown cells to explore (bool): ray gaps filled, cells near the trail and small enclosed holes removed."""
    info = msg.info
    res = info.resolution
    grid = grid_of(msg)
    known = grid >= 0
    if GAP_CELLS > 0:
        st = np.ones((3, 3), bool)
        filled = ndimage.binary_closing(known, structure=st, iterations=GAP_CELLS, border_value=0)
        grid = np.where(~known & filled, 0, grid)   # filled cells act as free (for frontier detection)
    unknown = open_unknown(grid, int(pocket_min / res ** 2))
    if len(visited) and VISITED_RADIUS > 0:
        seeds = np.zeros(grid.shape, bool)
        v = np.asarray(visited)
        c = ((v[:, 0] - info.origin.position.x) / res).astype(int)
        r = ((v[:, 1] - info.origin.position.y) / res).astype(int)
        ok = (r >= 0) & (r < info.height) & (c >= 0) & (c < info.width)
        seeds[r[ok], c[ok]] = True
        if seeds.any():
            unknown &= ndimage.distance_transform_edt(~seeds) * res > VISITED_RADIUS
    return grid, unknown


def frontier_mask(grid, unknown):
    """Frontier cells (bool): free cells with a (large-region) unknown 4-neighbour."""
    near = np.zeros_like(unknown)
    near[1:, :] |= unknown[:-1, :]
    near[:-1, :] |= unknown[1:, :]
    near[:, 1:] |= unknown[:, :-1]
    near[:, :-1] |= unknown[:, 1:]
    return (grid == 0) & near


def frontiers(front):
    """Frontier cells -> connected groups [ [(r, c), ...], ... ] (large groups split into CHUNK-cell pieces)."""
    h, w = front.shape
    seen = np.zeros_like(front)
    clusters = []
    for r0, c0 in zip(*np.nonzero(front)):
        if seen[r0, c0]:
            continue
        q, cells = deque([(r0, c0)]), []
        seen[r0, c0] = True
        while q:
            r, c = q.popleft()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and front[nr, nc] and not seen[nr, nc]:
                        seen[nr, nc] = True
                        q.append((nr, nc))
        if len(cells) >= MIN_CLUSTER:
            # BFS order is spatially contiguous, so slicing it in order yields pieces
            for i in range(0, len(cells), CHUNK):
                part = cells[i:i + CHUNK]
                if len(part) >= MIN_CLUSTER:
                    clusters.append(part)
    return clusters


def view_gain(rows, cols, occ, unknown, res, sensor_range=SENSOR_RANGE, rays=RAYS):
    """Unknown area visible from each candidate cell (rows[i], cols[i]) (m^2).
    For each ray, unknown cells passed before hitting an occupied cell are summed as sector area at that distance
    (r*dtheta*res). Outside the map counts as unknown."""
    n = int(sensor_range / res)
    ang = np.linspace(0, 2 * np.pi, rays, endpoint=False)
    k = np.arange(1, n + 1)
    dr = np.rint(np.sin(ang)[:, None] * k[None, :]).astype(int)        # (rays, n)
    dc = np.rint(np.cos(ang)[:, None] * k[None, :]).astype(int)
    rr = rows[:, None, None] + dr[None]                                 # (N, rays, n)
    cc = cols[:, None, None] + dc[None]
    h, w = occ.shape
    inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
    rcl, ccl = np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)
    hit = occ[rcl, ccl] & inside
    blocked = np.cumsum(hit, axis=2) > 0
    unk = (unknown[rcl, ccl] | ~inside) & ~blocked
    weight = k * res * (2 * np.pi / rays) * res                         # area represented by one cell at distance k
    return (unk * weight[None, None, :]).sum(axis=(1, 2))


def frontier_left(msg, fx, fy, pocket_min=POCKET_MIN, visited=()):
    """Number of frontier cells within SEEN_RADIUS of (fx, fy). Used to tell if the target was already seen on the way."""
    info = msg.info
    res = info.resolution
    front = frontier_mask(*effective_unknown(msg, pocket_min, visited))
    c = int((fx - info.origin.position.x) / res)
    r = int((fy - info.origin.position.y) / res)
    k = int(SEEN_RADIUS / res) + 1
    rr, cc = np.nonzero(front[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1])
    rr = rr + max(0, r - k)
    cc = cc + max(0, c - k)
    return int(np.count_nonzero((rr - r) ** 2 + (cc - c) ** 2 <= k * k))


def choose_by_heading(cands, robot_xy, heading=None, turn_weight=TURN_WEIGHT):
    """Best-scoring candidate in the first direction tier (DIRECTION_TIERS, in order) that has any candidate.
    Returns (x, y, yaw, visible area, frontier centre x, y, direction tier) or None."""
    for tier, limit in enumerate(DIRECTION_TIERS):
        best = None
        for gx, gy, bearing, gain, fx, fy, size in cands:
            travel = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
            cost = travel - SIZE_WEIGHT * size                 # prefer near and large frontiers
            if heading is not None:
                off = abs(math.atan2(math.sin(bearing - heading), math.cos(bearing - heading)))
                if math.degrees(off) > limit:
                    continue
                cost += turn_weight * off
            if best is None or cost < best[0]:
                best = (cost, gx, gy, bearing, gain, fx, fy, tier)
        if best is not None:
            return best[1:]
    return None


class Sweep:
    """Lane-by-lane sweep (boustrophedon, lawn-mower pattern). Picking 'the best frontier every time' zig-zags back and forth
    where structures repeat on both sides (e.g. rows of seats), because boundaries appear in every direction at once.
    - Lane direction (u) is the dominant wall direction found in the map (wall_axis) closest to the robot heading (the robot
      heading if no walls yet); lanes of width lane_width (sensor range x LANE_RATIO) are laid out along its left normal (v)
    - In the current lane, only candidates ahead in the travel direction are visited, nearest first
    - With no candidate ahead, move to the next lane and reverse direction (nearest lane with candidates; the first time,
      the side with more candidates)
    - The lane-end decision is not rushed: while moving, finish the current goal ('wait'); after stopping, change lanes only
      if there is still no candidate ahead after another map update (LANE_END_WAIT). A brief lack of candidates ahead is
      usually temporary (going a bit further lets the LiDAR find new boundaries), so switching immediately goes back and forth"""

    def __init__(self, x0, y0, yaw0, lane_width):
        self.x0, self.y0 = x0, y0
        self.ux, self.uy = math.cos(yaw0), math.sin(yaw0)
        self.vx, self.vy = -math.sin(yaw0), math.cos(yaw0)
        self.w = lane_width
        self.lane = 0
        self.dir = 1          # direction of travel within the lane (+u / -u)
        self.side = None      # side to step lanes toward (+v / -v), set on the first lane change to the side with more candidates
        self.end_since = None # time when no candidate ahead in the current lane was first seen

    def uv(self, x, y):
        dx, dy = x - self.x0, y - self.y0
        return dx * self.ux + dy * self.uy, dx * self.vx + dy * self.vy

    def lane_of(self, x, y):
        return int(math.floor(self.uv(x, y)[1] / self.w + 0.5))

    def pick(self, cands, robot_xy, moving, now, waypoint=None):
        """Returns (candidate, label) / (None, 'wait': not yet sure the lane has ended, retry shortly) / (None, ''): no candidate.
        waypoint(lane, dir): a reachable point (x, y) ahead on the lane centreline, or None.
        While frontiers remain ahead in the lane, drive straight via centreline points instead of frontier cells (so the
        robot doesn't go in and out of every side opening; the LiDAR sees inside while passing). If the centreline can't
        be followed further, go to the frontier cell."""
        if not cands:
            return None, ''
        ru, _ = self.uv(*robot_xy)
        lanes = {}
        for c in cands:
            lanes.setdefault(self.lane_of(c[0], c[1]), []).append(c)

        def ahead_in(lane, d):
            return [c for c in lanes.get(lane, []) if (self.uv(c[0], c[1])[0] - ru) * d > -0.3]

        ahead = ahead_in(self.lane, self.dir)
        if ahead:
            self.end_since = None
            arrow = "→" if self.dir > 0 else "←"
            far = max((self.uv(c[0], c[1])[0] - ru) * self.dir for c in ahead)
            wp = waypoint(self.lane, self.dir) if waypoint else None
            if wp is not None and (self.uv(*wp)[0] - ru) * self.dir <= far + 1.0:
                bearing = math.atan2(wp[1] - robot_xy[1], wp[0] - robot_xy[0])
                return (wp[0], wp[1], bearing, 0.0, None, None, 0), f'줄 {self.lane} {arrow} 중심선'
            c = min(ahead, key=lambda c: (self.uv(c[0], c[1])[0] - ru) * self.dir)
            return c, f'줄 {self.lane} {arrow}'
        if moving:
            return None, 'wait'
        if self.end_since is None:
            self.end_since = now
        if now - self.end_since < LANE_END_WAIT:
            return None, 'wait'
        # lane end: nearest next lane with candidates (same side first, else the other side)
        self.end_since = None
        if self.side is None:
            left = sum(len(v) for k, v in lanes.items() if k > self.lane)
            right = sum(len(v) for k, v in lanes.items() if k < self.lane)
            self.side = 1 if left >= right else -1
        nxt = [k for k in lanes if (k - self.lane) * self.side > 0]
        if not nxt:
            self.side = -self.side
            nxt = [k for k in lanes if (k - self.lane) * self.side > 0]
        if not nxt:   # candidates only behind in this lane: reverse along it
            self.dir = -self.dir
        else:
            self.lane = min(nxt, key=lambda k: abs(k - self.lane))
            self.dir = -self.dir
            if not ahead_in(self.lane, self.dir):   # new lane has candidates only in the opposite direction: go that way
                self.dir = -self.dir
        ahead = ahead_in(self.lane, self.dir)
        if not ahead:
            return None, ''
        c = min(ahead, key=lambda c: (self.uv(c[0], c[1])[0] - ru) * self.dir)
        return c, f'줄 {self.lane} {"→" if self.dir > 0 else "←"} (줄 바꿈)'


def wall_axis(msg, min_points=40):
    """Direction (rad, 0-90 deg) along which map walls (occupied cells) best line up in rows/columns, or None (too few walls).
    Same method as the skew estimate in tools/straighten_map.py (angle whose rotated projection histograms are sharpest)."""
    info = msg.info
    res = info.resolution
    rr, cc = np.nonzero(grid_of(msg) >= 65)
    if len(rr) < min_points:
        return None
    x, y = cc * res, rr * res
    best = (-1, 0.0)
    for deg in range(0, 90):
        t = math.radians(deg)
        c, s = math.cos(t), math.sin(t)
        u, v = c * x + s * y, -s * x + c * y
        hu = np.bincount(np.floor((u - u.min()) / res).astype(int))
        hv = np.bincount(np.floor((v - v.min()) / res).astype(int))
        score = int((hu.astype(np.int64) ** 2).sum() + (hv.astype(np.int64) ** 2).sum())
        if score > best[0]:
            best = (score, t)
    return best[1]


def lane_heading(msg, robot_yaw):
    """Lane direction: the dominant wall direction (of 4) closest to the robot heading; the robot heading if no walls found."""
    axis = wall_axis(msg)
    if axis is None:
        return robot_yaw
    options = [axis + k * math.pi / 2 for k in range(4)]
    return min(options, key=lambda a: abs(math.atan2(math.sin(a - robot_yaw), math.cos(a - robot_yaw))))


def lane_waypoint(msg, sweep, robot_xy, lane, d, allowed=None, blacklist=(), exclude=()):
    """A safe free cell (x, y) near the lane centreline (v = lane x lane width), WAYPOINT_AHEAD ahead of the robot in
    direction d, or None. Further ahead and closer to the centreline is better. allowed: reachable cells (no-go zones
    applied), blacklist/exclude: failed spots / current goal."""
    info = msg.info
    res = info.resolution
    raw = grid_of(msg)
    dist = obstacle_distance(raw >= 65, res, CLEARANCE + 0.1)
    safe = (raw == 0) & (dist >= CLEARANCE)
    if allowed is not None:
        safe &= allowed
    rr, cc = np.nonzero(safe)
    if len(rr) == 0:
        return None
    xs = info.origin.position.x + (cc + 0.5) * res
    ys = info.origin.position.y + (rr + 0.5) * res
    dx, dy = xs - sweep.x0, ys - sweep.y0
    u = dx * sweep.ux + dy * sweep.uy
    v = dx * sweep.vx + dy * sweep.vy
    ru = sweep.uv(*robot_xy)[0]
    prog = (u - ru) * d
    off = np.abs(v - lane * sweep.w)
    ok = (prog >= WAYPOINT_AHEAD[0]) & (prog <= WAYPOINT_AHEAD[1]) & (off <= sweep.w / 2)
    ok &= ~blocked_by(blacklist, xs, ys)
    for ex, ey in exclude:
        ok &= np.hypot(xs - ex, ys - ey) >= 0.8
    if not ok.any():
        return None
    score = np.where(ok, prog - 1.5 * off, -np.inf)
    i = int(np.argmax(score))
    return float(xs[i]), float(ys[i])


def _candidates(msg, robot_xy, blacklist, pocket_min, exclude, sensor_range, visited, allowed=None, zmask=None):
    """One goal candidate per frontier group: [(x, y, bearing, visible area m^2, frontier centre x, y, group cell count)].
    blacklist [(x, y, radius)] / exclude: candidates within that radius / 0.8 m are dropped (failed spots / current goal).
    allowed: cells reachable from the robot (reachable), zmask: no-go zone cells - frontiers and goals inside are dropped.
    The goal cell is CLEARANCE from walls and UNKNOWN_STEPS from unknown cells (the strictest margin the group has cells for).
    The margin is relaxed per group: relaxing over the whole map would drop narrow-corridor groups (no cells at the strict
    margin) whenever an open-area group has one, sending the robot backwards regardless of the direction rule."""
    info = msg.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    raw = grid_of(msg)
    occ = raw >= 65
    grid, unknown = effective_unknown(msg, pocket_min, visited)
    dist = obstacle_distance(occ, res, CLEARANCE + 0.1)
    base = (raw == 0) & (dist >= CLEARANCE)      # don't stand on filled gap cells
    if allowed is not None:
        base &= allowed
    dist_unknown = obstacle_distance(raw < 0, res, max(UNKNOWN_STEPS) + 0.1)
    levels = [(base & (dist_unknown >= u), mt) for u in UNKNOWN_STEPS for mt in (MIN_TRAVEL, 0.5)]
    k = int(SEARCH_RADIUS / res)
    out = []
    front = frontier_mask(grid, unknown)
    if zmask is not None:
        front &= ~zmask
    for cells in frontiers(front):
        cr = int(np.mean([c[0] for c in cells]))
        cc = int(np.mean([c[1] for c in cells]))
        r0, r1 = max(0, cr - k), min(info.height, cr + k + 1)
        c0, c1 = max(0, cc - k), min(info.width, cc + k + 1)
        for safe, min_travel in levels:
            rr, ccs = np.nonzero(safe[r0:r1, c0:c1])
            rr, ccs = rr + r0, ccs + c0
            if len(rr) == 0:
                continue
            xs, ys = ox + (ccs + 0.5) * res, oy + (rr + 0.5) * res
            ok = np.hypot(xs - robot_xy[0], ys - robot_xy[1]) >= min_travel
            ok &= ~blocked_by(blacklist, xs, ys)
            for ex, ey in exclude:
                ok &= np.hypot(xs - ex, ys - ey) >= 0.8
            if ok.any():
                break
        else:
            continue
        # nearest the frontier centre first: the first cell that actually sees some area (occluded spots skipped)
        order = np.flatnonzero(ok)[np.argsort(((rr - cr) ** 2 + (ccs - cc) ** 2)[ok])][:VIEW_SAMPLES]
        gains = view_gain(rr[order], ccs[order], occ, unknown, res, sensor_range)
        good = np.flatnonzero(gains >= MIN_GAIN)
        if len(good) == 0:
            continue
        j = order[good[0]]
        gx, gy = float(xs[j]), float(ys[j])
        # arrival heading = travel direction (the LiDAR is 360 deg, so no extra turn on arrival)
        bearing = math.atan2(gy - robot_xy[1], gx - robot_xy[0])
        out.append((gx, gy, bearing, float(gains[good[0]]), ox + (cc + 0.5) * res, oy + (cr + 0.5) * res, len(cells)))
    return out


def wait_idle(ws, timeout=3.0):
    """Wait after a cancel until the task-ended state arrives (so the next goto is not rejected as 'in progress')."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            m = recv(ws)
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'state' and not (m.get('task') and m['task']['status'] == 'running'):
            return


_last = {'task': None}   # id of the last move task sent (to tell it apart from the previous task when chaining)


def goto(ws, x, y, yaw, timeout, check=None, replace=False):
    """Send goto_point to the server and wait until the task ends or check() returns a reason.
    Returns (success, message). check(): called every 0.5 s on the way; if it returns a string, return (True, that string)
    without stopping (the move is still running -> send the next goto with replace=True to chain).
    replace: switch the ongoing move to this goal (without stopping the robot)."""
    ws.send(json.dumps({'action': 'goto_point', 'x': x, 'y': y, 'yaw': yaw, 'replace': replace}))
    end = time.time() + timeout
    task_id = None
    next_check = time.time() + 0.5
    while time.time() < end:
        if check is not None and task_id is not None and time.time() >= next_check:
            next_check = time.time() + 0.5
            why = check()
            if why:
                return True, why
        try:
            m = recv(ws)
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'error':
            return None, m['message']   # the request itself was rejected, not the move (task in progress etc.)
        if m['type'] == 'state' and m.get('task'):
            t = m['task']
            if task_id is None and t['status'] == 'running' and t['id'] != _last['task']:
                task_id = t['id']
                _last['task'] = task_id
            if t['id'] == task_id and t['status'] != 'running':
                return t['status'] == 'done', m['robot'].get('message', '')
    ws.send(json.dumps({'action': 'cancel_task'}))
    wait_idle(ws)
    return False, f'{timeout:.0f}초 안에 도착하지 못함'


def main():
    global VISITED_RADIUS
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-minutes', type=float, default=20.0)
    ap.add_argument('--goal-timeout', type=float, default=90.0)
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--turn-weight', type=float, default=TURN_WEIGHT, help='같은 방향 단계 안의 방향 벌점 (m/rad)')
    ap.add_argument('--no-early-switch', action='store_true', help='목표 프론티어가 보여도 도착까지 감')
    ap.add_argument('--no-continuous', action='store_true', help='목표마다 도착(정지)까지 기다림')
    ap.add_argument('--pocket-min', type=float, default=POCKET_MIN,
                    help='이보다 작은(m²) 막힌 미탐색 구멍은 무시, 0 이면 전부 탐색')
    ap.add_argument('--strategy', choices=('sweep', 'heading'), default='sweep',
                    help='sweep: 줄 단위로 훑기(기본), heading: 프론티어 + 방향 유지')
    ap.add_argument('--lane-width', type=float, default=0.0,
                    help='줄 단위 훑기의 줄 폭 (m). 0 이면 --sensor-range x %.2f' % LANE_RATIO)
    ap.add_argument('--visited-radius', type=float, default=VISITED_RADIUS,
                    help='지나온 경로에서 이 거리(m) 안의 미탐색은 본 것으로 침 (0 = 끔)')
    ap.add_argument('--debug-dir', default='', help='고를 때마다 맵/후보를 npz 로 저장 (분석용)')
    ap.add_argument('--sensor-range', type=float, default=SENSOR_RANGE, help='이득 계산에 쓰는 LiDAR 거리 (m)')
    args = ap.parse_args()
    VISITED_RADIUS = args.visited_radius

    rclpy.init()
    node = Node('explore_tool', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=os.environ.get('WAREHOUSE_SIM_TIME') == 'true')])
    buf = Buffer()
    TransformListener(buf, node)
    latest = {}
    node.create_subscription(OccupancyGrid, 'map', lambda m: latest.__setitem__('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    ws = websocket.create_connection(args.server, timeout=0.5)
    snap = recv(ws)
    if snap.get('task') and snap['task']['status'] == 'running':
        print('[explore] 진행 중인 작업을 취소하고 시작합니다', flush=True)
        ws.send(json.dumps({'action': 'cancel_task'}))
        wait_idle(ws)

    trail = []   # robot trail (every VISIT_STEP, map coordinates)
    last_m_o = []  # previous map->odom (SLAM correction); warn on sudden large jumps (e.g. a wrong loop closure in repetitive structure)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            x, y, _ = robot_pose()
        except Exception:
            return
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= VISIT_STEP:
            trail.append((x, y))
        try:
            t = buf.lookup_transform('map', 'odom', rclpy.time.Time()).transform.translation
        except Exception:
            return
        if last_m_o and math.hypot(t.x - last_m_o[0], t.y - last_m_o[1]) > SLAM_JUMP:
            print(f'[explore] 경고: SLAM 위치 보정이 갑자기 {math.hypot(t.x - last_m_o[0], t.y - last_m_o[1]):.2f}m 바뀜 '
                  '(잘못된 루프 클로저일 수 있음, 맵 확인 필요)', flush=True)
        last_m_o[:] = [t.x, t.y]

    def robot_pose():
        tf = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        q = tf.rotation
        return tf.translation.x, tf.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    blacklist = []  # [(x, y, radius)] failed spots
    failed_at = []  # failed goals (to detect a second failure at the same spot)
    fails = 0       # consecutive failures
    reached = []    # reached goals; if the frontier remains after reaching the same spot twice (not visible from there), exclude it
    start = time.time()
    visited = ok_count = switched = chained = 0
    moving = False  # still moving to the previous goal (next goto chains with replace)
    exclude = []    # when prefetching: don't re-pick near the current goal
    left_early = [] # goals left before arrival for the next one; driven to the end if picked again
    # right after SLAM starts the map is tiny and there is nothing to pick (would end as 'done'), so wait until it grows a bit
    wait_end = time.time() + 10.0
    while time.time() < wait_end:
        spin(0.5)
        m = latest.get('map')
        if m is not None and np.count_nonzero(np.asarray(m.data) >= 0) * m.info.resolution ** 2 >= 3.0:
            break
    heading = None  # exploration heading (rad); the robot pose before the first goal
    sweep = None    # lane sweep state (--strategy sweep)

    def choose(rx, ry, excl):
        """Returns (x, y, yaw, visible area, frontier centre x, y, label) or None."""
        m = latest['map']
        zm = zone_grid(m)
        reach = reachable(m, (rx, ry), zm)
        cands = _candidates(m, (rx, ry), blacklist, args.pocket_min, excl, args.sensor_range, trail, reach, zm)
        if not cands:
            return None
        if sweep is not None:
            c, label = sweep.pick(cands, (rx, ry), moving, time.time(),
                                  lambda lane, d: lane_waypoint(m, sweep, (rx, ry), lane, d, reach, blacklist, excl))
            if label == 'wait':
                return 'wait'
            if c is not None:
                return c[0], c[1], c[2], c[3], c[4], c[5], label
        g = choose_by_heading(cands, (rx, ry), heading, args.turn_weight)
        if g is None:
            return None
        return g[:6] + ((('자투리 ' if sweep is not None else '') + ('앞', '옆', '뒤')[g[6]]),)
    def fail(gx, gy, message):
        """Exclude a failed goal and count consecutive failures. Returns True if exploration should stop."""
        nonlocal fails
        again = any(math.hypot(gx - px, gy - py) < FAIL_RADIUS[0] for px, py in failed_at)
        r = FAIL_RADIUS[1] if again or '금지 구역' in message else FAIL_RADIUS[0]
        failed_at.append((gx, gy))
        blacklist.append((gx, gy, r))
        fails += 1
        if fails >= MAX_FAILS:
            print(f'[explore] 연속 {fails}번 실패해 탐색을 멈춥니다 (마지막: {message})', flush=True)
            return True
        return False

    while time.time() - start < args.max_minutes * 60:
        spin(0.05 if moving else 0.3)
        drain(ws)   # apply no-go zone changes
        if 'map' not in latest or not buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
            print('[explore] 맵/위치 대기 중', flush=True)
            spin(2.0)
            continue
        rx, ry, ryaw = robot_pose()
        if heading is None:
            heading = ryaw
            if args.strategy == 'sweep':
                lane_yaw = lane_heading(latest['map'], ryaw)
                lane_w = args.lane_width or args.sensor_range * LANE_RATIO
                sweep = Sweep(rx, ry, lane_yaw, lane_w)
                print(f'[explore] 줄 단위 훑기: 줄 방향 {math.degrees(lane_yaw):.0f}° (로봇 {math.degrees(ryaw):.0f}°), '
                      f'줄 폭 {lane_w:.1f}m', flush=True)
        goal = choose(rx, ry, exclude)
        if goal == 'wait' and not moving:
            spin(0.5)          # wait for a map update and recheck whether the lane has ended
            continue
        if goal == 'wait':
            goal = None        # moving: finish the current goal (below)
        if goal is None and moving:
            # nothing to chain to: finish the current goal
            exclude = []
            moving = False
            goto_wait(ws, args.goal_timeout)
            continue
        if goal is None:
            # the SLAM map update may lag, so wait once more and recheck
            spin(4.0)
            rx, ry, ryaw = robot_pose()
            goal = choose(rx, ry, ())
            while goal == 'wait':
                spin(0.5)
                goal = choose(rx, ry, ())
            if goal is None:
                print('[explore] 남은 프론티어 없음: 탐색 완료', flush=True)
                if args.debug_dir:
                    m = latest['map']
                    np.savez_compressed(os.path.join(args.debug_dir, 'final.npz'), grid=grid_of(m), res=m.info.resolution,
                                        origin=[m.info.origin.position.x, m.info.origin.position.y],
                                        robot=[rx, ry, ryaw], trail=np.array(trail),
                                        blacklist=np.array(blacklist).reshape(-1, 3))
                break
        gx, gy, gyaw, gain, fx, fy, label = goal
        tier = 0 if not label.endswith(('옆', '뒤')) else (1 if label.endswith('옆') else 2)
        visited += 1
        if args.debug_dir:   # for analysis: map, robot, heading and all candidates at pick time
            m = latest['map']
            np.savez_compressed(os.path.join(args.debug_dir, f'pick_{visited:03d}.npz'),
                                grid=grid_of(m), res=m.info.resolution,
                                origin=[m.info.origin.position.x, m.info.origin.position.y],
                                robot=[rx, ry, ryaw], heading=heading, goal=[gx, gy, tier], label=label,
                                trail=np.array(trail),
                                exclude=np.array(exclude).reshape(-1, 2), blacklist=np.array(blacklist).reshape(-1, 3))
        turn = math.degrees(abs(math.atan2(math.sin(gyaw - heading), math.cos(gyaw - heading))))
        print(f'[explore] #{visited} 목표 ({gx:.2f}, {gy:.2f}) 보일 넓이 {gain:.1f} m², 진행 방향과 {turn:.0f}° '
              f'[{label}]{" (이어 가기)" if moving else ""}', flush=True)
        # update heading: slightly if picked ahead, fully to the new direction if picked from the side/back tiers
        if tier == 0:
            hx = (1 - HEADING_BLEND) * math.cos(heading) + HEADING_BLEND * math.cos(gyaw)
            hy = (1 - HEADING_BLEND) * math.sin(heading) + HEADING_BLEND * math.sin(gyaw)
            heading = math.atan2(hy, hx)
        else:
            heading = gyaw

        # drive to the end if this goal was left before arrival earlier, or is about as close as the prefetch distance
        # (not for lane-centreline goals, which keep moving ahead)
        is_waypoint = fx is None
        commit = not is_waypoint and (any(math.hypot(gx - px, gy - py) < 0.8 for px, py in left_early)
                                      or math.hypot(gx - rx, gy - ry) < PREFETCH_DIST + 0.2)
        if commit and math.hypot(gx - rx, gy - ry) >= PREFETCH_DIST + 0.2:
            print('[explore]   전에 도착 전에 떠난 곳: 이번엔 끝까지 감', flush=True)

        zone_ver = ZONES['ver']

        def check():
            nonlocal zone_ver
            spin(0.05)
            if ZONES['ver'] != zone_ver:   # no-go zones changed on the way: is the goal now inside/behind one?
                zone_ver = ZONES['ver']
                m = latest['map']
                x, y, _ = robot_pose()
                info = m.info
                c = int((gx - info.origin.position.x) / info.resolution)
                r = int((gy - info.origin.position.y) / info.resolution)
                inside = 0 <= r < info.height and 0 <= c < info.width
                if not inside or not reachable(m, (x, y), zone_grid(m))[r, c]:
                    return '금지 구역 변경'
            if not args.no_continuous and not commit:
                x, y, _ = robot_pose()
                if math.hypot(gx - x, gy - y) < PREFETCH_DIST:
                    return '근접'
            if (not args.no_early_switch and fx is not None
                    and frontier_left(latest['map'], fx, fy, args.pocket_min, trail) < MIN_CLUSTER):
                return '이미 봄'
            return None

        ok, message = goto(ws, gx, gy, gyaw, args.goal_timeout, check, replace=moving)
        moving = False
        exclude = []
        if ok and message == '금지 구역 변경':
            print('[explore]   금지 구역이 바뀌어 지금 목표로 갈 수 없음: 멈추고 다시 고름', flush=True)
            ws.send(json.dumps({'action': 'cancel_task'}))
            wait_idle(ws)
            blacklist.append((gx, gy, FAIL_RADIUS[1]))
            continue
        if ok:
            fails = 0
        if ok and message in ('근접', '이미 봄'):
            if message == '이미 봄':
                switched += 1
                print('[explore]   가는 도중 목표 프론티어가 보임: 다음 목표로', flush=True)
            else:
                chained += 1
                if not is_waypoint:
                    exclude = [(gx, gy)]
                    left_early.append((gx, gy))
            if args.no_continuous:
                ws.send(json.dumps({'action': 'cancel_task'}))
                wait_idle(ws)
            else:
                moving = True
            continue
        if ok:
            ok_count += 1
            if is_waypoint:
                continue
            if any(math.hypot(gx - px, gy - py) < 0.4 for px, py in reached):
                blacklist.append((gx, gy, FAIL_RADIUS[0]))
                print('[explore]   같은 곳에 다시 왔지만 프론티어가 남음: 이 근처는 제외', flush=True)
            reached.append((gx, gy))
        elif ok is None and ('금지 구역' in message or '같은 지점' in message):
            # the server rejected the goal itself (inside a no-go zone, repeated failure at the same spot): count as a failure
            print(f'[explore]   요청 거절: {message}', flush=True)
            if fail(gx, gy, message):
                break
        elif ok is None:
            print(f'[explore]   요청 거절: {message} (잠시 뒤 다시 시도)', flush=True)
            spin(3.0)
        else:
            print(f'[explore]   실패: {message}', flush=True)
            if any(w in message for w in ('넘어', '직접 확인', '비상정지')):
                # the robot needs a person to check it: all further goals would be rejected, excluded, and wrongly end as 'done'
                print('[explore] 로봇 확인이 필요해 탐색을 멈춥니다 (상태 초기화 후 다시 시작)', flush=True)
                break
            if fail(gx, gy, message):
                break
    else:
        print(f'[explore] 시간 제한 {args.max_minutes:.0f}분 도달', flush=True)
    if moving:
        ws.send(json.dumps({'action': 'cancel_task'}))
        wait_idle(ws)
    m = latest.get('map')
    known = int(np.count_nonzero(np.array(m.data) >= 0)) * m.info.resolution ** 2 if m else 0
    print(f'[explore] 끝: 목표 {visited}개 (도착 {ok_count}, 이어 가기 {chained}, 도중 전환 {switched}), '
          f'탐색한 면적 약 {known:.1f} m², {(time.time() - start) / 60:.1f}분', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


def goto_wait(ws, timeout):
    """Wait only for the result of the ongoing goto (no new request). Returns (success, message)"""
    end = time.time() + timeout
    task_id = _last['task']
    if task_id in _finished:   # end state already read elsewhere (drain)
        return _finished[task_id]
    while time.time() < end:
        try:
            m = recv(ws)
        except websocket.WebSocketTimeoutException:
            continue
        if m['type'] == 'state' and m.get('task') and m['task']['id'] == task_id and m['task']['status'] != 'running':
            return m['task']['status'] == 'done', m['robot'].get('message', '')
        if m['type'] == 'state' and not m.get('task'):
            return True, ''
    return False, '시간 초과'


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  # web "Stop exploration" (SIGINT)
        print('[explore] 중지됨', flush=True)
        raise SystemExit(130)
