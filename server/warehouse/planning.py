"""Travel time prediction and visit order optimization.

- Path: compute path length and total turning with A* (astar.py from ros2_ws, no ROS dependency) on the active map.
  The planner is rebuilt when the map changes (its update time changes). Saved maps are processed with plan_map.py the same
  way as the robot's A* (fill stripes, inflate desk/chair leg points, keep-out zones). If the start is inside a zone/inflated
  point, that zone / 0.35 m around it is excluded, as on the robot.
- Travel time model: time = base + path length x distance coef + turning x turn coef.
  Learned from past travel records (TaskStep plan_length / plan_turn / actual time).
  It learns only the robot's driving characteristics, not locations, so it still applies after the map changes.
  With few records it is pulled toward the defaults (ridge); slow moves that got blocked (outliers) are dropped and it is refit.
- Order optimization: the order minimizing total travel time start (robot) -> selected locations -> (home).
  Exhaustive for 8 or fewer, otherwise nearest-first + 2-opt.
"""
import itertools
import math
import os
import sys
import threading

import numpy as np
from django.conf import settings
from django.utils import timezone

from .mapimage import decode_png, gray_to_occupancy

sys.path.insert(0, os.path.join(os.path.dirname(settings.BASE_DIR), 'ros2_ws', 'src', 'warehouse_robot'))
from warehouse_robot.astar import GridPlanner, PlanningError  # noqa: E402
from warehouse_robot.plan_map import INSIDE_MARGIN, near_mask, prepare, with_added, zone_contains, zone_mask  # noqa: E402

# Defaults when there are no records: base 2 s, 0.13 m/s, turning 0.8 rad/s
PRIOR = np.array([2.0, 1 / 0.13, 1 / 0.8])
RIDGE = 5.0        # Strength of the pull toward the defaults when records are few
MIN_SAMPLES = 3

_lock = threading.Lock()
_planner_cache = {}   # (map_id, updated_at, allow_unknown, zones) -> (planner, processed base grid, inflated cells, zone list)
START_CLEAR = 0.35    # astar_planner_node start_clear_radius


class PlanError(Exception):
    pass


# ---------------------------------------------------------------- path
def _planner(m):
    allow_unknown = m.name == 'slam_live'   # A map being built may pass through unknown cells (same as the bridge)
    zones = [z.to_dict() for z in m.zones.all()]
    key = (m.id, m.updated_at, allow_unknown, tuple(tuple(sorted(z.items())) for z in zones))
    with _lock:
        p = _planner_cache.get(key)
        if p is None:
            width, height, rows = decode_png(bytes(m.png))
            lut = [gray_to_occupancy(v) for v in range(256)]
            grid = np.array([[lut[v] for v in row] for row in rows], dtype=np.int16)[::-1]  # row 0 = bottom
            inflated = np.zeros(grid.shape, dtype=bool)
            if not allow_unknown:   # Same processing as the robot's A* (astar_planner_node defaults)
                grid, inflated, _ = prepare(grid, m.resolution)
            added = inflated | zone_mask(grid.shape, m.resolution, m.origin_x, m.origin_y, zones)
            planner = GridPlanner(with_added(grid, added, m.resolution, m.origin_x, m.origin_y).ravel(),
                                  width, height, m.resolution, m.origin_x, m.origin_y, allow_unknown=allow_unknown)
            p = (planner, grid, inflated, zones)
            _planner_cache.clear()
            _planner_cache[key] = p
        return p


def _angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def path_metrics(m, start, goal):
    """start, goal: (x, y, yaw). Returns (path length m, total turning rad). PlanError if there is no path."""
    planner, grid, inflated, zones = _planner(m)
    others = [z for z in zones if not zone_contains(z, start[:2], INSIDE_MARGIN)]
    added = inflated | zone_mask(grid.shape, m.resolution, m.origin_x, m.origin_y, others)
    if len(others) < len(zones) or (added & near_mask(grid.shape, m.resolution, m.origin_x, m.origin_y,
                                                      start[:2], START_CLEAR)).any():
        cleared = with_added(grid, added, m.resolution, m.origin_x, m.origin_y, start[:2], START_CLEAR)
        planner = GridPlanner(cleared.ravel(), planner.width, planner.height, m.resolution,
                              m.origin_x, m.origin_y, allow_unknown=m.name == 'slam_live')
    try:
        pts = planner.plan(start[:2], goal[:2])
    except PlanningError as e:
        raise PlanError(str(e))
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    # Headings are taken every 5 cells (~25 cm) to reduce jitter from grid staircasing
    sub = pts[::5] + ([pts[-1]] if (len(pts) - 1) % 5 else [])
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(sub, sub[1:])
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6]
    turn = 0.0
    if headings:
        turn += abs(_angle(headings[0] - start[2]))           # Align heading at the start
        turn += sum(abs(_angle(b - a)) for a, b in zip(headings, headings[1:]))
        turn += abs(_angle(goal[2] - headings[-1]))           # Align to the goal heading
    else:
        turn = abs(_angle(goal[2] - start[2]))
    return length, turn


# ---------------------------------------------------------------- travel time model
def _samples():
    from .models import TaskStep
    rows = TaskStep.objects.filter(status__in=[TaskStep.STATUS_DONE, TaskStep.STATUS_ARRIVED],
                                   plan_length__isnull=False, started_at__isnull=False,
                                   finished_at__isnull=False)
    X, y = [], []
    for s in rows:
        sec = (s.finished_at - s.started_at).total_seconds()
        if 0 < sec < 600:
            X.append([1.0, s.plan_length, s.plan_turn or 0.0])
            y.append(sec)
    return np.array(X).reshape(-1, 3), np.array(y)


def _fit(X, y):
    """Least squares pulled toward the defaults: min |Xw - y|^2 + RIDGE |w - PRIOR|^2, coefficients >= 0."""
    A = X.T @ X + RIDGE * np.eye(3)
    b = X.T @ y + RIDGE * PRIOR
    return np.maximum(np.linalg.solve(A, b), 0.0)


def travel_model():
    """Learned model and stats. {'w': [...], 'n': records used, 'outliers': records dropped, 'mae': mean error (s), ...}"""
    X, y = _samples()
    info = {'w': PRIOR.tolist(), 'n': 0, 'outliers': 0, 'mae': None, 'learned': False}
    if len(y) >= MIN_SAMPLES:
        w = _fit(X, y)
        resid = y - X @ w
        mad = np.median(np.abs(resid - np.median(resid))) * 1.4826
        keep = np.abs(resid) <= max(3 * mad, 5.0)      # Drop outliers such as moves that took long because they were blocked
        if keep.sum() >= MIN_SAMPLES:
            w = _fit(X[keep], y[keep])
        info.update(w=w.tolist(), n=int(keep.sum()), outliers=int((~keep).sum()),
                    mae=float(np.mean(np.abs(y[keep] - X[keep] @ w))), learned=True)
    w = info['w']
    info['summary'] = (f'기본 {w[0]:.1f}초 + 거리 {w[1]:.1f}초/m'
                       f'{f" (약 {1 / w[1]:.2f} m/s)" if w[1] > 0 else ""}'
                       f' + 회전 {w[2]:.1f}초/rad')
    return info


def predict(model, length, turn):
    w = model['w']
    return w[0] + w[1] * length + w[2] * turn


# ---------------------------------------------------------------- records (called from services)
def record_start(m, step, pose):
    """On departure: record start pose, path length/turning and predicted time (the move proceeds even if this fails)."""
    step.started_at = timezone.now()
    step.finished_at = None
    fields = ['started_at', 'finished_at']
    if m is not None and pose is not None:
        try:
            length, turn = path_metrics(m, (pose['x'], pose['y'], pose['yaw']), (step.x, step.y, step.yaw))
            step.start_x, step.start_y, step.start_yaw = pose['x'], pose['y'], pose['yaw']
            step.plan_length, step.plan_turn = length, turn
            step.predicted_sec = predict(travel_model(), length, turn)
            fields += ['start_x', 'start_y', 'start_yaw', 'plan_length', 'plan_turn', 'predicted_sec']
        except PlanError:
            pass
    step.save(update_fields=fields)


def record_arrival(step):
    step.finished_at = timezone.now()
    step.save(update_fields=['finished_at'])


# ---------------------------------------------------------------- order optimization
def _route_cost(order, cost, start_idx, end_idx):
    total, prev = 0.0, start_idx
    for i in order:
        total += cost[prev][i]
        prev = i
    if end_idx is not None:
        total += cost[prev][end_idx]
    return total


def optimize(m, pose, locations, home=None):
    """pose: robot {'x','y','yaw'}, locations: list of Location (in selected order), home: Location to visit last or None.
    Returns dict(order=[Location...], legs=[...], total, original_total, model)."""
    if pose is None:
        raise PlanError('로봇 위치를 모릅니다. 초기 위치를 먼저 지정하세요.')
    model = travel_model()
    nodes = [(pose['x'], pose['y'], pose['yaw'])] + [(l.x, l.y, l.yaw) for l in locations]
    if home is not None:
        nodes.append((home.x, home.y, home.yaw))
    n = len(nodes)
    cost = [[0.0] * n for _ in range(n)]
    metrics = {}
    for a in range(n):
        for b in range(1, n):
            if a == b:
                continue
            try:
                length, turn = path_metrics(m, nodes[a], nodes[b])
            except PlanError:
                length, turn = float('inf'), 0.0
            metrics[a, b] = (length, turn)
            cost[a][b] = predict(model, length, turn) if math.isfinite(length) else float('inf')

    k = len(locations)
    idx = list(range(1, k + 1))
    end = n - 1 if home is not None else None
    if k <= 8:
        best = min(itertools.permutations(idx), key=lambda o: _route_cost(o, cost, 0, end))
        best = list(best)
    else:
        # Pick nearest first, then improve with 2-opt
        best, left, cur = [], set(idx), 0
        while left:
            nxt = min(left, key=lambda j: cost[cur][j])
            best.append(nxt)
            left.remove(nxt)
            cur = nxt
        improved = True
        while improved:
            improved = False
            for i in range(len(best) - 1):
                for j in range(i + 1, len(best)):
                    cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                    if _route_cost(cand, cost, 0, end) + 1e-9 < _route_cost(best, cost, 0, end):
                        best, improved = cand, True
    total = _route_cost(best, cost, 0, end)
    if not math.isfinite(total):
        raise PlanError('갈 수 없는 위치가 있습니다 (경로 없음).')
    legs, prev = [], 0
    for i in best + ([end] if end is not None else []):
        length, turn = metrics[prev, i]
        legs.append({'to': i, 'length': round(length, 2), 'turn_deg': round(math.degrees(turn)),
                     'sec': round(cost[prev][i], 1)})
        prev = i
    return {
        'order': [locations[i - 1] for i in best],
        'legs': legs,
        'total': round(total, 1),
        'original_total': (round(_route_cost(idx, cost, 0, end), 1)
                           if math.isfinite(_route_cost(idx, cost, 0, end)) else None),
        'model': model,
    }
