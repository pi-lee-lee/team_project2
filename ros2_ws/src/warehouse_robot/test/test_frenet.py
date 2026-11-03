"""Frenet local planner (frenet.py): checked closed-loop (a robot that follows commands as is within the acceleration limit)."""
import math

import numpy as np

from warehouse_robot.frenet import FrenetConfig, FrenetFollower, body_circles, body_points, build_reference, lateral_shape, offset_grid


def drive(path, pts, start=(0.0, 0.0, 0.0), goal_yaw=0.0, steps=400, moving=None, cfg=None):
    f = FrenetFollower(path, goal_yaw, cfg or FrenetConfig())
    x, y, yaw = start
    v = 0.0
    offs, rad = body_circles(f.cfg.footprint)                # moving obstacles (same circle model as the planner)
    body = body_points(f.cfg.footprint, f.cfg.footprint_extra)   # static (outline of body + attached items)
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    log = {'v': [], 'state': [], 'clear': math.inf, 'xy': []}
    for k in range(steps):
        mv = moving(k * 0.1) if moving else np.zeros((0, 5))
        vc, wc, st = f.compute((x, y, yaw), v, 0.0, (0.0, 0.0, 0.0), pts, mv)
        log['state'].append(st)
        if st == 'done':
            break
        v += float(np.clip(vc - v, -0.06, 0.03))
        yaw += wc * 0.1
        x += v * math.cos(yaw) * 0.1
        y += v * math.sin(yaw) * 0.1
        log['v'].append(v)
        log['xy'].append((x, y))
        cx, cy = x + offs * math.cos(yaw), y + offs * math.sin(yaw)
        if len(pts):
            bx = x + body[:, 0] * math.cos(yaw) - body[:, 1] * math.sin(yaw)
            by = y + body[:, 0] * math.sin(yaw) + body[:, 1] * math.cos(yaw)
            log['clear'] = min(log['clear'], float(np.min(np.hypot(pts[:, None, 0] - bx, pts[:, None, 1] - by))))
        for o in mv:
            log['clear'] = min(log['clear'], float(np.min(np.hypot(o[0] - cx, o[1] - cy))) - rad - o[4])
    return f, log


STRAIGHT = [(x, 0.0) for x in np.arange(0.0, 3.01, 0.05)]
WALLS = [(x, s) for x in np.arange(-0.5, 3.5, 0.03) for s in (-0.46, 0.41)]


def test_reference_is_smooth_and_follows_path():
    # staircase A* path on a grid -> spline reference stays close to the points with small curvature
    pts = [(0.05 * i, 0.05 * (i // 6)) for i in range(60)]
    ref = build_reference(pts, 0.05, 0.02)
    d = np.min(np.hypot(ref.x[:, None] - np.array(pts)[None, :, 0], ref.y[:, None] - np.array(pts)[None, :, 1]), axis=1)
    on = ref.s <= ref.s_end
    assert d[on].max() < 0.05
    assert np.abs(ref.kappa[on]).max() < 5.0


def test_lateral_shape_ends_parallel_at_offset():
    # one candidate = one polynomial: parallel at the offset at D (no hold/return segment; beyond D the end state holds)
    q = np.arange(0.0, 1.0, 0.05)
    d, dp, dpp = lateral_shape(q, 0.0, 0.0, 0.2, 0.4)
    assert abs(d[0]) < 1e-9 and np.allclose(d[q >= 0.4], 0.2) and np.allclose(dp[q >= 0.4], 0.0)


def test_drives_straight_at_target_speed():
    f, log = drive(STRAIGHT, [])
    assert log['state'][-1] == 'done'
    assert np.mean(log['v']) > 0.13                      # target 0.15 (including accel/decel)
    assert max(abs(y) for _, y in log['xy']) < 0.01


def test_passes_obstacle_beside_path_without_slowing():
    # 0.9 m corridor, fire extinguisher (radius 0.066) 4 cm beside the path -> passes the 0.5 m gap without slowing
    ext = [(1.5 + 0.066 * math.cos(a), 0.11 + 0.066 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 24)]
    f, log = drive(STRAIGHT, WALLS + ext)
    assert log['state'][-1] == 'done'
    assert 'blocked' not in log['state']
    assert log['clear'] >= f.cfg.safety_margin - 0.01
    passing = [v for (x, _), v in zip(log['xy'], log['v']) if 1.2 < x < 1.8]
    assert min(passing) > 0.12                           # must not creep past the obstacle


def test_stops_when_corridor_fully_blocked():
    block = WALLS + [(1.5, s) for s in np.arange(-0.46, 0.42, 0.03)]
    f, log = drive(STRAIGHT, block, steps=200)
    assert log['state'][-1] == 'blocked'
    assert log['clear'] > 0.2                            # stops early instead of creeping right up to it
    assert log['v'][-1] < 1e-3


def test_turns_in_place_when_path_is_behind():
    back = [(-x, 0.0) for x in np.arange(0.0, 2.01, 0.05)]
    f, log = drive(back, [])
    assert log['state'][0] == 'rotating' and log['state'][-1] == 'done'


def test_avoids_crossing_mover():
    # object crossing the path at 0.1 m/s: constant-velocity prediction makes the robot wait or move aside without collision
    f, log = drive(STRAIGHT, WALLS, moving=lambda t: np.array([[1.2, -0.4 + 0.1 * t, 0.0, 0.1, 0.1]]))
    assert log['state'][-1] == 'done'
    assert log['clear'] > 0.0


def test_escapes_corner_then_turns():
    # Corner: front wall 13 cm, left wall 9 cm, path goes back and down -> turning in place hits a corner either way.
    # Make room with a short move, then turn onto the path
    walls = ([(x, 0.25) for x in np.arange(-0.6, 0.21, 0.02)] + [(0.2, y) for y in np.arange(-0.65, 0.26, 0.02)]
             + [(x, -0.65) for x in np.arange(-0.6, 0.21, 0.02)])
    path = [(0.0, 0.0)] + [(-0.05 * i, -0.3 * min(i, 6) / 6) for i in range(1, 30)]
    f, log = drive(path, walls, steps=600)
    assert log['state'][-1] == 'done'
    assert log['clear'] > 0.0


def test_actual_speed_stays_below_max_when_shifting_on_curve():
    # shifting to the outside of a curve must not push the actual speed over max_speed
    arc = [(1.0 * math.sin(a), 1.0 - 1.0 * math.cos(a)) for a in np.linspace(0, math.pi / 2, 40)]
    obstacle = [(0.71 + 0.05 * math.cos(a) - 0.0, 0.29 + 0.05 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 12)]
    f, log = drive(arc, obstacle, steps=300)
    assert max(log['v']) <= f.cfg.max_speed + 1e-6


def test_mover_behind_does_not_stop_robot():
    # object following from behind at a similar speed (0.12 m/s, 0.45 m behind): even when the growing predicted margin touches the rear end, keep going forward
    f, log = drive(STRAIGHT, WALLS, steps=120, moving=lambda t: np.array([[-0.45 + 0.12 * t, 0.0, 0.12, 0.0, 0.1]]))
    assert 'blocked' not in log['state'][:60]
    assert np.mean(log['v'][20:60]) > 0.12


def _arrived_at_origin():
    """Follower already at the goal (path end = origin)."""
    f = FrenetFollower([(-1.0, 0.0), (0.0, 0.0)], 0.0, FrenetConfig())
    f.arrived = True
    return f


def _align_clear(f, pts, yaw0, w_steps):
    """Run the post-arrival turn -> (final state, angle turned, min body-to-point distance)."""
    body = body_points(f.cfg.footprint, f.cfg.footprint_extra)
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    yaw, turned, clear, st = yaw0, 0.0, math.inf, None
    for _ in range(w_steps):
        v, w, st = f.compute((0.0, 0.0, yaw), 0.0, 0.0, (0.0, 0.0, 0.0), pts, np.zeros((0, 5)))
        if st == 'done':
            break
        yaw += w * 0.1
        turned += w * 0.1
        if len(pts):
            bx = body[:, 0] * math.cos(yaw) - body[:, 1] * math.sin(yaw)
            by = body[:, 0] * math.sin(yaw) + body[:, 1] * math.cos(yaw)
            clear = min(clear, float(np.min(np.hypot(pts[:, None, 0] - bx, pts[:, None, 1] - by))))
    return st, turned, clear


def test_arrival_turn_in_open_space():
    st, turned, _ = _align_clear(_arrived_at_origin(), [], math.radians(90), 100)
    assert st == 'done' and turned < -1.3          # about 90° the near way (right)


def test_arrival_turn_takes_far_side_when_near_side_hits():
    # Facing up (90°), must align to 0°. Turning the near way (right) sweeps the rear end (radius ~0.28 m) through the
    # lower-left diagonal to within 4.5 cm of an object near (-0.23, -0.23) -> turn 270° the other way (left) instead (final pose is over 8 cm away)
    f = _arrived_at_origin()
    chunk = [(-0.23 + dx, -0.23 + dy) for dx in (-0.01, 0.0, 0.01) for dy in (-0.01, 0.0, 0.01)]
    st, turned, clear = _align_clear(f, chunk, math.radians(90), 200)
    assert st == 'done' and turned > 3.5           # about 270° to the left
    assert clear >= f.cfg.safety_margin - 1e-3


def test_arrival_turn_skipped_in_tight_corner():
    # back/side walls closer than the turn radius (~0.28 m), like a small closet -> finish at the current heading instead of scraping the wall
    f = _arrived_at_origin()
    walls = [(-0.30, y) for y in np.arange(-0.5, 0.5, 0.02)] + [(x, 0.27) for x in np.arange(-0.3, 0.5, 0.02)]
    st, turned, clear = _align_clear(f, walls, math.radians(180), 50)
    assert st == 'done' and abs(turned) < 1e-6
    assert f.last_debug.get('align') == 'skipped'


def test_lateral_shape_quintic_boundary_conditions():
    # start (d, d', d'') = current state, (off, 0, 0) at D
    d0, dp0, dpp0, off, D = 0.05, 0.1, -0.8, 0.2, 0.4
    for q, want in ((0.0, (d0, dp0, dpp0)), (D, (off, 0.0, 0.0))):
        got = lateral_shape(np.array(q), d0, dp0, off, D, dpp0)
        assert np.allclose([float(v) for v in got], want, atol=1e-9), (q, got, want)


def test_offset_switch_keeps_curvature_continuous():
    # Change in w when switching offsets while avoiding a box on the path. The new candidate's start d'' = the previous
    # candidate's d'' at that point, so curvature stays continuous
    f = FrenetFollower([(0, 0), (4.0, 0.0)], 0.0, FrenetConfig())
    pts = np.array([(1.5 + i * 0.02, j * 0.02) for i in range(6) for j in range(-4, 5)])
    x = y = yaw = v = 0.0
    prev, jumps, st = None, [], None
    for _ in range(500):
        vc, wc, st = f.compute((x, y, yaw), v, 0.0, (0.0, 0.0, 0.0), pts, np.zeros((0, 5)))
        if st == 'done':
            break
        off = f.offset if f.last_debug.get('branch') == 'frenet' else None
        if prev is not None and None not in (off, prev[1]) and off != prev[1]:
            jumps.append(abs(wc - prev[0]))
        prev = (wc, off)
        v += float(np.clip(vc - v, -0.06, 0.03))
        yaw += wc * 0.1
        x, y = x + v * math.cos(yaw) * 0.1, y + v * math.sin(yaw) * 0.1
    assert st == 'done' and jumps
    assert max(jumps) < 0.35


def test_very_short_paths_build_reference():
    # paths shorter than 15 cm (goal right next to the robot): curvature averaging must not raise on a length mismatch
    for pts in ([(0, 0), (0.05, 0.0), (0.1, 0.02)], [(0, 0), (0.08, 0.0), (0.08, 0.06)], [(0, 0), (0.05, 0.0)],
                [(0, 0), (0.02, 0.01)]):
        ref = build_reference(np.array(pts, dtype=float))
        assert np.all(np.isfinite(ref.at(0.0)[0]))
    f, log = drive([(0, 0), (0.06, 0.0), (0.1, 0.03)], [])
    assert log['state'][-1] in ('done', 'rotating', 'driving')



def test_projection_ignores_extension_past_path_end():
    # U-shaped path ending back toward where it passed: the extension past the end runs beside the start section.
    # Even if the extension is closer, project onto the actual path (otherwise the end looks already passed)
    pts = [(x, 0.0) for x in np.arange(0.0, 1.5, 0.05)] + [(1.5, y) for y in np.arange(0.0, 0.4, 0.05)] + \
          [(x, 0.4) for x in np.arange(1.5, 0.89, -0.05)]
    ref = build_reference(np.array(pts, dtype=float))
    _, s0 = ref.project(0.3, 0.22, 0, 400)        # 0.22 m to the start section, 0.18 m to the extension
    assert s0 < 0.5 < ref.s_end


def _corridor(half_gap):
    """Narrow corridor only for x 0.3-1.5 m (walls at y = +-half_gap), wide elsewhere."""
    return [(x, s * half_gap) for x in np.arange(0.3, 1.5, 0.02) for s in (-1, 1)]


def test_narrow_gap_passed_slowly_with_speed_dependent_margin():
    # half width 0.153 + 0.045 -> about 4.5 cm clearance even in the middle: fast candidates (6 cm) fail, only slow ones (4 cm) pass -> goes through slowly
    walls = _corridor(0.153 + 0.045)
    f, log = drive(STRAIGHT, walls, steps=900)
    assert log['state'][-1] == 'done'
    assert log['clear'] > 0.03
    xs = np.array(log['xy'])[:, 0]
    v_gap = np.array(log['v'])[(xs > 0.5) & (xs < 1.2)]
    assert v_gap.max() < 0.08                       # slow in the narrow part
    # with a speed-independent 6 cm margin the same corridor is blocked
    cfg = FrenetConfig(safety_margin_slow=0.06)
    f2, log2 = drive(STRAIGHT, walls, steps=200, cfg=cfg)
    assert 'blocked' in log2['state'] and log2['state'][-1] != 'done'


def test_wide_corridor_keeps_full_speed():
    f, log = drive(STRAIGHT, _corridor(0.45), steps=400)
    assert log['state'][-1] == 'done'
    assert max(log['v']) > 0.13


def test_offset_grid_fine_inside_coarse_outside():
    g = offset_grid(0.3, 0.025, 0.1, 0.05)
    assert np.allclose(g, -g[::-1]) and 0.0 in g
    assert np.allclose(np.diff(g[np.abs(g) <= 0.1 + 1e-9]), 0.025)
    assert np.allclose(np.diff(g[g >= 0.1 - 1e-9]), 0.05) and abs(g[-1] - 0.3) < 1e-9
    assert len(offset_grid(0.3, 0.05, 0.1, 0.05)) == 13          # uniform if the outer spacing is not coarser than the inner one
