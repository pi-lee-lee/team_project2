"""Frenet-frame local planner + trajectory follower (no ROS dependency, numpy-vectorized).

Core idea from github.com/jeongseonarm/prediction-planning-control (MIT, planning/local_path_planning):
  turn the A* path into a smooth reference line (smoothing spline), generate 'lateral offset x speed'
  candidates in the (s: along the path, d: sideways) frame on it, check them for collisions and pick the
  cheapest one. If none is safe, stop.
  (The reference spline and the Frenet -> Cartesian conversion are the same as in that repo / PythonRobotics (MIT).)

Parts redesigned for the TurtleBot (differential drive, 0.05-0.18 m/s, narrow aisles):
  - Lateral shape: one candidate = one polynomial d(s) (no stitched segments). Based on the low-speed d(s) of
    Werling (2010) IV-B, with 7 conditions -> 6th order:
      start d, d', d'' = current state (d'' taken from the previously chosen trajectory at this point, as in Werling)
      end (length D) d = end offset, d' = 0, d'' = 0 (Werling's target state: parallel to the path, curvature continuous)
      via point (D x via_ratio) d = via offset
    Candidates exist only up to D and are checked only up to D. Returning to the path comes from the cost
    (end offset^2) plus replanning every cycle.
    The via offset lets the planner choose 'position inside a gap' and 'position after it' separately; a single
    end offset cannot, and gets stuck in narrow places.
    Why match d'' (Werling minimizes lateral jerk; our reason): the command is w = v*kappa, so a curvature jump is
    a jump in w. Matching the start d'' and ending with d'' = 0 keeps curvature continuous when switching offsets.
  - Speed: instead of a polynomial, accelerate/decelerate from the current speed to the target speed at max_accel
    (cmd_vel commands speed directly).
  - Shape and speed are separated: static obstacles are checked over distance 0..D using the shape only,
    independent of speed - a stationary obstacle's collision does not depend on when we get there. (Rolling out
    in time lets slow trajectories look safe just because they do not reach the obstacle within the horizon.)
    Only moving obstacles are checked in time (speed profile) against constant-velocity predicted circles
    (radius + dynamic_margin + dynamic_growth * t).
  - Candidates: via offset x end offset (+-max_offset, finer spacing inside +-fine_offset) x candidate length
    lateral_dists x speed (target and n_speed more, speed_step apart).
  - Cost (TurtleBot units instead of a car's jerk/time, each term roughly 0..1): end offset^2, lateral jerk,
    candidate length, change from the previous offset (oscillation), slowness, lack of static clearance
    (closer than clear_pref, over distance), lack of clearance to moving obstacles.
  - Clearance (safety_margin) is a hard constraint, above it a cost -> centered in gaps, on the path when wide.
    The static margin shrinks with candidate speed down to safety_margin_slow, so tight gaps are crept through slowly.
  - Already inside the margin (next to a wall): the slowest candidate may get up to 2 cm closer than now
    (to get out), never below 1 cm.
  - Static collision: look up a distance grid (grid_res) of scan points at points on the outline and center line
    of the body rectangle (+ attached items) -> distance to the rectangle. (Covering the body with circles would
    inflate front/back and block turning near walls.) Moving obstacles: the body is covered by 4 circles and checked
    against the predicted circles.
  - The speed profile is in terms of actual speed (so sidestepping / the outside of a curve does not go faster
    than the along-path speed).
  - Kinematic limits: |v*kappa| <= max_w (angular velocity), v <= max_speed.
  - Following: execute the start of the plan as is (v = planned speed 0.1 s ahead, w = v*kappa averaged over
    0.1-0.3 s). Every plan starts at the current pose, so errors are corrected by the next plan (a pure-pursuit
    follower cuts inside when sidestepping and moves aside too late).
  - Perception range: the static grid only covers where this cycle's candidates go + body + clear_pref (behind,
    only up to the rear end of the body); in-place rotation uses 0.6 m around the robot. Moving obstacles are used
    only if they can reach the candidate area within the horizon, and ignored while fully behind the body's rear end.
  - Turning before starting / aligning after arrival is done in place (candidates are forward trajectories only).
    If the near direction is blocked, turn the far way; if both are blocked (corner), make a short forward/back move
    to make room, then turn.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.ndimage import distance_transform_edt

# Why a candidate was rejected (last_eval['reason'])
OK, KINEMATIC, STATIC, DYNAMIC = 0, 1, 2, 3
REASONS = {OK: '통과', KINEMATIC: '운동 한계', STATIC: '정적 장애물', DYNAMIC: '움직이는 물체'}


def normalize(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def transform_points(points, x, y, yaw):
    """Rotate + translate (N, 2) points by (x, y, yaw)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                            y + s * points[:, 0] + c * points[:, 1]])


@dataclass
class FrenetConfig:
    dt: float = 0.1               # time step (s)
    horizon: float = 4.0          # time horizon for moving-obstacle checks (s)
    check_step: float = 0.05      # distance sample spacing along candidates (m)
    lateral_dists: tuple = (0.8,)            # candidate lengths (m); checks also go up to this length
    max_offset: float = 0.3       # lateral offset candidates +-max_offset (m)
    offset_step: float = 0.025     # fine enough to center the robot in a tight gap
    target_speed: float = 0.15    # (m/s) speed candidates: target, target - speed_step, ... (n_speed more)
    speed_step: float = 0.05
    n_speed: int = 2
    max_speed: float = 0.18
    max_accel: float = 0.3        # speed profile acceleration/deceleration (m/s²)
    max_w: float = 1.82           # angular velocity limit |v·κ| (rad/s)
    footprint: tuple = (-0.197, 0.069, 0.153)   # body rectangle (x_min, x_max, half width)
    footprint_extra: tuple = (-0.277, -0.197, -0.04, 0.04)   # items attached to the body (x_min, x_max, y_min, y_max) x N; included in static checks
    safety_margin: float = 0.06   # minimum clearance between body and obstacles (constraint, m)
    # Static clearance depends on candidate speed: safety_margin at target_speed, safety_margin_slow at the slowest
    # candidate (target - n_speed*speed_step), linear in between. Tight gaps are passed only by slow candidates,
    # so the robot creeps through them, while wide areas keep the larger margin.
    safety_margin_slow: float = 0.04
    clear_pref: float = 0.15      # clearance below this adds cost (m)
    dynamic_margin: float = 0.15  # extra clearance for moving obstacles (m)
    dynamic_growth: float = 0.03  # clearance growth per second of prediction (m/s)
    grid_res: float = 0.01        # static distance grid resolution (m); coarser grids are too inaccurate for tight gaps
    grid_half: float = 1.6        # max static grid extent: robot center +-grid_half (actual extent fits where candidates go, m)
    w_offset: float = 0.0         # (off: replaced by end offset²) mean |d| over the candidate / max_offset
    w_change: float = 1.0         # |offset - previous offset| / max_offset (1/4 when moving back toward the path). Too small a weight
                                  # lets the offset flip every cycle in narrow places, making w jump
    w_turn: float = 0.0           # (off: replaced by jerk) max|d'| (lateral slope, 1 = 45°)
    w_speed: float = 1.0          # (target - speed) / target
    w_clear: float = 1.0          # static: (clear_pref - min clearance) / clear_pref
    w_dynamic: float = 1.0        # moving obstacles: same formula
    # Terms of Werling (2010) low-speed lateral cost Cd = kj·Js + kt·S + kd·d1² (each normalized to about 0..1). 0 = off
    w_end: float = 1.0            # end offset² (d1 / max_offset)²: pulls the robot back to the path
    w_jerk: float = 0.3           # ∫ d'''² ds / (720·max_offset² / 0.6⁵) (1 = optimal quintic shifting by max_offset over 0.6 m)
    w_len: float = 0.2            # candidate length S / longest candidate (penalizes slowly converging candidates)
    via_ratio: float = 0.5        # via point position = candidate length x this (0 = no via point, quintic)
    end_offset_step: float = 0.025 # end offset spacing (via offsets use offset_step). candidates = via x end x length x speed
    fine_offset: float = 0.1      # offset_step / end_offset_step only for |offset| within this, coarse_offset_step outside (0 = uniform)
    coarse_offset_step: float = 0.05
    rotate_speed: float = 1.0     # in-place turn speed (rad/s). The bridge uses 0.6 in SLAM mode (fast turns skew scans and hurt matching)
    stop_decel: float = 0.3       # speed is capped so the robot can stop at the path end with this deceleration (m/s²)
    stop_margin: float = 0.05     # the stopping profile ends this far before the path end (m)
    reference_ds: float = 0.05    # reference line spacing (m); A* grid is 5 cm
    reference_sigma: float = 0.02  # allowed RMS deviation of the spline from the A* points (m)


# ------------------------------------------------------------------ reference line
class Reference:
    """A* path -> smooth reference line [s, x, y, yaw, kappa, dkappa]. The end is extended straight by `extend` m
    (so candidates past the path end can still be converted; collision checks stop at s_end)."""

    def __init__(self, s, x, y, yaw, kappa, dkappa, s_end):
        self.s, self.x, self.y, self.yaw, self.kappa, self.dkappa = s, x, y, np.unwrap(yaw), kappa, dkappa
        self.s_end = s_end

    def transformed(self, tx, ty, tyaw):
        """Reference rotated + translated by (tx, ty, tyaw) (map -> odom). Curvature unchanged."""
        c, sn = math.cos(tyaw), math.sin(tyaw)
        return Reference(self.s, tx + c * self.x - sn * self.y, ty + sn * self.x + c * self.y,
                         self.yaw + tyaw, self.kappa, self.dkappa, self.s_end)

    def at(self, s):
        return (np.interp(s, self.s, self.x), np.interp(s, self.s, self.y), np.interp(s, self.s, self.yaw),
                np.interp(s, self.s, self.kappa), np.interp(s, self.s, self.dkappa))

    def project(self, px, py, lo=0, hi=None):
        """Foot of the perpendicular from (px, py) onto the reference in [lo, hi) -> (index, s).
        The nearest point is searched only up to the path end (s_end) (the segment after it may go past s_end).
        Searching the extension too would break paths that end by coming back toward where they passed: the extension
        can run through the robot's position, making it look as if the end had already been passed."""
        end = int(np.searchsorted(self.s, self.s_end, side='right'))
        hi = end if hi is None else min(hi, end)
        lo = max(0, min(lo, hi - 1))
        i = lo + int(np.argmin(np.hypot(self.x[lo:hi] - px, self.y[lo:hi] - py)))
        best = (math.inf, self.s[i])
        for a, b in ((i - 1, i), (i, i + 1)):
            if a < 0 or b >= len(self.s):
                continue
            p0 = np.array([self.x[a], self.y[a]])
            seg = np.array([self.x[b], self.y[b]]) - p0
            t = float(np.clip(np.dot([px - p0[0], py - p0[1]], seg) / max(np.dot(seg, seg), 1e-12), 0.0, 1.0))
            dist = math.hypot(px - p0[0] - t * seg[0], py - p0[1] - t * seg[1])
            if dist < best[0]:
                best = (dist, self.s[a] + t * (self.s[b] - self.s[a]))
        return i, float(best[1])


def _polyline(xy, ds):
    """Fallback when a spline cannot be used (too few points, turn over 90°): sample the polyline every ds, curvature from heading change."""
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    sg = np.append(np.arange(0.0, s[-1], ds), s[-1]) if s[-1] > ds else np.array([0.0, s[-1]])
    x, y = np.interp(sg, s, xy[:, 0]), np.interp(sg, s, xy[:, 1])
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x))) if len(sg) > 1 else np.zeros(1)
    k = np.gradient(yaw, sg) if len(sg) > 2 else np.zeros(len(sg))
    # 5-point average. With fewer than 5 points convolve(mode='same') returns 5 values and the lengths mismatch
    kappa = np.convolve(k, np.ones(5) / 5, mode='same') if len(sg) >= 5 else k
    dkappa = np.gradient(kappa, sg) if len(sg) > 2 else np.zeros(len(sg))
    return sg, x, y, yaw, kappa, dkappa


def build_reference(points, ds=0.05, sigma=0.02, order=5, end_extension=0.3, extend=1.5):
    """A* path points -> Reference. Same method as the repo's build_reference_path (extend both ends straight, weighted
    smoothing spline, resample by arc length, curvature / curvature rate). end_extension is used only for fitting
    (scaled down from a car's 4 m to 0.3 m)."""
    xy = np.asarray(points, dtype=float).reshape(-1, 2)
    keep = [0]
    for i in range(1, len(xy)):
        if math.hypot(*(xy[i] - xy[keep[-1]])) > 1e-3:
            keep.append(i)
    xy = xy[keep]
    if len(xy) < 2:
        raise ValueError('경로 점이 2개 미만')
    out = None
    seg = np.diff(xy, axis=0)
    seg_len = np.hypot(*seg.T)
    cusp = len(xy) > 2 and np.any(np.sum(seg[:-1] * seg[1:], axis=1) < 0.0)   # turn over 90°
    if len(xy) >= 4 and not cusp:
        try:
            step = float(np.median(seg_len))
            n_ext = max(int(math.ceil(end_extension / step)), 1)
            off = step * np.arange(1, n_ext + 1)[:, None]
            t0 = (xy[min(3, len(xy) - 1)] - xy[0]) / np.hypot(*(xy[min(3, len(xy) - 1)] - xy[0]))
            t1 = (xy[-1] - xy[max(-4, -len(xy))]) / np.hypot(*(xy[-1] - xy[max(-4, -len(xy))]))
            fit = np.vstack([xy[0] - off[::-1] * t0, xy, xy[-1] + off * t1])
            u = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(fit, axis=0).T))])
            k = int(min(order, len(fit) - 1))
            tck, _ = splprep([fit[:, 0], fit[:, 1]], w=np.full(len(fit), 1.0 / sigma), u=u, k=k, s=float(len(fit)))
            u0, u1 = float(u[n_ext]), float(u[n_ext + len(xy) - 1])
            uf = np.append(np.arange(u0, u1, 0.005), u1)
            fdx, fdy = splev(uf, tck, der=1)
            sp = np.hypot(fdx, fdy)
            arc = np.concatenate([[0.0], np.cumsum(0.5 * (sp[1:] + sp[:-1]) * np.diff(uf))])
            sg = np.append(np.arange(0.0, arc[-1], ds), arc[-1])
            us = np.interp(sg, arc, uf)
            x, y = splev(us, tck)
            dx, dy = splev(us, tck, der=1)
            ddx, ddy = splev(us, tck, der=2)
            dddx, dddy = splev(us, tck, der=3)
            a, b = dx * ddy - dy * ddx, dx * dddy - dy * dddx
            c, d = dx * ddx + dy * ddy, dx * dx + dy * dy
            out = (sg, np.asarray(x), np.asarray(y), np.unwrap(np.arctan2(dy, dx)), a / d ** 1.5,
                   (b * d - 3.0 * a * c) / d ** 3)
            if not all(np.all(np.isfinite(v)) for v in out):
                out = None
        except (ValueError, TypeError, ZeroDivisionError):
            out = None
    if out is None:
        out = _polyline(xy, ds)
    s, x, y, yaw, kappa, dkappa = out
    # extend the end straight (curvature 0)
    se = s[-1] + np.arange(1, int(extend / ds) + 1) * ds
    c, sn = math.cos(yaw[-1]), math.sin(yaw[-1])
    return Reference(np.concatenate([s, se]), np.concatenate([x, x[-1] + c * (se - s[-1])]),
                     np.concatenate([y, y[-1] + sn * (se - s[-1])]), np.concatenate([yaw, np.full(len(se), yaw[-1])]),
                     np.concatenate([kappa, np.zeros(len(se))]), np.concatenate([dkappa, np.zeros(len(se))]),
                     float(s[-1]))


# ------------------------------------------------------------------ coordinate conversion
def frenet_to_cartesian(rx, ry, rtheta, rkappa, rdkappa, s_dot, s_ddot, d, d_prime, d_pprime):
    """(Same formula as the repo's frenet_vectorized.frenet_to_cartesian_batch) arrays -> (..., 6) [x, y, theta, kappa, v, a]."""
    cos_r, sin_r = np.cos(rtheta), np.sin(rtheta)
    x, y = rx - sin_r * d, ry + cos_r * d
    omk = 1.0 - rkappa * d
    delta = np.arctan2(d_prime, omk)
    tan_delta, cos_delta = d_prime / omk, np.cos(delta)
    theta = normalize(delta + rtheta)
    krd = rdkappa * d + rkappa * d_prime
    kappa = (((d_pprime + krd * tan_delta) * cos_delta * cos_delta) / omk + rkappa) * cos_delta / omk
    v = np.sqrt(omk * omk * s_dot * s_dot + (d_prime * s_dot) ** 2)
    dtp = omk / cos_delta * kappa - rkappa
    a = s_ddot * omk / cos_delta + s_dot * s_dot / cos_delta * (d_prime * dtp - krd)
    return np.stack((x, y, theta, kappa, v, a), axis=-1)


# ------------------------------------------------------------------ collision
class StaticGrid:
    """Distance grid to scan points, only for points inside the rectangle [x0, x1] x [y0, y1]. No obstacles outside.
    The distance transform costs grid area, so the grid covers only what is needed (candidate area + body + preferred clearance)."""

    def __init__(self, points, x0, y0, x1, y1, res):
        self.ox, self.oy, self.res = x0, y0, res
        self.nx = max(int(math.ceil((x1 - x0) / res)), 1)
        self.ny = max(int(math.ceil((y1 - y0) / res)), 1)
        free = np.ones((self.ny, self.nx), dtype=bool)
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts):
            j = np.floor((pts[:, 0] - self.ox) / res).astype(int)
            i = np.floor((pts[:, 1] - self.oy) / res).astype(int)
            ok = (i >= 0) & (i < self.ny) & (j >= 0) & (j < self.nx)
            free[i[ok], j[ok]] = False
        self.empty = bool(free.all())
        self.dist = None if self.empty else distance_transform_edt(free) * res   # distance between cell centers

    @classmethod
    def around(cls, points, cx, cy, half, res):
        return cls(points, cx - half, cy - half, cx + half, cy + half, res)

    def lookup(self, x, y):
        if self.empty:
            return np.full(np.shape(x), np.inf)
        j = np.floor((x - self.ox) / self.res).astype(int)
        i = np.floor((y - self.oy) / self.res).astype(int)
        inside = (i >= 0) & (i < self.ny) & (j >= 0) & (j < self.nx)
        out = np.full(np.shape(x), np.inf)
        out[inside] = self.dist[i[inside], j[inside]]
        return out


def _rect_points(x0, x1, y0, y1, step, middle=False):
    xs = np.linspace(x0, x1, max(int(round((x1 - x0) / step)) + 1, 2))
    ys = np.linspace(y0, y1, max(int(round((y1 - y0) / step)) + 1, 2))
    rows = (y0, (y0 + y1) / 2, y1) if middle else (y0, y1)
    return [np.c_[xs, np.full_like(xs, y)] for y in rows] + [np.c_[np.full_like(ys, x), ys] for x in (x0, x1)]


def offset_grid(max_offset, step, fine=0.0, coarse=0.0):
    """Lateral offset candidates -max_offset..max_offset. step spacing for |o| <= fine, coarse outside (fine 0 = all step)."""
    if fine <= 0.0 or coarse <= step:
        return np.round(np.arange(-max_offset, max_offset + 1e-9, step), 6)
    inner = np.arange(0.0, fine + 1e-9, step)
    outer = np.arange(inner[-1] + coarse, max_offset + 1e-9, coarse)
    half = np.concatenate([inner, outer])
    return np.round(np.unique(np.concatenate([-half, half])), 6)


def body_points(footprint, extra=(), step=0.02):
    """Points on the outline and center line of the body rectangle (x_min, x_max, half width) plus the outlines of
    attached-item rectangles (robot frame, (P, 2)). The minimum obstacle distance over these points = distance to the
    body (for points outside). The center line catches points that are deep inside the body."""
    x0, x1, hw = footprint
    pts = _rect_points(x0, x1, -hw, hw, step, middle=True)
    for i in range(0, len(extra) - 3, 4):
        pts += _rect_points(*extra[i:i + 4], step)
    return np.unique(np.round(np.vstack(pts), 6), axis=0)


def body_circles(footprint, n=4):
    """n circles covering the body rectangle (x_min, x_max, half width): center x positions, radius."""
    x0, x1, hw = footprint
    L = x1 - x0
    return x0 + L * (np.arange(n) + 0.5) / n, math.hypot(L / (2 * n), hw)


# ------------------------------------------------------------------ planner
def lateral_shape(q, d0, dp0, off, D, dpp0=0.0, mid=None, s_mid=None):
    """Lateral shape d(q), q = distance along the path. Returns (d, d', d'') (broadcast). One candidate = one polynomial.
    mid None: quintic, (d0, dp0, dpp0) -> (off, 0, 0) at D (Werling low-speed d(s)).
    mid given: 6th order, same start/end conditions + d = mid at distance s_mid (7 conditions). Lets the planner choose
    'position inside a gap' and 'position after it' separately (a quintic has only the end offset). No stitched segments.
    Beyond D the end state (off, 0, 0) holds (for interpolation/display; not used in checks)."""
    a2 = 0.5 * np.asarray(dpp0, dtype=float)
    if mid is None:
        A = off - (d0 + dp0 * D + a2 * D ** 2)                          # end condition minus the first three terms
        B = -(dp0 + 2.0 * a2 * D)
        C = -2.0 * a2
        co = [(10.0 * A - 4.0 * B * D + 0.5 * C * D ** 2) / D ** 3, (-15.0 * A + 7.0 * B * D - C * D ** 2) / D ** 4,
              (6.0 * A - 3.0 * B * D + 0.5 * C * D ** 2) / D ** 5]
    else:
        off, D, mid, s_mid, a2b = np.broadcast_arrays(*(np.asarray(v, dtype=float) for v in (off, D, mid, s_mid, a2)))
        m, S = s_mid, D
        M = np.stack([np.stack([m ** 3, m ** 4, m ** 5, m ** 6], -1),
                      np.stack([S ** 3, S ** 4, S ** 5, S ** 6], -1),
                      np.stack([3 * S ** 2, 4 * S ** 3, 5 * S ** 4, 6 * S ** 5], -1),
                      np.stack([6 * S, 12 * S ** 2, 20 * S ** 3, 30 * S ** 4], -1)], -2)
        rhs = np.stack([mid - (d0 + dp0 * m + a2b * m ** 2), off - (d0 + dp0 * S + a2b * S ** 2),
                        -(dp0 + 2.0 * a2b * S), -2.0 * a2b], -1)
        sol = np.linalg.solve(M, rhs[..., None])[..., 0]
        co = [sol[..., k] for k in range(4)]
    qq = np.minimum(q, D)
    d = d0 + dp0 * qq + a2 * qq ** 2
    dp = dp0 + 2.0 * a2 * qq
    dpp = 2.0 * a2 + 0.0 * qq
    for k, ak in enumerate(co, start=3):
        d = d + ak * qq ** k
        dp = dp + k * ak * qq ** (k - 1)
        dpp = dpp + k * (k - 1) * ak * qq ** (k - 2)
    past = q > D
    return np.where(past, off, d), np.where(past, 0.0, dp), np.where(past, 0.0, dpp)


def speed_profile(v0, vs, t, a_max):
    """Accelerate/decelerate from v0 to each target speed vs at a_max, then hold. Returns (v, a, distance) (nV, n)."""
    vs = np.asarray(vs, dtype=float)[:, None]
    tr = np.abs(vs - v0) / a_max                                         # time to reach the target speed
    sign = np.sign(vs - v0)
    v = np.where(t[None, :] < tr, v0 + sign * a_max * t[None, :], vs)
    a = np.where(t[None, :] < tr, sign * a_max, 0.0)
    tc = np.minimum(t[None, :], tr)
    dist = v0 * tc + 0.5 * sign * a_max * tc ** 2 + vs * np.maximum(t[None, :] - tr, 0.0)
    return v, a, dist


class FrenetPlanner:
    def __init__(self, cfg=None):
        self.cfg = cfg or FrenetConfig()
        self.last_eval = None

    def static_margins(self, vs):
        """Minimum static clearance per candidate speed: safety_margin at target_speed, safety_margin_slow at the slowest candidate."""
        c = self.cfg
        lo = max(c.target_speed - c.n_speed * c.speed_step, 0.0)
        span = max(c.target_speed - lo, 1e-6)
        r = np.clip((np.asarray(vs, dtype=float) - lo) / span, 0.0, 1.0)
        slow = min(c.safety_margin_slow, c.safety_margin)
        return slow + (c.safety_margin - slow) * r

    def plan(self, ref, pose, v0, static_points, moving, hint=0, prev_offset=None, prev_shape=None):
        """ref: Reference (same frame as pose), pose: (x, y, yaw), v0: current linear speed, static_points: (N, 2) scan points,
        moving: (M, 5) [x, y, vx, vy, r] moving obstacles, prev_offset: previously chosen offset (oscillation cost),
        prev_shape: previously chosen shape (s0, d0, dp0, dpp0, off, D) -> its d'' here becomes the start d'' (continuous curvature).
        Returns a trajectory dict (t, x, y, yaw, v, kappa, valid) or None (blocked). self.last_eval: per-candidate results (for records/analysis)."""
        c = self.cfg
        x, y, yaw = pose
        i0, s0 = ref.project(x, y, hint - 20, hint + 400)
        rx, ry, ryaw, rk, _ = (float(v) for v in ref.at(s0))
        d0 = (y - ry) * math.cos(ryaw) - (x - rx) * math.sin(ryaw)
        dth = float(np.clip(normalize(yaw - ryaw), -1.2, 1.2))
        dp0 = math.tan(dth) * (1.0 - rk * d0)                            # d' = slope relative to the path
        dpp0 = 0.0                                                        # d'': previous plan's value here (0 if none)
        if prev_shape is not None and 0.0 <= s0 - prev_shape['s0'] < prev_shape['D']:   # only within the previous candidate's length
            p = prev_shape
            dpp0 = float(lateral_shape(np.array(s0 - p['s0']), p['d0'], p['dp0'], p['off'], p['D'], p['dpp0'],
                                       p.get('mid'), p.get('s_mid'))[2])
        remaining = ref.s_end - s0
        v_end = math.sqrt(2.0 * c.stop_decel * max(remaining - c.stop_margin, 0.0))
        v_top = max(min(c.target_speed, c.max_speed, v_end), 0.02)
        vs = np.unique(np.clip(v_top - np.arange(c.n_speed + 1) * c.speed_step, 0.02, None))
        offs = offset_grid(c.max_offset, c.offset_step, c.fine_offset, c.coarse_offset_step)
        Ds = np.asarray(c.lateral_dists, dtype=float)
        # shape candidates = via offset x end offset x length, flattened to one axis (nS) (via_ratio 0: end offset x length, quintic)
        if c.via_ratio > 0.0:
            ends = offset_grid(c.max_offset, c.end_offset_step, c.fine_offset, c.coarse_offset_step)
            MID, END, DD = (a.ravel() for a in np.meshgrid(offs, ends, Ds, indexing='ij'))
            SM = DD * c.via_ratio
        else:
            END, DD = (a.ravel() for a in np.meshgrid(offs, Ds, indexing='ij'))
            MID = SM = None
        OFF = END if MID is None else MID          # 'offset' = the nearer position (for change cost, records)
        shape_args = lambda e: (END.reshape((-1,) + (1,) * e), DD.reshape((-1,) + (1,) * e), dpp0,   # noqa: E731
                                None if MID is None else MID.reshape((-1,) + (1,) * e),
                                None if SM is None else SM.reshape((-1,) + (1,) * e))
        nS, nV = len(OFF), len(vs)
        offs_c, rad = body_circles(c.footprint)
        body = body_points(c.footprint, c.footprint_extra)
        r_body = float(np.hypot(body[:, 0], body[:, 1]).max())            # from the reference point to the body's farthest point (incl. attachments)

        # ---- shape only: static obstacles over distance 0..D (candidate length), independent of speed
        q = np.arange(0.0, Ds.max() + 1e-9, c.check_step)                 # (nq,)
        dq, dpq, ddq = lateral_shape(q[None, :], d0, dp0, *shape_args(1))
        within = q[None, :] <= DD[:, None] + 1e-9                         # within the candidate length (nothing beyond it)
        sq = np.broadcast_to(s0 + q, dq.shape)
        g = frenet_to_cartesian(*ref.at(sq), np.ones_like(dq), np.zeros_like(dq), dq, dpq, ddq)
        on_path = (sq <= ref.s_end + 0.02) & within                       # nothing past the path end / candidate end is checked
        # Perception range: where this cycle's candidates go (+ current pose) + body extent + preferred clearance. Points
        # outside affect no candidate's clearance or cost (behind, only up to near the body's rear end). Capped at +-grid_half
        gx, gy = g[..., 0][on_path], g[..., 1][on_path]
        if not len(gx):               # past the path end or a very short path: only around the current pose (min() of an empty array raises)
            gx, gy = np.array([x]), np.array([y])
        pad = r_body + c.clear_pref + 2 * c.grid_res
        bx0, bx1 = max(min(gx.min(), x) - pad, x - c.grid_half), min(max(gx.max(), x) + pad, x + c.grid_half)
        by0, by1 = max(min(gy.min(), y) - pad, y - c.grid_half), min(max(gy.max(), y) + pad, y + c.grid_half)
        grid = StaticGrid(static_points, bx0, by0, bx1, by1, c.grid_res)
        stat = np.where(on_path, FrenetPlanner._static_clear(grid, g[..., 0], g[..., 1], g[..., 2], body),
                        np.inf).min(axis=1)                               # (nS,)
        now_static = float(FrenetPlanner._static_clear(grid, np.array([x]), np.array([y]), np.array([yaw]), body)[0])
        # Static clearance per speed (larger when faster, static_margins). If already inside the margin (next to a wall /
        # in a corner), only the slowest speed may get up to 2 cm closer (the rear corner briefly gets closer while getting
        # out; never below 1 cm). Faster speeds must always keep their own margin, otherwise fast candidates would pass
        # through tight gaps too.
        # The escape allowance applies when inside the largest (fastest) margin, not the slowest one: just outside the
        # slowest margin, the first move can bring the rear corner inside it and reject every candidate.
        margin_v = self.static_margins(vs)                                # (nV,)
        need_v = margin_v.copy()
        slowest = int(np.argmin(vs))
        if now_static < margin_v.max():
            need_v[slowest] = min(margin_v[slowest], max(now_static - 0.02, 0.01))
        curv = np.where(on_path, np.abs(g[..., 3]), 0.0).max(axis=1)      # max curvature of the shape
        steep = np.where(within, np.abs(dpq), 0.0).max(axis=1)
        d3 = np.gradient(ddq, c.check_step, axis=1)                        # d''' (low-order polynomial, numerical derivative is enough)
        jerk = np.where(within, d3 ** 2, 0.0).sum(axis=1) * c.check_step
        mean_d = np.where(within, np.abs(dq), 0.0).sum(axis=1) / within.sum(axis=1)

        # ---- speed profile x shape: time samples (moving obstacles, angular velocity limit, and the trajectory to execute)
        t = np.arange(0.0, c.horizon + 1e-9, c.dt)
        V, A, dist = speed_profile(max(v0, 0.0), vs, t, c.max_accel)       # (nV, n)

        def along(sdot, sddot, q):
            dd_, dp_, ddp_ = lateral_shape(q, d0, dp0, *shape_args(2))
            return frenet_to_cartesian(*ref.at(s0 + q), sdot, sddot, dd_, dp_, ddp_)

        # Profile speed is the actual speed: using it directly as the along-path speed ṡ makes the actual speed higher
        # when sidestepping / on the outside of a curve. Roll out once to get the actual/path speed ratio g, then redo with ṡ = V / g
        qt = np.broadcast_to(dist[None], (nS, nV, len(t)))
        Vb, Ab = np.broadcast_to(V, qt.shape), np.broadcast_to(A, qt.shape)
        cart = along(Vb, Ab, qt)
        g = np.where(Vb > 1e-6, cart[..., 4] / np.maximum(Vb, 1e-6), 1.0)
        sdot = Vb / np.clip(np.nan_to_num(g, nan=1.0), 0.5, 3.0)
        qt = np.concatenate([np.zeros(qt.shape[:-1] + (1,)),
                             np.cumsum(0.5 * (sdot[..., 1:] + sdot[..., :-1]) * c.dt, axis=-1)], axis=-1)
        cart = along(sdot, np.gradient(sdot, c.dt, axis=-1), qt)         # (nS, nV, n, 6)
        st = s0 + qt
        valid = (st <= ref.s_end + 0.02) & (qt <= DD[:, None, None] + 1e-9)   # only up to the path end and candidate end (D)
        valid[..., 0] = True
        wmax = np.where(valid, np.abs(cart[..., 4] * cart[..., 3]), 0.0).max(axis=2)
        # Actual speed must not exceed max_speed (if already faster, up to the current speed is allowed so decelerating
        # candidates pass). A small tolerance is needed because the correction above is not exact when starting at a
        # large angle to the path.
        over = np.where(valid, cart[..., 4] - max(c.max_speed * 1.05, v0 + 0.01), -1.0).max(axis=2)
        kin = (np.all(np.isfinite(cart).all(axis=3) | ~valid, axis=2) & (wmax <= c.max_w) & (over <= 0.0))
        mv = np.asarray(moving, dtype=float).reshape(-1, 5)
        if len(mv):   # only obstacles that can reach the candidate area (within reach of the current pose) within the horizon
            reach = float(np.hypot(cart[..., 0] - x, cart[..., 1] - y).max()) + r_body
            grow = c.dynamic_margin + c.dynamic_growth * c.horizon
            mv = mv[np.hypot(mv[:, 0] - x, mv[:, 1] - y) - mv[:, 4] - np.hypot(mv[:, 2], mv[:, 3]) * c.horizon - grow
                    <= reach]
        dyn = np.full((nS, nV), np.inf)
        need_d = c.safety_margin
        if len(mv):
            dclr = self._dynamic_clear(mv, t, cart[..., 0], cart[..., 1], cart[..., 2], offs_c, rad)
            dyn = np.where(valid, dclr, np.inf).min(axis=2)
            now_dyn = float(self._dynamic_clear(mv, t[:1], np.array([[x]]), np.array([[y]]), np.array([[yaw]]),
                                                offs_c, rad).min())
            need_d = min(c.safety_margin, now_dyn - 0.005)

        # ---- verdict + cost
        stat3 = np.broadcast_to(stat[:, None], (nS, nV))
        reason = np.where(~kin, KINEMATIC, np.where(stat3 < need_v[None, :], STATIC, np.where(dyn < need_d, DYNAMIC, OK)))
        prev = 0.0 if prev_offset is None else prev_offset
        if MID is not None and prev_shape is not None:     # previous plan's lateral position at this candidate's via point (Werling's temporal consistency)
            p = prev_shape
            prev = float(lateral_shape(np.array(s0 + float(SM.max()) - p['s0']), p['d0'], p['dp0'], p['off'], p['D'],
                                       p['dpp0'], p.get('mid'), p.get('s_mid'))[0])
        pref = max(c.clear_pref, 1e-6)
        terms = {
            'offset': c.w_offset * mean_d / c.max_offset,
            # Offset change: full cost when moving away from the path or crossing to the other side, 1/4 when returning
            # toward the path on the same side (a symmetric cost keeps the robot at its sidestep offset after the obstacle)
            'change': c.w_change * np.abs(OFF - prev) / c.max_offset
            * np.where((OFF * prev >= 0.0) & (np.abs(OFF) < abs(prev)), 0.25, 1.0),
            'turn': c.w_turn * steep,
            'clear': c.w_clear * np.clip((pref - stat) / pref, 0.0, 1.0),
            'end': c.w_end * (END / c.max_offset) ** 2,
            'jerk': c.w_jerk * jerk / (720.0 * c.max_offset ** 2 / 0.6 ** 5),
            'len': c.w_len * DD / Ds.max(),
        }
        terms = {kk: np.broadcast_to(v[:, None], (nS, nV)) for kk, v in terms.items()}
        terms['speed'] = np.broadcast_to(c.w_speed * (v_top - vs) / v_top, (nS, nV))
        terms['dynamic'] = c.w_dynamic * np.clip((pref - dyn) / pref, 0.0, 1.0)
        total = sum(terms.values())
        cost = np.where(reason == OK, total, np.inf)
        best = np.unravel_index(int(np.argmin(cost)), cost.shape) if np.isfinite(cost).any() else None
        self.last_eval = {
            's0': s0, 'd0': d0, 'dp0': dp0, 'dpp0': dpp0, 'v_top': v_top, 'index': i0, 'now_static': now_static,
            'need_static': float(need_v.min()), 'need_dynamic': need_d,
            'need_static_v': np.broadcast_to(need_v[None, :], cost.shape).ravel(),   # per candidate (differs by speed)
            'off': np.broadcast_to(OFF[:, None], cost.shape).ravel(),
            'off_end': np.broadcast_to(END[:, None], cost.shape).ravel(),
            's_mid': None if SM is None else np.broadcast_to(SM[:, None], cost.shape).ravel(),
            'D': np.broadcast_to(DD[:, None], cost.shape).ravel(),
            'v': np.broadcast_to(vs, cost.shape).ravel(),
            'min_static': stat3.ravel(), 'min_dynamic': dyn.ravel(), 'curv': np.broadcast_to(curv[:, None], cost.shape).ravel(),
            'w_max': wmax.ravel(), 'reason': reason.ravel(), 'total': total.ravel(),
            'terms': {kk: v.ravel() for kk, v in terms.items()},
            'best': None if best is None else int(np.ravel_multi_index(best, cost.shape)),
        }
        if best is None:
            return None
        b = best
        return {'t': t, 'x': cart[b][:, 0], 'y': cart[b][:, 1], 'yaw': cart[b][:, 2], 'kappa': cart[b][:, 3],
                'v': cart[b][:, 4], 'valid': valid[b], 'offset': float(OFF[b[0]]), 'D': float(DD[b[0]]),
                'speed': float(vs[b[1]]),
                'shape': {'s0': float(s0), 'd0': float(d0), 'dp0': float(dp0), 'dpp0': dpp0, 'off': float(END[b[0]]),
                          'D': float(DD[b[0]]), 'mid': None if MID is None else float(MID[b[0]]),
                          's_mid': None if SM is None else float(SM[b[0]])}}

    @staticmethod
    def _static_clear(grid, X, Y, TH, body):
        """Distance from the body (body_points) to the nearest scan point for each pose (X, Y, TH)."""
        c, s = np.cos(TH)[..., None], np.sin(TH)[..., None]
        bx, by = body[:, 0], body[:, 1]                                   # look up all body points at once (much faster than looping per point)
        return grid.lookup(X[..., None] + bx * c - by * s, Y[..., None] + bx * s + by * c).min(axis=-1)

    def _dynamic_clear(self, mv, t, X, Y, TH, offs, rad):
        """X, Y, TH: (..., n) poses at time samples t. Distance to constant-velocity predictions of moving obstacles minus clearance radius.
        Obstacles fully behind the body's rear end at that time are ignored: the robot cannot avoid something coming from
        behind by stopping, and stopping would only block its way (so objects behind do not reject every forward candidate)."""
        c = self.cfg
        px = mv[:, 0, None] + mv[:, 2, None] * t[None, :]                # (M, n)
        py = mv[:, 1, None] + mv[:, 3, None] * t[None, :]
        R = mv[:, 4, None] + c.dynamic_margin + c.dynamic_growth * t[None, :]
        cos, sin = np.cos(TH), np.sin(TH)
        rear = c.footprint[0]
        out = np.full(np.shape(X), np.inf)
        for m in range(len(mv)):
            behind = (px[m] - X) * cos + (py[m] - Y) * sin + mv[m, 4] < rear    # obstacle's front edge is behind the body's rear end
            for o in offs:
                d = np.hypot(X + o * cos - px[m], Y + o * sin - py[m]) - R[m]
                out = np.minimum(out, np.where(behind, np.inf, d))
        return out - rad


# ------------------------------------------------------------------ follower
class FrenetFollower:
    """Follows an A* path (map frame) with the Frenet planner (the bridge's only follower).
    Computation is in the odom frame so the robot/scan do not jump when AMCL corrects map->odom; only the reference moves.
    compute() state: driving | blocked (no safe candidate) | rotating (turning before start, aligning after arrival) | done"""

    def __init__(self, path_map, goal_yaw, cfg=None, goal_tolerance=0.08, yaw_tolerance=0.1):
        self.cfg = cfg or FrenetConfig()
        self.path_map = np.asarray(path_map, dtype=float).reshape(-1, 2)
        self.ref_map = build_reference(self.path_map, self.cfg.reference_ds, self.cfg.reference_sigma)
        self.planner = FrenetPlanner(self.cfg)
        self.goal_yaw = goal_yaw
        self.goal_tolerance = goal_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.progress = 0             # reference index (project only near it, so paths that double back do not jump)
        self.offset = None            # previously chosen lateral offset
        self.prev_shape = None        # shape chosen last cycle (start d'' of the next plan); kept only after cycles driven by a candidate
        self.arrived = False
        self.turning = False          # turning in place before driving
        self.turn_dir = 0.0           # turn direction (+1 / -1), kept until the turn ends
        self.align_dir = 0.0          # alignment turn direction after arrival (kept until done, 0 = not chosen yet)
        self.rotate_first = math.radians(60)   # when stopped, turn in place first if heading differs from the reference by more than this
        self.turn_done = math.radians(30)      # the turn ends inside this
        self.min_v = 0.03             # forward commands slower than this do not move the real robot
        self.last_trajectory = None
        self.last_debug = None

    @property
    def static_blocked(self):
        """Whether the last plan was blocked with every shape blocked by static obstacles alone (waiting will not help -> the bridge replans A* at once)."""
        ev = self.planner.last_eval
        if not ev or ev.get('best') is not None:
            return False
        return bool(np.all(np.asarray(ev['min_static']) < np.asarray(ev.get('need_static_v', ev['need_static']))))

    def _rotation_clear(self, grid, x, y, yaw, w, horizon):
        ts = np.arange(1, int(horizon / 0.1) + 1) * 0.1
        clear = FrenetPlanner._static_clear(grid, np.full_like(ts, x), np.full_like(ts, y), yaw + w * ts,
                                            body_points(self.cfg.footprint, self.cfg.footprint_extra))
        return float(clear.min()) > 0.01 if len(ts) else True

    def _turn(self, pts, x, y, yaw, err, was_turning, dbg):
        """Turn in place (v, w, state) or None (cannot turn). If turning the near way would hit a corner, turn the far way;
        if both are blocked, make a short move to make room. The direction already being turned is tried first (so that
        after turning past half a revolution the 'near side' does not flip back and forth), then the opposite one."""
        c = self.cfg
        near = math.copysign(1.0, err)
        dirs = [self.turn_dir, -self.turn_dir] if was_turning and self.turn_dir else [near, -near]
        # in-place turning / making room only needs the area around the robot (turn radius ~0.3 m + short move + free-radius comparison)
        grid = StaticGrid.around(pts, x, y, 0.6, c.grid_res)
        for k in dirs:
            rem = abs(err) if k == near else 2.0 * math.pi - abs(err)           # remaining angle in this direction
            w = k * max(0.3, min(c.max_w, c.rotate_speed, 2.0 * rem))
            if self._rotation_clear(grid, x, y, yaw, w, max(1.0, rem / abs(w))):
                self.turn_dir = k
                dbg.update(branch='turn_first', err=err, turn_dir=k)
                return 0.0, w, 'rotating'
        room = self._make_room(grid, x, y, yaw)
        if room is not None:          # both blocked: short move to make room (stay in turning state)
            self.turn_dir = 0.0
            dbg.update(branch='make_room', err=err)
            return room[0], room[1], 'driving'
        return None

    def _align(self, pts, x, y, yaw, err, dbg):
        """Turn in place to the goal heading after arrival (v, w, state). If the body (+ attachments) would enter the safety
        margin while turning that way, try the other way. If neither works, finish at the current heading ('done',
        dbg align='skipped'): the goal heading is nice-to-have, not worth scraping a wall for.
        If already standing inside the margin, allow it as long as it gets no closer. The chosen direction is kept until done."""
        c = self.cfg
        grid = StaticGrid.around(pts, x, y, 0.6, c.grid_res)
        body = body_points(c.footprint, c.footprint_extra)
        now = float(FrenetPlanner._static_clear(grid, np.array([x]), np.array([y]), np.array([yaw]), body)[0])
        need = min(c.safety_margin, now - 0.005)
        near = math.copysign(1.0, err)
        # keep the chosen direction, but when almost there (within 0.5 rad) use the near side so a slight overshoot does not cause a full extra turn
        dirs = [self.align_dir] if self.align_dir and abs(err) > 0.5 else [near, -near]
        for k in dirs:
            rem = abs(err) if k == near else 2.0 * math.pi - abs(err)        # remaining angle in this direction
            ang = np.linspace(0.0, k * rem, max(int(rem / 0.05), 1) + 1)
            clear = FrenetPlanner._static_clear(grid, np.full_like(ang, x), np.full_like(ang, y), yaw + ang, body)
            if float(clear.min()) >= need:
                self.align_dir = k
                # limited to rotate_speed like the start turn: turning at max_w overshoots on the real robot and
                # disturbs AMCL heading and scan-to-map matching
                lim = min(c.max_w, c.rotate_speed)
                w = float(np.clip(2.0 * k * rem, -lim, lim))
                if abs(w) < 0.3:
                    w = math.copysign(0.3, w)
                dbg.update(align=k, align_clear=float(clear.min()))
                return 0.0, w, 'rotating'
        dbg.update(align='skipped', align_err=err, align_now=now)
        return 0.0, 0.0, 'done'

    def _approach(self, pts, x, y, yaw, gx, gy, dbg):
        """Past the path end, toward goal (gx, gy): turn in place if far off, otherwise check the straight line the body will sweep and drive slowly."""
        c = self.cfg
        dist = math.hypot(gx - x, gy - y)
        err = math.atan2(gy - y, gx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        if abs(err) > 0.3:
            turn = self._turn(pts, x, y, yaw, err, self.turning, dbg)
            if turn is not None:
                self.turning = True
                return turn
            return 0.0, 0.0, 'blocked'
        grid = StaticGrid.around(pts, x, y, 0.6, c.grid_res)
        body = body_points(c.footprint, c.footprint_extra)
        ds = np.linspace(0.0, dist, max(int(dist / 0.02), 1) + 1)
        th = np.full_like(ds, yaw)
        clear = FrenetPlanner._static_clear(grid, x + ds * math.cos(yaw), y + ds * math.sin(yaw), th, body)
        need = c.safety_margin if clear[0] >= c.safety_margin else max(float(clear[0]) - 0.02, 0.01)
        if float(clear.min()) < need:
            return 0.0, 0.0, 'blocked'
        dbg.update(branch='approach', err=err, dist=dist)
        v = min(0.08, max(self.min_v, dist))
        return v, float(np.clip(2.0 * err, -c.max_w, c.max_w)), 'driving'

    def _make_room(self, grid, x, y, yaw):
        """When turning in place is blocked both ways: among short 1 s moves at +-0.05 m/s, the one (v, w) that does not bring
        the body closer to obstacles and maximizes the free radius around the turn center (room to turn). None if none improves it."""
        body = body_points(self.cfg.footprint, self.cfg.footprint_extra)
        now = float(FrenetPlanner._static_clear(grid, np.array([x]), np.array([y]), np.array([yaw]), body)[0])
        need = min(self.cfg.safety_margin, now - 0.005)
        free_now = float(grid.lookup(np.array([x]), np.array([y]))[0])
        best, best_free = None, free_now + 0.005
        ts = np.arange(1, 11) * 0.1
        for v in (-0.05, 0.05):
            for w in (-0.6, -0.3, 0.0, 0.3, 0.6):
                th = yaw + w * ts
                X = x + np.cumsum(v * np.cos(th) * 0.1)
                Y = y + np.cumsum(v * np.sin(th) * 0.1)
                if float(FrenetPlanner._static_clear(grid, X, Y, th, body).min()) < need:
                    continue
                free = float(grid.lookup(X[-1:], Y[-1:])[0])
                if free > best_free:
                    best, best_free = (v, w), free
        return best

    def compute(self, pose, v0, w0, map_to_odom, static_points, dynamic_obstacles, moving_speed=0.08):
        """pose: odom (x, y, yaw), map_to_odom: (x, y, yaw) transforming map points into odom.
        static_points: scan points in odom, dynamic_obstacles: odom [x, y, vx, vy, r]. Returns (v, w, state)."""
        c = self.cfg
        x, y, yaw = pose
        self.last_trajectory = None
        self.planner.last_eval = None
        dbg = self.last_debug = {'branch': None}
        # the previous shape carries over only when this cycle drives a candidate (turning/approach/blocked has no curvature to continue -> next starts at d''=0)
        prev_shape, self.prev_shape = self.prev_shape, None
        ref = self.ref_map.transformed(*map_to_odom)
        goal_yaw = self.goal_yaw + map_to_odom[2]
        gx, gy = (float(v) for v in ref.at(ref.s_end)[:2])
        # moving obstacles are handled by prediction; remove their scan points from the static set (do not count the same object twice)
        dyn = np.asarray(dynamic_obstacles, dtype=float).reshape(-1, 5)
        moving = dyn[np.hypot(dyn[:, 2], dyn[:, 3]) > moving_speed] if len(dyn) else dyn
        pts = np.asarray(static_points, dtype=float).reshape(-1, 2)
        if len(moving) and len(pts):
            dd = np.linalg.norm(pts[:, None, :] - moving[None, :, :2], axis=2)
            pts = pts[np.all(dd > moving[None, :, 4] + 0.1, axis=1)]
        if self.arrived or math.hypot(gx - x, gy - y) < self.goal_tolerance:
            self.arrived = True
            dbg['branch'] = 'arrived'
            err = math.atan2(math.sin(goal_yaw - yaw), math.cos(goal_yaw - yaw))
            if abs(err) < self.yaw_tolerance:
                return 0.0, 0.0, 'done'
            return self._align(pts, x, y, yaw, err, dbg)
        self.progress, s0 = ref.project(x, y, self.progress - 20, self.progress + 400)
        if s0 >= ref.s_end - 0.01:
            # Past the path end but outside the goal tolerance (passed a few cm to the side): count as arrived if close,
            # otherwise approach the goal slowly (instead of creeping along the straight extension past the end)
            dist_goal = math.hypot(gx - x, gy - y)
            if dist_goal < 2.0 * self.goal_tolerance:
                self.arrived = True
                return self.compute(pose, v0, w0, map_to_odom, static_points, dynamic_obstacles, moving_speed)
            return self._approach(pts, x, y, yaw, gx, gy, dbg)
        # Turn in place if the heading differs a lot from the reference direction at the current point; while moving
        # (v0 > 0.05) only beyond 100°. Using the local direction (not a point ahead) avoids repeated stop-and-turn at sharp bends.
        err = float(ref.at(s0)[2]) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        near_moving = len(moving) and float(np.min(np.hypot(moving[:, 0] - x, moving[:, 1] - y))) < 1.5
        was_turning = self.turning
        start = self.rotate_first if v0 < 0.05 else math.radians(100)
        self.turning = abs(err) > (self.turn_done if self.turning else start) and not near_moving
        if self.turning:
            turn = self._turn(pts, x, y, yaw, err, was_turning, dbg)
            if turn is not None:
                return turn
            self.turning = False      # cannot turn: leave it to the planner (if all blocked -> blocked -> the bridge backs off/replans)
        self.turn_dir = 0.0
        traj = self.planner.plan(ref, (x, y, yaw), v0, pts, moving, hint=self.progress, prev_offset=self.offset,
                                 prev_shape=prev_shape)
        dbg.update(branch='frenet', s0=s0)
        if traj is None:
            # Blocked while more than 30° off the path direction: try turning in place (when the start turn was skipped
            # because the error was under 60° or a moving obstacle was near)
            ev = self.planner.last_eval or {}
            if abs(math.atan(ev.get('dp0', 0.0))) > self.turn_done:   # relative to the local path direction (no turn just because the path bends ahead)
                turn = self._turn(pts, x, y, yaw, err, False, dbg)
                if turn is not None:
                    self.turning = True
                    return turn
            return 0.0, 0.0, 'blocked'
        self.offset = traj['offset']
        self.prev_shape = traj['shape']
        self.last_trajectory = np.column_stack([traj['x'][traj['valid']], traj['y'][traj['valid']]])
        # execute the start of the plan: speed 0.1 s ahead, mean curvature over 0.1-0.3 s x speed
        v = min(float(traj['v'][1]), c.max_speed)
        if 0.0 < v < self.min_v:
            v = self.min_v
        kappa = float(np.mean(traj['kappa'][1:4]))
        w = float(np.clip(v * kappa, -c.max_w, c.max_w))
        dbg.update(offset=traj['offset'], D=traj['D'], speed=traj['speed'], kappa=kappa)
        return v, w, 'driving'
