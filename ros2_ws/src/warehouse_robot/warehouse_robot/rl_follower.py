"""강화학습(PPO) 지역 계획기 + 추종기. 브리지의 DWAFollower 자리에 들어간다 (controller=rl).

학습: D:\\webserver_rl (tools/rl_env.py, rl_train.py). 정책 가중치는 numpy 로 내보낸 .npz (torch 불필요).
관측 165개 (학습 환경과 똑같이 만듦):
  LiDAR 5° 구간 72개 최소거리 (0~3.5m → -1~1) x 2 (지금 + 0.1초 전), A* 경로 0.25~2.0m 앞 8점 (로봇 좌표 / 2),
  목표 (거리, cos, sin), 현재 v / 0.26, w / 1.82
행동 2개 [-1, 1] → v 목표 -0.05~0.26 m/s, w 목표 ±1.82 rad/s, 가속 0.3 m/s², 1.5 rad/s² 제한.
안전 필터: 명령을 한 주기 낸 뒤 최대 감속으로 멈추는 1.5초 궤적을 차체(+뒤 부착물) 직사각형으로 근처 1m 스캔 점 /
  움직이는 track 에 대 보고, 여유 3cm 안이면 정책 명령에 가장 가까운 안전한 명령으로 바꿈. 정지밖에 없으면 'blocked'
  (브리지가 기다리다 blocked_timeout 뒤 복구 → 재계획).
"""
import math

import numpy as np

from .dwa import transform_points

MAX_V, MIN_V, MAX_W = 0.26, -0.05, 1.82
ACC_V, ACC_W = 0.3, 1.5
DT = 0.1
N_SECT, SCAN_MAX = 72, 3.5
PATH_PTS = np.arange(1, 9) * 0.25
LIDAR_X = -0.064
BODY = (-0.197, 0.069, 0.153)       # (x_min, x_max, 반폭)
EXTRA = (-0.277, -0.197, 0.04)      # 뒤에 붙인 물건


class MLPPolicy:
    def __init__(self, path):
        d = np.load(path)
        n = len([k for k in d.files if k.startswith('w') and k[1:].isdigit()])
        self.layers = [(d[f'w{i}'], d[f'b{i}']) for i in range(n)]
        self.out = (d['wa'], d['ba'])

    def __call__(self, obs):
        h = np.asarray(obs, dtype=np.float32)
        for w, b in self.layers:
            h = np.tanh(w @ h + b)
        return np.clip(self.out[0] @ h + self.out[1], -1.0, 1.0)


class SafetyFilter:
    def __init__(self, horizon=1.5, margin=0.03, rects=(BODY, EXTRA)):
        self.T = int(round(horizon / DT))
        self.margin = margin
        self.rects = rects

    def rollout(self, pose, v1, w1):
        """첫 주기 (v1, w1), 그 뒤 최대 감속으로 멈춤 (K, T+1, 3), 0번 = 지금 자세."""
        v, w = np.asarray(v1, float).copy(), np.asarray(w1, float).copy()
        x, y, yaw = (np.full(len(v), pose[i], float) for i in range(3))
        out = [np.stack([x, y, yaw], axis=1)]
        for k in range(self.T):
            if k:
                v = np.sign(v) * np.maximum(np.abs(v) - ACC_V * DT, 0.0)
                w = np.sign(w) * np.maximum(np.abs(w) - ACC_W * DT, 0.0)
            yaw = yaw + w * DT
            x, y = x + v * np.cos(yaw) * DT, y + v * np.sin(yaw) * DT
            out.append(np.stack([x, y, yaw], axis=1))
        return np.stack(out, axis=1)

    def _gap(self, traj, P, r=0.0):
        x, y, yaw = (traj[..., i][..., None] for i in range(3))
        c, s = np.cos(yaw), np.sin(yaw)
        dx, dy = P[:, 0] - x, P[:, 1] - y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        best = np.full(traj.shape[:2], np.inf)
        for x0, x1, hw in self.rects:
            cx, hx = (x0 + x1) / 2, (x1 - x0) / 2
            ex, ey = np.maximum(np.abs(lx - cx) - hx, 0.0), np.maximum(np.abs(ly) - hw, 0.0)
            inside = (ex == 0) & (ey == 0)
            d = np.where(inside, -np.minimum(hx - np.abs(lx - cx), hw - np.abs(ly)), np.hypot(ex, ey)) - r
            best = np.minimum(best, d.min(axis=2))
        return best

    def clearance(self, pose, vs, ws, pts, dyn):
        traj = self.rollout(pose, vs, ws)
        out = np.full(traj.shape[:2], np.inf)
        if len(pts):
            near = pts[np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1]) < 1.0]
            if len(near):
                out = self._gap(traj, near)
        t = np.arange(traj.shape[1]) * DT
        for mx, my, vx, vy, r in dyn:
            P = np.column_stack([mx + vx * t, my + vy * t])
            for k in range(traj.shape[1]):
                out[:, k] = np.minimum(out[:, k], self._gap(traj[:, k:k + 1], P[k:k + 1], r)[:, 0])
        return out.min(axis=1), traj

    def __call__(self, pose, v0, w0, tv, tw, pts, dyn):
        """((v, w) 목표, 바꿨는지, 고른 궤적 (T+1, 3))."""
        cands = sorted({(v, w) for v in (tv, tv / 2, 0.0, MIN_V) for w in (tw, tw / 2, 0.0, 0.8, -0.8)},
                       key=lambda c: abs(c[0] - tv) / (MAX_V - MIN_V) + abs(c[1] - tw) / (2 * MAX_W))
        nxt = [(v0 + np.clip(v - v0, -ACC_V * DT, ACC_V * DT), w0 + np.clip(w - w0, -ACC_W * DT, ACC_W * DT))
               for v, w in cands]
        cl, traj = self.clearance(pose, [n[0] for n in nxt], [n[1] for n in nxt], pts, dyn)
        stop = cands.index((0.0, 0.0))
        need = min(self.margin, cl[stop])        # 바로 정지했을 때 여유보다 나빠지면 안 됨
        for k, c in enumerate(cl):
            if c >= need:
                return cands[k], k > 0, traj[k]
        k = int(np.argmax(cl))
        return cands[k], True, traj[k]


class RLFollower:
    """compute() 형식은 DWAFollower 와 같음: pose / 점 / track 은 odom, 경로는 map."""

    def __init__(self, path_map, goal_yaw, policy, goal_tolerance=0.15, yaw_tolerance=0.1, safety=True):
        self.path_map = np.asarray(path_map, dtype=float)
        seg = np.hypot(*np.diff(self.path_map, axis=0).T) if len(self.path_map) > 1 else np.zeros(0)
        self.path_s = np.concatenate([[0.0], np.cumsum(seg)])
        self.goal_yaw = goal_yaw
        self.policy = policy
        self.goal_tolerance = goal_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.filter = SafetyFilter() if safety else None
        self.progress = 0
        self.arrived = False
        self.turning = False
        self.sidestep = None
        self.last_cmd = None
        self.last_trajectory = None
        self.prev_scan = None
        self.cmd = (0.0, 0.0)
        self.overrides = 0
        sect = (np.arange(360) + 2.5) % 360 // 5
        self.sect_of_deg = sect.astype(int)

    def scan_features(self, pts, x, y, yaw):
        lx, ly = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        m = np.full(N_SECT, SCAN_MAX)
        if len(pts):
            dx, dy = pts[:, 0] - lx, pts[:, 1] - ly
            r = np.hypot(dx, dy)
            deg = np.floor(np.degrees(np.arctan2(dy, dx) - yaw)).astype(int) % 360
            np.minimum.at(m, self.sect_of_deg[deg], np.minimum(r, SCAN_MAX))
        return np.clip(m / SCAN_MAX, 0, 1) * 2 - 1

    def observation(self, path, x, y, yaw, v0, w0, pts):
        hi = min(len(path), self.progress + 40)
        d = np.hypot(path[self.progress:hi, 0] - x, path[self.progress:hi, 1] - y)
        self.progress += int(np.argmin(d))
        cur = self.scan_features(pts, x, y, yaw)
        prev = self.prev_scan if self.prev_scan is not None else cur
        self.prev_scan = cur
        c, s = math.cos(yaw), math.sin(yaw)
        s0 = self.path_s[self.progress]
        pp = []
        for dd in PATH_PTS:
            k = min(np.searchsorted(self.path_s, s0 + dd), len(path) - 1)
            dx, dy = path[k, 0] - x, path[k, 1] - y
            pp += [(c * dx + s * dy) / 2.0, (-s * dx + c * dy) / 2.0]
        gx, gy = path[-1, 0] - x, path[-1, 1] - y
        ga = math.atan2(gy, gx) - yaw
        gd = math.hypot(gx, gy)
        return np.concatenate([cur, prev, np.clip(pp, -1, 1), [min(gd, 5.0) / 2.5 - 1, math.cos(ga), math.sin(ga)],
                               [v0 / MAX_V, w0 / MAX_W]]).astype(np.float32)

    def compute(self, pose, v0, w0, map_to_odom, static_points, dynamic_obstacles, moving_speed=0.05,
                unmapped_points=None):
        x, y, yaw = pose
        self.last_trajectory = None
        path = transform_points(self.path_map, *map_to_odom)
        goal_yaw = self.goal_yaw + map_to_odom[2]
        if self.arrived or np.hypot(path[-1, 0] - x, path[-1, 1] - y) < self.goal_tolerance:
            self.arrived = True
            self.cmd = (0.0, 0.0)
            err = math.atan2(math.sin(goal_yaw - yaw), math.cos(goal_yaw - yaw))
            if abs(err) < self.yaw_tolerance:
                return 0.0, 0.0, 'done'
            w = float(np.clip(2.0 * err, -MAX_W, MAX_W))
            return 0.0, (0.3 * np.sign(w) if abs(w) < 0.3 else w), 'rotating'
        pts = np.asarray(static_points, dtype=float).reshape(-1, 2)
        obs = self.observation(path, x, y, yaw, v0, w0, pts)
        a = self.policy(obs)
        tv, tw = MIN_V + (a[0] + 1) / 2 * (MAX_V - MIN_V), a[1] * MAX_W
        pv, pw = self.cmd
        state = 'rl'
        if self.filter is not None:
            dyn = np.asarray(dynamic_obstacles, dtype=float).reshape(-1, 5)
            dyn = dyn[np.hypot(dyn[:, 2], dyn[:, 3]) > moving_speed] if len(dyn) else dyn
            (tv, tw), changed, traj = self.filter((x, y, yaw), pv, pw, tv, tw, pts, dyn)
            self.last_trajectory = traj
            if changed:
                self.overrides += 1
                state = 'rl_filtered'
                if tv <= 0.0 and abs(tw) < 0.05:
                    self.cmd = (0.0, 0.0)
                    return 0.0, 0.0, 'blocked'
        v = pv + float(np.clip(tv - pv, -ACC_V * DT, ACC_V * DT))
        w = pw + float(np.clip(tw - pw, -ACC_W * DT, ACC_W * DT))
        self.cmd = (v, w)
        if self.last_trajectory is None:
            self.last_trajectory = SafetyFilter().rollout((x, y, yaw), [v], [w])[0]
        return v, w, state
