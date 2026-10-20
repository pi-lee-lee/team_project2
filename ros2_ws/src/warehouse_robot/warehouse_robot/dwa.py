"""Dynamic Window Approach. ROS 의존성 없음 (numpy 벡터화).
(~/tb3_nav_ws/src/tb3_nav/tb3_nav/core/dwa.py + controller_base.py 에서 가져옴)

1) 현재 속도 + 가속도 한계로 이번 주기에 낼 수 있는 (v, w) 범위 = dynamic window
2) 그 안에서 (v, w) 를 격자로 샘플링 -> 각각 predict_time 동안 굴려본 궤적(rollout)
3) 정적 점(현재 위치) / 동적 장애물(속도로 미래 위치 예측)과의 여유거리 계산
   -> 첫 충돌 지점 전에 멈출 수 없는 궤적 제거 (admissible velocity: v^2/2a <= 충돌까지 거리)
4) 비용 = 방향 + 경로이탈 + 목표거리 + 장애물근접 + 느림  -> 최소 비용 (v, w) 선택
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # scipy 가 없으면 전부 비교 (느림)
    cKDTree = None


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    v: float
    w: float


@dataclass
class ControlResult:
    v: float
    w: float
    ok: bool                                  # False = 안전한 궤적 없음
    trajectory: Optional[np.ndarray] = None   # (T, 3) [x, y, yaw]


@dataclass
class DWAConfig:
    max_v: float = 0.22          # waffle_pi 하드웨어 한계 0.26
    min_v: float = -0.08         # 느린 후진 허용: 다가오는 장애물에서 물러날 수 있게
    max_w: float = 1.5           # 한계 1.82
    max_acc_v: float = 0.5
    max_acc_w: float = 2.5
    v_samples: int = 9
    w_samples: int = 21
    sim_dt: float = 0.1
    predict_time: float = 2.0
    control_period: float = 0.1
    robot_radius: float = 0.22
    # 충돌 검사용 원들의 중심 (로봇 진행 방향 x 위치, m). 기본은 원 하나 (tb3_nav 그대로).
    # 이식하면서 추가: Waffle Pi 직사각형(x -0.197~0.069, 폭 0.306)은 원 하나로 덮으면 옆으로 너무 커서
    # 좁은 통로에서 앞으로 가는 궤적이 전부 막혔다 → 반경 0.17 원 두 개 (x=-0.13, 0.0) 로 덮는다
    circle_offsets: tuple = (0.0,)
    # 직사각형 외곽 (x_min, x_max, 반폭) [m]. 주면 원 대신 실제 직사각형까지 거리로 충돌을 본다 (이식하면서 추가).
    # 원 두 개는 앞뒤로 10cm 씩 실제보다 커서, 벽 옆에 선 로봇이 '이미 닿음' 으로 판정되어 갇혔었다
    footprint: tuple = ()
    # 이미 safety_margin 보다 가까우면(벽 옆에 섬) 여유가 줄지 않는 궤적은 허용해 빠져나올 수 있게 (이식하면서 추가)
    allow_escape: bool = False
    # 후진은 전진/회전 후보에 충돌 위험(soft)이 없는 게 하나도 없을 때만 (이식하면서 추가).
    # 아니면 마주 오는 물체 앞에서 옆으로 비키지 않고 경로를 따라 30초 넘게 뒤로만 물러났음
    reverse_only_when_needed: bool = False
    safety_margin: float = 0.05
    dynamic_margin: float = 0.15      # 움직이는 장애물에는 여유를 더 둔다 (방향을 바꿀 수 있으므로)
    dynamic_growth: float = 0.05      # 예측 불확실성: 예측 시간 1초당 반경 증가 [m/s]
    obstacle_influence: float = 0.5   # 이 거리 안쪽부터 장애물 비용이 붙음
    w_heading: float = 0.4
    w_path: float = 1.0
    w_goal: float = 1.0
    w_obstacle: float = 0.6
    w_speed: float = 0.3
    w_collision: float = 5.0          # 나중에 부딪힐 궤적의 벌점 (빨리 부딪힐수록 큼)


class DWAController:
    def __init__(self, cfg=None):
        self.cfg = cfg or DWAConfig()

    def _sample(self, v0, w0):
        c = self.cfg
        dv = c.max_acc_v * c.control_period
        dw = c.max_acc_w * c.control_period
        v_lo, v_hi = max(c.min_v, v0 - dv), min(c.max_v, v0 + dv)
        w_lo, w_hi = max(-c.max_w, w0 - dw), min(c.max_w, w0 + dw)
        vs = np.linspace(v_lo, v_hi, c.v_samples)
        ws = np.linspace(w_lo, w_hi, c.w_samples)
        if w_lo < 0.0 < w_hi:
            ws = np.append(ws, 0.0)   # 직진 후보는 항상 넣는다
        if v_lo <= 0.0:
            vs = np.append(vs, 0.0)   # 제자리 회전 후보
        vv, ww = np.meshgrid(np.unique(vs), np.unique(ws), indexing='ij')
        return vv.ravel(), ww.ravel()

    def _rollout(self, state, vs, ws):
        c = self.cfg
        steps = int(round(c.predict_time / c.sim_dt))
        traj = np.empty((len(vs), steps, 3))
        x = np.full(len(vs), state.x)
        y = np.full(len(vs), state.y)
        yaw = np.full(len(vs), state.yaw)
        for k in range(steps):
            yaw = yaw + ws * c.sim_dt
            x = x + vs * np.cos(yaw) * c.sim_dt
            y = y + vs * np.sin(yaw) * c.sim_dt
            traj[:, k, 0], traj[:, k, 1], traj[:, k, 2] = x, y, yaw
        return traj

    def _static_clearance(self, traj, static_points):
        """정적 점까지 여유거리 (K, T). hard/soft 가 같으므로 한 번만 계산한다 (이식하면서 추가:
        브리지 안에서 돌리려면 빨라야 해서 KD-tree 사용. 점 360개가 0.6m 안에 있을 때 83ms → 수 ms)."""
        K, T, _ = traj.shape
        if not len(static_points):
            return np.full((K, T), np.inf)
        if self.cfg.footprint:
            return self._rect_static_clearance(traj, static_points)
        tree = cKDTree(static_points) if cKDTree is not None else None
        out = np.full(K * T, np.inf)
        for xy in self._circle_centers(traj):
            if tree is not None:
                d, _ = tree.query(xy)
            else:
                d = np.linalg.norm(xy[:, None, :] - static_points[None, :, :], axis=2).min(axis=1)
            out = np.minimum(out, d)
        return out.reshape(K, T) - self.cfg.robot_radius

    def _rect_local(self, traj, pts):
        """pts (P, k, 2) 를 각 자세(P) 의 로봇 좌표로."""
        x, y, yaw = traj[:, :, 0].ravel(), traj[:, :, 1].ravel(), traj[:, :, 2].ravel()
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        dx, dy = pts[..., 0] - x[:, None], pts[..., 1] - y[:, None]
        return c * dx + s * dy, -s * dx + c * dy

    def _rect_distance(self, lx, ly):
        x0, x1, hw = self.cfg.footprint
        cx, hx = (x0 + x1) / 2, (x1 - x0) / 2
        ex = np.maximum(np.abs(lx - cx) - hx, 0.0)
        ey = np.maximum(np.abs(ly) - hw, 0.0)
        inside = (ex == 0) & (ey == 0)
        # 안쪽이면 음수 (얼마나 파고들었는지)
        depth = -np.minimum(hx - np.abs(lx - cx), hw - np.abs(ly))
        return np.where(inside, depth, np.hypot(ex, ey))

    def _rect_static_clearance(self, traj, static_points):
        """직사각형 외곽에서 가장 가까운 정적 점까지 거리 (K, T). 직사각형 중심에서 가까운 점 12개만 비교."""
        K, T, _ = traj.shape
        x0, x1, _ = self.cfg.footprint
        off = (x0 + x1) / 2
        x, y, yaw = traj[:, :, 0].ravel(), traj[:, :, 1].ravel(), traj[:, :, 2].ravel()
        centers = np.column_stack([x + off * np.cos(yaw), y + off * np.sin(yaw)])
        k = min(12, len(static_points))
        if cKDTree is not None:
            _, idx = cKDTree(static_points).query(centers, k=k)
            idx = idx.reshape(len(centers), k)
        else:
            d = np.linalg.norm(centers[:, None, :] - static_points[None, :, :], axis=2)
            idx = np.argsort(d, axis=1)[:, :k]
        lx, ly = self._rect_local(traj, static_points[idx])
        return self._rect_distance(lx, ly).min(axis=1).reshape(K, T)

    def _circle_centers(self, traj):
        """충돌 원 중심들, 각각 (K*T, 2)."""
        x, y, yaw = traj[:, :, 0].ravel(), traj[:, :, 1].ravel(), traj[:, :, 2].ravel()
        return [np.column_stack([x + o * np.cos(yaw), y + o * np.sin(yaw)]) for o in self.cfg.circle_offsets]

    def _clearance(self, traj, static_clear, dynamic_obstacles, soft):
        """각 궤적, 각 시점의 여유거리 [m] (로봇 반경 제외 후), (K, T). 장애물 없으면 inf.

        soft=False : 실제 반경만 -> 진짜 충돌 판정용
        soft=True  : 동적 장애물에 dynamic_margin + 시간에 따른 불확실성 추가 -> 비용용
        """
        c = self.cfg
        K, T, _ = traj.shape
        clear = static_clear.copy()

        if len(dynamic_obstacles):
            t = (np.arange(T) + 1) * c.sim_dt                      # (T,)
            pos = dynamic_obstacles[None, :, 0:2] \
                + dynamic_obstacles[None, :, 2:4] * t[:, None, None]   # (T, M, 2)
            r = np.broadcast_to(dynamic_obstacles[None, :, 4], (T, len(dynamic_obstacles)))
            if soft:
                r = r + c.dynamic_margin + c.dynamic_growth * t[:, None]
            if c.footprint:
                M = len(dynamic_obstacles)
                obs = np.broadcast_to(pos[None], (K, T, M, 2)).reshape(K * T, M, 2)
                lx, ly = self._rect_local(traj, obs)
                d = self._rect_distance(lx, ly).reshape(K, T, M) - r[None]
            else:
                d = np.full((K, T, len(dynamic_obstacles)), np.inf)
                for xy in self._circle_centers(traj):
                    d = np.minimum(d, np.linalg.norm(xy.reshape(K, T, 1, 2) - pos[None], axis=3))  # (K, T, M)
                d = d - r[None] - c.robot_radius
            clear = np.minimum(clear, d.min(axis=2))
        return clear

    def compute(self, state, local_path, local_goal, static_points, dynamic_obstacles):
        """
        state             : RobotState (odom frame)
        local_path        : (P, 2) 로봇 앞쪽 전역 경로 일부
        local_goal        : (2,) 이번 주기의 목표점 (경로 위 lookahead 점)
        static_points     : (N, 2) 정적 장애물 점 (스캔)
        dynamic_obstacles : (M, 5) [x, y, vx, vy, radius] 추적된 움직이는 장애물
        """
        c = self.cfg
        static_points = np.asarray(static_points, dtype=float).reshape(-1, 2)
        dynamic_obstacles = np.asarray(dynamic_obstacles, dtype=float).reshape(-1, 5)
        local_path = np.asarray(local_path, dtype=float).reshape(-1, 2)
        goal = np.asarray(local_goal, dtype=float)

        # 로봇에서 먼 정적 점은 계산에서 뺀다 (속도 향상)
        if len(static_points):
            reach = (c.max_v * c.predict_time + c.robot_radius + c.obstacle_influence
                     + max(abs(o) for o in c.circle_offsets))
            near = np.hypot(static_points[:, 0] - state.x,
                            static_points[:, 1] - state.y) < reach
            static_points = static_points[near]

        vs, ws = self._sample(state.v, state.w)
        traj = self._rollout(state, vs, ws)
        static_clear = self._static_clearance(traj, static_points)
        hard_t = self._clearance(traj, static_clear, dynamic_obstacles, soft=False)
        soft_t = self._clearance(traj, static_clear, dynamic_obstacles, soft=True)
        T = hard_t.shape[1]

        # 충돌 판정 (hard: 실제 반경 기준)
        #  - 궤적 전체가 safety_margin 밖이면 OK
        #  - 아니면 첫 충돌 시점까지 이동거리 안에 멈출 수 있어야 OK (속도를 줄이는 선택지를 살림)
        #  - 제자리 회전(v=0)은 실제 접촉만 금지
        clear_t = hard_t
        clear = clear_t.min(axis=1)
        hit = clear_t < c.safety_margin
        first_hit = np.where(hit.any(axis=1), hit.argmax(axis=1), clear_t.shape[1])
        dist_to_hit = np.abs(vs) * first_hit * c.sim_dt
        can_stop = vs ** 2 / (2.0 * c.max_acc_v) <= dist_to_hit
        now_clear = clear_t[:, 0] >= 0.0
        valid = (clear >= c.safety_margin) | (can_stop & now_clear & (vs != 0.0)) \
            | ((vs == 0.0) & now_clear)
        if c.allow_escape and not np.any(valid & (vs != 0.0)):
            # 벽 옆에 바짝 서 있어 앞뒤로 갈 궤적이 없으면: 여유가 지금보다 줄지 않는 이동은 허용 (빠져나오기)
            start = clear_t[:, 0]
            valid = valid | ((vs != 0.0) & (clear >= np.minimum(start, c.safety_margin) - 0.005)
                             & (clear_t[:, -1] > start))
        if not np.any(valid):
            if c.allow_escape:
                # 갈 곳이 없으면(움직이는 물체가 다가와 멈춰 있어도 부딪힐 때 등) 여유가 가장 커지는 쪽으로 천천히 피한다.
                # 그 자리에 멈추면 다가오는 물체에 그대로 부딪혔음 (sim: 원기둥에 밀려 넘어짐)
                stay = (vs == 0.0) & (ws == 0.0)
                stay_clear = clear[stay].max() if np.any(stay) else -np.inf
                best = int(np.argmax(clear))
                if clear[best] > stay_clear + 0.005:   # 가만히 있는 것보다 여유가 클 때만
                    return ControlResult(float(vs[best]), float(ws[best]), True, traj[best])
            return ControlResult(0.0, 0.0, False, None)

        end = traj[:, -1, :]
        to_goal = goal[None, :] - end[:, :2]
        bearing = np.arctan2(to_goal[:, 1], to_goal[:, 0])
        heading_err = np.abs(np.arctan2(np.sin(bearing - end[:, 2]),
                                        np.cos(bearing - end[:, 2]))) / np.pi
        goal_dist = np.linalg.norm(to_goal, axis=1)
        if len(local_path):
            path_dist = np.linalg.norm(
                end[:, None, :2] - local_path[None, :, :], axis=2).min(axis=1)
        else:
            path_dist = np.zeros(len(vs))
        soft_clear = soft_t.min(axis=1)
        obstacle = np.clip((c.obstacle_influence - soft_clear) / c.obstacle_influence, 0.0, 1.0)
        slowness = (c.max_v - vs) / max(c.max_v, 1e-6)   # 후진은 1 보다 커져서 자연히 덜 선택됨

        # 여유(soft) 영역을 침범하는 궤적에 벌점: 빨리 침범할수록, 깊이 침범할수록 큼
        # -> 안전한 궤적이 있으면 그쪽이 이기고, 없으면 '가장 덜 위험한' 쪽(후진/감속)을 고른다
        soft_hit = soft_t < c.safety_margin
        first_soft = np.where(soft_hit.any(axis=1), soft_hit.argmax(axis=1), T)
        depth = np.clip(c.safety_margin - soft_clear, 0.0, None) / c.robot_radius
        collision = np.where(first_soft < T, 1.0 - first_soft / T, 0.0) + depth

        cost = (c.w_heading * heading_err + c.w_path * path_dist + c.w_goal * goal_dist
                + c.w_obstacle * obstacle + c.w_speed * slowness + c.w_collision * collision)
        cost[~valid] = np.inf
        if c.reverse_only_when_needed:
            fwd = np.where(valid & (vs >= 0.0) & (collision <= 0.0), cost, np.inf)
            if np.isfinite(fwd).any():
                cost = fwd
        best = int(np.argmin(cost))
        return ControlResult(float(vs[best]), float(ws[best]), True, traj[best])


def transform_points(points, x, y, yaw):
    """(N, 2) 점들을 (x, y, yaw) 만큼 회전+이동."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                            y + s * points[:, 0] + c * points[:, 1]])


class DWAFollower:
    """A* 경로(map) 를 DWA 로 따라가는 추종기. 브리지의 PurePursuit 자리에 들어간다.

    계산은 odom 프레임에서 한다 (tb3_nav local_planner 와 같음): AMCL 이 map->odom 을 고칠 때마다
    map 좌표가 몇 cm 씩 튀는데, 로봇/스캔/추적 장애물을 odom 에 두면 연속적이고 경로만 살짝 옮겨진다.
    compute() 반환 state: driving | blocked (안전한 궤적 없음) | rotating (도착 후 방향 맞춤) | done
    """

    def __init__(self, path_map, goal_yaw, cfg=None, goal_tolerance=0.1, lookahead=0.5,
                 local_path_length=1.2, yaw_tolerance=0.1):
        self.path_map = np.asarray(path_map, dtype=float).reshape(-1, 2)
        self.goal_yaw = goal_yaw
        self.dwa = DWAController(cfg)
        self.goal_tolerance = goal_tolerance
        self.lookahead = lookahead
        self.local_path_length = local_path_length
        self.yaw_tolerance = yaw_tolerance
        self.progress = 0
        self.arrived = False
        self.last_trajectory = None
        self.sidestep = None      # 마주 오는 물체를 피해 잠시 가는 옆 지점 (odom)
        self.sidestep_enabled = True

    def local_target(self, path, x, y):
        window = path[self.progress:self.progress + 60]
        self.progress += int(np.argmin(np.hypot(window[:, 0] - x, window[:, 1] - y)))
        ahead = path[self.progress:]
        seg = np.hypot(*np.diff(ahead, axis=0).T) if len(ahead) > 1 else np.zeros(0)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        idx = np.searchsorted(s, self.lookahead)
        return ahead[s <= self.local_path_length], ahead[min(idx, len(ahead) - 1)]

    def sidestep_target(self, x, y, moving, pts, path_dir, look=1.5, offset=0.6):
        """정면으로 마주 오는 움직이는 물체가 있으면 옆으로 offset 만큼 비켜선 지점, 없으면 None.
        DWA 는 몇 초 앞만 보므로 '옆으로 크게 비키기' 를 스스로 못 찾는다 (마주 오는 물체 앞에서 뒤로만 물러나다 부딪힘).
        경로를 가로지르는 물체는 제외한다 (그 진행 방향의 옆으로 비키면 물체가 지나가는 줄을 건너게 되어 sim 에서 부딪혔음)
        → 물체가 경로 방향과 거의 반대로(120° 넘게) 움직일 때만."""
        threat = None
        for ox, oy, vx, vy, r in moving:
            rel = np.array([ox - x, oy - y])
            dist = np.hypot(*rel)
            speed = np.hypot(vx, vy)
            if dist > look or speed < 0.03:
                continue
            u = np.array([vx, vy]) / speed
            closing = -rel @ u                       # 물체가 로봇 쪽으로 오는 정도 (>0 이면 다가옴)
            lateral = abs(rel[0] * u[1] - rel[1] * u[0])   # 물체 진행선과 로봇 사이 거리
            head_on = u @ path_dir < -0.5
            if head_on and closing > 0.0 and lateral < 0.153 + r + 0.15:
                threat = (u, r)
                break
        if threat is None:
            self.sidestep = None
            return None
        if self.sidestep is None:
            u, _ = threat
            n = np.array([-u[1], u[0]])
            cands = [np.array([x, y]) + s * offset * n for s in (1.0, -1.0)]
            # 정적 장애물(벽)에서 더 먼 쪽으로
            def room(p):
                return np.min(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])) if len(pts) else np.inf
            self.sidestep = max(cands, key=room)
        return self.sidestep

    def compute(self, pose, v0, w0, map_to_odom, static_points, dynamic_obstacles, moving_speed=0.05):
        """pose: odom 프레임 (x, y, yaw), map_to_odom: map 점을 odom 으로 옮기는 (x, y, yaw).
        static_points: odom 스캔 점, dynamic_obstacles: odom [x, y, vx, vy, r]. 반환 (v, w, state)."""
        x, y, yaw = pose
        path = transform_points(self.path_map, *map_to_odom)
        goal_yaw = self.goal_yaw + map_to_odom[2]
        if self.arrived or np.hypot(path[-1, 0] - x, path[-1, 1] - y) < self.goal_tolerance:
            # 도착: 제자리에서 목표 방향 맞추기 (회전은 DWA 대신 단순 비례 제어)
            self.arrived = True
            err = np.arctan2(np.sin(goal_yaw - yaw), np.cos(goal_yaw - yaw))
            if abs(err) < self.yaw_tolerance:
                return 0.0, 0.0, 'done'
            w = float(np.clip(2.0 * err, -self.dwa.cfg.max_w, self.dwa.cfg.max_w))
            if abs(w) < 0.3:
                w = 0.3 * np.sign(w)
            return 0.0, w, 'rotating'
        local_path, local_goal = self.local_target(path, x, y)
        # 움직이는 장애물은 예측해서 쓰고, 그 자리의 스캔 점은 정적 목록에서 뺀다 (중복 방지)
        dyn = np.asarray(dynamic_obstacles, dtype=float).reshape(-1, 5)
        moving = dyn[np.hypot(dyn[:, 2], dyn[:, 3]) > moving_speed] if len(dyn) else dyn
        pts = np.asarray(static_points, dtype=float).reshape(-1, 2)
        if len(pts):
            # 5cm 격자로 솎기 (가까운 벽은 점이 촘촘해서 계산만 늘어남)
            _, keep = np.unique(np.floor(pts / 0.05).astype(np.int64), axis=0, return_index=True)
            pts = pts[keep]
        if len(moving) and len(pts):
            d = np.linalg.norm(pts[:, None, :] - moving[None, :, :2], axis=2)
            pts = pts[np.all(d > moving[None, :, 4] + 0.1, axis=1)]
        ahead = path[min(self.progress + 6, len(path) - 1)] - path[self.progress]
        path_dir = ahead / np.hypot(*ahead) if np.hypot(*ahead) > 1e-6 else np.array([np.cos(yaw), np.sin(yaw)])
        side = self.sidestep_target(x, y, moving, pts, path_dir) if self.sidestep_enabled else None
        if side is not None:
            local_path, local_goal = np.array([side]), side
        r = self.dwa.compute(RobotState(x, y, yaw, v0, w0), local_path, local_goal, pts, moving)
        self.last_trajectory = r.trajectory
        if not r.ok:
            return 0.0, 0.0, 'blocked'
        return r.v, r.w, 'driving'
