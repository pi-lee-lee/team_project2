"""Dynamic Window Approach. ROS 의존성 없음 (numpy 벡터화).
(~/tb3_nav_ws/src/tb3_nav/tb3_nav/core/dwa.py + controller_base.py 에서 가져옴)

1) 현재 속도 + 가속도 한계로 이번 주기에 낼 수 있는 (v, w) 범위 = dynamic window
2) 그 안에서 (v, w) 를 격자로 샘플링 -> 각각 predict_time 동안 굴려본 궤적(rollout)
3) 정적 점(현재 위치) / 동적 장애물(속도로 미래 위치 예측)과의 여유거리 계산
   -> 첫 충돌 지점 전에 멈출 수 없는 궤적 제거 (admissible velocity: v^2/2a <= 충돌까지 거리)
   -> 움직이는 물체가 있으면 추가로: 이번 주기만 그 (v, w) 로 가고 최대 감속으로 서는 궤적을 물체 예측 위치와
      직접 비교 (v^2/2a 는 '멈추면 안전' 을 가정하지만 움직이는 물체는 로봇이 멈춰도 다가온다)
   -> 여유(움직이는 물체엔 dynamic_margin 포함)를 끝까지 지키는 궤적이 있으면 그중에서만 고른다 (비용이 아니라 제약)
4) 비용 = 방향 + 경로이탈 + 목표거리 + 진행도 + 장애물근접 + 느림 + 여유침범 -> 최소 비용 (v, w) 선택
   거리 항은 무차원으로 맞춘다 (가중치끼리 비교할 수 있게):
     path     = 경로이탈 / path_scale                       (path_scale 만큼 벗어나면 1)
     goal     = 목표거리 / 한 번 굴려 갈 수 있는 거리(max_v * predict_time)
     progress = 1 - 경로를 따라 전진한 길이 / (max_v * predict_time)   (최대 속도로 따라가면 0, 멈추면 1, 후진 최대 2)
   경로이탈/목표거리만으로는 '경로 옆에 가만히 있기' 와 '경로를 따라 가기' 의 차이가 작다.
   안전은 비용이 아니라 3) 의 제약으로 지킨다.
"""
import math
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
    path_scale: float = 0.5           # 경로이탈 정규화 기준 [m] (비키는 차선 0.6m 와 비슷한 크기)
    w_heading: float = 0.4
    # w_path / w_goal 은 정규화된 항의 계수. 예전(미터 단위) params.yaml 값 1.5 / 2.3 과 같은 효과
    # (params.yaml 의 max_v 0.15, predict_time 2.5 기준): 1.5 * path_scale 0.5 = 0.75, 2.3 * 0.15 * 2.5 = 0.86
    # params.yaml 에 예전 값(1.5 / 2.3)이 남아 있으면 경로/목표 쪽 힘이 2~2.7배가 된다
    w_path: float = 0.75
    w_goal: float = 0.86
    # 경로를 따라 전진한 정도. dwa_tune 시나리오 + 검증 시나리오에서 0~1.0 은 시간 차이가 없고 최소 여유만 줄어서
    # (0.098 -> 0.076) 기본은 끔. 실물에서 '경로 옆에 서 있기' 가 보이면 0.3 정도부터
    w_progress: float = 0.0
    w_obstacle: float = 0.6
    w_speed: float = 0.3
    w_collision: float = 2.0          # 나중에 부딪힐 궤적의 벌점 (빨리 부딪힐수록 큼)


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

    def _braking_rollout(self, state, vs, ws):
        """(v, w) 를 한 제어 주기 낸 뒤 최대 감속으로 멈추는 궤적 (K, T, 3). 멈춘 뒤엔 그 자리."""
        c = self.cfg
        steps = int(round(c.predict_time / c.sim_dt))
        t = (np.arange(steps) + 1) * c.sim_dt
        brake = np.clip(t - c.control_period, 0.0, None)                       # 감속 시작 후 시간
        sv = np.clip(np.abs(vs)[:, None] - c.max_acc_v * brake[None], 0.0, None) * np.sign(vs)[:, None]
        sw = np.clip(np.abs(ws)[:, None] - c.max_acc_w * brake[None], 0.0, None) * np.sign(ws)[:, None]
        yaw = state.yaw + np.cumsum(sw * c.sim_dt, axis=1)
        x = state.x + np.cumsum(sv * np.cos(yaw) * c.sim_dt, axis=1)
        y = state.y + np.cumsum(sv * np.sin(yaw) * c.sim_dt, axis=1)
        return np.stack([x, y, yaw], axis=2)

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
        if len(dynamic_obstacles):
            # v^2/2a 검사는 '멈추면 안전' 이라고 보지만 움직이는 물체는 로봇이 멈춰도 계속 다가온다.
            # → 이번 주기만 그 (v, w) 로 가고 최대 감속으로 서는 궤적을 물체 예측 위치와 직접 비교해서 추가로 거른다.
            #   지금 바로 감속해도(달리던 로봇은 그 자리에 못 선다) 가까워지는 상황이면, 그보다 나빠지지 않을 때만.
            # (이게 없으면 진행 쪽 계수를 키웠을 때 막 움직이기 시작한 물체 앞으로 끼어들어 겹쳤다: dwa_tune cross_3.5s)
            risky = np.flatnonzero(can_stop & (clear < c.safety_margin) & (vs != 0.0))
            if len(risky):
                bt = self._braking_rollout(state, vs[risky], ws[risky])
                b_dyn = self._clearance(bt, np.full(bt.shape[:2], np.inf), dynamic_obstacles, soft=False).min(axis=1)
                ref = self._braking_rollout(state, np.array([state.v]), np.array([state.w]))
                ref_dyn = self._clearance(ref, np.full(ref.shape[:2], np.inf), dynamic_obstacles, soft=False).min()
                can_stop[risky] = b_dyn >= min(c.safety_margin, ref_dyn - 0.005)
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
        reach = max(c.max_v * c.predict_time, 1e-6)     # 한 번 굴려 갈 수 있는 최대 거리
        goal_dist = np.linalg.norm(to_goal, axis=1) / reach
        if len(local_path):
            d = np.linalg.norm(end[:, None, :2] - local_path[None, :, :], axis=2)   # (K, P)
            path_dist = d.min(axis=1) / c.path_scale
        else:
            path_dist = np.zeros(len(vs))
        if len(local_path) >= 2:
            # 끝점과 로봇을 경로의 가장 가까운 점에 투영해서 호 길이 차이 = 경로를 따라 전진한 길이
            arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(local_path, axis=0).T))])
            i_robot = np.argmin(np.hypot(local_path[:, 0] - state.x, local_path[:, 1] - state.y))
            advance = arc[d.argmin(axis=1)] - arc[i_robot]
        else:   # 경로가 없으면 목표까지 거리가 줄어든 만큼
            advance = np.hypot(goal[0] - state.x, goal[1] - state.y) - goal_dist * reach
        progress = 1.0 - np.clip(advance / reach, -1.0, 1.0)
        soft_clear = soft_t.min(axis=1)
        obstacle = np.clip((c.obstacle_influence - soft_clear) / c.obstacle_influence, 0.0, 1.0)
        slowness = (c.max_v - vs) / max(c.max_v, 1e-6)   # 후진은 1 보다 커져서 자연히 덜 선택됨

        # 여유(soft) 영역을 침범하는 궤적에 벌점: 빨리 침범할수록, 깊이 침범할수록 큼
        # -> 안전한 궤적이 있으면 그쪽이 이기고, 없으면 '가장 덜 위험한' 쪽(후진/감속)을 고른다
        soft_hit = soft_t < c.safety_margin
        first_soft = np.where(soft_hit.any(axis=1), soft_hit.argmax(axis=1), T)
        depth = np.clip(c.safety_margin - soft_clear, 0.0, None) / c.robot_radius
        collision = np.where(first_soft < T, 1.0 - first_soft / T, 0.0) + depth

        terms = {
            'heading': c.w_heading * heading_err,
            'path': c.w_path * path_dist,
            'goal': c.w_goal * goal_dist,
            'progress': c.w_progress * progress,
            'obstacle': c.w_obstacle * obstacle,
            'speed': c.w_speed * slowness,
            'collision': c.w_collision * collision,
        }
        cost = sum(terms.values())
        cost[~valid] = np.inf
        # 여유(soft: 움직이는 물체엔 dynamic_margin 포함)를 끝까지 지키는 궤적이 있으면 그중에서만 고른다.
        # 비용(w_collision)으로만 막으면 경로/목표 계수를 키웠을 때 천천히 가로지르는 물체 앞으로 끼어들어,
        # 뒤따라온 물체와 6mm 까지 가까워졌다 (원래 계수는 기다렸음). 없을 때만 비용으로 '덜 위험한' 쪽을 고름
        safe = valid & (collision <= 0.0)
        if np.any(safe):
            cost[~safe] = np.inf
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
    compute() 반환 state: driving | blocked (안전한 궤적 없음) | rotating (도착 후 방향 맞춤, 또는 출발 전 돌아서기) | done

    가야 할 방향이 rotate_first(90°) 보다 크게 뒤쪽이면 DWA 대신 제자리에서 먼저 돈다 (방향이 45° 안으로 들어올
    때까지). DWA 는 지금 속도에서 한 주기에 낼 수 있는 범위만 보므로, 멈춘 채 목표가 정확히 뒤에 있으면
    후보 회전이 ±0.15 rad/s 뿐이라 굴려 봐도 방향이 거의 안 바뀌고 좌우가 대칭이어서 '가만히 있기' 가
    가장 싸다 → 속도 0 을 계속 내며 멈춰 있었다 (sim 강당 자동 탐색, 줄을 바꿀 때 다음 목표가 뒤쪽).
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
        self.rotate_first = math.radians(90)
        self.turning = False      # 출발 전 돌아서는 중 (45° 안으로 들어오면 끝)
        # 돌아서기 회전 속도 (rad/s): LiDAR(5Hz) 한 바퀴 동안 많이 돌면 스캔이 휘어 SLAM 매칭이 나빠짐
        self.rotate_speed = 0.6
        # 목표점을 장애물 너머로: lookahead 점 근처에 장애물이 있으면 경로를 따라 더 앞의, 장애물에서 이 거리 넘게
        # 떨어진 첫 경로점(target_max 까지)을 목표로 한다. 0 이면 끔.
        # 경로 이탈 비용(w_path)을 빼면 목표점만이 경로 쪽으로 당기는 힘인데, 목표점이 상자 바로 뒤에 있으면
        # '상자 쪽으로 가기' 와 '피하기' 가 싸워 상자 앞에 멈췄다 → 장애물 너머의 점을 보면 옆으로 돌아가는 궤적이 싸짐
        self.target_clear = 0.0
        self.target_max = 2.0

    def local_target(self, path, x, y):
        window = path[self.progress:self.progress + 60]
        self.progress += int(np.argmin(np.hypot(window[:, 0] - x, window[:, 1] - y)))
        ahead = path[self.progress:]
        seg = np.hypot(*np.diff(ahead, axis=0).T) if len(ahead) > 1 else np.zeros(0)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        idx = np.searchsorted(s, self.lookahead)
        return ahead[s <= self.local_path_length], ahead[min(idx, len(ahead) - 1)]

    def clear_target(self, path, x, y, pts, local_path, local_goal):
        """local_goal 이 장애물에 가까우면 경로를 따라 장애물에서 target_clear 넘게 떨어진 첫 점으로 옮긴다."""
        if self.target_clear <= 0.0 or not len(pts):
            return local_path, local_goal
        ahead = path[self.progress:]
        if len(ahead) < 2:
            return local_path, local_goal
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(ahead, axis=0).T))])
        sel = (s >= self.lookahead) & (s <= self.target_max)
        cand = ahead[sel]
        if not len(cand):
            return local_path, local_goal
        near = pts[np.hypot(pts[:, 0] - x, pts[:, 1] - y) < self.target_max + self.target_clear]
        if not len(near):
            return local_path, local_goal
        d = np.hypot(cand[:, None, 0] - near[None, :, 0], cand[:, None, 1] - near[None, :, 1]).min(axis=1)
        blocked = d < self.target_clear
        if not blocked.any():
            return local_path, local_goal
        # 앞쪽 경로가 장애물에 걸리면 처음 막힌 구간 다음의 빈 점 (없으면 target_max 끝점)
        j = int(np.argmax(blocked))
        free = np.flatnonzero(~blocked[j:])
        i = j + int(free[0]) if len(free) else len(cand) - 1
        goal = cand[i]
        return ahead[s <= s[sel][i] + 0.2], goal

    def _rotation_clear(self, x, y, yaw, w, pts, horizon=1.0):
        """제자리에서 w 로 horizon 초 도는 동안 차체가 정적 점에 닿지 않는지."""
        if not len(pts):
            return True
        t = np.arange(1, int(horizon / 0.1) + 1) * 0.1
        traj = np.stack([np.full_like(t, x), np.full_like(t, y), yaw + w * t], axis=1)[None]
        return float(self.dwa._static_clearance(traj, pts).min()) > 0.01

    def _escape_speed(self, x, y, yaw, pts, speed=0.05, need=0.10):
        """앞/뒤 중 차체 앞뒤 끝에서 장애물까지 더 많이 비어 있는 쪽으로 천천히 (m/s). 둘 다 need 보다 좁으면 0."""
        if not len(pts):
            return speed
        x0, x1, hw = self.dwa.cfg.footprint if self.dwa.cfg.footprint else (-self.dwa.cfg.robot_radius,
                                                                             self.dwa.cfg.robot_radius,
                                                                             self.dwa.cfg.robot_radius)
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = c * (pts[:, 0] - x) - s * (pts[:, 1] - y)
        ly = s * (pts[:, 0] - x) + c * (pts[:, 1] - y)
        side = np.abs(ly) <= hw + 0.03
        front = lx[side & (lx > 0)] - x1
        back = -lx[side & (lx < 0)] + x0
        gap_f = float(front.min()) if len(front) else np.inf
        gap_b = float(back.min()) if len(back) else np.inf
        if max(gap_f, gap_b) < need:
            return 0.0
        return speed if gap_f >= gap_b else -speed

    def sidestep_target(self, x, y, moving, pts, path_dir, look=1.5, offset=0.6):
        """정면으로 마주 오는 움직이는 물체가 있으면 경로를 옆으로 옮길 벡터 (odom, 2), 없으면 None.
        DWA 는 몇 초 앞만 보므로 '옆으로 크게 비키기' 를 스스로 못 찾는다 (마주 오는 물체 앞에서 뒤로만 물러나다 부딪힘).
        경로를 가로지르는 물체는 제외한다 (그 진행 방향의 옆으로 비키면 물체가 지나가는 줄을 건너게 되어 sim 에서 부딪혔음)
        → 물체가 경로 방향과 거의 반대로(120° 넘게) 움직일 때만.
        옆 지점 하나로 비키면 그 자리에 서서 물체가 다 지나갈 때까지 기다렸다 (dwa_tune headon 30초, 그중 ~10초 정지)
        → 경로 전체를 옆으로 옮긴 '옆 차선' 을 따라 계속 가고, 물체가 로봇 뒤로 지나가면 원래 경로로 돌아온다."""
        n_path = np.array([-path_dir[1], path_dir[0]])
        if self.sidestep is not None:
            # 이미 비키는 중: 물체가 로봇 옆을 다 지나갈 때까지 (경로 방향으로 뒤쪽) 옆 차선 유지
            ahead = [(ox - x) * path_dir[0] + (oy - y) * path_dir[1] > -(r + 0.3)
                     for ox, oy, vx, vy, r in moving
                     if np.hypot(ox - x, oy - y) < look + offset and (vx * path_dir[0] + vy * path_dir[1]) < 0.0]
            if any(ahead):
                return self.sidestep
            self.sidestep = None
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
                threat = (rel, r)
                break
        if threat is None:
            return None
        rel, _ = threat
        # 물체가 경로 중심선의 어느 쪽에 있는지 보고 반대쪽 우선, 같으면 벽에서 먼 쪽
        def room(p):
            return np.min(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])) if len(pts) else np.inf
        cands = [s * offset * n_path for s in (1.0, -1.0)]
        here = np.array([x, y])
        self.sidestep = max(cands, key=lambda sh: (min(room(here + sh), 1.0), -np.sign(rel @ n_path) * np.sign(sh @ n_path)))
        return self.sidestep

    def compute(self, pose, v0, w0, map_to_odom, static_points, dynamic_obstacles, moving_speed=0.05,
                unmapped_points=None):
        """pose: odom 프레임 (x, y, yaw), map_to_odom: map 점을 odom 으로 옮기는 (x, y, yaw).
        static_points: odom 스캔 점, dynamic_obstacles: odom [x, y, vx, vy, r]. 반환 (v, w, state).
        unmapped_points: 지도에 없는 물체의 스캔 점 (odom). 목표점을 장애물 너머로 옮길 때만 씀 (벽은 빼야
        좁은 통로/모서리에서 목표가 멀리 튀지 않음). None 이면 static_points 전체."""
        x, y, yaw = pose
        self.last_trajectory = None   # 이번 주기에 DWA 로 고른 궤적만 남김 (제자리 회전/빠져나오기는 None)
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
        # 움직이는 물체는 넣지 않는다 (옆 차선 비키기/예측이 맡음. 넣었더니 목표가 사람 너머로 튀어 제자리에서 돌다 부딪힘)
        obst = pts if unmapped_points is None else np.asarray(unmapped_points, dtype=float).reshape(-1, 2)
        if len(moving) and len(obst):
            obst = obst[np.all(np.hypot(obst[:, None, 0] - moving[None, :, 0], obst[:, None, 1] - moving[None, :, 1])
                               > moving[None, :, 4] + 0.1, axis=1)]
        local_path, local_goal = self.clear_target(path, x, y, obst, local_path, local_goal)
        side = self.sidestep_target(x, y, moving, pts, path_dir) if self.sidestep_enabled else None
        if side is not None:
            local_path, local_goal = local_path + side, local_goal + side
        else:
            err = math.atan2(local_goal[1] - y, local_goal[0] - x) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            limit = math.radians(45) if self.turning else self.rotate_first
            # 가까이 움직이는 물체가 있으면 제자리 회전 대신 DWA (예측해서 피함). 돌아서기는 정적 점만 보므로
            # 피하던 중 돌다가 다가오는 물체에 부딪혔음 (dwa_tune cross_back_5.0s)
            near_moving = len(moving) and float(np.min(np.hypot(moving[:, 0] - x, moving[:, 1] - y))) < 1.5
            self.turning = abs(err) > limit and not near_moving
            if self.turning:
                max_w = min(self.dwa.cfg.max_w, self.rotate_speed)
                w = float(np.clip(2.0 * err, -max_w, max_w))
                if abs(w) < 0.3:
                    w = math.copysign(0.3, err)
                # 필요한 회전 전체에 대해 검사 (1초 앞만 보면 돌다 멈추고 물러나기를 번갈아 해서, 옆으로 선 차체가
                # 벽과 나란히 미끄러지기만 했음)
                if self._rotation_clear(x, y, yaw, w, pts, horizon=max(1.0, abs(err) / abs(w))):
                    return 0.0, w, 'rotating'
                # 벽에 붙어 돌 공간이 없으면 앞/뒤 중 트인 쪽으로 천천히 떨어진 뒤 돈다
                # (그냥 DWA 에 맡기면 목표가 뒤라 멈춰 있다가, 브리지 회복 동작이 벽에 붙은 채 돌아 넘어졌음)
                v = self._escape_speed(x, y, yaw, pts)
                if v != 0.0:
                    return v, 0.0, 'driving'
                self.turning = False
        r = self.dwa.compute(RobotState(x, y, yaw, v0, w0), local_path, local_goal, pts, moving)
        self.last_trajectory = r.trajectory
        if not r.ok:
            return 0.0, 0.0, 'blocked'
        return r.v, r.w, 'driving'
