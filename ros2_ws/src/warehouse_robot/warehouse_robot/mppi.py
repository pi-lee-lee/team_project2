"""MPPI (Model Predictive Path Integral) 지역 경로 추종기. ROS 의존성 없음 (numpy 벡터화).

DWA 와 입력/출력/충돌 검사가 같다 (DWAController 를 상속해 차체 직사각형 여유, 동적 장애물 예측을 그대로 씀).
다른 점은 후보 궤적을 만드는 방법과 고르는 방법:
  DWA : (v, w) 한 쌍을 predict_time 동안 그대로 → 원호. 그중 비용 최소 하나
  MPPI: 시점마다 다른 (v_t, w_t) 순서 = 직전 해(한 칸 민 것) + 가우시안 잡음 K 개
        → 각 순서를 굴려 비용 J_k → 가중치 exp(-(J_k - min J) / λ) 로 순서들을 평균 → 새 해의 첫 (v, w) 실행
  그래서 '옆으로 빠졌다 경로로 돌아오기' 같은 S 자 궤적이 나오고, 매 주기 직전 해에서 이어서 덜 흔들린다.

비용 (시점별 평균, 거리 항은 DWA 처럼 무차원):
  path      = 경로에서 떨어진 거리 / path_scale
  progress  = 1 - 경로를 따라 전진한 길이 / (max_v * predict_time)
  goal      = 끝점 ~ 목표점 / (max_v * predict_time)
  heading   = 끝점에서 목표점 방향 오차 / pi
  obstacle  = ((obstacle_influence - 여유) / obstacle_influence)^2  (여유: 움직이는 물체엔 dynamic_margin 포함)
  collision = 여유가 safety_margin 보다 작아지는 시점부터 큰 벌점 (빨리 닿을수록 큼)
  smooth    = 각속도 변화량
안전 필터: 가중 평균은 '왼쪽으로 피하기' 와 '오른쪽으로 피하기' 를 섞어 정면(장애물)으로 갈 수 있다.
  평균 궤적이 DWA 와 같은 기준(safety_margin, 멈출 수 있는지)으로 안전하지 않으면 안전한 샘플 중 비용 최소를 쓰고,
  그것도 없으면 DWA 처럼 'blocked'.
"""
import math
from dataclasses import dataclass

import numpy as np

from .dwa import ControlResult, DWAController, DWAFollower


@dataclass
class MPPIConfig:
    samples: int = 500            # 잡음 샘플 수 K
    noise_v: float = 0.04         # 선속도 잡음 표준편차 [m/s]
    noise_w: float = 0.6          # 각속도 잡음 표준편차 [rad/s]
    temperature: float = 0.05     # λ: 작을수록 비용 최소 샘플 쪽으로 몰림
    w_path: float = 1.0
    w_progress: float = 1.0
    w_goal: float = 0.5
    w_heading: float = 0.2
    w_obstacle: float = 1.0
    w_collision: float = 50.0
    w_smooth: float = 0.02
    seed: int = 0


class MPPIController(DWAController):
    """DWAController 와 같은 compute() 인터페이스. cfg: DWAConfig (속도/가속/외곽/여유), mcfg: MPPIConfig."""

    def __init__(self, cfg=None, mcfg=None):
        super().__init__(cfg)
        self.mcfg = mcfg or MPPIConfig()
        self.rng = np.random.default_rng(self.mcfg.seed)
        self.T = int(round(self.cfg.predict_time / self.cfg.sim_dt))
        self.U = np.zeros((self.T, 2))   # 직전 해 (v, w) 순서

    def reset(self):
        self.U[:] = 0.0

    def _rollout_seq(self, state, V, W):
        """시점별 (v, w) 순서 (K, T) 를 가속 한계로 자르고 굴린 궤적 (K, T, 3), 잘린 V, W."""
        c = self.cfg
        dt = c.sim_dt
        K, T = V.shape
        V, W = V.copy(), W.copy()
        v, w = np.full(K, state.v), np.full(K, state.w)
        dv, dw = c.max_acc_v * dt, c.max_acc_w * dt
        for k in range(T):
            v = np.clip(V[:, k], np.maximum(c.min_v, v - dv), np.minimum(c.max_v, v + dv))
            w = np.clip(W[:, k], np.maximum(-c.max_w, w - dw), np.minimum(c.max_w, w + dw))
            V[:, k], W[:, k] = v, w
        yaw = state.yaw + np.cumsum(W * dt, axis=1)
        x = state.x + np.cumsum(V * np.cos(yaw) * dt, axis=1)
        y = state.y + np.cumsum(V * np.sin(yaw) * dt, axis=1)
        return np.stack([x, y, yaw], axis=2), V, W

    def _candidates(self, state):
        """잡음 샘플 + 직전 해 그대로 + 정지 + DWA 같은 일정 원호 몇 개 (벽 옆/뒤돌기에서 잡음만으로 못 찾는 해)."""
        c, m = self.cfg, self.mcfg
        T = self.T
        U = np.vstack([self.U[1:], self.U[-1:]])          # 한 칸 밀기 (warm start)
        n = max(m.samples, 1)
        V = U[None, :, 0] + self.rng.normal(0.0, m.noise_v, (n, T))
        W = U[None, :, 1] + self.rng.normal(0.0, m.noise_w, (n, T))
        arcs_v = np.array([c.max_v, 0.5 * c.max_v, 0.0, c.min_v])
        arcs_w = np.linspace(-c.max_w, c.max_w, 9) * 0.5
        av, aw = np.meshgrid(arcs_v, arcs_w, indexing='ij')
        self.stop_index = n + 1
        V = np.vstack([V, U[None, :, 0], np.zeros((1, T)), np.repeat(av.ravel()[:, None], T, axis=1)])
        W = np.vstack([W, U[None, :, 1], np.zeros((1, T)), np.repeat(aw.ravel()[:, None], T, axis=1)])
        return V, W

    def _costs(self, state, traj, V, W, local_path, goal, hard_t, soft_t):
        c, m = self.cfg, self.mcfg
        K, T, _ = traj.shape
        reach = max(c.max_v * c.predict_time, 1e-6)
        xy = traj[:, :, :2]
        if len(local_path):
            d = np.linalg.norm(xy[:, :, None, :] - local_path[None, None, :, :], axis=3)   # (K, T, P)
            path_cost = (d.min(axis=2) / c.path_scale).mean(axis=1)
        else:
            d = None
            path_cost = np.zeros(K)
        if d is not None and len(local_path) >= 2:
            arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(local_path, axis=0).T))])
            i_robot = np.argmin(np.hypot(local_path[:, 0] - state.x, local_path[:, 1] - state.y))
            advance = arc[d[:, -1, :].argmin(axis=1)] - arc[i_robot]
        else:
            advance = np.hypot(goal[0] - state.x, goal[1] - state.y) - np.linalg.norm(goal[None] - xy[:, -1], axis=1)
        progress = 1.0 - np.clip(advance / reach, -1.0, 1.0)
        to_goal = goal[None, :] - xy[:, -1]
        goal_cost = np.linalg.norm(to_goal, axis=1) / reach
        bearing = np.arctan2(to_goal[:, 1], to_goal[:, 0])
        heading = np.abs(np.arctan2(np.sin(bearing - traj[:, -1, 2]), np.cos(bearing - traj[:, -1, 2]))) / np.pi
        obstacle = (np.clip((c.obstacle_influence - soft_t) / c.obstacle_influence, 0.0, 1.0) ** 2).mean(axis=1)
        # 여유 침범: 침범한 시점부터 끝까지의 비율 + 깊이 (빨리, 깊이 닿을수록 큼)
        hit = soft_t < c.safety_margin
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), T)
        depth = np.clip(c.safety_margin - soft_t.min(axis=1), 0.0, None) / max(c.robot_radius, 1e-6)
        collision = np.where(first < T, 1.0 - first / T, 0.0) + depth
        smooth = (np.abs(np.diff(W, axis=1)).mean(axis=1) + abs(W[:, 0] - state.w) / T) / max(c.max_w, 1e-6)
        terms = {
            'path': m.w_path * path_cost,
            'progress': m.w_progress * progress,
            'goal': m.w_goal * goal_cost,
            'heading': m.w_heading * heading,
            'obstacle': m.w_obstacle * obstacle,
            'collision': m.w_collision * collision,
            'smooth': m.w_smooth * smooth,
        }
        return sum(terms.values()), terms

    def _safe(self, state, V, W, hard_t, dynamic_obstacles):
        """DWA 와 같은 안전 기준: 끝까지 safety_margin 밖이거나, 첫 침범 전에 멈출 수 있고 지금 닿아 있지 않음.
        움직이는 물체가 있으면 '멈추면 안전' 이 틀릴 수 있다 (물체는 계속 다가옴) → 첫 (v, w) 를 한 주기 내고
        최대 감속으로 서는 궤적이 물체 예측 위치와 여유를 지키는지(지금 바로 서는 것보다 나빠지지 않는지)도 본다.
        (sim: 옆에 서 있던 사람이 걸어 나올 때 그 자리에 멈춰서 부딪혔음)"""
        c = self.cfg
        T = hard_t.shape[1]
        hit = hard_t < c.safety_margin
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), T)
        v0 = np.abs(V[:, 0])
        dist = np.where(np.arange(T)[None, :] < first[:, None], np.abs(V), 0.0).sum(axis=1) * c.sim_dt
        can_stop = (v0 ** 2 / (2.0 * c.max_acc_v) <= dist) & (hard_t[:, 0] >= 0.0)
        if len(dynamic_obstacles):
            risky = np.flatnonzero(can_stop & (first < T))
            if len(risky):
                bt = self._braking_rollout(state, V[risky, 0], W[risky, 0])
                b_dyn = self._clearance(bt, np.full(bt.shape[:2], np.inf), dynamic_obstacles, soft=False).min(axis=1)
                ref = self._braking_rollout(state, np.array([state.v]), np.array([state.w]))
                ref_dyn = self._clearance(ref, np.full(ref.shape[:2], np.inf), dynamic_obstacles, soft=False).min()
                can_stop[risky] = b_dyn >= min(c.safety_margin, ref_dyn - 0.005)
        return (first == T) | can_stop

    def compute(self, state, local_path, local_goal, static_points, dynamic_obstacles):
        c, m = self.cfg, self.mcfg
        static_points = np.asarray(static_points, dtype=float).reshape(-1, 2)
        dynamic_obstacles = np.asarray(dynamic_obstacles, dtype=float).reshape(-1, 5)
        local_path = np.asarray(local_path, dtype=float).reshape(-1, 2)
        goal = np.asarray(local_goal, dtype=float)
        if len(static_points):
            reach = c.max_v * c.predict_time + c.robot_radius + c.obstacle_influence + 0.3
            near = np.hypot(static_points[:, 0] - state.x, static_points[:, 1] - state.y) < reach
            static_points = static_points[near]

        V, W = self._candidates(state)
        traj, V, W = self._rollout_seq(state, V, W)
        static_clear = self._static_clearance(traj, static_points)
        hard_t = self._clearance(traj, static_clear, dynamic_obstacles, soft=False)
        soft_t = self._clearance(traj, static_clear, dynamic_obstacles, soft=True)
        cost, terms = self._costs(state, traj, V, W, local_path, goal, hard_t, soft_t)

        # 가중 평균 (정보 이론적 MPPI 갱신). 비용이 무한대인 샘플은 없음 (벌점은 유한)
        beta = cost.min()
        wgt = np.exp(-(cost - beta) / max(m.temperature, 1e-9))
        wgt /= wgt.sum()
        U = np.stack([(wgt[:, None] * V).sum(axis=0), (wgt[:, None] * W).sum(axis=0)], axis=1)

        # 평균 해를 굴려 안전 확인
        mt, mv, mw = self._rollout_seq(state, U[None, :, 0], U[None, :, 1])
        m_static = self._static_clearance(mt, static_points)
        m_hard = self._clearance(mt, m_static, dynamic_obstacles, soft=False)
        m_soft = self._clearance(mt, m_static, dynamic_obstacles, soft=True)
        m_cost, m_terms = self._costs(state, mt, mv, mw, local_path, goal, m_hard, m_soft)
        if self._safe(state, mv, mw, m_hard, dynamic_obstacles)[0]:
            self.U = np.stack([mv[0], mw[0]], axis=1)
            self.last_terms = {k: float(v[0]) for k, v in m_terms.items()}
            return ControlResult(float(mv[0, 0]), float(mw[0, 0]), True, mt[0])
        safe = self._safe(state, V, W, hard_t, dynamic_obstacles)
        if not np.any(safe):
            # 안전한 게 없으면(다가오는 물체를 피할 수 없음) 여유가 가장 커지는 쪽으로. 멈춰 있는 것보다 나을 때만
            clear = hard_t.min(axis=1)
            stay = clear[self.stop_index]
            best = int(np.argmax(clear))
            if clear[best] > stay + 0.005:
                self.U = np.stack([V[best], W[best]], axis=1)
                return ControlResult(float(V[best, 0]), float(W[best, 0]), True, traj[best])
            self.U[:] = 0.0
            return ControlResult(0.0, 0.0, False, None)
        best = int(np.argmin(np.where(safe, cost, np.inf)))
        self.U = np.stack([V[best], W[best]], axis=1)
        self.last_terms = {k: float(v[best]) for k, v in terms.items()}
        return ControlResult(float(V[best, 0]), float(W[best, 0]), True, traj[best])


class MPPIFollower(DWAFollower):
    """DWAFollower 와 같고 지역 계획만 MPPI. 출발 전 돌아서기, 옆 차선 비키기, 도착 후 방향 맞추기는 그대로."""

    def __init__(self, path_map, goal_yaw, cfg=None, mcfg=None, controller=None, **kw):
        super().__init__(path_map, goal_yaw, cfg, **kw)
        # 재계획으로 추종기를 새로 만들 때 직전 해(warm start)를 이어 쓰려면 controller 를 넘긴다
        self.dwa = controller if controller is not None else MPPIController(cfg, mcfg)
