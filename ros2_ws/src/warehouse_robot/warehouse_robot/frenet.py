"""Frenet 좌표 로컬 플래너 + 궤적 추종 (ROS 의존성 없음, numpy 벡터화).

핵심 개념은 github.com/jeongseonarm/prediction-planning-control (MIT, planning/local_path_planning) 에서:
  A* 경로를 매끄러운 기준선(스무딩 스플라인)으로 만들고, 그 위의 (s: 경로를 따라, d: 옆으로) 좌표에서
  '옆 오프셋 x 속도' 후보를 만들어 충돌 검사 후 비용이 가장 작은 것을 고른다. 막히면 선다.
  (기준선 스플라인, Frenet→직교 변환 식은 리포/PythonRobotics(MIT) 와 같음)

터틀봇(차동 구동, 0.05~0.18 m/s, 폭 0.9 m 통로)에 맞게 다시 짠 부분:
  - 옆 모양: 후보 하나 = d(s) 다항식 하나 (구간을 이어 붙이지 않음). Werling(2010) IV-B 저속 d(s) 를 바탕으로, 조건 7개 → 6차:
      시작 d, d', d'' = 지금 상태 (d'' 는 Werling 처럼 직전에 고른 궤적의 그 자리 값)        ← 갈아탈 때 w 튐 (아래)
      끝(길이 D) d = 끝 오프셋, d' = 0, d'' = 0 (Werling 목표 상태: 경로와 평행, 곡률 이어짐)
      중간(D x via_ratio) d = 중간 오프셋                                                     ← 좁은 곳 (아래)
    후보는 D 까지만 있고 검사도 D 까지만 (유지/복귀 구간 없음). 경로로 돌아오는 건 비용(끝 오프셋²) + 매 주기 재계획.
    중간 오프셋 근거: 5차 한 구간(끝 오프셋 하나)은 좁은 곳에서 '틈 안 위치'와 '그 뒤 위치'를 따로 못 골라 막혔음
    (실물 기록 57개 재주행: 주행 중 돌아서기 4 → 10~13). 6차, D 1.0 m: 도착 41 → 43, 막힘 108 → 96 s, 오프셋 갈아타기 314 → 208,
    갈아탈 때 |Δw| 최대 0.30 → 0.24, 최소 여유 같음 (기준 = 3차/5차 + 유지/복귀, 2026-10-29). 길이 0.8 m: 1.0 은 좁은 틈 앞에서 미리 멈춤
    5차인 이유 (Werling 은 횡 저크 최소화, 우리는): 명령이 w = v·κ 라 곡률이 끊기면 w 가 튐. 예전 3차(d'' 안 맞춤)는
    같은 계획을 이어 갈 땐 문제없었지만(실물 37분: |Δw| 0.2 rad/s 넘음 0.3%) 오프셋을 갈아탈 때 6.6%, 0.5 넘음 1.4%
    (최대 1.69)였고, 그 순간 IMU-바퀴 각속도 차이가 평소의 14배. 시작 d'' + 끝 d''=0 (유지 구간과 곡률 연속) = 조건 6개 → 5차.
    (예전: D 뒤 유지/복귀 구간을 붙이고 1.0 m 까지 검사. 논문에 없고 근거가 기록 한 건뿐이라 뺐음, 2026-10-29)
  - 속도: 다항식 대신 지금 속도에서 max_accel 로 목표 속도까지 가/감속 (cmd_vel 로 속도를 직접 주므로)
  - 경로 모양과 속도를 나눔: 정적 장애물은 모양만으로 '거리' 0~D 를 검사 → 속도와 무관.
    (예전 DWA 는 '시간' 으로 굴려서, 느린 궤적이 장애물까지 못 가 안전으로 남아 장애물 옆/앞에서 기어갔음)
    움직이는 물체만 시간(속도 프로필)으로 등속 예측 원과 검사 (반경 + dynamic_margin + dynamic_growth·t)
  - 후보: 오프셋 ±max_offset (offset_step) x 후보 길이 lateral_dists x 속도 (target 에서
    speed_step 씩 n_speed 개 더) → 약 450 개, 0.1초 주기에 10 ms 안팎
  - 비용(자동차의 저크/시간 대신 터틀봇 단위, 각 항 0~1 근처): 경로에서 벗어난 정도(검사 구간 평균 |d|), 직전 오프셋과 다름(흔들림),
    옆으로 꺾는 가파름(max|d'|), 느림, 정적 여유 부족(clear_pref 보다 가까움, 거리 구간에서), 움직이는 물체 여유 부족
  - 여유(safety_margin)는 제약, 그 위로는 비용 → 틈에서는 가운데 쪽, 넓으면 경로 위
  - 이미 여유 안에 있으면(벽 옆) 지금보다 2 cm 넘게 가까워지지 않는 후보는 허용 (빠져나오기, 1 cm 아래는 안 됨)
  - 충돌(정적): 차체 직사각형 둘레 + 가운데 줄의 점(2 cm 간격)에서 스캔 점 거리 격자(grid_res)를 조회 → 직사각형까지 거리.
    (원 4개로 덮으면 앞뒤가 12 cm 커져서, 실물에서 벽 8 cm 앞에 비스듬히 선 로봇이 돌지도 못하고 막혔음)
    움직이는 물체는 차체를 원 4개로 덮고 등속 예측 원과 검사
  - 속도 프로필은 '실제 속도' 기준 (옆으로 비킬 때/커브 바깥쪽에서 경로 방향 속도보다 커지지 않게, 실물: 0.15 설정에 0.25 m/s)
  - 운동 한계: |v·κ| <= max_w (각속도), v <= max_speed
  - 추종: 계획의 앞부분을 그대로 실행 (v = 0.1초 뒤 계획 속도, w = v·κ(0.1~0.3초 평균)). 계획이 늘 지금 위치/방향에서
    시작하므로 오차는 다음 계획이 바로잡음 (pure pursuit 로 따라가면 옆으로 비킬 때 안쪽으로 질러가 늦게 비켰음)
  - 인지 범위: 정적 격자는 이번 후보들이 지나가는 곳 + 차체 + clear_pref 만 (뒤는 차체 뒤 끝 근처까지), 제자리 회전은
    로봇 둘레 0.6 m. 움직이는 물체는 horizon 안에 후보 영역에 닿을 수 있는 것만, 차체 뒤 끝보다 뒤에 있는 순간은 무시
  - 출발 전 돌아서기 / 도착 후 방향 맞추기는 제자리 회전 (후보는 앞으로 가는 궤적만). 가까운 쪽이 막히면 먼 쪽으로,
    양쪽 다 막히면(구석: 실물에서 벽 13 cm 앞, 옆 9 cm) 짧게 앞/뒤로 움직여 돌 자리를 만든 뒤 돈다
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.ndimage import distance_transform_edt

# 후보가 빠진 이유 (last_eval['reason'])
OK, KINEMATIC, STATIC, DYNAMIC = 0, 1, 2, 3
REASONS = {OK: '통과', KINEMATIC: '운동 한계', STATIC: '정적 장애물', DYNAMIC: '움직이는 물체'}


def normalize(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def transform_points(points, x, y, yaw):
    """(N, 2) 점들을 (x, y, yaw) 만큼 회전+이동."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                            y + s * points[:, 0] + c * points[:, 1]])


@dataclass
class FrenetConfig:
    dt: float = 0.1               # 시간 샘플 (s)
    horizon: float = 4.0          # 움직이는 물체 검사 시간 (s)
    check_step: float = 0.05      # 그 거리 샘플 간격 (m)
    lateral_dists: tuple = (0.8,)            # 후보 길이 (m). 검사도 이 길이까지
    max_offset: float = 0.3       # 옆 오프셋 후보 ±max_offset (m)
    offset_step: float = 0.025     # 실제 소화기 틈(양쪽 9 cm)은 5 cm 간격으로는 가운데를 못 맞춰 막혔음
    target_speed: float = 0.15    # (m/s) 속도 후보: target, target - speed_step, ... (n_speed 개 더)
    speed_step: float = 0.05
    n_speed: int = 2
    max_speed: float = 0.18
    max_accel: float = 0.3        # 속도 프로필 가/감속 (m/s²)
    max_w: float = 1.82           # 각속도 한계 |v·κ| (rad/s)
    footprint: tuple = (-0.197, 0.069, 0.153)   # 차체 직사각형 (x_min, x_max, 반폭)
    footprint_extra: tuple = (-0.277, -0.197, -0.04, 0.04)   # 차체에 붙인 물건 (x_min, x_max, y_min, y_max) x N. 정적 검사에 포함
    safety_margin: float = 0.06   # 차체와 장애물 사이 최소 여유 (제약, m). 0.08 은 실제 소화기 옆 0.5 m 틈(경로가 꺾여 비스듬히 지남)을 못 지나갔음
    # 정적 장애물 여유는 후보 속도에 따라: target_speed 에서 safety_margin, 가장 느린 후보(target - n_speed*speed_step)에서
    # safety_margin_slow, 그 사이는 비례. 좁은 틈은 느린 후보만 통과해 천천히 비집고, 넓은 곳은 그대로 넉넉하게.
    # 실물: 약 0.5 m 틈/90° 모서리에서 가장 나은 후보가 0.040~0.054 라 0.06 고정이면 막혔음 (2026-10-27)
    safety_margin_slow: float = 0.04
    clear_pref: float = 0.15      # 여유가 이보다 작으면 비용 (m)
    dynamic_margin: float = 0.15  # 움직이는 물체 추가 여유 (m)
    dynamic_growth: float = 0.03  # 예측 1초당 여유 증가 (m/s)
    grid_res: float = 0.01        # 정적 거리 격자 (m). 2 cm 는 오차 때문에 소화기 옆 0.5 m 틈을 못 지나갔음
    grid_half: float = 1.6        # 정적 격자 최대 범위: 로봇 중심 ±grid_half (실제 범위는 후보가 지나가는 곳에 맞춤, m)
    w_offset: float = 0.0         # (끔: 끝 오프셋² 로) 후보 구간 평균 |d| / max_offset
    w_change: float = 1.0         # |오프셋 - 직전 오프셋| / max_offset (경로로 돌아오는 쪽은 1/4). 0.3 은 좁은 곳에서 오프셋이 주기마다 바뀌어 w 가 튐
                                  # (실물 기록 18개 폐루프: 0.3 → 1.0 에 오프셋 바뀜 181 → 81, 도착 수 같음, 2.5 는 1개 실패)
    w_turn: float = 0.0           # (끔: jerk 로) max|d'| (옆으로 꺾는 기울기, 1 = 45°)
    w_speed: float = 1.0          # (목표 - 속도) / 목표
    w_clear: float = 1.0          # 정적: (clear_pref - 최소 여유) / clear_pref
    w_dynamic: float = 1.0        # 움직이는 물체: 같은 식
    # Werling(2010) 저속 횡 비용 Cd = kj·Js + kt·S + kd·d1² 의 항목 (각 항 0~1 근처로 나눔). 0 이면 끔
    w_end: float = 1.0            # 끝 오프셋² (d1 / max_offset)²: 경로로 돌아오게 하는 힘
    w_jerk: float = 0.3           # ∫ d'''² ds / (720·max_offset² / 0.6⁵) (0.6 m 에 max_offset 만큼 비키는 최적 5차의 값 = 1)
    w_len: float = 0.2            # 후보 길이 S / 가장 긴 후보 (천천히 수렴하는 후보에 벌점)
    via_ratio: float = 0.5        # 중간 위치를 고르는 지점 = 후보 길이 x 이 값 (0 이면 중간점 없이 5차)
    end_offset_step: float = 0.05 # 끝 오프셋 간격 (중간 오프셋은 offset_step). 후보 수 = 중간 x 끝 x 길이 x 속도
    rotate_speed: float = 1.0     # 출발 전 돌아서기 각속도 (rad/s). SLAM 모드는 브리지가 0.6 으로 (빨리 돌면 스캔이 휘어 매칭이 나빠짐)
    stop_decel: float = 0.3       # 경로 끝에서 이 감속으로 멈출 수 있는 속도 이하 (m/s²)
    stop_margin: float = 0.05     # 경로 끝 이만큼 앞에서 멈추는 프로필 (m)
    reference_ds: float = 0.05    # 기준선 간격 (m). A* 격자 5 cm
    reference_sigma: float = 0.02  # 스플라인이 A* 점에서 벗어나도 되는 RMS (m)


# ------------------------------------------------------------------ 기준선
class Reference:
    """A* 경로 → 매끄러운 기준선 [s, x, y, yaw, kappa, dkappa]. 끝은 직선으로 extend m 늘려 둔다
    (후보가 경로 끝을 넘어도 변환되게, 충돌 검사는 s_end 까지만)."""

    def __init__(self, s, x, y, yaw, kappa, dkappa, s_end):
        self.s, self.x, self.y, self.yaw, self.kappa, self.dkappa = s, x, y, np.unwrap(yaw), kappa, dkappa
        self.s_end = s_end

    def transformed(self, tx, ty, tyaw):
        """(tx, ty, tyaw) 만큼 회전+이동한 기준선 (map → odom). 곡률은 그대로."""
        c, sn = math.cos(tyaw), math.sin(tyaw)
        return Reference(self.s, tx + c * self.x - sn * self.y, ty + sn * self.x + c * self.y,
                         self.yaw + tyaw, self.kappa, self.dkappa, self.s_end)

    def at(self, s):
        return (np.interp(s, self.s, self.x), np.interp(s, self.s, self.y), np.interp(s, self.s, self.yaw),
                np.interp(s, self.s, self.kappa), np.interp(s, self.s, self.dkappa))

    def project(self, px, py, lo=0, hi=None):
        """(px, py) 에서 기준선 [lo, hi) 구간에 내린 수선의 발 → (index, s).
        가장 가까운 점은 경로 끝(s_end)까지에서만 찾는다 (끝 다음 선분으로는 s_end 를 넘을 수 있음). 끝 너머 연장선까지 찾으면
        경로가 지나온 쪽으로 되돌아와 끝나는 경우(홈에 서쪽을 보며 도착) 연장선이 로봇 자리를 지나가, 1 m 앞인데 끝을 지났다고
        보고 멈췄음 (실물 2760)"""
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
    """스플라인을 못 쓸 때(점이 적음, 90° 넘게 꺾임): 선분 위를 ds 간격으로, 곡률은 방향 변화로."""
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    sg = np.append(np.arange(0.0, s[-1], ds), s[-1]) if s[-1] > ds else np.array([0.0, s[-1]])
    x, y = np.interp(sg, s, xy[:, 0]), np.interp(sg, s, xy[:, 1])
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x))) if len(sg) > 1 else np.zeros(1)
    k = np.gradient(yaw, sg) if len(sg) > 2 else np.zeros(len(sg))
    # 5점 평균. 점이 5개보다 적으면 convolve(mode='same') 가 5개를 돌려줘 길이가 어긋나 예외가 났음
    # (15 cm 안 되는 짧은 경로 → 브리지가 죽음, 실물 2026-10-27 step-2738)
    kappa = np.convolve(k, np.ones(5) / 5, mode='same') if len(sg) >= 5 else k
    dkappa = np.gradient(kappa, sg) if len(sg) > 2 else np.zeros(len(sg))
    return sg, x, y, yaw, kappa, dkappa


def build_reference(points, ds=0.05, sigma=0.02, order=5, end_extension=0.3, extend=1.5):
    """A* 경로 점 → Reference. 리포 build_reference_path 와 같은 방법(양 끝 직선 연장 후 가중 스무딩 스플라인,
    호 길이로 다시 샘플, 곡률/곡률 변화율). end_extension 은 맞출 때만 붙이는 연장(자동차 4 m → 0.3 m)."""
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
    cusp = len(xy) > 2 and np.any(np.sum(seg[:-1] * seg[1:], axis=1) < 0.0)   # 90° 넘게 꺾임
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
    # 끝을 직선으로 연장 (곡률 0)
    se = s[-1] + np.arange(1, int(extend / ds) + 1) * ds
    c, sn = math.cos(yaw[-1]), math.sin(yaw[-1])
    return Reference(np.concatenate([s, se]), np.concatenate([x, x[-1] + c * (se - s[-1])]),
                     np.concatenate([y, y[-1] + sn * (se - s[-1])]), np.concatenate([yaw, np.full(len(se), yaw[-1])]),
                     np.concatenate([kappa, np.zeros(len(se))]), np.concatenate([dkappa, np.zeros(len(se))]),
                     float(s[-1]))


# ------------------------------------------------------------------ 좌표 변환
def frenet_to_cartesian(rx, ry, rtheta, rkappa, rdkappa, s_dot, s_ddot, d, d_prime, d_pprime):
    """(리포 frenet_vectorized.frenet_to_cartesian_batch 와 같은 식) 배열 → (..., 6) [x, y, theta, kappa, v, a]."""
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


# ------------------------------------------------------------------ 충돌 (차체 = 원 4개)
class StaticGrid:
    """스캔 점까지 거리 격자, 사각형 [x0, x1] x [y0, y1] 안의 점만. 격자 밖은 장애물 없음.
    계산(거리 변환)은 격자 넓이에 비례 → 쓸 곳(후보가 지나가는 곳 + 차체 + 선호 여유)만 만든다."""

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
        self.dist = None if self.empty else distance_transform_edt(free) * res   # 칸 중심 사이 거리

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


def body_points(footprint, extra=(), step=0.02):
    """차체 직사각형 (x_min, x_max, 반폭) 의 둘레 + 가운데 줄 점, 붙인 물건 사각형들의 둘레 (로봇 좌표, (P, 2)).
    이 점들에서 장애물까지 거리의 최소 = 차체까지 거리 (밖의 점). 가운데 줄은 점이 차체 안으로 깊이 들어온 경우를 잡는다."""
    x0, x1, hw = footprint
    pts = _rect_points(x0, x1, -hw, hw, step, middle=True)
    for i in range(0, len(extra) - 3, 4):
        pts += _rect_points(*extra[i:i + 4], step)
    return np.unique(np.round(np.vstack(pts), 6), axis=0)


def body_circles(footprint, n=4):
    """차체 직사각형 (x_min, x_max, 반폭) 을 덮는 원 n 개: 중심 x 위치들, 반경."""
    x0, x1, hw = footprint
    L = x1 - x0
    return x0 + L * (np.arange(n) + 0.5) / n, math.hypot(L / (2 * n), hw)


# ------------------------------------------------------------------ 플래너
def lateral_shape(q, d0, dp0, off, D, dpp0=0.0, mid=None, s_mid=None):
    """옆 모양 d(q), q = 경로를 따라 간 거리. 반환 (d, d', d'') (브로드캐스트). 후보 하나 = 다항식 하나.
    mid 없음: 5차, (d0, dp0, dpp0) → D 에서 (off, 0, 0) (Werling 저속 d(s)).
    mid 있음: 6차, 같은 시작/끝 조건 + 중간 거리 s_mid 에서 d = mid (조건 7개). 좁은 곳에서 '틈 안 위치'와 '그 뒤 위치'를
    따로 고를 수 있게 (5차는 끝 오프셋 하나뿐이라 틈 뒤에서 벽에 걸렸음). 구간을 이어 붙이지 않는다.
    D 너머는 끝 상태 (off, 0, 0) (보간/표시용, 검사에는 안 씀)."""
    a2 = 0.5 * np.asarray(dpp0, dtype=float)
    if mid is None:
        A = off - (d0 + dp0 * D + a2 * D ** 2)                          # 끝 조건에서 앞 세 항을 뺀 나머지
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
    """지금 속도 v0 에서 각 목표 속도 vs 까지 a_max 로 가/감속, 그 뒤 유지. 반환 (v, a, 간 거리) (nV, n)."""
    vs = np.asarray(vs, dtype=float)[:, None]
    tr = np.abs(vs - v0) / a_max                                         # 바뀌는 데 걸리는 시간
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
        """후보 속도별 정적 장애물 최소 여유: target_speed 에서 safety_margin, 가장 느린 후보 속도에서 safety_margin_slow."""
        c = self.cfg
        lo = max(c.target_speed - c.n_speed * c.speed_step, 0.0)
        span = max(c.target_speed - lo, 1e-6)
        r = np.clip((np.asarray(vs, dtype=float) - lo) / span, 0.0, 1.0)
        slow = min(c.safety_margin_slow, c.safety_margin)
        return slow + (c.safety_margin - slow) * r

    def plan(self, ref, pose, v0, static_points, moving, hint=0, prev_offset=None, prev_shape=None):
        """ref: Reference (pose 와 같은 좌표계), pose: (x, y, yaw), v0: 지금 선속도, static_points: (N, 2) 스캔 점,
        moving: (M, 5) [x, y, vx, vy, r] 움직이는 물체, prev_offset: 직전에 고른 오프셋 (흔들림 비용),
        prev_shape: 직전에 고른 모양 (s0, d0, dp0, dpp0, off, D) → 그 자리 d'' 를 시작 d'' 로 (곡률 이어짐).
        반환 궤적 dict (t, x, y, yaw, v, kappa, valid) 또는 None (막힘). self.last_eval: 후보별 판정 (기록/분석용)."""
        c = self.cfg
        x, y, yaw = pose
        i0, s0 = ref.project(x, y, hint - 20, hint + 400)
        rx, ry, ryaw, rk, _ = (float(v) for v in ref.at(s0))
        d0 = (y - ry) * math.cos(ryaw) - (x - rx) * math.sin(ryaw)
        dth = float(np.clip(normalize(yaw - ryaw), -1.2, 1.2))
        dp0 = math.tan(dth) * (1.0 - rk * d0)                            # d' = 경로에 대한 기울기
        dpp0 = 0.0                                                        # d'': 직전 계획의 그 자리 값 (없으면 0)
        if prev_shape is not None and 0.0 <= s0 - prev_shape['s0'] < prev_shape['D']:   # 직전 후보 길이 안에서만
            p = prev_shape
            dpp0 = float(lateral_shape(np.array(s0 - p['s0']), p['d0'], p['dp0'], p['off'], p['D'], p['dpp0'],
                                       p.get('mid'), p.get('s_mid'))[2])
        remaining = ref.s_end - s0
        v_end = math.sqrt(2.0 * c.stop_decel * max(remaining - c.stop_margin, 0.0))
        v_top = max(min(c.target_speed, c.max_speed, v_end), 0.02)
        vs = np.unique(np.clip(v_top - np.arange(c.n_speed + 1) * c.speed_step, 0.02, None))
        offs = np.round(np.arange(-c.max_offset, c.max_offset + 1e-9, c.offset_step), 6)
        Ds = np.asarray(c.lateral_dists, dtype=float)
        # 모양 후보 = 중간 오프셋 x 끝 오프셋 x 길이 → 한 축 (nS) 으로 펴서 (via_ratio 0 이면 끝 오프셋 x 길이, 5차)
        if c.via_ratio > 0.0:
            ends = np.round(np.arange(-c.max_offset, c.max_offset + 1e-9, c.end_offset_step), 6)
            MID, END, DD = (a.ravel() for a in np.meshgrid(offs, ends, Ds, indexing='ij'))
            SM = DD * c.via_ratio
        else:
            END, DD = (a.ravel() for a in np.meshgrid(offs, Ds, indexing='ij'))
            MID = SM = None
        OFF = END if MID is None else MID          # '오프셋' = 가까운 쪽 위치 (갈아타기 비용, 기록)
        shape_args = lambda e: (END.reshape((-1,) + (1,) * e), DD.reshape((-1,) + (1,) * e), dpp0,   # noqa: E731
                                None if MID is None else MID.reshape((-1,) + (1,) * e),
                                None if SM is None else SM.reshape((-1,) + (1,) * e))
        nS, nV = len(OFF), len(vs)
        offs_c, rad = body_circles(c.footprint)
        body = body_points(c.footprint, c.footprint_extra)
        r_body = float(np.hypot(body[:, 0], body[:, 1]).max())            # 기준점에서 차체 끝까지 (부착물 포함)

        # ---- 모양만: 정적 장애물을 거리 0 ~ D (후보 길이) 에서 (속도와 무관)
        q = np.arange(0.0, Ds.max() + 1e-9, c.check_step)                 # (nq,)
        dq, dpq, ddq = lateral_shape(q[None, :], d0, dp0, *shape_args(1))
        within = q[None, :] <= DD[:, None] + 1e-9                         # 후보 길이 안 (그 너머는 그 후보에 없음)
        sq = np.broadcast_to(s0 + q, dq.shape)
        g = frenet_to_cartesian(*ref.at(sq), np.ones_like(dq), np.zeros_like(dq), dq, dpq, ddq)
        on_path = (sq <= ref.s_end + 0.02) & within                       # 경로 끝/후보 끝 너머는 검사 안 함
        # 인지 범위: 이번 후보들이 지나가는 곳(+ 지금 자세) + 차체 끝 + 선호 여유. 그 밖의 점은 어느 후보의 여유/비용에도
        # 영향이 없다 (뒤쪽은 차체 뒤 끝 근처까지만). ±grid_half 를 넘지 않게
        gx, gy = g[..., 0][on_path], g[..., 1][on_path]
        if not len(gx):               # 경로 끝을 지났거나 아주 짧은 경로: 지금 자세 둘레만 (빈 배열이면 min() 이 예외 → 브리지가 죽음)
            gx, gy = np.array([x]), np.array([y])
        pad = r_body + c.clear_pref + 2 * c.grid_res
        bx0, bx1 = max(min(gx.min(), x) - pad, x - c.grid_half), min(max(gx.max(), x) + pad, x + c.grid_half)
        by0, by1 = max(min(gy.min(), y) - pad, y - c.grid_half), min(max(gy.max(), y) + pad, y + c.grid_half)
        grid = StaticGrid(static_points, bx0, by0, bx1, by1, c.grid_res)
        stat = np.where(on_path, FrenetPlanner._static_clear(grid, g[..., 0], g[..., 1], g[..., 2], body),
                        np.inf).min(axis=1)                               # (nS,)
        now_static = float(FrenetPlanner._static_clear(grid, np.array([x]), np.array([y]), np.array([yaw]), body)[0])
        # 정적 여유는 속도마다 (빠를수록 크게, static_margins). 이미 여유 안이면(벽 옆/구석) 가장 느린 속도만 2 cm 까지 더 가까워져도
        # 됨 (빠져나오며 뒤 모서리가 잠깐 가까워짐, 1 cm 아래로는 안 됨). 빠른 속도는 늘 제 여유를 지켜야 함: 이걸 모든 속도에 주면
        # 좁은 틈에 들어선 뒤 빠른 후보도 통과해 0.15 m/s 로 지나갔음 (단위 테스트)
        # 빠져나오기 허용은 '가장 큰 여유(빠를 때) 안에 있으면' (예전과 같은 조건): 가장 느린 여유(4 cm) 기준으로 하면 4.2 cm 에서
        # 출발할 때 첫 움직임에 뒤 모서리가 3.6 cm 로 다가가 모든 후보가 빠졌음 (실물 기록 2728 폐루프)
        margin_v = self.static_margins(vs)                                # (nV,)
        need_v = margin_v.copy()
        slowest = int(np.argmin(vs))
        if now_static < margin_v.max():
            need_v[slowest] = min(margin_v[slowest], max(now_static - 0.02, 0.01))
        curv = np.where(on_path, np.abs(g[..., 3]), 0.0).max(axis=1)      # 모양의 최대 곡률
        steep = np.where(within, np.abs(dpq), 0.0).max(axis=1)
        d3 = np.gradient(ddq, c.check_step, axis=1)                        # d''' (5차라 2차식, 수치 미분으로 충분)
        jerk = np.where(within, d3 ** 2, 0.0).sum(axis=1) * c.check_step
        mean_d = np.where(within, np.abs(dq), 0.0).sum(axis=1) / within.sum(axis=1)

        # ---- 속도 프로필 x 모양: 시간 샘플 (움직이는 물체, 각속도 한계, 그리고 실행할 궤적)
        t = np.arange(0.0, c.horizon + 1e-9, c.dt)
        V, A, dist = speed_profile(max(v0, 0.0), vs, t, c.max_accel)       # (nV, n)

        def along(sdot, sddot, q):
            dd_, dp_, ddp_ = lateral_shape(q, d0, dp0, *shape_args(2))
            return frenet_to_cartesian(*ref.at(s0 + q), sdot, sddot, dd_, dp_, ddp_)

        # 프로필 속도는 '실제 속도' 로: 경로 방향 속도 ṡ 를 그대로 쓰면 옆으로 비킬 때/커브 바깥쪽에서 실제 속도가 더 커짐
        # (실물 기록: 0.15 설정에 0.25 m/s). 한 번 굴려서 실제/경로 속도 비율 g 를 구하고 ṡ = V / g 로 다시
        qt = np.broadcast_to(dist[None], (nS, nV, len(t)))
        Vb, Ab = np.broadcast_to(V, qt.shape), np.broadcast_to(A, qt.shape)
        cart = along(Vb, Ab, qt)
        g = np.where(Vb > 1e-6, cart[..., 4] / np.maximum(Vb, 1e-6), 1.0)
        sdot = Vb / np.clip(np.nan_to_num(g, nan=1.0), 0.5, 3.0)
        qt = np.concatenate([np.zeros(qt.shape[:-1] + (1,)),
                             np.cumsum(0.5 * (sdot[..., 1:] + sdot[..., :-1]) * c.dt, axis=-1)], axis=-1)
        cart = along(sdot, np.gradient(sdot, c.dt, axis=-1), qt)         # (nS, nV, n, 6)
        st = s0 + qt
        valid = (st <= ref.s_end + 0.02) & (qt <= DD[:, None, None] + 1e-9)   # 경로 끝, 후보 끝(D)까지만
        valid[..., 0] = True
        wmax = np.where(valid, np.abs(cart[..., 4] * cart[..., 3]), 0.0).max(axis=2)
        # 실제 속도가 max_speed 를 넘지 않는지 (이미 빠르면 지금 속도까지는 허용 → 감속 후보 통과). 프로필과 딱 맞추면
        # 경로와 60° 넘게 틀어진 출발에서 위 보정이 완벽하지 않아 후보가 전부 빠졌음 (실물 기록 2596, 2597)
        over = np.where(valid, cart[..., 4] - max(c.max_speed * 1.05, v0 + 0.01), -1.0).max(axis=2)
        kin = (np.all(np.isfinite(cart).all(axis=3) | ~valid, axis=2) & (wmax <= c.max_w) & (over <= 0.0))
        mv = np.asarray(moving, dtype=float).reshape(-1, 5)
        if len(mv):   # horizon 안에 후보 영역(지금 자세에서 reach 안)에 닿을 수 있는 물체만
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

        # ---- 판정 + 비용
        stat3 = np.broadcast_to(stat[:, None], (nS, nV))
        reason = np.where(~kin, KINEMATIC, np.where(stat3 < need_v[None, :], STATIC, np.where(dyn < need_d, DYNAMIC, OK)))
        prev = 0.0 if prev_offset is None else prev_offset
        if MID is not None and prev_shape is not None:     # 직전 계획이 이 후보의 중간 지점에서 있던 옆 위치 (Werling 의 시간 일관성)
            p = prev_shape
            prev = float(lateral_shape(np.array(s0 + float(SM.max()) - p['s0']), p['d0'], p['dp0'], p['off'], p['D'],
                                       p['dpp0'], p.get('mid'), p.get('s_mid'))[0])
        pref = max(c.clear_pref, 1e-6)
        terms = {
            'offset': c.w_offset * mean_d / c.max_offset,
            # 오프셋 바뀜: 경로에서 멀어지거나 반대쪽으로 넘어가면 다, 같은 쪽에서 경로로 돌아오면 1/4 만
            # (대칭이면 장애물을 지나서도 비킨 오프셋에 머물러 경로로 안 돌아왔음)
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
            'need_static_v': np.broadcast_to(need_v[None, :], cost.shape).ravel(),   # 후보별 (속도마다 다름)
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
        """자세 (X, Y, TH) 마다 차체(body_points) 에서 가장 가까운 스캔 점까지 거리."""
        c, s = np.cos(TH)[..., None], np.sin(TH)[..., None]
        bx, by = body[:, 0], body[:, 1]                                   # 차체 점 전부 한 번에 조회 (점마다 돌면 3배 느림)
        return grid.lookup(X[..., None] + bx * c - by * s, Y[..., None] + bx * s + by * c).min(axis=-1)

    def _dynamic_clear(self, mv, t, X, Y, TH, offs, rad):
        """X, Y, TH: (..., n) 시간 샘플 t 의 자세. 움직이는 물체 등속 예측까지 거리 - 여유 반경.
        그 시각 물체가 차체 뒤 끝보다 완전히 뒤에 있으면 빼다: 뒤에서 오는 물체는 로봇이 서도 피할 수 없고,
        서면 오히려 길을 막는다 (뒤 물체 때문에 앞으로 가는 후보가 다 빠지지 않게)."""
        c = self.cfg
        px = mv[:, 0, None] + mv[:, 2, None] * t[None, :]                # (M, n)
        py = mv[:, 1, None] + mv[:, 3, None] * t[None, :]
        R = mv[:, 4, None] + c.dynamic_margin + c.dynamic_growth * t[None, :]
        cos, sin = np.cos(TH), np.sin(TH)
        rear = c.footprint[0]
        out = np.full(np.shape(X), np.inf)
        for m in range(len(mv)):
            behind = (px[m] - X) * cos + (py[m] - Y) * sin + mv[m, 4] < rear    # 물체 앞 끝이 차체 뒤 끝보다 뒤
            for o in offs:
                d = np.hypot(X + o * cos - px[m], Y + o * sin - py[m]) - R[m]
                out = np.minimum(out, np.where(behind, np.inf, d))
        return out - rad


# ------------------------------------------------------------------ 추종기
class FrenetFollower:
    """A* 경로(map)를 Frenet 플래너로 따라가는 추종기 (브리지의 유일한 추종기).
    계산은 odom 프레임: AMCL 이 map->odom 을 고칠 때 로봇/스캔이 튀지 않게, 기준선만 옮김.
    compute() 반환 state: driving | blocked (안전한 후보 없음) | rotating (출발 전 돌아서기, 도착 후 방향 맞춤) | done"""

    def __init__(self, path_map, goal_yaw, cfg=None, goal_tolerance=0.08, yaw_tolerance=0.1):
        self.cfg = cfg or FrenetConfig()
        self.path_map = np.asarray(path_map, dtype=float).reshape(-1, 2)
        self.ref_map = build_reference(self.path_map, self.cfg.reference_ds, self.cfg.reference_sigma)
        self.planner = FrenetPlanner(self.cfg)
        self.goal_yaw = goal_yaw
        self.goal_tolerance = goal_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.progress = 0             # 기준선 index (투영을 이 근처에서만 → 되돌아오는 경로에서 튀지 않게)
        self.offset = None            # 직전에 고른 옆 오프셋
        self.prev_shape = None        # 직전 주기에 고른 모양 (다음 계획의 시작 d''). 후보로 주행한 주기에만 남음
        self.arrived = False
        self.turning = False          # 출발 전 돌아서는 중
        self.turn_dir = 0.0           # 돌아서는 방향 (+1 / -1). 돌기 시작하면 끝날 때까지 유지
        self.align_dir = 0.0          # 도착 후 방향 맞추기 회전 방향 (정하면 끝날 때까지 유지, 0 = 아직)
        self.rotate_first = math.radians(60)   # 서 있을 때 기준선 방향과 이보다 차이 나면 제자리에서 먼저 돎
        self.turn_done = math.radians(30)      # 이 안으로 들어오면 돌기 끝
        self.min_v = 0.03             # 이보다 느린 전진 명령은 실제 로봇이 안 움직임 (기록: 0.007 → 실제 0)
        self.last_trajectory = None
        self.last_debug = None

    @property
    def static_blocked(self):
        """마지막 계획이 막혔고, 정적 장애물만으로도 모든 모양이 막혔는지 (기다려도 안 풀림 → 브리지가 바로 A* 재계획)."""
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
        """제자리 돌기 (v, w, state) 또는 None (못 돎). 가까운 쪽으로 돌면 모서리가 닿으면 먼 쪽으로 (실물: 벽 8 cm 앞에서
        130° 오른쪽은 막히고 230° 왼쪽은 됨), 둘 다 막히면 짧게 움직여 돌 자리를 만든다. 돌기 시작한 방향을 먼저 본다
        (먼 쪽으로 반 바퀴 넘게 돌면 '가까운 쪽' 이 바뀌어 왔다 갔다 하지 않게). 그 방향이 막히면 반대쪽도."""
        c = self.cfg
        near = math.copysign(1.0, err)
        dirs = [self.turn_dir, -self.turn_dir] if was_turning and self.turn_dir else [near, -near]
        # 제자리 회전/돌 자리 만들기는 로봇 둘레만 (회전 반경 약 0.3 m + 짧은 이동 + 빈 반경 비교)
        grid = StaticGrid.around(pts, x, y, 0.6, c.grid_res)
        for k in dirs:
            rem = abs(err) if k == near else 2.0 * math.pi - abs(err)           # 이 방향으로 남은 각도
            w = k * max(0.3, min(c.max_w, c.rotate_speed, 2.0 * rem))
            if self._rotation_clear(grid, x, y, yaw, w, max(1.0, rem / abs(w))):
                self.turn_dir = k
                dbg.update(branch='turn_first', err=err, turn_dir=k)
                return 0.0, w, 'rotating'
        room = self._make_room(grid, x, y, yaw)
        if room is not None:          # 양쪽 다 막힘: 돌 자리를 만들러 짧게 움직임 (돌기 상태는 유지)
            self.turn_dir = 0.0
            dbg.update(branch='make_room', err=err)
            return room[0], room[1], 'driving'
        return None

    def _align(self, pts, x, y, yaw, err, dbg):
        """도착 후 목표 방향으로 제자리 회전 (v, w, state). 회전 중 차체(+부착물)가 안전 여유 안으로 들어가면 그 방향은 안 돌고
        반대쪽을 본다. 둘 다 안 되면 지금 방향에서 끝낸다 ('done', dbg align='skipped'): 도착 방향은 '되면 좋은 것' 이라
        벽을 긁으며 맞추지 않는다. 예전에는 검사 없이 돌아 뒤 부착물이 벽 0~0.8 cm 까지 붙었음 (실물 2609, 2627, 2636 등).
        이미 여유 안에 서 있으면 지금보다 가까워지지만 않으면 허용. 정한 방향은 끝날 때까지 유지 (예전: ±w 로 번갈아 냄)"""
        c = self.cfg
        grid = StaticGrid.around(pts, x, y, 0.6, c.grid_res)
        body = body_points(c.footprint, c.footprint_extra)
        now = float(FrenetPlanner._static_clear(grid, np.array([x]), np.array([y]), np.array([yaw]), body)[0])
        need = min(c.safety_margin, now - 0.005)
        near = math.copysign(1.0, err)
        # 정한 방향 유지. 다만 거의 다 왔으면(0.5 rad 안) 가까운 쪽으로: 살짝 넘쳤을 때 한 바퀴를 더 돌지 않게
        dirs = [self.align_dir] if self.align_dir and abs(err) > 0.5 else [near, -near]
        for k in dirs:
            rem = abs(err) if k == near else 2.0 * math.pi - abs(err)        # 이 방향으로 남은 각도
            ang = np.linspace(0.0, k * rem, max(int(rem / 0.05), 1) + 1)
            clear = FrenetPlanner._static_clear(grid, np.full_like(ang, x), np.full_like(ang, y), yaw + ang, body)
            if float(clear.min()) >= need:
                self.align_dir = k
                # 돌아서기와 같은 rotate_speed 까지: max_w(1.82) 로 돌면 실제 2.0~2.4 rad/s 가 나와 회전마다 AMCL 방향이
                # 최대 8° 틀어졌다 바로잡히고 회전 끝 LiDAR-지도 맞음이 67~80% 로 떨어졌음 (실물 141220 세션 11번 중 6번)
                lim = min(c.max_w, c.rotate_speed)
                w = float(np.clip(2.0 * k * rem, -lim, lim))
                if abs(w) < 0.3:
                    w = math.copysign(0.3, w)
                dbg.update(align=k, align_clear=float(clear.min()))
                return 0.0, w, 'rotating'
        dbg.update(align='skipped', align_err=err, align_now=now)
        return 0.0, 0.0, 'done'

    def _approach(self, pts, x, y, yaw, gx, gy, dbg):
        """경로 끝 너머에서 목표 (gx, gy) 로: 크게 틀어졌으면 제자리에서 돌고, 아니면 차체가 지나갈 직선을 검사한 뒤 천천히."""
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
        """제자리에서 양쪽 다 못 돌 때: 1초 동안 ±0.05 m/s 로 짧게 움직이는 후보 중, 차체가 지금보다 장애물에 가까워지지
        않으면서 회전 중심 둘레의 빈 반경(돌 수 있는지)이 가장 커지는 것 (v, w). 넓어지는 게 없으면 None."""
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
        """pose: odom (x, y, yaw), map_to_odom: map 점을 odom 으로 옮기는 (x, y, yaw).
        static_points: odom 스캔 점, dynamic_obstacles: odom [x, y, vx, vy, r]. 반환 (v, w, state)."""
        c = self.cfg
        x, y, yaw = pose
        self.last_trajectory = None
        self.planner.last_eval = None
        dbg = self.last_debug = {'branch': None}
        # 직전 모양은 이번에 후보로 주행할 때만 이어 쓴다 (돌기/다가가기/막힘이면 곡률을 이을 계획이 없음 → 다음은 d''=0 에서)
        prev_shape, self.prev_shape = self.prev_shape, None
        ref = self.ref_map.transformed(*map_to_odom)
        goal_yaw = self.goal_yaw + map_to_odom[2]
        gx, gy = (float(v) for v in ref.at(ref.s_end)[:2])
        # 움직이는 물체는 예측으로, 그 자리의 스캔 점은 정적 목록에서 뺀다 (같은 물체를 두 번 세지 않게)
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
            # 경로 끝을 지났는데 아직 목표 허용 오차 밖 (옆으로 몇 cm 비켜 지나감): 가까우면 도착으로, 아니면 목표를 보고
            # 천천히 다가감. 그대로 두면 끝 너머 연장선을 따라 최저 속도로 계속 갔음 (실물 기록 폐루프 2595, 2597)
            dist_goal = math.hypot(gx - x, gy - y)
            if dist_goal < 2.0 * self.goal_tolerance:
                self.arrived = True
                return self.compute(pose, v0, w0, map_to_odom, static_points, dynamic_obstacles, moving_speed)
            return self._approach(pts, x, y, yaw, gx, gy, dbg)
        # 돌아서기: 그 자리 기준선 방향과 크게 다르면 제자리에서. 달리는 중(v0 > 0.05)에는 100° 넘게 틀어질 때만.
        # (예전: 기준선 15 cm 앞 점 방향 기준 → 경로가 급하게 꺾이는 곳에서 달리다 60° 가 넘어 서서 돌기를 반복했음, 실물 2610)
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
            self.turning = False      # 못 돌면 플래너에 맡김 (다 막히면 blocked → 브리지가 물러나기/재계획)
        self.turn_dir = 0.0
        traj = self.planner.plan(ref, (x, y, yaw), v0, pts, moving, hint=self.progress, prev_offset=self.offset,
                                 prev_shape=prev_shape)
        dbg.update(branch='frenet', s0=s0)
        if traj is None:
            # 막혔는데 경로 방향과 30° 넘게 틀어져 있으면 제자리에서 돌아 본다 (60° 미만이라 돌아서기를 안 했거나 움직이는 물체가
            # 가까워서 안 했을 때. 실물: 64~68° 틀어진 채 후보가 다 빠져 서 있었음)
            ev = self.planner.last_eval or {}
            if abs(math.atan(ev.get('dp0', 0.0))) > self.turn_done:   # 그 자리 경로 방향 기준 (앞이 꺾인 경로에서는 안 돎)
                turn = self._turn(pts, x, y, yaw, err, False, dbg)
                if turn is not None:
                    self.turning = True
                    return turn
            return 0.0, 0.0, 'blocked'
        self.offset = traj['offset']
        self.prev_shape = traj['shape']
        self.last_trajectory = np.column_stack([traj['x'][traj['valid']], traj['y'][traj['valid']]])
        # 계획의 앞부분을 그대로: 0.1초 뒤 속도, 0.1~0.3초 곡률 평균 x 속도
        v = min(float(traj['v'][1]), c.max_speed)
        if 0.0 < v < self.min_v:
            v = self.min_v
        kappa = float(np.mean(traj['kappa'][1:4]))
        w = float(np.clip(v * kappa, -c.max_w, c.max_w))
        dbg.update(offset=traj['offset'], D=traj['D'], speed=traj['speed'], kappa=kappa)
        return v, w, 'driving'
