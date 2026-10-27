"""Frenet 로컬 플래너 (frenet.py): 폐루프로 굴려서 확인 (명령을 가속 한계 안에서 그대로 따르는 로봇)."""
import math

import numpy as np

from warehouse_robot.frenet import FrenetConfig, FrenetFollower, body_circles, body_points, build_reference, lateral_shape


def drive(path, pts, start=(0.0, 0.0, 0.0), goal_yaw=0.0, steps=400, moving=None, cfg=None):
    f = FrenetFollower(path, goal_yaw, cfg or FrenetConfig())
    x, y, yaw = start
    v = 0.0
    offs, rad = body_circles(f.cfg.footprint)                # 움직이는 물체 (플래너와 같은 원 모델)
    body = body_points(f.cfg.footprint, f.cfg.footprint_extra)   # 정적 (차체 + 붙인 물건 둘레)
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
    # 격자처럼 계단진 A* 경로 → 스플라인 기준선이 점에서 많이 벗어나지 않고 곡률이 작게
    pts = [(0.05 * i, 0.05 * (i // 6)) for i in range(60)]
    ref = build_reference(pts, 0.05, 0.02)
    d = np.min(np.hypot(ref.x[:, None] - np.array(pts)[None, :, 0], ref.y[:, None] - np.array(pts)[None, :, 1]), axis=1)
    on = ref.s <= ref.s_end
    assert d[on].max() < 0.05
    assert np.abs(ref.kappa[on]).max() < 5.0


def test_lateral_shape_returns_to_path():
    q = np.arange(0.0, 2.0, 0.05)
    d, dp, _ = lateral_shape(q, 0.0, 0.0, 0.2, 0.4, 0.3)
    assert abs(d[0]) < 1e-9 and abs(d[np.searchsorted(q, 0.5)] - 0.2) < 1e-9 and abs(d[-1]) < 1e-9
    held, _, _ = lateral_shape(q, 0.0, 0.0, 0.2, 0.4, -1.0)
    assert abs(held[-1] - 0.2) < 1e-9


def test_drives_straight_at_target_speed():
    f, log = drive(STRAIGHT, [])
    assert log['state'][-1] == 'done'
    assert np.mean(log['v']) > 0.13                      # 목표 0.15 (가/감속 포함)
    assert max(abs(y) for _, y in log['xy']) < 0.01


def test_passes_obstacle_beside_path_without_slowing():
    # 실제 기록과 같은 크기: 0.9 m 통로, 경로 4 cm 옆 소화기(반지름 0.066) → 0.5 m 틈으로 감속 없이 지나감
    ext = [(1.5 + 0.066 * math.cos(a), 0.11 + 0.066 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 24)]
    f, log = drive(STRAIGHT, WALLS + ext)
    assert log['state'][-1] == 'done'
    assert 'blocked' not in log['state']
    assert log['clear'] >= f.cfg.safety_margin - 0.01
    passing = [v for (x, _), v in zip(log['xy'], log['v']) if 1.2 < x < 1.8]
    assert min(passing) > 0.12                           # 예전 DWA 는 여기서 0.03 아래로 기어갔음


def test_stops_when_corridor_fully_blocked():
    block = WALLS + [(1.5, s) for s in np.arange(-0.46, 0.42, 0.03)]
    f, log = drive(STRAIGHT, block, steps=200)
    assert log['state'][-1] == 'blocked'
    assert log['clear'] > 0.2                            # 바로 앞까지 기어가지 않고 미리 섬
    assert log['v'][-1] < 1e-3


def test_turns_in_place_when_path_is_behind():
    back = [(-x, 0.0) for x in np.arange(0.0, 2.01, 0.05)]
    f, log = drive(back, [])
    assert log['state'][0] == 'rotating' and log['state'][-1] == 'done'


def test_avoids_crossing_mover():
    # 경로를 0.1 m/s 로 가로지르는 물체: 등속 예측으로 기다리거나 비켜서 부딪히지 않음
    f, log = drive(STRAIGHT, WALLS, moving=lambda t: np.array([[1.2, -0.4 + 0.1 * t, 0.0, 0.1, 0.1]]))
    assert log['state'][-1] == 'done'
    assert log['clear'] > 0.0


def test_escapes_corner_then_turns():
    # 실물 기록과 같은 구석: 앞 벽 13 cm, 왼쪽 벽 9 cm, 경로는 뒤쪽 아래 → 제자리에서는 양쪽 다 모서리가 닿음.
    # 짧게 움직여 돌 자리를 만들고, 돌아서 경로로
    walls = ([(x, 0.25) for x in np.arange(-0.6, 0.21, 0.02)] + [(0.2, y) for y in np.arange(-0.65, 0.26, 0.02)]
             + [(x, -0.65) for x in np.arange(-0.6, 0.21, 0.02)])
    path = [(0.0, 0.0)] + [(-0.05 * i, -0.3 * min(i, 6) / 6) for i in range(1, 30)]
    f, log = drive(path, walls, steps=600)
    assert log['state'][-1] == 'done'
    assert log['clear'] > 0.0


def test_actual_speed_stays_below_max_when_shifting_on_curve():
    # 커브 바깥쪽으로 비켜 가도 실제 속도는 max_speed 를 넘지 않음 (실물: 0.15 설정에 0.25 m/s 가 나왔음)
    arc = [(1.0 * math.sin(a), 1.0 - 1.0 * math.cos(a)) for a in np.linspace(0, math.pi / 2, 40)]
    obstacle = [(0.71 + 0.05 * math.cos(a) - 0.0, 0.29 + 0.05 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 12)]
    f, log = drive(arc, obstacle, steps=300)
    assert max(log['v']) <= f.cfg.max_speed + 1e-6


def test_mover_behind_does_not_stop_robot():
    # 뒤에서 비슷한 속도로 따라오는 물체 (0.12 m/s, 0.45 m 뒤): 예측 여유가 커지며 뒤 끝에 닿는 순간이 있어도 서지 않고 앞으로
    f, log = drive(STRAIGHT, WALLS, steps=120, moving=lambda t: np.array([[-0.45 + 0.12 * t, 0.0, 0.12, 0.0, 0.1]]))
    assert 'blocked' not in log['state'][:60]
    assert np.mean(log['v'][20:60]) > 0.12


def _arrived_at_origin():
    """목표에 이미 도착한 추종기 (경로 끝 = 원점)."""
    f = FrenetFollower([(-1.0, 0.0), (0.0, 0.0)], 0.0, FrenetConfig())
    f.arrived = True
    return f


def _align_clear(f, pts, yaw0, w_steps):
    """도착 후 회전을 굴려서 (마지막 상태, 회전한 각도, 차체~점 최소 거리)."""
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
    assert st == 'done' and turned < -1.3          # 가까운 쪽(오른쪽)으로 약 90°


def test_arrival_turn_takes_far_side_when_near_side_hits():
    # 위를 보고(90°) 서서 0° 로 맞춰야 함. 가까운 쪽(오른쪽)으로 돌면 뒤 끝(반경 약 0.28 m)이 왼쪽 아래 대각선을 지나며
    # (-0.23, -0.23) 근처 물체에 4.5 cm 까지 다가감 → 반대쪽(왼쪽)으로 270° 돌아서 맞춤 (끝 자세에서는 8 cm 넘게 떨어짐)
    f = _arrived_at_origin()
    chunk = [(-0.23 + dx, -0.23 + dy) for dx in (-0.01, 0.0, 0.01) for dy in (-0.01, 0.0, 0.01)]
    st, turned, clear = _align_clear(f, chunk, math.radians(90), 200)
    assert st == 'done' and turned > 3.5           # 왼쪽으로 약 270°
    assert clear >= f.cfg.safety_margin - 1e-3


def test_arrival_turn_skipped_in_tight_corner():
    # 옛 골방처럼 뒤/옆 벽이 회전 반경(약 0.28 m)보다 가까움 → 벽을 긁으며 돌지 않고 지금 방향에서 끝냄
    f = _arrived_at_origin()
    walls = [(-0.30, y) for y in np.arange(-0.5, 0.5, 0.02)] + [(x, 0.27) for x in np.arange(-0.3, 0.5, 0.02)]
    st, turned, clear = _align_clear(f, walls, math.radians(180), 50)
    assert st == 'done' and abs(turned) < 1e-6
    assert f.last_debug.get('align') == 'skipped'


def test_very_short_paths_build_reference():
    # 15 cm 안 되는 짧은 경로 (목표가 바로 옆): 예전엔 곡률 평균이 길이가 어긋나 예외 → 브리지가 죽었음 (실물 step-2738)
    for pts in ([(0, 0), (0.05, 0.0), (0.1, 0.02)], [(0, 0), (0.08, 0.0), (0.08, 0.06)], [(0, 0), (0.05, 0.0)],
                [(0, 0), (0.02, 0.01)]):
        ref = build_reference(np.array(pts, dtype=float))
        assert np.all(np.isfinite(ref.at(0.0)[0]))
    f, log = drive([(0, 0), (0.06, 0.0), (0.1, 0.03)], [])
    assert log['state'][-1] in ('done', 'rotating', 'driving')


def _corridor(half_gap):
    """x 0.3~1.5 m 구간만 좁은 통로 (벽 y = ±half_gap), 나머지는 넓음."""
    return [(x, s * half_gap) for x in np.arange(0.3, 1.5, 0.02) for s in (-1, 1)]


def test_narrow_gap_passed_slowly_with_speed_dependent_margin():
    # 차체 반폭 0.153 + 0.045 → 가운데로 가도 여유 약 4.5 cm: 빠른 후보(6 cm)는 탈락, 느린 후보(4 cm)만 통과 → 천천히 지나감
    walls = _corridor(0.153 + 0.045)
    f, log = drive(STRAIGHT, walls, steps=900)
    assert log['state'][-1] == 'done'
    assert log['clear'] > 0.03
    xs = np.array(log['xy'])[:, 0]
    v_gap = np.array(log['v'])[(xs > 0.5) & (xs < 1.2)]
    assert v_gap.max() < 0.08                       # 좁은 곳에서는 느리게
    # 여유를 속도와 상관없이 6 cm 로 두면 같은 통로에서 막힌다 (예전 동작)
    cfg = FrenetConfig(safety_margin_slow=0.06)
    f2, log2 = drive(STRAIGHT, walls, steps=200, cfg=cfg)
    assert 'blocked' in log2['state'] and log2['state'][-1] != 'done'


def test_wide_corridor_keeps_full_speed():
    f, log = drive(STRAIGHT, _corridor(0.45), steps=400)
    assert log['state'][-1] == 'done'
    assert max(log['v']) > 0.13
