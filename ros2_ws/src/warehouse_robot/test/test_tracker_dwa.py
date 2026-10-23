"""추적(tracker.py) + DWA(dwa.py) 테스트. ~/tb3_nav_ws/src/tb3_nav/test/test_core.py 에서 가져옴."""
import numpy as np

from warehouse_robot.dwa import DWAController, RobotState
from warehouse_robot.tracker import MultiObjectTracker, detect_clusters


def test_tracker_estimates_velocity():
    tr = MultiObjectTracker(min_hits=3)
    tracks = []
    for k in range(20):
        t = k * 0.1
        cx = 1.0 + 0.3 * t
        arc = np.array([[cx - 0.1 + 0.02 * i, 2.0 - 0.05 * np.sin(i / 10 * np.pi)]
                        for i in range(11)])
        tracks = tr.update(detect_clusters(arc, 0.15, 3, 0.8, 0.08), t)
    assert len(tracks) == 1
    assert abs(tracks[0].x[2] - 0.3) < 0.05   # 상태 [x, y, vx, vy]
    assert abs(tracks[0].x[3]) < 0.05


def test_cluster_center_corrected_away_from_sensor():
    # 반지름 0.12 원기둥을 원점의 센서가 볼 때 보이는 앞면 호 (중심 (1.0, 0))
    ang = np.linspace(np.pi - 0.9, np.pi + 0.9, 15)
    arc = np.column_stack([1.0 + 0.12 * np.cos(ang), 0.12 * np.sin(ang)])
    raw = detect_clusters(arc, 0.15, 3, 0.8, 0.08)[0]
    fixed = detect_clusters(arc, 0.15, 3, 0.8, 0.08, sensor_xy=(0.0, 0.0))[0]
    assert abs(fixed[0] - 1.0) < abs(raw[0] - 1.0)
    assert abs(fixed[0] - 1.0) < 0.04


def test_dwa_backs_away_from_approaching_obstacle():
    """멈춰 있는데 옆에서 장애물이 다가오면 (후진/회전 포함) 안전한 선택을 해야 한다."""
    dwa = DWAController()
    incoming = [[-0.6, 0.35, 0.25, 0.0, 0.12]]   # 로봇 앞쪽 0.35 m 선을 따라 다가옴
    r = dwa.compute(RobotState(0, 0, np.pi / 2, 0.0, 0.0), [[0, 1.0]], [0, 1.0], [], incoming)
    assert r.ok and r.v <= 0.0


def test_dwa_drives_to_goal_when_free():
    dwa = DWAController()
    r = dwa.compute(RobotState(0, 0, 0, 0.2, 0), [[0.5, 0], [1.0, 0]], [1.0, 0], [], [])
    assert r.ok and r.v > 0.1 and abs(r.w) < 0.2


def simulate(dwa, goal, static_pts, dyn, steps=300, dt=0.1, path=None):
    """DWA 를 폐루프로 돌려 (최소 여유거리, 마지막 위치) 반환. dyn 은 시간에 따라 이동."""
    if path is None:
        path = np.column_stack([np.linspace(0, goal[0], 41), np.linspace(0, goal[1], 41)])
    s = RobotState(0.0, 0.0, 0.0, 0.0, 0.0)
    dyn = np.array(dyn, dtype=float).reshape(-1, 5)
    min_clear = np.inf
    for _ in range(steps):
        idx = int(np.argmin(np.hypot(path[:, 0] - s.x, path[:, 1] - s.y)))
        look = path[min(idx + 16, len(path) - 1)]
        r = dwa.compute(s, path[idx:], look, static_pts, dyn)
        s = RobotState(s.x + r.v * np.cos(s.yaw) * dt, s.y + r.v * np.sin(s.yaw) * dt,
                       s.yaw + r.w * dt, r.v, r.w)
        dyn[:, :2] += dyn[:, 2:4] * dt
        pts = np.asarray(static_pts).reshape(-1, 2)
        if len(pts):
            min_clear = min(min_clear, np.hypot(pts[:, 0] - s.x, pts[:, 1] - s.y).min() - 0.22)
        if len(dyn):
            min_clear = min(min_clear,
                            (np.hypot(dyn[:, 0] - s.x, dyn[:, 1] - s.y) - dyn[:, 4]).min() - 0.22)
        if np.hypot(goal[0] - s.x, goal[1] - s.y) < 0.12:
            break
    return min_clear, (s.x, s.y)


def test_dwa_follows_detour_around_obstacle():
    """A* 가 돌아가는 경로를 줬을 때 DWA 가 상자에 닿지 않고 따라가는지."""
    box = [[x, y] for x in np.linspace(0.9, 1.1, 5) for y in np.linspace(-0.1, 0.1, 5)]
    corners = np.array([[0, 0], [0.6, 0.45], [1.4, 0.45], [2.0, 0]])
    path = np.vstack([np.linspace(a, b, 15, endpoint=False) for a, b in zip(corners, corners[1:])]
                     + [corners[-1:]])
    min_clear, end = simulate(DWAController(), (2.0, 0.0), box, [], path=path)
    assert min_clear > 0.0
    assert np.hypot(end[0] - 2.0, end[1]) < 0.12


def test_dwa_avoids_crossing_obstacle():
    crossing = [[1.0, -1.2, 0.0, 0.25, 0.12]]    # 경로를 아래->위로 가로지름
    min_clear, end = simulate(DWAController(), (2.0, 0.0), [], crossing)
    assert min_clear > 0.0
    assert np.hypot(end[0] - 2.0, end[1]) < 0.12


def test_dwa_considers_future_obstacle_position():
    """지금은 경로 밖이지만 1~2초 뒤 경로를 가로지르는 장애물."""
    dwa = DWAController()
    path = [[x, 0.0] for x in np.linspace(0, 1.0, 11)]
    crossing = [[0.35, -0.5, 0.0, 0.3, 0.12]]    # 아래에서 위로 0.3 m/s
    r_dyn = dwa.compute(RobotState(0, 0, 0, 0.2, 0), path, [1.0, 0], [], crossing)
    r_free = dwa.compute(RobotState(0, 0, 0, 0.2, 0), path, [1.0, 0], [], [])
    assert r_dyn.ok
    assert (r_dyn.v, r_dyn.w) != (r_free.v, r_free.w)


def run_follower(follower, steps=400, dt=0.1, map_to_odom=(0.0, 0.0, 0.0), static=(), dyn=None):
    x = y = yaw = v = w = 0.0
    dyn = np.array(dyn if dyn is not None else [], dtype=float).reshape(-1, 5)
    state = None
    for _ in range(steps):
        v, w, state = follower.compute((x, y, yaw), v, w, map_to_odom, static, dyn)
        if state == 'done':
            break
        yaw += w * dt
        x += v * np.cos(yaw) * dt
        y += v * np.sin(yaw) * dt
        dyn[:, :2] += dyn[:, 2:4] * dt
    return state, (x, y, yaw)


def test_follower_reaches_goal_and_turns_to_yaw():
    from warehouse_robot.dwa import DWAFollower
    path = [[x, 0.0] for x in np.linspace(0, 1.5, 31)]
    state, (x, y, yaw) = run_follower(DWAFollower(path, np.pi / 2))
    assert state == 'done'
    assert np.hypot(x - 1.5, y) < 0.12
    assert abs(np.arctan2(np.sin(yaw - np.pi / 2), np.cos(yaw - np.pi / 2))) < 0.12


def test_follower_uses_map_to_odom():
    """map 경로를 odom 으로 옮겨서 따라감 (map 원점이 odom 에서 (0.5, 0) 에 있으면 끝점은 odom (2.0, 0))."""
    from warehouse_robot.dwa import DWAFollower
    path = [[x, 0.0] for x in np.linspace(-0.5, 1.5, 41)]
    state, (x, y, _) = run_follower(DWAFollower(path, 0.0), map_to_odom=(0.5, 0.0, 0.0))
    assert state == 'done' and np.hypot(x - 2.0, y) < 0.12


def test_follower_blocked_by_wall():
    from warehouse_robot.dwa import DWAFollower
    path = [[x, 0.0] for x in np.linspace(0, 2.0, 41)]
    wall = [[0.3, yy] for yy in np.linspace(-1.0, 1.0, 41)]
    f = DWAFollower(path, 0.0)
    v, w, state = f.compute((0.0, 0.0, 0.0), 0.0, 0.0, (0, 0, 0), wall, [])
    # 바로 앞 벽: 앞으로는 못 감 (멈추거나 돌거나 막힘)
    assert v <= 0.05


def test_target_moves_beyond_obstacle_on_path():
    # 앞쪽 경로가 지도에 없는 상자에 걸리면 목표점을 상자 너머의 경로점으로 (w_path 0 인 hybrid DWA 가 상자 앞에 서지 않게)
    import numpy as np
    from warehouse_robot.dwa import DWAConfig, DWAFollower
    path = np.array([[x, 0.0] for x in np.linspace(0, 3.0, 61)])
    f = DWAFollower(path, 0.0, DWAConfig(), lookahead=0.3)
    f.target_clear = 0.233
    box = np.array([[x, y] for x in np.linspace(0.9, 1.1, 5) for y in np.linspace(-0.1, 0.1, 5)])
    lp, goal = f.local_target(path, 0.0, 0.0)
    assert abs(goal[0] - 0.3) < 0.06
    lp2, goal2 = f.clear_target(path, 0.0, 0.0, box, lp, goal)
    assert goal2[0] > 1.1 + 0.2 and len(lp2) >= len(lp)
    # 장애물이 없거나 끔(0)이면 그대로
    assert f.clear_target(path, 0.0, 0.0, box[:0], lp, goal)[1] is goal
    f.target_clear = 0.0
    assert f.clear_target(path, 0.0, 0.0, box, lp, goal)[1] is goal
