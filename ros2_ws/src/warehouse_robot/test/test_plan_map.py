import numpy as np

from warehouse_robot.astar import GridPlanner, PlanningError
from warehouse_robot.plan_map import fill_stripes, inflate_clusters, prepare, with_added, zone_contains, zone_mask

RES = 0.05


def planner(grid):
    h, w = grid.shape
    return GridPlanner(grid.ravel(), w, h, RES, 0.0, 0.0, robot_radius=0.25)


def cell(m):
    return int(round(m / RES))


def hall():
    """6m x 4m 방. 아래/위에 의자 다리 줄(다리 점 1칸, 0.6m 간격), 두 줄 사이 0.8m 통로."""
    g = np.zeros((cell(4.0), cell(6.0)), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    for y in (1.0, 1.6, 2.4, 3.0):          # 다리 줄 y (1.6 과 2.4 사이 = 통로 0.8m)
        for x in np.arange(1.0, 5.01, 0.6):
            g[cell(y), cell(x)] = 100
    return g


def test_leg_rows_become_blocks_and_aisle_stays():
    g = hall()
    p = planner(g)
    # 가공 전: 다리 사이(0.6m) 로 좌석 줄을 세로로 가로지를 수 있음
    p.plan((1.3, 0.4), (1.3, 3.6))
    _, added, _ = prepare(g, RES)
    assert added.any()
    p2 = planner(with_added(g, added, RES, 0.0, 0.0))
    # 가공 후: 좌석 줄 (다리 0.6m 간격) 사이로 못 지나가고 줄 끝(x < 1.0 또는 > 5.0)으로 돌아감
    path = p2.plan((1.3, 0.4), (1.3, 3.6))
    crossing = [x for x, y in path if 1.0 <= y <= 1.6 or 2.4 <= y <= 3.0]
    assert all(x < 1.0 or x > 5.0 for x in crossing)
    # 0.8m 통로는 그대로 다닐 수 있음
    p2.plan((0.5, 2.0), (5.5, 2.0))
    path = p2.plan((1.0, 2.0), (5.0, 2.0))
    assert all(1.7 < y < 2.3 for _, y in path)


def test_walls_are_not_inflated():
    """큰 덩어리(벽)는 부풀리지 않으므로 벽 사이 0.6m 문은 그대로 지나갈 수 있다."""
    g = np.zeros((cell(3.0), cell(3.0)), dtype=np.int16)
    g[:, cell(1.5)] = 100
    g[cell(1.2):cell(1.8), cell(1.5)] = 0   # 0.6m 문
    added = inflate_clusters(g, RES)
    assert not added.any()
    planner(with_added(g, added, RES, 0.0, 0.0)).plan((0.5, 1.5), (2.5, 1.5))


def test_fill_stripes_only_between_free_cells():
    g = np.full((40, 40), -1, dtype=np.int16)
    g[10:30, 10:30] = 0
    g[10:30, 19:21] = -1        # 빈 칸 사이 폭 2칸 미탐색 줄무늬
    g[10:30, 25] = 100          # 장애물은 그대로
    out, n = fill_stripes(g)
    assert n == 40
    assert (out[10:30, 19:21] == 0).all()
    assert (out[10:30, 25] == 100).all()
    assert (out[:10] == -1).all() and (out[:, :10] == -1).all()   # 빈 영역 바깥으로 안 넓어짐


def test_zone_rect_and_line():
    m = zone_mask((40, 40), RES, 0.0, 0.0, [
        {'shape': 'rect', 'x1': 0.5, 'y1': 0.5, 'x2': 0.25, 'y2': 0.25},
        {'shape': 'line', 'x1': 1.0, 'y1': 1.0, 'x2': 1.5, 'y2': 1.0},
    ])
    assert m[cell(0.375), cell(0.375)] and not m[cell(0.6), cell(0.6)]
    assert m[cell(1.0), cell(1.25)] and not m[cell(1.3), cell(1.25)]


def test_zone_blocks_but_robot_inside_can_leave():
    g = np.zeros((cell(3.0), cell(3.0)), dtype=np.int16)
    zone = zone_mask(g.shape, RES, 0.0, 0.0, [{'shape': 'rect', 'x1': 1.0, 'y1': 0.0, 'x2': 2.0, 'y2': 3.0}])
    blocked = with_added(g, zone, RES, 0.0, 0.0, start=(0.5, 1.5))
    try:
        planner(blocked).plan((0.5, 1.5), (2.5, 1.5))
        assert False, '금지 구역을 지나가면 안 됨'
    except PlanningError:
        pass
    # 출발 위치가 들어 있는 금지 구역은 그 계획에서 뺀다 (노드/서버가 zone_contains 로 고름)
    z = {'shape': 'rect', 'x1': 1.0, 'y1': 0.0, 'x2': 2.0, 'y2': 3.0}
    assert zone_contains(z, (1.5, 1.5)) and zone_contains(z, (0.9, 1.5), 0.15) and not zone_contains(z, (0.5, 1.5))
    assert zone_contains({'shape': 'line', 'x1': 0, 'y1': 0, 'x2': 1, 'y2': 0}, (0.5, 0.2), 0.2)
    # 금지 구역 가장자리 안에서 출발하면 출발 주변은 비워서 밖으로 나갈 수 있음
    start = (1.1, 1.5)
    inside = with_added(g, zone, RES, 0.0, 0.0, start=start)
    path = planner(inside).plan(start, (0.4, 1.5))
    assert path[-1] == (0.4, 1.5)
