import numpy as np

from warehouse_robot.astar import GridPlanner, PlanningError
from warehouse_robot.plan_map import (INSIDE_MARGIN, fill_stripes, inflate_clusters, prepare, with_added, zone_contains,
                                      zone_mask)

RES = 0.05


def planner(grid):
    h, w = grid.shape
    return GridPlanner(grid.ravel(), w, h, RES, 0.0, 0.0, robot_radius=0.25)


def cell(m):
    return int(round(m / RES))


def hall():
    """6 m x 4 m room. Rows of chair legs at bottom/top (1-cell leg points, 0.6 m apart), 0.8 m aisle between the rows."""
    g = np.zeros((cell(4.0), cell(6.0)), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    for y in (1.0, 1.6, 2.4, 3.0):          # leg row y (between 1.6 and 2.4 = 0.8 m aisle)
        for x in np.arange(1.0, 5.01, 0.6):
            g[cell(y), cell(x)] = 100
    return g


def test_leg_rows_become_blocks_and_aisle_stays():
    g = hall()
    p = planner(g)
    # before processing: can cross the seat rows vertically between legs (0.6 m)
    p.plan((1.3, 0.4), (1.3, 3.6))
    _, added, _ = prepare(g, RES)
    assert added.any()
    p2 = planner(with_added(g, added, RES, 0.0, 0.0))
    # after processing: cannot pass between legs (0.6 m apart), goes around the row ends (x < 1.0 or > 5.0)
    path = p2.plan((1.3, 0.4), (1.3, 3.6))
    crossing = [x for x, y in path if 1.0 <= y <= 1.6 or 2.4 <= y <= 3.0]
    assert all(x < 1.0 or x > 5.0 for x in crossing)
    # the 0.8 m aisle is still passable
    p2.plan((0.5, 2.0), (5.5, 2.0))
    path = p2.plan((1.0, 2.0), (5.0, 2.0))
    assert all(1.7 < y < 2.3 for _, y in path)


def test_walls_are_not_inflated():
    """Large clusters (walls) are not inflated, so a 0.6 m door between walls stays passable."""
    g = np.zeros((cell(3.0), cell(3.0)), dtype=np.int16)
    g[:, cell(1.5)] = 100
    g[cell(1.2):cell(1.8), cell(1.5)] = 0   # 0.6 m door
    added = inflate_clusters(g, RES)
    assert not added.any()
    planner(with_added(g, added, RES, 0.0, 0.0)).plan((0.5, 1.5), (2.5, 1.5))


def test_float32_resolution_gives_same_result():
    """OccupancyGrid resolution is float32 (0.05 -> 0.0500000007). Results must match the yaml 0.05."""
    g = hall()
    a = prepare(g, 0.05)[1]
    b = prepare(g, float(np.float32(0.05)))[1]
    assert a.sum() > 0 and (a == b).all()


def test_fill_stripes_only_between_free_cells():
    g = np.full((40, 40), -1, dtype=np.int16)
    g[10:30, 10:30] = 0
    g[10:30, 19:21] = -1        # 2-cell-wide unknown stripe between free cells
    g[10:30, 25] = 100          # obstacles unchanged
    out, n = fill_stripes(g)
    assert n == 40
    assert (out[10:30, 19:21] == 0).all()
    assert (out[10:30, 25] == 100).all()
    assert (out[:10] == -1).all() and (out[:, :10] == -1).all()   # does not grow outside the free area


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
    # a no-go zone containing the start is left out of that plan (node/server pick it with zone_contains)
    z = {'shape': 'rect', 'x1': 1.0, 'y1': 0.0, 'x2': 2.0, 'y2': 3.0}
    assert zone_contains(z, (1.5, 1.5)) and zone_contains(z, (0.9, 1.5), 0.15) and not zone_contains(z, (0.5, 1.5))
    assert zone_contains({'shape': 'line', 'x1': 0, 'y1': 0, 'x2': 1, 'y2': 0}, (0.5, 0.2), 0.2)
    # starting just inside a zone edge: the area around the start is cleared so the robot can get out
    start = (1.1, 1.5)
    inside = with_added(g, zone, RES, 0.0, 0.0, start=start)
    path = planner(inside).plan(start, (0.4, 1.5))
    assert path[-1] == (0.4, 1.5)


def test_robot_beside_zone_does_not_cross_it():
    """Replanning right next to a zone (0.2 m outside) does not drop the zone (only the start area is cleared)
    -> the path does not cut across the zone."""
    g = np.zeros((cell(3.0), cell(4.0)), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    z = {'shape': 'rect', 'x1': 1.5, 'y1': 0.0, 'x2': 2.5, 'y2': 2.2}   # only the top 0.8 m is open
    start = (1.3, 1.0)                                                  # 0.2 m left of the zone
    assert not zone_contains(z, start, INSIDE_MARGIN)
    zone = zone_mask(g.shape, RES, 0.0, 0.0, [z])
    path = planner(with_added(g, zone, RES, 0.0, 0.0, start=start)).plan(start, (3.0, 1.0))
    inside = [(x, y) for x, y in path if 1.6 < x < 2.4 and y < 2.1]
    assert not inside and max(y for _, y in path) > 2.2
