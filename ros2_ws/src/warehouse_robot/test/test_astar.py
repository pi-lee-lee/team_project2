import math

import numpy as np
import pytest

from warehouse_robot.astar import GridPlanner, PlanningError, path_yaws


def make_planner(grid, **kw):
    """grid: 2D 배열 (row 0 = 아래쪽). 해상도 0.05m."""
    grid = np.asarray(grid, dtype=np.int8)
    h, w = grid.shape
    return GridPlanner(grid.ravel(), w, h, 0.05, 0.0, 0.0, **kw)


def room_with_wall():
    """4m x 4m 방, 가운데 세로 벽(위쪽에 통로)."""
    g = np.zeros((80, 80), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:60, 40] = 100
    return g


def test_path_goes_around_wall():
    p = make_planner(room_with_wall())
    path = p.plan((0.5, 0.5), (3.5, 0.5))
    assert path[0] == (0.5, 0.5) and path[-1] == (3.5, 0.5)
    # 벽(y < 3.0) 위쪽 통로를 지나야 함
    assert max(y for _, y in path) > 3.0
    for x, y in path:
        assert p.is_free(x, y) or math.hypot(x - 0.5, y - 0.5) < 0.3


def test_no_path():
    g = room_with_wall()
    g[:, 40] = 100  # 통로 막기
    with pytest.raises(PlanningError):
        make_planner(g).plan((0.5, 0.5), (3.5, 0.5))


def test_goal_near_wall_is_moved():
    p = make_planner(room_with_wall())
    path = p.plan((0.5, 0.5), (3.5, 0.08))  # 벽에 붙은 목표
    assert p.is_free(*path[-1])


def test_start_inside_inflation_escapes():
    p = make_planner(room_with_wall())
    path = p.plan((0.1, 0.1), (1.0, 1.0))
    assert p.is_free(*path[-1])


def test_unknown_is_blocked_by_default():
    g = np.zeros((40, 40), dtype=np.int8)
    g[:, 20] = -1
    with pytest.raises(PlanningError):
        make_planner(g).plan((0.3, 1.0), (1.7, 1.0))
    assert make_planner(g, allow_unknown=True).plan((0.3, 1.0), (1.7, 1.0))


def test_path_yaws():
    yaws = path_yaws([(0, 0), (1, 0), (1, 1)], 3.0)
    assert yaws == pytest.approx([0.0, math.pi / 2, 3.0])
