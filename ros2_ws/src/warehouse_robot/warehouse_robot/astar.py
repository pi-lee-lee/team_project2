"""격자 맵 위의 A* 경로 계획 (ROS 의존성 없음, numpy만 사용).

1. 장애물까지의 거리를 계산해 로봇 반경 안쪽은 통과 불가(lethal)로 만든다.
2. 반경 바깥 inflation_radius까지는 벽에 가까울수록 비용을 더해 통로 가운데로 가게 한다.
3. 8방향 이동, octile 휴리스틱으로 A* 탐색.
"""
import heapq
import math

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except Exception:  # 라즈베리 파이 등 scipy가 없거나 numpy와 버전이 안 맞는 환경
    distance_transform_edt = None

SQRT2 = math.sqrt(2.0)
# (drow, dcol, 이동 거리)
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
ESCAPE_PENALTY = 20.0  # 로봇이 벽에 붙어 있을 때 lethal 영역을 빠져나가는 비용


class PlanningError(Exception):
    pass


def obstacle_distance(obstacle, resolution, max_dist):
    """각 셀에서 가장 가까운 장애물까지의 거리(m)."""
    if not obstacle.any():
        return np.full(obstacle.shape, np.inf)
    if distance_transform_edt is not None:
        return distance_transform_edt(~obstacle) * resolution
    # scipy가 없으면 팽창을 반복해서 근사 (max_dist까지만)
    dist = np.full(obstacle.shape, np.inf)
    dist[obstacle] = 0.0
    reached = obstacle.copy()
    for k in range(1, int(math.ceil(max_dist / resolution)) + 1):
        grown = reached.copy()
        grown[1:, :] |= reached[:-1, :]
        grown[:-1, :] |= reached[1:, :]
        grown[:, 1:] |= reached[:, :-1]
        grown[:, :-1] |= reached[:, 1:]
        new = grown & ~reached
        dist[new] = k * resolution
        reached = grown
    return dist


def pad_grid(data, width, height, resolution, origin_x, origin_y, points, margin=1.0):
    """points(월드 좌표)가 모두 들어가도록 격자를 미탐색(-1) 칸으로 넓힌다 (SLAM 중 아직 맵 밖인 목표용).

    반환: (data 1차원 배열, width, height, origin_x, origin_y)
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    cells = int(math.ceil(margin / resolution))
    left = max(0, int(math.ceil((origin_x - min(xs)) / resolution)) + cells) if min(xs) < origin_x else 0
    bottom = max(0, int(math.ceil((origin_y - min(ys)) / resolution)) + cells) if min(ys) < origin_y else 0
    right_edge = origin_x + width * resolution
    top_edge = origin_y + height * resolution
    right = int(math.ceil((max(xs) - right_edge) / resolution)) + cells if max(xs) >= right_edge else 0
    top = int(math.ceil((max(ys) - top_edge) / resolution)) + cells if max(ys) >= top_edge else 0
    if not (left or bottom or right or top):
        return grid.ravel(), width, height, origin_x, origin_y
    out = np.full((height + bottom + top, width + left + right), -1, dtype=np.int16)
    out[bottom:bottom + height, left:left + width] = grid
    return (out.ravel(), out.shape[1], out.shape[0],
            origin_x - left * resolution, origin_y - bottom * resolution)


class GridPlanner:
    """OccupancyGrid 데이터로 만든 A* 플래너.

    data: 길이 width*height, row 0 = 맵 아래쪽 (OccupancyGrid와 같은 순서), 값 -1 또는 0~100
    """

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 robot_radius=0.22, inflation_radius=0.55, cost_weight=3.0,
                 occupied_threshold=65, allow_unknown=False, unknown_cost=2.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.obstacle = grid >= occupied_threshold
        if not allow_unknown:
            self.obstacle |= grid < 0

        self.dist = obstacle_distance(self.obstacle, resolution, inflation_radius)
        self.lethal = self.obstacle | (self.dist < robot_radius)

        # 셀에 들어갈 때 이동 거리에 곱하는 비용 배율 (1 이상)
        span = max(inflation_radius - robot_radius, 1e-6)
        closeness = np.clip((inflation_radius - self.dist) / span, 0.0, 1.0)
        self.cost = 1.0 + cost_weight * closeness ** 2
        if allow_unknown:
            # 미탐색 칸은 지나갈 수 있지만, 아는 빈 칸을 우선 지나가도록 비용을 더한다 (SLAM 중 탐색용)
            self.cost = self.cost + unknown_cost * (grid < 0)

        # 탐색 루프 속도를 위해 1차원 리스트로 변환
        self._obstacle = self.obstacle.ravel().tolist()
        self._lethal = self.lethal.ravel().tolist()
        self._cost = self.cost.ravel().tolist()

    # -------------------------------------------------------------- 좌표 변환
    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row, col):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def inside(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, x, y):
        row, col = self.world_to_cell(x, y)
        return self.inside(row, col) and not self.lethal[row, col]

    def nearest_free(self, row, col, max_cells):
        """(row, col) 주변 max_cells 안에서 가장 가까운 통과 가능 셀."""
        r0, r1 = max(0, row - max_cells), min(self.height, row + max_cells + 1)
        c0, c1 = max(0, col - max_cells), min(self.width, col + max_cells + 1)
        free = ~self.lethal[r0:r1, c0:c1]
        if not free.any():
            return None
        rr, cc = np.nonzero(free)
        d2 = (rr + r0 - row) ** 2 + (cc + c0 - col) ** 2
        i = int(np.argmin(d2))
        if d2[i] > max_cells ** 2:
            return None
        return int(rr[i] + r0), int(cc[i] + c0)

    # -------------------------------------------------------------- 탐색
    def plan(self, start, goal, goal_tolerance=0.3, smooth=True):
        """start, goal: (x, y) [m]. 반환: [(x, y), ...] (start -> goal)."""
        sr, sc = self.world_to_cell(*start)
        gr, gc = self.world_to_cell(*goal)
        if not self.inside(sr, sc):
            raise PlanningError('현재 위치가 맵 밖에 있습니다.')
        if not self.inside(gr, gc):
            raise PlanningError('목표 위치가 맵 밖에 있습니다.')

        exact_goal = True
        if self.lethal[gr, gc]:
            cell = self.nearest_free(gr, gc, int(goal_tolerance / self.resolution))
            if cell is None:
                raise PlanningError('목표 위치가 장애물에 너무 가깝습니다.')
            gr, gc = cell
            exact_goal = False

        cells = self._astar(sr, sc, gr, gc)
        if cells is None:
            raise PlanningError('목표까지 갈 수 있는 경로가 없습니다.')

        points = [self.cell_to_world(r, c) for r, c in cells]
        points[0] = tuple(start)
        if exact_goal:
            points[-1] = tuple(goal)
        if smooth:
            points = smooth_path(points)
        return points

    def _astar(self, sr, sc, gr, gc):
        w = self.width
        n = w * self.height
        start, goal = sr * w + sc, gr * w + gc
        obstacle, lethal, cost = self._obstacle, self._lethal, self._cost

        g = [math.inf] * n
        parent = [-1] * n
        closed = bytearray(n)
        g[start] = 0.0

        def h(idx):
            dr = abs(idx // w - gr)
            dc = abs(idx % w - gc)
            return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)

        heap = [(h(start), 0.0, start)]
        while heap:
            _, gc_cur, cur = heapq.heappop(heap)
            if closed[cur]:
                continue
            if cur == goal:
                break
            closed[cur] = 1
            r, c = divmod(cur, w)
            cur_lethal = lethal[cur]
            for dr, dc, step in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if nr < 0 or nc < 0 or nr >= self.height or nc >= w:
                    continue
                nxt = nr * w + nc
                if closed[nxt] or obstacle[nxt]:
                    continue
                extra = 0.0
                if lethal[nxt]:
                    # lethal 영역은 시작점이 이미 그 안에 있을 때만 빠져나가는 용도로 허용
                    if not cur_lethal:
                        continue
                    extra = ESCAPE_PENALTY
                if dr and dc and (obstacle[r * w + nc] or obstacle[nr * w + c]):
                    continue  # 대각선으로 장애물 모서리를 가로지르지 않기
                ng = gc_cur + step * cost[nxt] + extra
                if ng < g[nxt]:
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(heap, (ng + h(nxt), ng, nxt))
        else:
            return None

        if g[goal] == math.inf:
            return None
        path = []
        idx = goal
        while idx != -1:
            path.append(divmod(idx, w))
            idx = parent[idx]
        path.reverse()
        return path


def smooth_path(points, window=2):
    """양 끝점은 고정하고 이동 평균으로 계단 모양을 완화."""
    if len(points) <= 2 * window + 1:
        return points
    arr = np.asarray(points)
    out = arr.copy()
    for i in range(window, len(arr) - window):
        out[i] = arr[i - window:i + window + 1].mean(axis=0)
    return [tuple(p) for p in out]


def path_yaws(points, goal_yaw):
    """경로 각 점의 진행 방향. 마지막 점은 goal_yaw."""
    yaws = []
    for i in range(len(points) - 1):
        (x0, y0), (x1, y1) = points[i], points[i + 1]
        yaws.append(math.atan2(y1 - y0, x1 - x0))
    yaws.append(goal_yaw)
    return yaws
