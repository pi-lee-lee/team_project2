"""경로 계획용 지도 가공 (ROS 의존성 없음, numpy만 사용).

위치 추정(AMCL)은 SLAM 지도를 그대로 쓰고, A* 는 이 모듈로 가공한 격자를 쓴다 (원본 지도 파일은 그대로).
강당·교실처럼 책상/의자가 흩어진 곳은 LiDAR(높이 약 13cm)에 다리 단면만 점으로 찍혀서,
  - 점 사이 빈 칸으로 A* 가 좌석 사이를 지나가는 경로를 만들고
  - 먼 곳은 빔 간격(1.5°, 3.4m 에서 약 9cm)이 칸(5cm)보다 넓어 빈 칸 사이에 미탐색 줄무늬가 남아 길이 막힌다.

1. fill_stripes: 빈 칸 사이의 좁은 미탐색 줄무늬(폭 2*cells 칸 미만)를 빈 칸으로. 장애물 칸은 건드리지 않음
2. inflate_clusters: 작은 장애물 덩어리(다리 점, 가로·세로 모두 max_blob_size 이하)만 inflate 만큼 부풀린다.
   벽(긴 덩어리)은 그대로.
   A* 는 로봇 반경(0.25m) 안을 못 지나가므로, 다리 점 사이 틈은 2*(0.25 + inflate) 미만이면 막힌다
   (기본 0.1 → 0.7m 미만). 다리가 격자로 늘어선 좌석 줄은 한 덩어리처럼 막히고, 0.7m 이상 통로는 남는다.
   닫힘 연산(점끼리 잇기)은 반경을 0.35m 로 해도 0.5m 격자까지만 이어서 A* 반경과 거의 차이가 없었다.
   다리 간격이 이보다 넓은 책상 밑 등은 웹의 금지 구역으로 막는다
3. zone_mask: 웹에서 그린 금지 구역(사각형/선)을 칸으로
4. with_added: 더한 칸을 격자에 장애물로 넣되, 출발 위치 주변은 빼서 그 안에 서 있는 로봇이 빠져나올 수 있게
   (출발 위치가 들어 있는 금지 구역은 zone_contains 로 골라 그 계획에서 통째로 뺀다)

격자 값은 OccupancyGrid 와 같음: -1 미탐색, 0~100 (occupied_threshold 이상 = 장애물), 행 0 = 아래쪽.
"""
import math
from collections import deque

import numpy as np

from .astar import obstacle_distance

try:
    from scipy.ndimage import label as _scipy_label
except Exception:  # 라즈베리 파이 등 scipy가 없거나 numpy와 버전이 안 맞는 환경
    _scipy_label = None

LINE_WIDTH = 0.1  # m, 선(가상 벽) 금지 구역의 두께


def _dilate(mask, k):
    """8방향 k칸 팽창."""
    out = mask.copy()
    for _ in range(k):
        g = out.copy()
        g[1:, :] |= out[:-1, :]
        g[:-1, :] |= out[1:, :]
        g[:, 1:] |= out[:, :-1]
        g[:, :-1] |= out[:, 1:]
        g[1:, 1:] |= out[:-1, :-1]
        g[1:, :-1] |= out[:-1, 1:]
        g[:-1, 1:] |= out[1:, :-1]
        g[:-1, :-1] |= out[1:, 1:]
        out = g
    return out


def _components(mask):
    """8방향 연결 성분. (라벨 배열, 성분별 칸 수 배열) - 라벨 0 = 마스크 밖."""
    if _scipy_label is not None:
        labels, n = _scipy_label(mask, structure=np.ones((3, 3), dtype=int))
        return labels, np.bincount(labels.ravel(), minlength=n + 1)
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    sizes = [0]
    for r0, c0 in zip(*np.nonzero(mask)):
        if labels[r0, c0]:
            continue
        label = len(sizes)
        labels[r0, c0] = label
        q, n = deque([(r0, c0)]), 0
        while q:
            r, c = q.popleft()
            n += 1
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not labels[nr, nc]:
                        labels[nr, nc] = label
                        q.append((nr, nc))
        sizes.append(n)
    return labels, np.array(sizes)


def fill_stripes(grid, cells=2, occupied_threshold=65):
    """빈 칸 사이의 좁은 미탐색 줄무늬를 빈 칸(0)으로 바꾼 새 격자와 바꾼 칸 수.
    빈 칸 영역의 닫힘 연산(cells 칸)에 들어가는 미탐색 칸만 바꾸므로, 빈 영역 바깥으로 넓어지지 않는다."""
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < occupied_threshold)
    if cells <= 0 or not free.any():
        return grid.copy(), 0
    closed = ~_dilate(~_dilate(free, cells), cells)
    fill = closed & (grid < 0)
    out = grid.copy()
    out[fill] = 0
    return out, int(fill.sum())


def inflate_clusters(grid, resolution, inflate=0.1, max_blob_size=0.4, occupied_threshold=65):
    """작은 장애물 덩어리를 inflate(m) 만큼 부풀려 새로 막을 칸 (bool 마스크, 원래 장애물 칸은 제외).
    가로나 세로가 max_blob_size(m)보다 긴 덩어리(벽, 책상 앞판 등)는 부풀리지 않는다.
    (넓이로 가르면 얇은 벽 조각도 작은 덩어리가 되어 문이 좁아졌음)"""
    grid = np.asarray(grid)
    occ = grid >= occupied_threshold
    if inflate <= 0 or not occ.any():
        return np.zeros(grid.shape, dtype=bool)
    labels, sizes = _components(occ)
    n = len(sizes)
    rows, cols = np.nonzero(labels)
    lab = labels[rows, cols]
    r_min = np.full(n, np.iinfo(np.int64).max)
    r_max = np.full(n, -1)
    c_min, c_max = r_min.copy(), r_max.copy()
    np.minimum.at(r_min, lab, rows)
    np.maximum.at(r_max, lab, rows)
    np.minimum.at(c_min, lab, cols)
    np.maximum.at(c_max, lab, cols)
    extent = (np.maximum(r_max - r_min, c_max - c_min) + 1) * resolution
    small = extent <= max_blob_size + 1e-9
    small[0] = False
    blobs = small[labels]
    if not blobs.any():
        return np.zeros(grid.shape, dtype=bool)
    return (obstacle_distance(blobs, resolution, inflate + resolution) <= inflate + 1e-9) & ~occ


def zone_mask(shape, resolution, origin_x, origin_y, zones):
    """금지 구역 목록을 칸 마스크로. zone: {'shape': 'rect'|'line', 'x1', 'y1', 'x2', 'y2'} (m)."""
    h, w = shape
    mask = np.zeros(shape, dtype=bool)
    if not zones:
        return mask
    xs = origin_x + (np.arange(w) + 0.5) * resolution
    ys = origin_y + (np.arange(h) + 0.5) * resolution
    for z in zones:
        x1, y1, x2, y2 = (float(z[k]) for k in ('x1', 'y1', 'x2', 'y2'))
        if z.get('shape') == 'line':
            # 선분에서 LINE_WIDTH/2 (최소 한 칸) 안의 칸
            half = max(LINE_WIDTH / 2, resolution * 0.75)
            c0 = max(0, int((min(x1, x2) - half - origin_x) / resolution))
            c1 = min(w, int(math.ceil((max(x1, x2) + half - origin_x) / resolution)) + 1)
            r0 = max(0, int((min(y1, y2) - half - origin_y) / resolution))
            r1 = min(h, int(math.ceil((max(y1, y2) + half - origin_y) / resolution)) + 1)
            if c0 >= c1 or r0 >= r1:
                continue
            px, py = np.meshgrid(xs[c0:c1], ys[r0:r1])
            dx, dy = x2 - x1, y2 - y1
            t = np.clip(((px - x1) * dx + (py - y1) * dy) / max(dx * dx + dy * dy, 1e-12), 0.0, 1.0)
            d = np.hypot(px - (x1 + t * dx), py - (y1 + t * dy))
            mask[r0:r1, c0:c1] |= d <= half
        else:
            cols = (xs >= min(x1, x2)) & (xs <= max(x1, x2))
            rows = (ys >= min(y1, y2)) & (ys <= max(y1, y2))
            mask[np.ix_(rows, cols)] = True
    return mask


def zone_contains(z, point, margin=0.0):
    """금지 구역 z 가 point(x, y) 를 margin 안에 포함하는지."""
    x1, y1, x2, y2 = (float(z[k]) for k in ('x1', 'y1', 'x2', 'y2'))
    px, py = point
    if z.get('shape') == 'line':
        dx, dy = x2 - x1, y2 - y1
        t = min(1.0, max(0.0, ((px - x1) * dx + (py - y1) * dy) / max(dx * dx + dy * dy, 1e-12)))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) <= LINE_WIDTH / 2 + margin
    return (min(x1, x2) - margin <= px <= max(x1, x2) + margin
            and min(y1, y2) - margin <= py <= max(y1, y2) + margin)


def with_added(grid, added, resolution, origin_x, origin_y, start=None, clear_radius=0.35):
    """grid 에 added 칸을 장애물(100)로 넣은 새 격자. start(x, y) 에서 clear_radius 안의 칸은 넣지 않는다
    (금지 구역/좌석 덩어리 안에 서 있는 로봇이 출발 칸부터 막혀 꼼짝 못 하지 않게)."""
    out = np.asarray(grid).copy()
    if added is None or not added.any():
        return out
    add = added
    if start is not None and clear_radius > 0:
        near = near_mask(out.shape, resolution, origin_x, origin_y, start, clear_radius)
        if (add & near).any():
            add = add & ~near
    out[add] = 100
    return out


def near_mask(shape, resolution, origin_x, origin_y, point, radius):
    h, w = shape
    xs = origin_x + (np.arange(w) + 0.5) * resolution
    ys = origin_y + (np.arange(h) + 0.5) * resolution
    return ((xs[None, :] - point[0]) ** 2 + (ys[:, None] - point[1]) ** 2) <= radius * radius


def prepare(grid, resolution, stripe_cells=2, inflate=0.1, max_blob_size=0.4, occupied_threshold=65):
    """경로 계획용 기본 격자와 더할 칸. (줄무늬 메운 격자, 다리 점을 부풀려 더할 칸 마스크, 통계 dict)."""
    filled, n_fill = fill_stripes(grid, stripe_cells, occupied_threshold)
    added = inflate_clusters(filled, resolution, inflate, max_blob_size, occupied_threshold)
    return filled, added, {'줄무늬 메움': n_fill, '다리 점 부풀림': int(added.sum())}
