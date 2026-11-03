"""Map processing for path planning (no ROS dependency, numpy only).

Localization (AMCL) uses the SLAM map as is; A* uses a grid processed by this module (the map file is unchanged).
Where desks/chairs are scattered (auditoriums, classrooms), the LiDAR (about 13 cm high) only sees leg cross-sections
as points, so
  - A* finds paths between seats through the free cells between the points, and
  - far away the beam spacing (1.5°, about 9 cm at 3.4 m) is wider than a cell (5 cm), leaving unknown stripes
    between free cells that block the way.

1. fill_stripes: narrow unknown stripes (less than 2*cells wide) between free cells become free. Obstacle cells are untouched
2. inflate_clusters: only small obstacle clusters (leg points, both width and height <= max_blob_size) are inflated
   by inflate. Walls (long clusters) stay as they are.
   A* cannot pass within the robot radius (0.25 m), so a gap between leg points is blocked if it is narrower than
   2*(0.25 + inflate) (default 0.1 -> under 0.7 m). Seat rows with legs on a grid are blocked like one block, while
   aisles of 0.7 m or more remain. A closing operation (joining points) is not used: it barely differs from the A* radius.
   Places with wider leg spacing (e.g. under desks) are blocked with no-go zones on the web
3. zone_mask: no-go zones drawn on the web (rectangles/lines) -> cells
4. with_added: add the extra cells to the grid as obstacles, except around the start position so a robot standing
   inside can get out (a no-go zone containing the start is picked with zone_contains and left out of that plan entirely)

Grid values are as in OccupancyGrid: -1 unknown, 0..100 (>= occupied_threshold = obstacle), row 0 = bottom.
"""
import math
from collections import deque

import numpy as np

from .astar import obstacle_distance

try:
    from scipy.ndimage import label as _scipy_label
except Exception:  # environments without scipy or with a mismatched numpy (e.g. Raspberry Pi)
    _scipy_label = None

LINE_WIDTH = 0.1  # m, thickness of line (virtual wall) no-go zones
# Margin (m) within which the start counts as 'inside' a no-go zone; such a zone is left out of that plan.
# Kept small so that replanning while passing next to a zone does not drop the whole zone and cut across it;
# near a zone's edge, clearing around the start in with_added is enough.
INSIDE_MARGIN = 0.05


def _dilate(mask, k):
    """8-connected dilation by k cells."""
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
    """8-connected components. (label array, cell count per component) - label 0 = outside the mask."""
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
    """New grid with narrow unknown stripes between free cells set to free (0), and the number of cells changed.
    Only unknown cells inside the closing (cells) of the free area change, so the free area does not grow outward."""
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
    """Cells newly blocked by inflating small obstacle clusters by inflate (m) (bool mask, original obstacle cells excluded).
    Clusters longer than max_blob_size (m) in width or height (walls, desk front panels, etc.) are not inflated.
    (Classifying by area instead would make thin wall pieces count as small clusters and narrow doorways.)"""
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
    # resolution comes from OccupancyGrid as float32 (0.0500000007), which breaks boundary comparisons, so allow 1% of a cell
    eps = 0.01 * resolution
    small = extent <= max_blob_size + eps
    small[0] = False
    blobs = small[labels]
    if not blobs.any():
        return np.zeros(grid.shape, dtype=bool)
    return (obstacle_distance(blobs, resolution, inflate + resolution) <= inflate + eps) & ~occ


def zone_mask(shape, resolution, origin_x, origin_y, zones):
    """No-go zone list -> cell mask. zone: {'shape': 'rect'|'line', 'x1', 'y1', 'x2', 'y2'} (m)."""
    h, w = shape
    mask = np.zeros(shape, dtype=bool)
    if not zones:
        return mask
    xs = origin_x + (np.arange(w) + 0.5) * resolution
    ys = origin_y + (np.arange(h) + 0.5) * resolution
    for z in zones:
        x1, y1, x2, y2 = (float(z[k]) for k in ('x1', 'y1', 'x2', 'y2'))
        if z.get('shape') == 'line':
            # cells within LINE_WIDTH/2 (at least one cell) of the segment
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
    """Whether no-go zone z contains point (x, y) within margin."""
    x1, y1, x2, y2 = (float(z[k]) for k in ('x1', 'y1', 'x2', 'y2'))
    px, py = point
    if z.get('shape') == 'line':
        dx, dy = x2 - x1, y2 - y1
        t = min(1.0, max(0.0, ((px - x1) * dx + (py - y1) * dy) / max(dx * dx + dy * dy, 1e-12)))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) <= LINE_WIDTH / 2 + margin
    return (min(x1, x2) - margin <= px <= max(x1, x2) + margin
            and min(y1, y2) - margin <= py <= max(y1, y2) + margin)


def with_added(grid, added, resolution, origin_x, origin_y, start=None, clear_radius=0.35):
    """New grid with the added cells set to obstacle (100). Cells within clear_radius of start (x, y) are not added
    (so a robot standing inside a no-go zone / seat block is not stuck with its own start cell blocked)."""
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
    """Base grid for path planning and cells to add. (stripe-filled grid, mask of cells added by inflating leg points, stats dict)."""
    filled, n_fill = fill_stripes(grid, stripe_cells, occupied_threshold)
    added = inflate_clusters(filled, resolution, inflate, max_blob_size, occupied_threshold)
    return filled, added, {'줄무늬 메움': n_fill, '다리 점 부풀림': int(added.sum())}
