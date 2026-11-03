"""Correct the map's tilt and redraw walls as straight lines. The original is not modified.

    python3 tools/straighten_map.py ~/maps/maze_map.yaml [-o ~/maps/maze_map_straight] [--seed 0 0]

Steps
  1. Same cleanup as clean_map.py (removing noise outside the room, etc.)
  2. Tilt estimate: rotate the wall points and search for the angle at which they line up best on rows/columns
  3. Rotate back by that angle (the map origin (0,0) stays fixed)
  4. Find horizontal/vertical wall segments and redraw them as 1-cell-thick straight lines, snapping corner endpoints together
     (short parallel lines duplicated from thick walls are dropped)
  5. Inside the room = free, outside = unknown, outer border sealed. Objects enclosed in the room get a 1-cell rectangular outline,
     wall fragments not touching the room (noise outside the room) are removed

--auto: if the fraction of wall points fitting straight lines is below --min-coverage (diagonal walls, cylinders, etc.), save
        the clean_map.py cleanup (1-cell outline) without rotating/straightening. Default of scripts/save_map.sh (<name>_clean)

Because the map is rotated, location coordinates registered on it must be rotated by the same angle. The angle is kept in the yaml as
'# rotation_deg: <angle>', and the web 'save map' copies locations/no-go zones registered during SLAM rotated by it (services.carry_over_locations).
If an already registered map is rotated afterwards, on the server:  python manage.py rotate_locations <angle>
"""
import argparse
import math
import os

import numpy as np

from clean_map import FREE, OCC, UNKNOWN, clean, components, dilate, preview, read_pgm, read_yaml, write_pgm

MIN_LEN = 6        # only runs of at least this many cells (30 cm) count as straight walls
SNAP = 3           # distance (cells) within which corner endpoints are snapped together
MIN_ROTATE_DEG = 0.2   # don't rotate if tilted less than this (under 2 cm at the end of a 5 m wall = under half a cell; better to keep coordinates unchanged)


def estimate_angle(points, res, max_deg=15.0, step=0.05):
    """Rotation angle (rad) at which wall points line up best on rows/columns. Rotating by minus this angle straightens the map."""
    best = (-1, 0.0)
    for deg in np.arange(-max_deg, max_deg + 1e-9, step):
        t = math.radians(deg)
        c, s = math.cos(t), math.sin(t)
        x = c * points[:, 0] + s * points[:, 1]      # apply R(-t)
        y = -s * points[:, 0] + c * points[:, 1]
        hx = np.bincount(np.floor((x - x.min()) / res).astype(int))
        hy = np.bincount(np.floor((y - y.min()) / res).astype(int))
        score = int((hx.astype(np.int64) ** 2).sum() + (hy.astype(np.int64) ** 2).sum())
        if score > best[0]:
            best = (score, t)
    return best[1]


def rotate_grid(img, res, origin, theta, margin=5):
    """Rotate img (row 0 = top) by -theta about the world origin. Returns (new image, new origin)."""
    h, w = img.shape
    known = img != UNKNOWN
    rows, cols = np.nonzero(known)
    # rotate the corners of the known area to determine the new extent
    xs = origin[0] + np.array([cols.min(), cols.max() + 1]) * res
    ys = origin[1] + (h - np.array([rows.max() + 1, rows.min()])) * res
    corners = np.array([(x, y) for x in xs for y in ys])
    c, s = math.cos(theta), math.sin(theta)
    nx = c * corners[:, 0] + s * corners[:, 1]
    ny = -s * corners[:, 0] + c * corners[:, 1]
    ox = math.floor(nx.min() / res) * res - margin * res
    oy = math.floor(ny.min() / res) * res - margin * res
    nw = int(math.ceil((nx.max() - ox) / res)) + margin
    nh = int(math.ceil((ny.max() - oy) / res)) + margin

    # new cell center -> original world coordinates (R(theta)) -> original cell (nearest)
    cc, rr = np.meshgrid(np.arange(nw), np.arange(nh))
    wx = ox + (cc + 0.5) * res
    wy = oy + (nh - rr - 0.5) * res
    px = c * wx - s * wy
    py = s * wx + c * wy
    src_c = np.floor((px - origin[0]) / res).astype(int)
    src_r = h - 1 - np.floor((py - origin[1]) / res).astype(int)
    out = np.full((nh, nw), UNKNOWN, dtype=np.uint8)
    ok = (src_c >= 0) & (src_c < w) & (src_r >= 0) & (src_r < h)
    out[ok] = img[src_r[ok], src_c[ok]]
    return out, [ox, oy, 0.0]


def runs(mask_1d, max_gap=2):
    """True runs as a list of (start, end), bridging gaps of up to max_gap."""
    idx = np.flatnonzero(mask_1d)
    if len(idx) == 0:
        return []
    out, start, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > max_gap + 1:
            out.append((start, prev))
            start = i
        prev = i
    out.append((start, prev))
    return out


def extract_segments(occ):
    """Extract horizontal/vertical straight segments from wall points, largest first.
    Returns: [('h', row, c0, c1) | ('v', col, r0, r1)], remaining points (not part of any line)"""
    remaining = occ.copy()
    segments = []
    h, w = occ.shape
    while True:
        best = None
        for axis in ('h', 'v'):
            grid = remaining if axis == 'h' else remaining.T
            n = grid.shape[0]
            for r in range(n):
                band = grid[max(0, r - 1):r + 2]
                for a, b in runs(band.any(axis=0)):
                    if b - a + 1 < MIN_LEN:
                        continue
                    count = int(band[:, a:b + 1].sum())
                    if best is None or count > best[0]:
                        best = (count, axis, r, a, b)
        if best is None or best[0] < MIN_LEN:
            break
        _, axis, r, a, b = best
        grid = remaining if axis == 'h' else remaining.T
        lo = max(0, r - 1)
        band = grid[lo:r + 2, a:b + 1]
        rr, _ = np.nonzero(band)
        line = int(round(np.median(rr))) + lo   # the line where most points are concentrated
        band[:] = False                          # mark this segment's points as used (a view, so it updates remaining)
        segments.append((axis, line, a, b))
    return segments, remaining


def drop_parallel(segments):
    """Drop the shorter of parallel lines (within 2 rows) duplicated from a thick wall (short lines inside the wall)."""
    kept = []
    for s in sorted(segments, key=lambda s: abs(s[3] - s[2]), reverse=True):
        if not any(t[0] == s[0] and abs(t[1] - s[1]) <= 2
                   and min(t[2], t[3]) - SNAP <= min(s[2], s[3]) and max(s[2], s[3]) <= max(t[2], t[3]) + SNAP
                   for t in kept):
            kept.append(s)
    return kept


def snap_segments(segments):
    """If an endpoint is near a perpendicular wall (within SNAP cells), extend or trim it to that wall so the corner meets."""
    segs = [list(s) for s in segments]
    for s in segs:
        for t in segs:
            if s[0] == t[0]:
                continue
            # snap s's endpoint coordinates (s[2], s[3]) to t's position (t[1]). Check that t covers s's line (s[1])
            if t[2] - SNAP <= s[1] <= t[3] + SNAP:
                if abs(s[2] - t[1]) <= SNAP:
                    s[2] = t[1]
                if abs(s[3] - t[1]) <= SNAP:
                    s[3] = t[1]
    return [tuple(s) for s in segs]


def extend_segments(segments, rot, max_ext=20):
    """If there is a perpendicular wall within max_ext cells beyond an endpoint in the same direction, with no free cells in between, extend to that wall.
    (To join outer walls broken by areas the LiDAR could not see, e.g. behind objects)"""
    free = rot == FREE
    segs = [list(s) for s in segments]
    for s in segs:
        for end in (2, 3):
            step = -1 if end == 2 else 1
            for t in segs:
                if t[0] == s[0] or not (t[2] - SNAP <= s[1] <= t[3] + SNAP):
                    continue
                dist = (t[1] - s[end]) * step
                if not 0 < dist <= max_ext:
                    continue
                lo, hi = sorted((s[end], t[1]))
                cells = free[s[1], lo:hi + 1] if s[0] == 'h' else free[lo:hi + 1, s[1]]
                if not cells.any():
                    s[end] = t[1]
    return [tuple(s) for s in segs]


def fill_enclosed(region, walls, max_cells=400):
    """Turn the bounding rectangle of unknown areas enclosed in the room (inside objects, etc.) into a 1-cell outline wall (the inside stays unknown = obstacle for path planning)."""
    unknown = ~region & ~walls
    labels, sizes = components(unknown)
    h, w = unknown.shape
    filled = 0
    for label, size in enumerate(sizes):
        if label == 0 or size > max_cells:
            continue
        rr, cc = np.nonzero(labels == label)
        if rr.min() == 0 or cc.min() == 0 or rr.max() == h - 1 or cc.max() == w - 1:
            continue  # unknown connected to the outside
        r0, r1, c0, c1 = rr.min(), rr.max(), cc.min(), cc.max()
        # if wall fragments (bumps) stick to the row just outside the rectangle, widen it to absorb them (up to 2 rows)
        for _ in range(2):
            if r1 + 1 < h and walls[r1 + 1, c0:c1 + 1].any():
                r1 += 1
            if r0 > 0 and walls[r0 - 1, c0:c1 + 1].any():
                r0 -= 1
            if c1 + 1 < w and walls[r0:r1 + 1, c1 + 1].any():
                c1 += 1
            if c0 > 0 and walls[r0:r1 + 1, c0 - 1].any():
                c0 -= 1
        walls[r0:r1 + 1, c0:c1 + 1] = False
        walls[[r0, r1], c0:c1 + 1] = True
        walls[r0:r1 + 1, [c0, c1]] = True
        filled += size
    return filled


def draw_segments(shape, segments):
    walls = np.zeros(shape, dtype=bool)
    for axis, line, a, b in segments:
        a, b = min(a, b), max(a, b)
        if axis == 'h':
            walls[line, a:b + 1] = True
        else:
            walls[a:b + 1, line] = True
    return walls


def flood(passable, seed):
    """Region reached from seed by 4-connected flood fill over passable."""
    from collections import deque
    region = np.zeros_like(passable)
    r0, c0 = seed
    if not passable[r0, c0]:
        # if seed is on a wall, start from a nearby passable cell
        rr, cc = np.nonzero(passable)
        i = int(np.argmin((rr - r0) ** 2 + (cc - c0) ** 2))
        r0, c0 = rr[i], cc[i]
    q = deque([(r0, c0)])
    region[r0, c0] = True
    h, w = passable.shape
    while q:
        r, c = q.popleft()
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if 0 <= nr < h and 0 <= nc < w and passable[nr, nc] and not region[nr, nc]:
                region[nr, nc] = True
                q.append((nr, nc))
    return region


def world_to_rc(x, y, origin, res, h):
    return h - 1 - int(math.floor((y - origin[1]) / res)), int(math.floor((x - origin[0]) / res))


def write_map(base, out, res, origin, rotation_deg=0.0, source=None):
    write_pgm(base + '.pgm', out)
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        if rotation_deg:
            f.write(f'# straighten_map.py: 원본 {source} 을 원점 기준 {rotation_deg:+.2f}도 회전\n'
                    f'# rotation_deg: {rotation_deg:.4f}\n')
    print(f'저장: {base}.yaml, {base}.pgm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('-o', '--output', help='출력 경로(확장자 제외). 기본: <원본>_straight')
    ap.add_argument('--seed', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'))
    ap.add_argument('--angle', type=float, help='기울기(도)를 직접 지정 (기본: 자동 추정)')
    ap.add_argument('--auto', action='store_true',
                    help='벽이 가로/세로 직선에 잘 안 맞으면 clean_map.py 정리(회전 없음)로 대신 저장')
    ap.add_argument('--min-coverage', type=float, default=0.85,
                    help='--auto: 벽 점 중 직선(1칸 이내)에 맞아야 하는 비율')
    args = ap.parse_args()
    base = os.path.expanduser(args.output) if args.output else os.path.splitext(os.path.expanduser(args.yaml_path))[0] + '_straight'

    yaml_path = os.path.expanduser(args.yaml_path)
    conf = read_yaml(yaml_path)
    res = float(conf['resolution'])
    origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
    raw = read_pgm(os.path.join(os.path.dirname(yaml_path), conf['image'].strip('\'"')))
    if conf.get('negate', '0') == '1':
        raw = 255 - raw
    h = raw.shape[0]

    # 1. Cleanup (sealing is skipped since it is redone at the end)
    cleaned, _ = clean(raw, world_to_rc(args.seed[0], args.seed[1], origin, res, h), 2, 30, 3, False)

    # 2. Tilt
    rows, cols = np.nonzero(cleaned == OCC)
    pts = np.column_stack([origin[0] + (cols + 0.5) * res, origin[1] + (h - rows - 0.5) * res])
    theta = math.radians(args.angle) if args.angle is not None else estimate_angle(pts, res)
    if args.angle is None and abs(math.degrees(theta)) < MIN_ROTATE_DEG:
        print(f'기울기: {math.degrees(theta):+.2f}° → {MIN_ROTATE_DEG}° 미만이라 돌리지 않음')
        theta = 0.0
    else:
        print(f'기울기: {math.degrees(theta):+.2f}° → 반대로 돌려서 바로잡음 (rotation_deg = {-math.degrees(theta):+.2f})')

    # 3. Rotation
    rot, new_origin = rotate_grid(cleaned, res, origin, theta)
    nh = rot.shape[0]
    occ = rot == OCC
    known = rot != UNKNOWN

    # 4. Wall straightening
    segments, leftover = extract_segments(occ)
    segments = drop_parallel(segments)
    coverage = float((occ & dilate(draw_segments(occ.shape, segments), 1)).sum()) / max(int(occ.sum()), 1)
    print(f'벽 점 중 직선에 맞는 비율: {coverage:.0%}')
    if args.auto and coverage < args.min_coverage:
        print(f'{args.min_coverage:.0%} 미만 (대각선 벽/둥근 물체가 많음) → 회전/직선화 없이 정리만 (1칸 외곽선)')
        out, stats = clean(raw, world_to_rc(args.seed[0], args.seed[1], origin, res, h), 2, 30, 3, thin=True)
        for k, v in stats.items():
            print(f'{k}: {v}칸')
        write_map(base, out, res, origin)
        preview(base + '_preview.png', raw, out)
        return
    segments = snap_segments(extend_segments(segments, rot))
    walls = draw_segments(occ.shape, segments)
    # of the points not on any line, those right next to a line (2 cells) are jagged wall bits and are dropped; separate ones (objects) are kept
    leftover &= ~dilate(walls, 2)
    walls |= leftover
    print(f'벽 직선 {len(segments)}개 (가로 {sum(s[0] == "h" for s in segments)}, '
          f'세로 {sum(s[0] == "v" for s in segments)}), 직선 밖 물체 점 {int(leftover.sum())}칸')

    # 5. Refill the room + seal
    seed_rc = world_to_rc(args.seed[0], args.seed[1], new_origin, res, nh)
    region = flood(known & ~walls, seed_rc)
    enclosed = fill_enclosed(region, walls)
    if enclosed:
        region = flood(known & ~walls, seed_rc)
        print(f'갇힌 미탐색 영역 {enclosed}칸 → 사각형 장애물로 채움')
    border = dilate(region, 1) & ~region & ~walls
    walls |= border
    stray = walls & ~dilate(region, 1)          # wall fragments not touching the room (noise outside the room, line ends sticking outward)
    walls &= ~stray
    out = np.full(rot.shape, UNKNOWN, dtype=np.uint8)
    out[region] = FREE
    out[walls] = OCC
    print(f'빈 칸 {int((cleaned == FREE).sum())} → {int(region.sum())}, 바깥 경계 봉합 {int(border.sum())}칸, '
          f'방 밖 벽 조각 제거 {int(stray.sum())}칸')

    write_map(base, out, res, new_origin, -math.degrees(theta), os.path.basename(yaml_path))
    # comparison image: the cleaned map before rotation and the result at the same size
    pad_h, pad_w = max(cleaned.shape[0], out.shape[0]), max(cleaned.shape[1], out.shape[1])

    def pad(a):
        p = np.full((pad_h, pad_w), UNKNOWN, dtype=np.uint8)
        p[:a.shape[0], :a.shape[1]] = a
        return p
    if preview(base + '_preview.png', pad(cleaned), pad(out)):
        print(f'비교 이미지: {base}_preview.png (왼쪽 정리만 한 것, 오른쪽 반듯하게 한 것)')


if __name__ == '__main__':
    main()
