"""맵의 기울기를 바로잡고 벽을 곧은 선으로 다시 그린다. 원본은 건드리지 않는다.

    python3 tools/straighten_map.py ~/maps/maze_map.yaml [-o ~/maps/maze_map_straight] [--seed 0 0]

순서
  1. clean_map.py 와 같은 정리 (방 밖 노이즈 제거 등)
  2. 기울기 추정: 벽 점들을 돌려 보며 가로/세로 줄에 가장 잘 모이는 각도 탐색
  3. 그 각도만큼 반대로 회전 (맵 원점 (0,0) 은 그대로 유지)
  4. 가로/세로 벽 구간을 찾아 1칸 두께 직선으로 다시 그림, 모서리 끝점 맞물림
  5. 방 안 = 빈 칸, 밖 = 미탐색, 바깥 경계 봉합

회전했으므로 이 맵에서 등록한 위치 좌표도 같은 각도로 돌려야 한다.
출력되는 각도(rotation_deg)로 서버에서:  python manage.py rotate_locations <각도>
"""
import argparse
import math
import os

import numpy as np

from clean_map import FREE, OCC, UNKNOWN, clean, components, dilate, preview, read_pgm, read_yaml, write_pgm

MIN_LEN = 6        # 이 칸 수(30cm) 이상 이어진 것만 벽 직선으로 인정
SNAP = 3           # 모서리 끝점을 맞물리게 붙이는 거리(칸)


def estimate_angle(points, res, max_deg=15.0, step=0.05):
    """벽 점들이 가로/세로 줄에 가장 잘 모이는 회전 각도(rad). 이 각도만큼 -회전하면 반듯해진다."""
    best = (-1, 0.0)
    for deg in np.arange(-max_deg, max_deg + 1e-9, step):
        t = math.radians(deg)
        c, s = math.cos(t), math.sin(t)
        x = c * points[:, 0] + s * points[:, 1]      # R(-t) 적용
        y = -s * points[:, 0] + c * points[:, 1]
        hx = np.bincount(np.floor((x - x.min()) / res).astype(int))
        hy = np.bincount(np.floor((y - y.min()) / res).astype(int))
        score = int((hx.astype(np.int64) ** 2).sum() + (hy.astype(np.int64) ** 2).sum())
        if score > best[0]:
            best = (score, t)
    return best[1]


def rotate_grid(img, res, origin, theta, margin=5):
    """img(행 0 = 위쪽)를 월드 원점 기준 -theta 회전. (새 이미지, 새 origin) 반환."""
    h, w = img.shape
    known = img != UNKNOWN
    rows, cols = np.nonzero(known)
    # 아는 영역 모서리를 돌려서 새 범위 결정
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

    # 새 칸 중심 → 원래 월드 좌표(R(theta)) → 원래 칸 (최근접)
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
    """True 구간을 max_gap 이하 틈은 이어서 (시작, 끝) 목록으로."""
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
    """벽 점에서 가로/세로 직선 구간을 큰 것부터 차례로 뽑는다.
    반환: [('h', row, c0, c1) | ('v', col, r0, r1)], 남은 점(직선에 안 속한 것)"""
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
        line = int(round(np.median(rr))) + lo   # 점들이 가장 많이 몰린 줄
        band[:] = False                          # 이 구간의 점은 사용함 (view라 remaining에 반영)
        segments.append((axis, line, a, b))
    return segments, remaining


def snap_segments(segments):
    """끝점이 수직 벽 근처(SNAP 칸 이내)면 그 벽까지 늘리거나 줄여 모서리를 맞물리게."""
    segs = [list(s) for s in segments]
    for s in segs:
        for t in segs:
            if s[0] == t[0]:
                continue
            # s의 끝점 좌표축 값(s[2], s[3])을 t의 위치(t[1])에 맞추기. t가 s의 줄(s[1])을 덮는지 확인
            if t[2] - SNAP <= s[1] <= t[3] + SNAP:
                if abs(s[2] - t[1]) <= SNAP:
                    s[2] = t[1]
                if abs(s[3] - t[1]) <= SNAP:
                    s[3] = t[1]
    return [tuple(s) for s in segs]


def extend_segments(segments, rot, max_ext=20):
    """끝점에서 같은 방향으로 max_ext 칸 안에 수직 벽이 있고, 그 사이에 빈 칸이 없으면 그 벽까지 늘린다.
    (물체 뒤처럼 LiDAR가 못 본 곳 때문에 끊긴 바깥 벽을 잇기 위함)"""
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
    """방 안에 갇힌 미탐색 영역(물체 안쪽 등)을 경계 사각형째 벽(장애물)으로 채운다."""
    unknown = ~region & ~walls
    labels, sizes = components(unknown)
    h, w = unknown.shape
    filled = 0
    for label, size in enumerate(sizes):
        if label == 0 or size > max_cells:
            continue
        rr, cc = np.nonzero(labels == label)
        if rr.min() == 0 or cc.min() == 0 or rr.max() == h - 1 or cc.max() == w - 1:
            continue  # 바깥과 이어진 미탐색
        r0, r1, c0, c1 = rr.min(), rr.max(), cc.min(), cc.max()
        # 사각형 바로 바깥 줄에 붙은 벽 조각(돌기)이 있으면 그 줄까지 넓혀 흡수 (최대 2줄)
        for _ in range(2):
            if r1 + 1 < h and walls[r1 + 1, c0:c1 + 1].any():
                r1 += 1
            if r0 > 0 and walls[r0 - 1, c0:c1 + 1].any():
                r0 -= 1
            if c1 + 1 < w and walls[r0:r1 + 1, c1 + 1].any():
                c1 += 1
            if c0 > 0 and walls[r0:r1 + 1, c0 - 1].any():
                c0 -= 1
        walls[r0:r1 + 1, c0:c1 + 1] = True
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
    """seed에서 4방향으로 passable을 따라 퍼진 영역."""
    from collections import deque
    region = np.zeros_like(passable)
    r0, c0 = seed
    if not passable[r0, c0]:
        # seed가 벽 위면 가까운 통과 가능 칸에서 시작
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('-o', '--output', help='출력 경로(확장자 제외). 기본: <원본>_straight')
    ap.add_argument('--seed', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'))
    ap.add_argument('--angle', type=float, help='기울기(도)를 직접 지정 (기본: 자동 추정)')
    args = ap.parse_args()

    yaml_path = os.path.expanduser(args.yaml_path)
    conf = read_yaml(yaml_path)
    res = float(conf['resolution'])
    origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
    raw = read_pgm(os.path.join(os.path.dirname(yaml_path), conf['image'].strip('\'"')))
    if conf.get('negate', '0') == '1':
        raw = 255 - raw
    h = raw.shape[0]

    # 1. 정리 (봉합은 마지막에 다시 하므로 생략)
    cleaned, _ = clean(raw, world_to_rc(args.seed[0], args.seed[1], origin, res, h), 2, 30, 3, False)

    # 2. 기울기
    rows, cols = np.nonzero(cleaned == OCC)
    pts = np.column_stack([origin[0] + (cols + 0.5) * res, origin[1] + (h - rows - 0.5) * res])
    theta = math.radians(args.angle) if args.angle is not None else estimate_angle(pts, res)
    print(f'기울기: {math.degrees(theta):+.2f}° → 반대로 돌려서 바로잡음 (rotation_deg = {-math.degrees(theta):+.2f})')

    # 3. 회전
    rot, new_origin = rotate_grid(cleaned, res, origin, theta)
    nh = rot.shape[0]
    occ = rot == OCC
    known = rot != UNKNOWN

    # 4. 벽 직선화
    segments, leftover = extract_segments(occ)
    segments = snap_segments(extend_segments(segments, rot))
    walls = draw_segments(occ.shape, segments)
    # 직선에 안 속한 점 중 직선 바로 옆(1칸)은 들쑥날쑥한 벽 조각이므로 버리고, 떨어진 것(물체)은 유지
    leftover &= ~dilate(walls, 1)
    walls |= leftover
    print(f'벽 직선 {len(segments)}개 (가로 {sum(s[0] == "h" for s in segments)}, '
          f'세로 {sum(s[0] == "v" for s in segments)}), 직선 밖 물체 점 {int(leftover.sum())}칸')

    # 5. 방 안 다시 채우기 + 봉합
    seed_rc = world_to_rc(args.seed[0], args.seed[1], new_origin, res, nh)
    region = flood(known & ~walls, seed_rc)
    enclosed = fill_enclosed(region, walls)
    if enclosed:
        region = flood(known & ~walls, seed_rc)
        print(f'갇힌 미탐색 영역 {enclosed}칸 → 사각형 장애물로 채움')
    border = dilate(region, 1) & ~region & ~walls
    walls |= border
    out = np.full(rot.shape, UNKNOWN, dtype=np.uint8)
    out[region] = FREE
    out[walls] = OCC
    print(f'빈 칸 {int((cleaned == FREE).sum())} → {int(region.sum())}, 바깥 경계 봉합 {int(border.sum())}칸')

    base = os.path.expanduser(args.output) if args.output else os.path.splitext(yaml_path)[0] + '_straight'
    write_pgm(base + '.pgm', out)
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{new_origin[0]:.4f}, {new_origin[1]:.4f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                f'# straighten_map.py: 원본 {os.path.basename(yaml_path)} 을 원점 기준 '
                f'{-math.degrees(theta):+.2f}도 회전 (rotation_deg)\n')
    print(f'저장: {base}.yaml, {base}.pgm')
    # 비교 이미지: 회전 전 정리본과 결과를 같은 크기로
    pad_h, pad_w = max(cleaned.shape[0], out.shape[0]), max(cleaned.shape[1], out.shape[1])

    def pad(a):
        p = np.full((pad_h, pad_w), UNKNOWN, dtype=np.uint8)
        p[:a.shape[0], :a.shape[1]] = a
        return p
    if preview(base + '_preview.png', pad(cleaned), pad(out)):
        print(f'비교 이미지: {base}_preview.png (왼쪽 정리만 한 것, 오른쪽 반듯하게 한 것)')


if __name__ == '__main__':
    main()
