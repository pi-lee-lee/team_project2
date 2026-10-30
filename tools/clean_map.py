"""Clean up a SLAM-built map (pgm + yaml) and save it as a new file. The original is not modified.

    python3 tools/clean_map.py ~/maps/maze_map.yaml [-o ~/maps/maze_map_clean] [--seed 0 0]

Cleanup steps
  0. Opening removes thin free-cell stripes up to 2 cells wide (LiDAR rays in an under-explored SLAM map, --open)
  1. Keep only the free space the robot drives in (the region containing the seed point, else the largest region); other free cells become unknown
  2. Obstacle points more than 2 cells away from that region become unknown (noise outside the room)
  3. Small isolated obstacle points inside free space (up to --speck cells) become free (sensor noise)
  4. Small unknown holes surrounded by free space (up to --hole cells) become free
  5. Fill gaps of up to --gap cells between walls in the same row/column (walls broken like dotted lines)
  6. Border sealing: unknown cells touching free space become walls (outer walls become continuous, disabled by --no-seal)
  7. 1-cell outline: walls = only the 1-cell border around free space. The outside of thick walls and the inside of objects become unknown
     (path planning treats unknown as an obstacle too, so drivable space is unchanged, disabled by --thick)
Comparison image (<output>_preview.png): left original, right cleaned
"""
import argparse
import math
import os
from collections import deque

import numpy as np

FREE, OCC, UNKNOWN = 254, 0, 205


def read_yaml(path):
    conf = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                k, v = line.split(':', 1)
                conf[k.strip()] = v.strip()
    return conf


def read_pgm(path):
    data = open(path, 'rb').read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w).copy()


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n# cleaned by tools/clean_map.py\n{w} {h}\n255\n'.encode())
        f.write(img.astype(np.uint8).tobytes())


def components(mask):
    """4-connected components. Returns (label array, list of component sizes). Label 0 = outside the mask."""
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
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not labels[nr, nc]:
                    labels[nr, nc] = label
                    q.append((nr, nc))
        sizes.append(n)
    return labels, sizes


def dilate(mask, k=1):
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


def erode(mask, k=1):
    return ~dilate(~mask, k)


def clean(img, seed_rc, speck, hole, max_gap=3, seal=True, opening=1, thin=False):
    # map_saver values: 254 free, 205 unknown, 0 obstacle.
    # Note: if free_thresh is above 0.196 (e.g. 0.25), map_server reads 205 (unknown) as free
    occ = img < 89            # occupied_thresh 0.65
    free = img > 210          # 205 (unknown) is not free
    stats = {}

    # 0. Opening: remove thin ray-shaped free cells (up to 2*opening cells wide) from an under-explored SLAM map.
    #    The robot can't pass through that width, so drivable space is unchanged
    if opening > 0:
        opened = dilate(erode(free, opening), opening) & free
        stats['가느다란 빈 칸 제거'] = int((free & ~opened).sum())
        free = opened

    # 1. Free space the robot drives in
    labels, sizes = components(free)
    main = 0
    if seed_rc is not None:
        r, c = seed_rc
        if 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
            main = labels[r, c]
    if main == 0:
        main = int(np.argmax(sizes[1:])) + 1 if len(sizes) > 1 else 0
    region = labels == main
    stats['떨어진 빈 칸 제거'] = int((free & ~region).sum())

    # 2. Obstacle points outside the room
    near = dilate(region, 2)
    stats['방 밖 장애물 점 제거'] = int((occ & ~near).sum())
    occ &= near

    # 3. Small obstacle points inside free space
    olabels, osizes = components(occ)
    removed = 0
    for label, size in enumerate(osizes):
        if label == 0 or size > speck:
            continue
        blob = olabels == label
        ring = dilate(blob, 1) & ~blob
        if region[ring].all():
            occ &= ~blob
            region |= blob
            removed += size
    stats['방 안 노이즈 점 제거'] = removed

    # 4. Small unknown holes surrounded by free space
    unknown = ~occ & ~region
    ulabels, usizes = components(unknown)
    filled = 0
    for label, size in enumerate(usizes):
        if label == 0 or size > hole:
            continue
        blob = ulabels == label
        rows, cols = np.nonzero(blob)
        if rows.min() == 0 or cols.min() == 0 or rows.max() == img.shape[0] - 1 or cols.max() == img.shape[1] - 1:
            continue
        ring = dilate(blob, 1) & ~blob
        if (region | occ)[ring].all() and region[ring].any():
            region |= blob
            filled += size
    stats['작은 구멍 메움'] = filled

    # 5. Fill gaps of up to max_gap cells between walls in the same row/column (walls broken like dotted lines).
    #    The robot can't pass through that width, so drivable space does not shrink
    gap = np.zeros_like(occ)
    for grid, out_view in ((occ, gap), (occ.T, gap.T)):
        for r in range(grid.shape[0]):
            cols = np.flatnonzero(grid[r])
            for a, b in zip(cols[:-1], cols[1:]):
                if 1 < b - a <= max_gap + 1:
                    out_view[r, a + 1:b] = True
    gap &= ~occ
    stats['끊긴 벽 이음'] = int(gap.sum())
    occ |= gap
    region &= ~occ

    # 6. Border sealing: unknown cells touching free space (incl. diagonally) become walls -> continuous outer walls
    if seal:
        border = dilate(region, 1) & ~region & ~occ
        stats['바깥 경계 봉합'] = int(border.sum())
        occ |= border

    # 7. 1-cell outline: only cells touching free space (incl. diagonally) are walls. This line is the surface the LiDAR actually hits
    if thin:
        outline = dilate(region, 1) & ~region
        stats['두꺼운 벽 1칸으로'] = int((occ & ~outline).sum())
        occ = outline

    out = np.full(img.shape, UNKNOWN, dtype=np.uint8)
    out[region] = FREE
    out[occ] = OCC
    return out, stats


def preview(path, before, after, scale=4):
    try:
        from PIL import Image
    except ImportError:
        return False
    gap = np.full((before.shape[0], 4), 128, dtype=np.uint8)
    both = np.hstack([before, gap, after])
    Image.fromarray(both).resize((both.shape[1] * scale, both.shape[0] * scale), Image.NEAREST).save(path)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('-o', '--output', help='출력 경로(확장자 제외). 기본: <원본>_clean')
    ap.add_argument('--seed', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='로봇이 다니는 공간 안의 한 점 (m). 기본: 원점')
    ap.add_argument('--speck', type=int, default=2, help='이 칸 수 이하의 홀로 있는 장애물 점은 노이즈로 제거')
    ap.add_argument('--hole', type=int, default=30, help='이 칸 수 이하의 미탐색 구멍은 빈 칸으로')
    ap.add_argument('--gap', type=int, default=3, help='같은 행/열에서 이 칸 수 이하로 끊긴 벽은 이음')
    ap.add_argument('--no-seal', action='store_true', help='바깥 경계 봉합 안 함')
    ap.add_argument('--open', type=int, default=1, help='열림 연산 칸 수 (가느다란 빈 칸 제거, 0 이면 끔)')
    ap.add_argument('--thick', action='store_true', help='벽을 1칸 외곽선으로 줄이지 않음 (예전 방식)')
    args = ap.parse_args()

    yaml_path = os.path.expanduser(args.yaml_path)
    conf = read_yaml(yaml_path)
    pgm_path = os.path.join(os.path.dirname(yaml_path), conf['image'].strip('\'"'))
    res = float(conf['resolution'])
    origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
    img = read_pgm(pgm_path)
    if conf.get('negate', '0') == '1':
        img = 255 - img

    h = img.shape[0]
    seed_rc = (h - 1 - int(math.floor((args.seed[1] - origin[1]) / res)),
               int(math.floor((args.seed[0] - origin[0]) / res)))
    out, stats = clean(img, seed_rc, args.speck, args.hole, args.gap, not args.no_seal, args.open, not args.thick)

    base = os.path.expanduser(args.output) if args.output else os.path.splitext(yaml_path)[0] + '_clean'
    write_pgm(base + '.pgm', out)
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{origin[0]}, {origin[1]}, {origin[2] if len(origin) > 2 else 0}]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    for k, v in stats.items():
        print(f'{k}: {v}칸')
    print(f'빈 칸 {int((img > 210).sum())} → {int((out == FREE).sum())}, '
          f'장애물 {int((img < 89).sum())} → {int((out == OCC).sum())}')
    print(f'저장: {base}.yaml, {base}.pgm')
    if preview(base + '_preview.png', img, out):
        print(f'비교 이미지: {base}_preview.png (왼쪽 원본, 오른쪽 정리본)')


if __name__ == '__main__':
    main()
