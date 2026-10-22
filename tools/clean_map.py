"""SLAM으로 만든 맵(pgm + yaml)을 정리해서 새 파일로 저장한다. 원본은 건드리지 않는다.

    python3 tools/clean_map.py ~/maps/maze_map.yaml [-o ~/maps/maze_map_clean] [--seed 0 0]

정리 내용
  0. 열림 연산으로 폭 2칸 이하의 가느다란 빈 칸 줄무늬 제거 (탐색을 덜 한 SLAM 맵의 LiDAR 빛줄기, --open)
  1. 로봇이 다니는 빈 공간(seed 위치가 속한 영역, 없으면 가장 큰 영역)만 남기고 나머지 빈 칸은 미탐색으로
  2. 그 영역에서 2칸보다 멀리 떨어진 장애물 점은 미탐색으로 (방 밖 노이즈)
  3. 빈 공간 안에 홀로 있는 작은 장애물 점(--speck 칸 이하)은 빈 칸으로 (센서 노이즈)
  4. 빈 공간에 둘러싸인 작은 미탐색 구멍(--hole 칸 이하)은 빈 칸으로
  5. 같은 행/열에서 벽 사이 --gap 칸 이하 틈 메우기 (점선처럼 끊긴 벽)
  6. 경계 봉합: 빈 공간과 맞닿은 미탐색 칸을 벽으로 (바깥 벽이 끊김 없이 이어짐, --no-seal 로 끔)
  7. 1칸 외곽선: 벽 = 빈 공간을 둘러싼 1칸 테두리만. 두꺼운 벽의 바깥쪽과 물체 안쪽은 미탐색으로
     (경로 계획은 미탐색도 장애물로 보므로 지나갈 곳은 그대로, --thick 로 끔)
비교 이미지(<출력>_preview.png): 왼쪽 원본, 오른쪽 정리본
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
    """4방향 연결 성분. (라벨 배열, 성분별 크기 리스트) 반환. 라벨 0 = 마스크 밖."""
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
    # map_saver 값: 254 빈 칸, 205 미탐색, 0 장애물.
    # 주의: free_thresh가 0.196보다 크면(예: 0.25) map_server가 205(미탐색)를 빈 칸으로 읽는다
    occ = img < 89            # occupied_thresh 0.65
    free = img > 210          # 205(미탐색)는 빈 칸이 아님
    stats = {}

    # 0. 열림 연산: 탐색을 덜 한 SLAM 맵의 LiDAR 빛줄기 모양 가느다란 빈 칸(폭 2*opening 칸 이하) 제거
    #    로봇이 지나갈 수 없는 폭이므로 주행 공간은 그대로
    if opening > 0:
        opened = dilate(erode(free, opening), opening) & free
        stats['가느다란 빈 칸 제거'] = int((free & ~opened).sum())
        free = opened

    # 1. 로봇이 다니는 빈 공간
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

    # 2. 방 밖 장애물 점
    near = dilate(region, 2)
    stats['방 밖 장애물 점 제거'] = int((occ & ~near).sum())
    occ &= near

    # 3. 빈 공간 안의 작은 장애물 점
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

    # 4. 빈 공간에 둘러싸인 작은 미탐색 구멍
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

    # 5. 같은 행/열에서 벽 사이 max_gap 칸 이하 틈 메우기 (점선처럼 끊긴 벽).
    #    로봇이 지나갈 수 없는 폭이므로 주행 공간은 줄지 않는다
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

    # 6. 경계 봉합: 빈 공간과 맞닿은(대각선 포함) 미탐색 칸을 벽으로 → 바깥 벽이 끊김 없이 이어짐
    if seal:
        border = dilate(region, 1) & ~region & ~occ
        stats['바깥 경계 봉합'] = int(border.sum())
        occ |= border

    # 7. 1칸 외곽선: 빈 공간과 맞닿은(대각선 포함) 칸만 벽. LiDAR 가 실제로 닿는 면이 이 줄이다
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
