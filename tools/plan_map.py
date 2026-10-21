"""경로 계획용 지도 미리 보기/확인. 로봇의 A* 플래너가 맵을 받을 때 하는 가공(plan_map.py)을 똑같이 해서 보여 준다.
원본 맵은 건드리지 않는다 (AMCL 은 원본을 그대로 씀). A* 는 실행 중에 직접 가공하므로 이 파일을 map_server 에 줄 필요는 없다.

    python3 tools/plan_map.py ~/maps/hall_clean.yaml [--inflate 0.1] [--stripe 2] [--max-size 0.4]
           [--check X1 Y1 X2 Y2] ...      두 점 사이 경로가 가공 전/후 모두 있는지 (홈 ↔ LED 위치 등)
           [-o 출력경로]                  가공한 맵을 pgm/yaml 로도 저장 (확인용)

비교 이미지(<맵>_plan_preview.png): 왼쪽 원본, 오른쪽 가공본 (빨강 = 다리 점을 부풀려 막은 칸, 하늘색 = 메운 줄무늬)
값을 바꿔 보고 마음에 들면 ros2_ws/src/warehouse_robot/config/params.yaml 의 astar_planner 값(stripe_cells,
cluster_inflate, cluster_max_size)을 같게 바꾸고 브리지를 다시 실행하세요.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'warehouse_robot'))
from clean_map import read_pgm, read_yaml, write_pgm  # noqa: E402
from warehouse_robot.astar import GridPlanner, PlanningError  # noqa: E402
from warehouse_robot.plan_map import prepare, with_added  # noqa: E402

ROBOT_RADIUS = 0.25   # params.yaml astar_planner.robot_radius


def load(yaml_path):
    conf = read_yaml(yaml_path)
    img = read_pgm(os.path.join(os.path.dirname(yaml_path), conf['image'].strip('\'"')))
    if conf.get('negate', '0') == '1':
        img = 255 - img
    occ_t = float(conf.get('occupied_thresh', 0.65))
    free_t = float(conf.get('free_thresh', 0.196))
    p = (255 - img.astype(float)) / 255.0       # map_server trinary 방식
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[p > occ_t] = 100
    grid[p < free_t] = 0
    origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
    return grid[::-1].copy(), float(conf['resolution']), origin, img   # 행 0 = 아래쪽 (OccupancyGrid 와 같게)


def to_img(grid):
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == 0] = 254
    img[grid >= 65] = 0
    return img[::-1]


def try_plan(grid, res, origin, a, b):
    h, w = grid.shape
    p = GridPlanner(grid.ravel(), w, h, res, origin[0], origin[1], robot_radius=ROBOT_RADIUS)
    try:
        pts = p.plan(a, b)
    except PlanningError as e:
        return None, str(e)
    return sum(np.hypot(q[0] - p_[0], q[1] - p_[1]) for p_, q in zip(pts, pts[1:])), ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('--inflate', type=float, default=0.1, help='다리 점 부풀리는 거리 m (params cluster_inflate)')
    ap.add_argument('--stripe', type=int, default=2, help='줄무늬 메우기 칸 수 (params stripe_cells)')
    ap.add_argument('--max-size', type=float, default=0.4, help='가로/세로가 이보다 긴 덩어리(벽)는 부풀리지 않음 m (params cluster_max_size)')
    ap.add_argument('--check', nargs=4, type=float, action='append', default=[], metavar=('X1', 'Y1', 'X2', 'Y2'),
                    help='두 점 사이 경로 확인 (여러 번 가능)')
    ap.add_argument('-o', '--output', help='가공한 맵 저장 경로(확장자 제외)')
    args = ap.parse_args()

    yaml_path = os.path.expanduser(args.yaml_path)
    grid, res, origin, img = load(yaml_path)
    filled, added, stats = prepare(grid, res, args.stripe, args.inflate, args.max_size)
    planned = with_added(filled, added, res, origin[0], origin[1])
    for k, v in stats.items():
        print(f'{k}: {v}칸 ({v * res * res:.2f} m²)')
    print(f'빈 칸 {int((grid == 0).sum())} → {int((planned == 0).sum())}')

    for x1, y1, x2, y2 in args.check:
        before, _ = try_plan(grid, res, origin, (x1, y1), (x2, y2))
        after, why = try_plan(planned, res, origin, (x1, y1), (x2, y2))
        b = f'{before:.2f} m' if before is not None else '없음'
        a = f'{after:.2f} m' if after is not None else f'없음 ({why})'
        print(f'경로 ({x1:.2f}, {y1:.2f}) → ({x2:.2f}, {y2:.2f}): 가공 전 {b}, 가공 후 {a}')

    base = os.path.splitext(yaml_path)[0]
    if args.output:
        out = os.path.expanduser(args.output)
        write_pgm(out + '.pgm', to_img(planned))
        with open(out + '.yaml', 'w', encoding='utf-8') as f:
            f.write(f'image: {os.path.basename(out)}.pgm\nmode: trinary\nresolution: {res}\n'
                    f'origin: [{origin[0]}, {origin[1]}, {origin[2] if len(origin) > 2 else 0}]\n'
                    f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        print(f'저장: {out}.yaml (확인용. run.sh --map 에는 원본을 주세요)')
    try:
        from PIL import Image
    except ImportError:
        return
    rgb = np.stack([to_img(planned)] * 3, axis=-1)
    rgb[added[::-1]] = (220, 38, 38)
    rgb[((filled == 0) & (grid < 0))[::-1]] = (125, 211, 252)
    left = np.stack([img] * 3, axis=-1)
    gap = np.full((img.shape[0], 4, 3), 128, dtype=np.uint8)
    both = np.hstack([left, gap, rgb])
    path = base + '_plan_preview.png'
    Image.fromarray(both).resize((both.shape[1] * 3, both.shape[0] * 3), Image.NEAREST).save(path)
    print(f'비교 이미지: {path} (왼쪽 원본, 오른쪽 가공본: 빨강 = 부풀려 막은 칸, 하늘색 = 메운 줄무늬)')


if __name__ == '__main__':
    main()
