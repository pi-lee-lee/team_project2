"""정리된 맵(pgm + yaml)의 벽을 Gazebo 월드(.world)로 만든다. 실제 방을 시뮬레이션에서 똑같이 시험하기 위함.

    python3 tools/map_to_world.py ~/maps/backup/backup2/maze_map_straight.yaml -o worlds/maze_room.world

- 벽 칸(점유)을 직사각형 몇 개로 덮어서(같은 행의 연속 구간 → 아래로 같은 구간이면 합침) 각각 상자로 만든다
- 월드 좌표 = 맵 좌표 (맵 원점 (0,0) 이 월드 원점). 로봇 출발 위치도 맵 좌표 그대로 쓰면 된다
- 직선으로 정리된 맵(straighten_map.py 결과)을 쓰면 상자 수가 적고 벽이 반듯하다
"""
import argparse
import os

import numpy as np

from clean_map import OCC, read_pgm, read_yaml


def cover_rectangles(occ):
    """점유 칸을 겹치지 않는 직사각형들로 덮는다. 반환: [(r0, c0, r1, c1), ...] (끝 포함)"""
    h, w = occ.shape
    used = np.zeros_like(occ)
    rects = []
    for r in range(h):
        c = 0
        while c < w:
            if not occ[r, c] or used[r, c]:
                c += 1
                continue
            c1 = c
            while c1 + 1 < w and occ[r, c1 + 1] and not used[r, c1 + 1]:
                c1 += 1
            r1 = r
            while r1 + 1 < h and occ[r1 + 1, c:c1 + 1].all() and not used[r1 + 1, c:c1 + 1].any():
                r1 += 1
            used[r:r1 + 1, c:c1 + 1] = True
            rects.append((r, c, r1, c1))
            c = c1 + 1
    return rects


def world_sdf(boxes, height, name):
    links = []
    for i, (x, y, sx, sy) in enumerate(boxes):
        geom = f'<box><size>{sx:.3f} {sy:.3f} {height:.3f}</size></box>'
        links.append(f'''        <collision name="c{i}">
          <pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose>
          <geometry>{geom}</geometry>
        </collision>
        <visual name="v{i}">
          <pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose>
          <geometry>{geom}</geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>''')
    body = '\n'.join(links)
    return f'''<?xml version="1.0"?>
<!-- tools/map_to_world.py 로 맵에서 만든 월드: {name} -->
<sdf version="1.6">
  <world name="{name}">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <scene>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <model name="walls">
      <static>true</static>
      <link name="link">
{body}
      </link>
    </model>
  </world>
</sdf>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('-o', '--output', required=True, help='출력 .world 파일')
    ap.add_argument('--height', type=float, default=0.4, help='벽 높이 (m)')
    args = ap.parse_args()

    yaml_path = os.path.expanduser(args.yaml_path)
    conf = read_yaml(yaml_path)
    res = float(conf['resolution'])
    ox, oy = [float(v) for v in conf['origin'].strip('[]').split(',')][:2]
    img = read_pgm(os.path.join(os.path.dirname(yaml_path), conf['image'].strip('\'"')))
    h = img.shape[0]

    rects = cover_rectangles(img == OCC)
    boxes = []
    for r0, c0, r1, c1 in rects:
        # 이미지 행 0 = 맵 위쪽(y 최대)
        x0, x1 = ox + c0 * res, ox + (c1 + 1) * res
        y0, y1 = oy + (h - 1 - r1) * res, oy + (h - r0) * res
        boxes.append(((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0))

    name = os.path.splitext(os.path.basename(args.output))[0]
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(world_sdf(boxes, args.height, name))
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    print(f'벽 칸 {int((img == OCC).sum())}개 → 상자 {len(boxes)}개, 범위 x {min(xs):.2f}~{max(xs):.2f}, '
          f'y {min(ys):.2f}~{max(ys):.2f} m, 높이 {args.height} m')
    print(f'저장: {args.output}')


if __name__ == '__main__':
    main()
