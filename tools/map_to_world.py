"""Turn the walls of a cleaned map (pgm + yaml) into a Gazebo world (.world), so the real room can be tested identically in simulation.

    python3 tools/map_to_world.py ~/maps/backup/backup2/maze_map_straight.yaml -o worlds/maze_room.world

- Covers wall (occupied) cells with a few rectangles (runs within a row -> merged downward while the run is identical) and makes each a box
- World coordinates = map coordinates (map origin (0,0) is the world origin). The robot start pose can use map coordinates as is
- A straightened map (output of straighten_map.py) gives fewer boxes and straight walls
"""
import argparse
import os

import numpy as np

from clean_map import OCC, read_pgm, read_yaml


def cover_rectangles(occ):
    """Cover occupied cells with non-overlapping rectangles. Returns: [(r0, c0, r1, c1), ...] (inclusive)"""
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
    <!-- 모델 위치를 서비스로 바꿀 수 있게 (/set_entity_state): tools/sim_mover.py 로 움직이는 장애물 시험 -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
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
        # image row 0 = top of the map (max y)
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
