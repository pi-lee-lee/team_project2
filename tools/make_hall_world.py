"""Build the lecture hall world (lecture_hall): stage (open area) at the top + 6 horizontal x 5 vertical aisles, with 1 desk + 2 chairs per cell in the seat rows between horizontal aisles.

    python3 tools/make_hall_world.py [-o worlds/lecture_hall.world]
    python3 tools/make_hall_world.py --real        # similar to the real classroom -> worlds/lecture_hall_real.world
           (= --no-panel --no-side-aisles --row-gap 0.25 --jitter 0.12 --yaw-jitter 15 --seed 1 --name lecture_hall_real)

Differences from the real classroom (lecture_hall default): desk front panels show up as solid lines to the LiDAR, and chairs are neatly aligned.
  --no-panel     no desk front panel -> seat rows appear only as leg points, like the real thing
  --no-side-aisles  no leftmost/rightmost vertical aisle -> the end-column seats touch the walls (real classroom). Interior width 8.3 m,
                 origin (start pose) = intersection of horizontal 6 / vertical 2
  --row-gap M    gap between seat rows (chair back ~ next row's desk) of M m -> if narrower than 0.31 m (robot width) the robot cannot
                 pass between seat rows and only uses vertical aisles and the front (stage side)/back (start) horizontal aisles (real classroom). Front/back horizontal aisles stay 0.9 m
  --jitter M     randomly move chairs within ±M m (as if people had used them), rotate by --yaw-jitter degrees
  --seed N       random layout number (same number gives the same layout)

- Aisles 0.9 m, seat rows (bands between horizontal aisles) 1.0 m, cells between vertical aisles 1.4 m, stage depth 2.0 m -> interior 10.1 x 12.4 m
- Seat row = desk on the north (stage) side + 2 chairs on the south side (chairs face the stage)
- Desk 1.15 x 0.4 m, height 0.72 m: 4 legs + top + front panel (0.1 m ~ 0.7 m above the floor, seen by the LiDAR as a solid line -> seat rows don't look like free space)
- Chair 0.45 m: 4 legs (5 cm diameter, seen by the LiDAR as 4 points) + seat (0.45 m high, above the LiDAR/robot) + backrest
- Gap between the two chairs in a cell 0.25 m: visible, but the robot (0.31 m wide) cannot pass
- Origin (0,0) = center of the intersection of horizontal 6 (bottom) / vertical 1 (leftmost) = robot start pose (start_sim.sh/run.sh default)
"""
import argparse
import math
import os
import random

L = 0.9       # aisle width
B = 1.0       # seat row width (desk + chairs)
S = 1.4       # cell length between vertical aisles
C = 0.45      # chair size
G = 0.25      # gap between chairs
LEG_R = 0.025
SEAT_Z = 0.45
STAGE = 2.0
DESK_D, DESK_Z = 0.4, 0.72   # desk depth, height (length = two chairs + gap = 1.15 m)
PANEL_Z0 = 0.1               # bottom edge of the front panel (below the LiDAR height of about 0.17 m)
EDGE = 0.05                  # margin between the seat row edge and desk/chairs
NH, NV = 6, 5
WALL_T, WALL_H = 0.1, 0.5

W = NV * L + (NV - 1) * S        # 10.1
HA = NH * L + (NH - 1) * B       # seating area 10.4
H = HA + STAGE                   # 12.4
X0, Y0 = -L / 2, -L / 2          # interior bottom-left corner (so the origin is the center of the start intersection)


def layout(row_gap=None):
    """(horizontal aisle widths [top (stage side) -> bottom], seating area height, interior height).
    row_gap: free gap between seat rows (chair back ~ next row's desk, m). Chairs/desks are inset by EDGE from the seat row edges, so
    the band between rows = row_gap - 2 x EDGE. None means all horizontal aisles are L."""
    aisles = [L] * NH
    if row_gap is not None:
        aisles[1:-1] = [max(0.0, row_gap - 2 * EDGE)] * (NH - 2)
    ha = sum(aisles) + (NH - 1) * B
    return aisles, ha, ha + STAGE


def room(side_aisles=True):
    """(seat column reference x0, interior left edge x, interior width, start vertical aisle number).
    side_aisles=False: removes the leftmost/rightmost vertical aisles so the end-column seats touch the walls. The origin is moved onto vertical 2."""
    if side_aisles:
        return X0, X0, W, 1
    x0 = -(L + S + L / 2)
    return x0, x0 + L, W - 2 * L, 2

GRAY = '<material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>'
WOOD = '<material><ambient>0.45 0.25 0.1 1</ambient><diffuse>0.55 0.3 0.12 1</diffuse></material>'
STAGE_MAT = '<material><ambient>0.5 0.4 0.6 1</ambient><diffuse>0.6 0.5 0.7 1</diffuse></material>'


def part(name, pose, geom, material, collide=True):
    x, y, z = pose
    p = f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
    out = ''
    if collide:
        out += f'''        <collision name="c_{name}">
          {p}
          <geometry>{geom}</geometry>
        </collision>
'''
    out += f'''        <visual name="v_{name}">
          {p}
          <geometry>{geom}</geometry>
          {material}
        </visual>
'''
    return out


def box(sx, sy, sz):
    return f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'


def walls(xl=X0, w=W, ha=HA, h=H):
    cx, cy = xl + w / 2, Y0 + h / 2
    z = WALL_H / 2
    body = part('s', (cx, Y0 - WALL_T / 2, z), box(w + 2 * WALL_T, WALL_T, WALL_H), GRAY)
    body += part('n', (cx, Y0 + h + WALL_T / 2, z), box(w + 2 * WALL_T, WALL_T, WALL_H), GRAY)
    body += part('w', (xl - WALL_T / 2, cy, z), box(WALL_T, h, WALL_H), GRAY)
    body += part('e', (xl + w + WALL_T / 2, cy, z), box(WALL_T, h, WALL_H), GRAY)
    # stage floor marking (thin colored layer, no collision)
    body += part('stage', (cx, Y0 + ha + STAGE / 2, 0.002), box(w, STAGE, 0.004), STAGE_MAT, collide=False)
    return f'''    <model name="walls">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
'''


def chair(name, x, y, back_north, yaw=0.0):
    """4 legs + seat + backrest. back_north: backrest to the north (front faces away from the stage) = faces the stage. yaw: rotation (rad)."""
    h = C / 2 - LEG_R
    body = ''
    for i, (dx, dy) in enumerate(((-h, -h), (-h, h), (h, -h), (h, h))):
        body += part(f'leg{i}', (dx, dy, SEAT_Z / 2),
                     f'<cylinder><radius>{LEG_R}</radius><length>{SEAT_Z:.3f}</length></cylinder>', WOOD)
    body += part('seat', (0, 0, SEAT_Z + 0.015), box(C, C, 0.03), WOOD)
    by = -(C / 2 - 0.015) if back_north else (C / 2 - 0.015)
    body += part('back', (0, by, SEAT_Z + 0.03 + 0.2), box(C, 0.03, 0.4), WOOD)
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {round(yaw, 3):g}</pose>
      <link name="link">
{body}      </link>
    </model>
'''


def desk(name, x, y, panel=True):
    """4 legs + top + front panel (north, stage side; omitted if panel=False). (x, y) = desk center."""
    length = 2 * C + G
    hx, hy = length / 2 - LEG_R, DESK_D / 2 - LEG_R
    legz = DESK_Z - 0.03
    body = ''
    for i, (dx, dy) in enumerate(((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy))):
        body += part(f'leg{i}', (dx, dy, legz / 2),
                     f'<cylinder><radius>{LEG_R}</radius><length>{legz:.3f}</length></cylinder>', WOOD)
    body += part('top', (0, 0, DESK_Z - 0.015), box(length, DESK_D, 0.03), WOOD)
    pz = (DESK_Z - 0.03 - PANEL_Z0)
    if panel:
        body += part('panel', (0, DESK_D / 2 - 0.01, PANEL_Z0 + pz / 2), box(length - 2 * LEG_R, 0.02, pz), WOOD)
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
{body}      </link>
    </model>
'''


def seats(panel=True, jitter=0.0, yaw_jitter=0.0, seed=0, x0=X0, row_gap=None):
    """For each seat row and each cell between vertical aisles: 1 desk (north) + 2 chairs (south). Returns: (sdf, desk count, chair count)
    jitter/yaw_jitter(rad): randomly move and rotate chairs. Toward the desk (north) only as far as the front legs stay clear of the desk."""
    rnd = random.Random(seed)
    out, nd, nc = [], 0, 0
    aisles, ha, _ = layout(row_gap)
    # keep chairs pushed south from overlapping the next row's desk (with narrow row gaps)
    south = min(jitter, aisles[1] + 2 * EDGE - 0.02) if row_gap is not None else jitter
    top = Y0 + ha - aisles[0]
    for i in range(NH - 1):                       # seat row between horizontal i+1 and i+2
        bottom = top - B
        desk_y = top - EDGE - DESK_D / 2
        chair_y = bottom + EDGE + C / 2
        for j in range(NV - 1):                   # cell between vertical j+1 and j+2
            xc = x0 + j * (L + S) + L + S / 2
            out.append(desk(f'desk_{i + 1}_{j + 1}', xc, desk_y, panel))
            nd += 1
            for k, off in enumerate((-(C + G) / 2, (C + G) / 2)):
                # chairs face the stage: backrest to the south (robot start side)
                dx = rnd.uniform(-jitter, jitter)
                dy = rnd.uniform(-south, min(jitter, 0.03))   # barely moved toward the desk
                out.append(chair(f'chair_{i + 1}_{j + 1}_{k + 1}', xc + off + dx, chair_y + dy, back_north=True,
                                 yaw=rnd.uniform(-yaw_jitter, yaw_jitter)))
                nc += 1
        top = bottom - aisles[i + 1]
    return ''.join(out), nd, nc


def world_sdf(name='lecture_hall', panel=True, jitter=0.0, yaw_jitter=0.0, seed=0, side_aisles=True, row_gap=None):
    x0, xl, w, start_col = room(side_aisles)
    _, ha, h = layout(row_gap)
    body, nd, n = seats(panel, jitter, yaw_jitter, seed, x0, row_gap)
    extra = '' if panel and not jitter and side_aisles and row_gap is None else (
        f'\n     실물처럼: 책상 앞판 {"있음" if panel else "없음"}, 좌우 끝 세로 복도 {"있음" if side_aisles else "없음(끝 열이 벽에 붙음)"}, '
        + (f'좌석 줄 사이 빈 틈 {row_gap:.2f}m (로봇 폭 0.31m 보다 좁아 못 지나감, 맨 앞/뒤 가로 복도만 {L}m), ' if row_gap is not None else '')
        + f'의자 ±{jitter:.2f}m / ±{math.degrees(yaw_jitter):.0f}° 무작위 (seed {seed})')
    return f'''<?xml version="1.0"?>
<!-- tools/make_hall_world.py 로 만든 월드: {name}
     강단(위, 공터 {STAGE}m) + 가로 복도 {NH} x 세로 복도 {NV if side_aisles else NV - 2} (폭 {L}m), 가로 복도 사이 좌석 줄({B}m)에 칸마다 책상 1개 + 의자 2개 (책상 {nd}개, 의자 {n}개)
     실내 {w:.1f} x {h:.1f}m, 원점 = 가로 {NH} / 세로 {start_col} 교차점 (로봇 출발 위치){extra} -->
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
{walls(xl, w, ha, h)}{body}  </world>
</sdf>
''', nd, n


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output')
    ap.add_argument('--name', default='lecture_hall', help='월드 이름 (기본 출력 worlds/<이름>.world)')
    ap.add_argument('--no-panel', action='store_true', help='책상 앞판 없음 (실물처럼 다리만)')
    ap.add_argument('--no-side-aisles', action='store_true', help='맨 왼쪽/오른쪽 세로 복도 없음 (끝 열 좌석이 벽에 붙음)')
    ap.add_argument('--row-gap', type=float, default=None,
                    help='좌석 줄 사이 빈 틈 m (의자 등받이 ~ 다음 줄 책상). 0.31 보다 작으면 로봇이 못 지나감')
    ap.add_argument('--jitter', type=float, default=0.0, help='의자 위치 무작위 범위 ±m')
    ap.add_argument('--yaw-jitter', type=float, default=0.0, help='의자 방향 무작위 범위 ±도')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--real', action='store_true',
                    help='실제 학원처럼: --no-panel --no-side-aisles --row-gap 0.25 --jitter 0.12 --yaw-jitter 15 --seed 1 '
                         '--name lecture_hall_real')
    args = ap.parse_args()
    if args.real:
        args.no_panel, args.no_side_aisles, args.jitter, args.yaw_jitter = True, True, 0.12, 15.0
        args.row_gap = 0.25 if args.row_gap is None else args.row_gap
        args.seed = args.seed or 1
        args.name = 'lecture_hall_real' if args.name == 'lecture_hall' else args.name
    if not args.output:
        args.output = os.path.join(here, '..', 'worlds', f'{args.name}.world')
    sdf, nd, n = world_sdf(args.name, not args.no_panel, args.jitter, math.radians(args.yaw_jitter), args.seed,
                           not args.no_side_aisles, args.row_gap)
    _, xl, w, _ = room(not args.no_side_aisles)
    _, _, h = layout(args.row_gap)
    with open(args.output, 'w') as f:
        f.write(sdf)
    print(f'{os.path.normpath(args.output)}: 실내 {w:.1f} x {h:.1f}m, 책상 {nd}개, 의자 {n}개, '
          f'x {xl:.2f}~{xl + w:.2f}, y {Y0:.2f}~{Y0 + h:.2f}')


if __name__ == '__main__':
    main()
