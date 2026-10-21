"""강당 월드(lecture_hall) 만들기: 위쪽 강단(공터) + 가로 복도 6 × 세로 복도 5, 가로 복도 사이 좌석 줄에 칸마다 책상 1개 + 의자 2개.

    python3 tools/make_hall_world.py [-o worlds/lecture_hall.world]
    python3 tools/make_hall_world.py --real        # 실제 학원과 비슷하게 → worlds/lecture_hall_real.world
           (= --no-panel --jitter 0.12 --yaw-jitter 15 --seed 1 --name lecture_hall_real)

실제 학원과 다른 점 (lecture_hall 기본): 책상 앞판이 LiDAR 에 막힌 선으로 보이고, 의자가 줄을 맞춰 놓여 있다.
  --no-panel     책상 앞판 없음 → 실물처럼 좌석 줄이 다리 점으로만 보임
  --jitter M     의자를 ±M m 안에서 무작위로 옮김 (사람이 쓰고 난 뒤처럼), --yaw-jitter 도(°) 만큼 돌림
  --seed N       무작위 배치 번호 (같은 번호면 같은 배치)

- 복도 0.9m, 좌석 줄(가로 복도 사이 띠) 1.0m, 세로 복도 사이 칸 1.4m, 강단 깊이 2.0m → 실내 10.1 x 12.4m
- 좌석 줄 = 북쪽(강단 쪽) 책상 + 남쪽 의자 2개 (의자는 강단을 바라봄)
- 책상 1.15 x 0.4m, 높이 0.72m: 다리 4개 + 상판 + 앞판(바닥 0.1m ~ 0.7m, LiDAR 에 막힌 선으로 보임 → 좌석 줄이 빈 공간으로 안 보임)
- 의자 0.45m: 다리 4개(지름 5cm, LiDAR 에 점 4개로 보임) + 상판(높이 0.45m, LiDAR/로봇보다 높음) + 등받이
- 칸 안 의자 두 개 사이 틈 0.25m: 보이지만 로봇(폭 0.31m)은 못 지나감
- 원점(0,0) = 가로 6(맨 아래) / 세로 1(맨 왼쪽) 교차점 가운데 = 로봇 출발 위치 (start_sim.sh/run.sh 기본값)
"""
import argparse
import math
import os
import random

L = 0.9       # 복도 폭
B = 1.0       # 좌석 줄 폭 (책상 + 의자)
S = 1.4       # 세로 복도 사이 칸 길이
C = 0.45      # 의자 크기
G = 0.25      # 의자 사이 틈
LEG_R = 0.025
SEAT_Z = 0.45
STAGE = 2.0
DESK_D, DESK_Z = 0.4, 0.72   # 책상 깊이, 높이 (길이는 의자 두 개 + 틈 = 1.15m)
PANEL_Z0 = 0.1               # 앞판 아래 끝 (LiDAR 높이 약 0.17m 보다 낮게)
EDGE = 0.05                  # 좌석 줄 가장자리와 책상/의자 사이 여백
NH, NV = 6, 5
WALL_T, WALL_H = 0.1, 0.5

W = NV * L + (NV - 1) * S        # 10.1
HA = NH * L + (NH - 1) * B       # 좌석 구역 10.4
H = HA + STAGE                   # 12.4
X0, Y0 = -L / 2, -L / 2          # 실내 왼쪽 아래 모서리 (원점이 출발 교차점 가운데가 되게)

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


def walls():
    cx, cy = X0 + W / 2, Y0 + H / 2
    z = WALL_H / 2
    body = part('s', (cx, Y0 - WALL_T / 2, z), box(W + 2 * WALL_T, WALL_T, WALL_H), GRAY)
    body += part('n', (cx, Y0 + H + WALL_T / 2, z), box(W + 2 * WALL_T, WALL_T, WALL_H), GRAY)
    body += part('w', (X0 - WALL_T / 2, cy, z), box(WALL_T, H, WALL_H), GRAY)
    body += part('e', (X0 + W + WALL_T / 2, cy, z), box(WALL_T, H, WALL_H), GRAY)
    # 강단 바닥 표시 (얇은 색칠, 충돌 없음)
    body += part('stage', (cx, Y0 + HA + STAGE / 2, 0.002), box(W, STAGE, 0.004), STAGE_MAT, collide=False)
    return f'''    <model name="walls">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
'''


def chair(name, x, y, back_north, yaw=0.0):
    """다리 4개 + 상판 + 등받이. back_north: 등받이가 북쪽(강단 반대가 앞) = 강단을 바라봄. yaw: 돌린 각도(rad)."""
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
    """다리 4개 + 상판 + 앞판(북쪽, 강단 쪽, panel=False 면 없음). (x, y) = 책상 가운데."""
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


def seats(panel=True, jitter=0.0, yaw_jitter=0.0, seed=0):
    """좌석 줄마다, 세로 복도 사이 칸마다 책상 1개(북쪽) + 의자 2개(남쪽). 반환: (sdf, 책상 수, 의자 수)
    jitter/yaw_jitter(rad): 의자를 무작위로 옮기고 돌림. 책상 쪽(북쪽)으로는 앞 다리가 책상에 닿지 않을 만큼만."""
    rnd = random.Random(seed)
    out, nd, nc = [], 0, 0
    for i in range(NH - 1):                       # 가로 i+1 과 i+2 사이 좌석 줄
        top = Y0 + HA - L - i * (L + B)           # 좌석 줄 북쪽 끝
        bottom = top - B
        desk_y = top - EDGE - DESK_D / 2
        chair_y = bottom + EDGE + C / 2
        for j in range(NV - 1):                   # 세로 j+1 과 j+2 사이 칸
            xc = X0 + j * (L + S) + L + S / 2
            out.append(desk(f'desk_{i + 1}_{j + 1}', xc, desk_y, panel))
            nd += 1
            for k, off in enumerate((-(C + G) / 2, (C + G) / 2)):
                # 의자는 강단을 바라본다: 등받이가 남쪽(로봇 출발 쪽)
                dx = rnd.uniform(-jitter, jitter)
                dy = rnd.uniform(-jitter, min(jitter, 0.03))   # 책상 쪽으로는 거의 안 밀림
                out.append(chair(f'chair_{i + 1}_{j + 1}_{k + 1}', xc + off + dx, chair_y + dy, back_north=True,
                                 yaw=rnd.uniform(-yaw_jitter, yaw_jitter)))
                nc += 1
    return ''.join(out), nd, nc


def world_sdf(name='lecture_hall', panel=True, jitter=0.0, yaw_jitter=0.0, seed=0):
    body, nd, n = seats(panel, jitter, yaw_jitter, seed)
    extra = '' if panel and not jitter else (
        f'\n     실물처럼: 책상 앞판 {"있음" if panel else "없음"}, 의자 ±{jitter:.2f}m / ±{math.degrees(yaw_jitter):.0f}° 무작위 (seed {seed})')
    return f'''<?xml version="1.0"?>
<!-- tools/make_hall_world.py 로 만든 월드: {name}
     강단(위, 공터 {STAGE}m) + 가로 복도 {NH} x 세로 복도 {NV} (폭 {L}m), 가로 복도 사이 좌석 줄({B}m)에 칸마다 책상 1개 + 의자 2개 (책상 {nd}개, 의자 {n}개)
     실내 {W:.1f} x {H:.1f}m, 원점 = 가로 {NH} / 세로 1 교차점 (로봇 출발 위치){extra} -->
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
{walls()}{body}  </world>
</sdf>
''', nd, n


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output')
    ap.add_argument('--name', default='lecture_hall', help='월드 이름 (기본 출력 worlds/<이름>.world)')
    ap.add_argument('--no-panel', action='store_true', help='책상 앞판 없음 (실물처럼 다리만)')
    ap.add_argument('--jitter', type=float, default=0.0, help='의자 위치 무작위 범위 ±m')
    ap.add_argument('--yaw-jitter', type=float, default=0.0, help='의자 방향 무작위 범위 ±도')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--real', action='store_true',
                    help='실제 학원처럼: --no-panel --jitter 0.12 --yaw-jitter 15 --seed 1 --name lecture_hall_real')
    args = ap.parse_args()
    if args.real:
        args.no_panel, args.jitter, args.yaw_jitter = True, 0.12, 15.0
        args.seed = args.seed or 1
        args.name = 'lecture_hall_real' if args.name == 'lecture_hall' else args.name
    if not args.output:
        args.output = os.path.join(here, '..', 'worlds', f'{args.name}.world')
    sdf, nd, n = world_sdf(args.name, not args.no_panel, args.jitter, math.radians(args.yaw_jitter), args.seed)
    with open(args.output, 'w') as f:
        f.write(sdf)
    print(f'{os.path.normpath(args.output)}: 실내 {W:.1f} x {H:.1f}m, 책상 {nd}개, 의자 {n}개, '
          f'x {X0:.2f}~{X0 + W:.2f}, y {Y0:.2f}~{Y0 + H:.2f}')


if __name__ == '__main__':
    main()
