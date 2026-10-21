# 현재 상태 기록

기록 시각: 2026-10-19 14:07
실행 방법은 [README.md](README.md), 기획은 [PROJECT_PLAN.md](PROJECT_PLAN.md) 참고.

---

## 1. 한 줄 요약

웹에서 지점을 찍으면 실제 TurtleBot3 Waffle Pi가 **직접 구현한 A* 경로 + 직접 구현한 pure pursuit 추종**으로 이동하는 것까지 확인됨.
작업 흐름(적재 확인 → 다음 위치 → 홈 복귀)은 코드는 있으나 실제 로봇 전체 시험은 아직.

## 2. 지금 실행 중인 것

| 장비 | 프로세스 | 상태 |
|---|---|---|
| Pi (192.168.0.181) | `turtlebot3_bringup robot.launch.py` | 실행 중, OpenCR 오류 0건 |
| PC (192.168.0.188) | Django 서버 (`scripts/start_server.sh`, :8000) | 실행 중 |
| PC | Nav2 위치 추정 map_server + amcl (`scripts/start_nav2.sh`) | 실행 중 |
| PC | A* 플래너 + 브리지 (`scripts/start_bridge.sh`) | 실행 중, 서버 연결됨 |
| PC | MySQL `warehouse` DB | 실행 중 |
| PC | Firefox 웹 화면 (http://localhost:8000/) | 열림 |

- 로봇 상태: 대기(IDLE), 배터리 11.3V, 원래 출발점 근처 (0.21, 0.05)
- 활성 맵: `maze_map`, 파일은 `~/maps/maze_map_straight.yaml` (73×91, 9번 참고)
- 로그: `logs/server.log`, `logs/nav2.log`, `logs/bridge.log`, Pi `~/warehouse_bringup.log`

## 3. 사용 중인 알고리즘

### 경로 계획: A* (`ros2_ws/src/warehouse_robot/warehouse_robot/astar.py`)
- 장애물(점유값 ≥ 65, 미탐색 영역 포함)까지 거리 변환
- 0.25m(로봇 반경, 뒤쪽 모서리까지) 이내 통과 불가, 0.25~0.55m는 벽에 가까울수록 비용 증가 → 통로 가운데로 경로
  (2026-10-20 에 0.22 → 0.25: 제자리 회전 때 뒤쪽 모서리가 닿을 수 있었음)
- AMCL 모드: LiDAR 로 본, 맵에 없는 장애물을 20초 기억해 함께 피함. 부딪힌 자리(LiDAR 에 안 보이는 물체)는 120초 기억
- SLAM 모드: 아직 맵 밖인 목표도 미탐색 칸으로 넓혀서 계획
- 8방향 이동(직선 1, 대각선 √2 × 비용 배율), octile 휴리스틱, 장애물 모서리 대각선 통과 금지
- 목표가 벽에 붙어 있으면 0.3m 안의 가장 가까운 빈 칸으로 이동
- 이동 평균으로 경로 다듬기, 마지막 점에 도착 방향 지정

### 경로 추종: Pure Pursuit (`follower.py`)
- 0.3m 앞 경로점을 목표로 곡률 2·sin(α)/거리 로 각속도 결정
- 방향 차이 > 0.8 rad 이면 제자리 회전
- 최고 0.15 m/s, 방향 차이·목표 거리에 따라 감속
- 목표 8cm 이내 → 정지 후 도착 방향(±0.1 rad) 맞춤

### 위치 추정: Nav2 AMCL (이 프로젝트 설정 `config/amcl_waffle_pi.yaml`)
- 초기 위치는 웹 "초기 위치 지정" 또는 `tools/localize.py` (스캔-맵 전수 탐색)
- AMCL/SLAM 은 브리지가 넘겨주는 수평 스캔(`scan_level`)만 사용 (부딪혀 기울면 바닥을 벽으로 봄)

### 안전장치 (`bridge_node.py`)
- 앞쪽 ±25° 0.35m 안 물체 → 정지 후 대기, 5초 넘으면 장애물을 피하는 경로로 재계획
- 경로 이탈 0.5m / 15초 진행 없음 → 재계획 (최대 2회) → 실패
- 위치 TF 0.5초 이상 갱신 없음 → 즉시 정지
- 부딪힘(평소보다 6° 넘게 기울거나 2.5초 동안 못 나아감) → 정지, 들린 쪽 반대로 15cm 빠져나옴, 부딪힌 자리 기억 후 재계획.
  한 이동에서 두 번 부딪히면 멈추고 확인 요청. 기운 채로는 출발하지 않음

## 4. 확인된 동작 (실제 로봇)

| 항목 | 결과 |
|---|---|
| cmd_vel 직접 주행 | 10cm·80cm 전진/후진, 복귀 오차 1cm 이내 |
| 위치 추정 | 스캔 점 96~100%가 벽과 20cm 이내 |
| 웹 "여기로 이동" | 0.45m 전진: 4.9초, 오차 9cm |
| | 원위치 복귀(180° 회전 포함): 14초, 오차 7cm |
| 단위 테스트 | A* 6개 + 추종기 6개 통과 |

## 5. 아직 확인 안 한 것

- [ ] 여러 위치 작업 전체 흐름: 물류 위치 등록 → 작업 시작 → 적재 완료 → 다음 위치 → 홈 복귀 (홈 복귀 단독은 확인됨)
- [ ] 주행 중 앞쪽 장애물 정지/대기, 막힌 뒤 재계획
- [ ] 긴 거리 이동, 좁은 통로 통과
- [ ] 위치 TF 끊김 시 정지 (일부러 끊어 보는 시험)
- [ ] 다른 기기(휴대폰)에서 웹 접속

## 6. 알려진 문제

| 문제 | 현재 대응 | 비고 |
|---|---|---|
| Nav2 controller_server 안에서 TF 수신이 멈춤 → 목표를 지나쳐 계속 주행(2.8m까지, 비상정지로 멈춤) / 안 움직이고 "도착" 처리 | controller_server 사용 안 함, 직접 구현한 추종기 사용 | 근본 원인 미확인. Fast DDS SHM 정리, UDP 전용, Cyclone DDS, 노드 분리, 실행 순서 변경으로도 해결 안 됨 |
| ~~A* 맵이 정적 맵이라 새 장애물을 피해 돌아가지 못함~~ | 2026-10-20 해결: LiDAR 장애물을 20초 기억해 우회 (sim 확인) | |
| LiDAR(높이 약 13cm)보다 낮은 물체는 보이지 않음 | 부딪힌 뒤 감지 → 물러남 → 두 번이면 멈추고 확인 요청 | sim 에서 8cm 상자에 올라타 걸리는 일이 남음. 바닥 정리(운영 규칙) 또는 범퍼/깊이 카메라 필요 |
| Pi `turtlebot3_node`가 OpenCR 통신 오류 후 종료된 적 있음 (로봇을 옮길 때) | bringup 재실행 | 이후 재발 없음 |
| 마우스가 두 번 눌려 명령이 중복 | 웹에서 0.8초 안의 같은 명령 무시 | |

## 7. 오늘 바뀐 환경 설정

- ROS_DOMAIN_ID: PC 16 → **44**, Pi 226 → **44** (`~/.bashrc`)
  - 원래 파일 백업: Pi `~/.bashrc.bak-domain226`
  - PC 백업은 세션 임시 폴더에 두었으나 삭제되어 없음 (바뀐 줄: 120행 `export ROS_DOMAIN_ID`, 125행 `humble` alias, 원래 값 16)
- PC → Pi SSH 키 인증 (`test@192.168.0.181`)
- PC 쪽 ROS 노드는 Cyclone DDS 사용 (`scripts/env.sh` 의 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), Pi는 Fast DDS 그대로
- DB: SQLite → MySQL (`warehouse` 계정, `server/.env`). 예전 `server/db.sqlite3` 은 남아 있음 (시뮬레이션 시험 기록만 있음)

## 8. 다음 할 일 (제안)

1. 웹에서 홈 위치 + 물류 위치 2~3개 등록 → 작업 전체 흐름 실제 시험
2. 주행 중 사람/상자를 앞에 두고 정지·대기·재계획 시험
3. LiDAR로 본 장애물을 A* 맵에 임시 반영 → 막히면 돌아가는 경로 계획
4. 재부팅 후 자동 실행 (systemd 서비스) 검토

---

## 9. 추가 기록 (2026-10-19 14:25)

### 홈 복귀가 안 되던 문제 → 해결
- 원인: 홈 위치가 등록되어 있지 않았음 (등록된 위치 0개). 서버는 오류만 돌려주고 버튼은 눌리는 상태였음
- 수정: 웹에 "현재 위치를 홈으로" 버튼 추가, 홈이 없으면 "홈 복귀" 비활성화 + 안내
- 홈 = 출발점 (0.20, 0.00, 0°) 로 등록. 홈 복귀 실제 시험: 2.1초, 목표 오차 7cm (로봇이 홈 근처에 있어 짧은 이동)

### 맵 정리 → 적용
- `tools/clean_map.py` 로 `~/maps/maze_map_clean.yaml/.pgm` 생성 (원본 유지), `start_nav2.sh` 기본 맵으로 변경
- 방 밖 빈 칸 148칸, 장애물 점 37칸 제거, 벽 틈 4칸 메움
- **발견한 문제**: 원본 `maze_map.yaml` 의 `free_thresh: 0.25` 때문에 map_server가 미탐색(회색) 영역을 빈 칸으로 읽고 있었음
  → A*가 방 밖으로도 경로를 만들 수 있었음. 정리본은 `free_thresh: 0.196`
- 확인: 방 안 목표는 경로 생성, 방 밖 (-1.5, -4.0) 은 경로 없음. 웹 지도도 정리본으로 바뀜
- 방 범위: x -0.43 ~ 2.37 m, y -3.30 ~ 0.40 m

### 등록된 위치 (maze_map)
- 홈 (0.20, 0.00, 0°)
- gdayu (0.05, -2.75), 물류 위치 - 웹에서 사용자가 추가

### 끊긴 벽 채우기 (14:27)
- `clean_map.py` 에 두 단계 추가: 같은 행/열 3칸 이하 벽 틈 이음(20칸), 바깥 경계 봉합(16칸)
- 결과: 바깥 벽이 끊김 없이 이어지고, 점선처럼 끊긴 안쪽 벽(오른쪽 아래 칸막이 등)도 이어짐
- 로봇 이동 가능 칸 2,139 → 2,138 (주행 공간 변화 없음), 홈 → gdayu/방 안 여러 지점 A* 경로 정상
- 적용 완료: map_server 재시작, 초기 위치 재지정 (0.25, 0.05, -2°, 스캔 점 100%가 벽과 20cm 이내), 웹 지도 갱신

### 웹 고정 / 로봇 사라짐 표시 / 배터리 / 맵 반듯하게 (15:20)
- 웹 지도: 확대·축소·이동 제거, 방 범위에 맞춰 고정
- 로봇이 사라지면 화면에서도 사라짐 (브리지 종료, Pi 응답 없음, 위치 2초 이상 없음) + 상태 점(브리지/LiDAR/오도메트리/위치 추정). 확인 완료
- 배터리 %: TurtleBot3 값(10.99V에 27%) 대신 LiPo 3셀 곡선으로 계산 (10.99V ≈ 8%)
- 맵: `tools/straighten_map.py` 로 기울기 -2.40° 보정 + 벽 직선화 → `~/maps/maze_map_straight.yaml` 적용
  - 오른쪽 위 작은 칸(안쪽 미탐색)은 직사각형 장애물로 채움
  - 등록 위치도 +2.40° 회전 (`manage.py rotate_locations 2.40`): 홈 (0.20, 0.01, 2.4°), gdayu (0.17, -2.75, 93.4°)
  - 다닐 수 있는 칸 2,138 → 2,200, 홈 → 방 안 여러 지점 A* 경로 정상
- **현재 로봇 꺼짐** (15:02 이후 Pi 접속 불가, 직전 배터리 10.99V ≈ 8% → 방전 추정). 웹에는 "로봇(Pi) 응답 없음"으로 표시됨
- 할 일: 충전 후 Pi bringup → 초기 위치 지정(새 맵 좌표) → 새 맵에서 위치 추정 품질 확인 → 실패로 남은 작업 #27(홈 복귀) 취소 또는 재시도

### 실행 모드 추가 (17:10)
- `scripts/run.sh --robot real|sim --mode slam|map` : 실물/시뮬레이션 × 맵 만들기(SLAM)/기존 맵(AMCL)
  - 실물 도메인 44, 시뮬레이션 도메인 77 (섞이지 않게). 현재 모드는 `logs/run_mode.env`
  - SLAM: slam_toolbox(기본) 또는 cartographer. 서버에는 `slam_live` 이름으로 보내 기존 맵/위치 보호
  - 웹: 모드 배지, SLAM 모드에서 "맵 저장" 버튼, "초기 위치 지정" 비활성화, 맵이 커질 때만 화면 다시 맞춤
  - 맵 저장: `save_map.sh` (map_saver + clean_map, `--straighten` 선택). 정리 기준점 = 저장 시 로봇 위치
- 고친 문제
  - Gazebo 로봇 생성 실패: 기본 모델 경로 누락 → env.sh 에서 /usr/share/gazebo/setup.sh 읽기
  - SLAM 중 바로 앞도 "경로 없음": SLAM 모드는 A* `allow_unknown:=true` (미탐색 칸 비용 +2)
  - 단일 이동 실패 시 작업이 "진행 중"으로 남아 다음 명령이 막힘 → 단일 이동은 실패로 종료하고 대기로
  - 맵 파일이 없으면 start_nav2.sh 가 바로 오류로 종료 (전에는 map_server 가 조용히 멈춘 채 남음)
  - clean_map: 탐색 덜 한 SLAM 맵의 빛줄기 모양 줄무늬 제거(열림 연산). straighten 은 직각 구조 전용이라 기본 끔
  - stop_ros.sh: 시험 셸까지 잡던 패턴 수정, Gazebo/SLAM 포함, `--all`
- 시뮬레이션으로 전 모드 확인 완료 (README 6번). 실물 로봇은 꺼져 있어 실물 모드는 아직 시험 안 함
- 맵 파일 위치가 바뀜 (사용자가 옮김): `~/maps/backup/backup2/maze_map*.yaml`
- 시험으로 만든 맵: `~/maps/sim_world_test.yaml`, `~/maps/sim_world_test_clean.yaml` (지워도 됨)
- 현재 PC 쪽 전부 꺼진 상태

### SLAM + Nav2 옵션, 상태 초기화, Nav2 TF 멈춤 재조사 (17:30)
- `run.sh --nav2` 를 SLAM 모드에서도 사용 가능 (주행 노드만, map→odom 나온 뒤 시작). `--planner nav2`, `--controller nav2` 추가
- 웹 "상태 초기화": 작업 취소 + 정지 + (시뮬레이션) Gazebo /reset_world 로 출발 위치에 세움 + (기존 맵) 초기 위치 재지정. 확인 완료
- 넘어짐 감지(IMU 기울기 35° 0.5초 → 정지/실패, 상태 점 "넘어짐"), 내장 추종기 제자리 회전 15초 제한
- 브리지 재연결로 목표를 잃은 단일 이동이 "진행 중"으로 남던 문제 수정
- **Nav2 TF 멈춤이 시뮬레이션에서도 재현됨** (Cyclone DDS, sim time):
  - 기본 설정: map→odom 오류 376건 / voxel_layer 제거: 약 2분 뒤 멈춤(630건)
  - 스캔 레이어 전부 제거: "too old" 는 없었지만 bt_navigator/controller 가 0.9m 예전 위치를 쓰고 "진행 없음" → TF 전체가 멈춘 것
  - → MessageFilter(스캔 레이어) 가설은 틀림. Nav2 C++ 노드 안 TF 수신 자체가 멈춤. 근본 원인 미확인
  - 다음 조사 후보: gdb 로 멈춘 controller_server 스레드 확인 (ptrace_scope=1 이라 직접 띄운 프로세스로 붙이거나 sudo 필요)
- 기본(A* + 내장 추종기)은 영향 없음. 현재 시뮬레이션 SLAM 모드(기본 방식)로 실행 중
- 실험 파일: `config/nav2_waffle_pi.yaml` (voxel 제거, 해결 안 됨), `/tmp/claude-1000/nav2exp/no_scan_layers.yaml`

### Nav2 TF 멈춤 근본 원인 확인 → tf2 0.25.24 로 해결 (17:45)
- **원인: tf2 / tf2_ros 0.25.23 의 ABBA 교착(deadlock)**
  - gdb 로 멈춘 controller_server 스레드 확인:
    - TF 수신 스레드: `TransformListener::subscription_callback → BufferCore::setTransform → testTransformableRequests` 에서 tf2_ros Buffer 잠금 대기
    - LiDAR 스레드(costmap obstacle layer MessageFilter): `Buffer::waitForTransform → BufferCore::addTransformableRequest` 에서 tf2 요청 목록 잠금 대기
  - 서로 상대 잠금을 쥔 채 멈춤 → 그 프로세스는 이후 TF 를 하나도 못 받음
  - 0.25.24 변경 기록: "Fix ABBA deadlock between waitForTransform and testTransformableRequests (geometry2 #982 백포트)"
  - 오늘 겪은 현상(AMCL 시작 직후 멈춤, controller 가 목표 지나쳐 2.8m 주행, 안 움직였는데 도착 처리, sim 에서 몇 분 뒤 진행 없음)이 모두 이것으로 설명됨. DDS(Fast/Cyclone) 와 무관
  - 앞서 "스캔 레이어를 빼도 멈췄다 → MessageFilter 가설 틀림" 이라고 판단했던 것은 해석 오류였음 (그때는 실제로 멈춘 것이 아니었음)
- **해결**: `sudo apt install --only-upgrade ros-humble-tf2 ros-humble-tf2-ros` → 0.25.24 설치 완료 (PC)
- 확인 시험 (sim, SLAM + Nav2 planner/controller, 목표 5곳 반복):
  - 0.25.23: 1분 26초 만에 교착 (too old 오류)
  - 0.25.24 라이브러리: 5분 동안 목표 20개 연속 성공, too old 0건 (10분 시험 진행 중)
- 남은 일
  - 시스템에 설치된 0.25.24 그대로 짧게 재확인
  - Pi 도 tf2 업데이트 권장 (Pi 는 Nav2/AMCL 을 안 돌려 영향은 적음)
  - 이제 Nav2 controller/planner 모드도 쓸 수 있으므로 README 의 "시험용" 경고와 메모리 갱신
  - AMCL 도 같은 버그 영향을 받았으므로, 실물 로봇 기존 맵 모드도 업데이트 후 재확인
- 현재: sim(SLAM) + 브리지(기본 방식) 실행 중, Nav2 는 시험용으로 별도 실행 중. 실물 로봇은 꺼져 있음 (Pi 접속 불가)
- 디버깅 기록: `logs/nav2_gdb.log` (교착 순간 스레드 덤프), `logs/nav2_tf2fix.log` (0.25.24 시험)

### 2026-10-20 09:20 현재 상태
- PC 재부팅 후 전부 꺼진 상태 (웹 서버, Gazebo, ROS 노드). 마지막 모드: sim / slam / astar+builtin (`logs/run_mode.env`)
- tf2 / tf2_ros 0.25.24 설치 유지
- 시스템 tf2 0.25.24 로 Nav2 연속 주행 확인 시험: 약 2분(목표 9개) 진행 중 PC 종료로 중단, 그때까지 TF 멈춤 0건
  (설치 전 받은 0.25.24 라이브러리로 한 10분 시험은 완료: 목표 42개 중 41개 성공, 멈춤 0건)
- 실물 로봇(Pi) 접속 불가 (No route to host) → 전원/배터리 확인 필요
- 다음: 시스템 tf2 로 Nav2 시험 다시 끝까지, 실물 로봇 켜지면 AMCL(기존 맵 모드) 재확인, Pi tf2 업데이트

### 웹에서 실행 모드 전환 + AMCL 위치 자동 찾기 (2026-10-20)
- 웹 "실행 모드" 카드: 로봇(시뮬레이션/실물) + 맵 선택 → "저장된 맵으로 위치 추정 (AMCL)", "맵 만들기 (SLAM)", "전체 정지", "위치 자동 찾기"
  - 서버(`server/warehouse/runner.py`)가 `scripts/run.sh` / `stop_ros.sh` / `tools/localize.py --global` 을 별도 스레드로 실행 (한 번에 하나)
  - ROS 가 꺼져 있어도 웹에서 시작 가능. 주행 중이면 먼저 비상정지. 결과는 알림 + 기록
- `tools/localize.py --global`: 맵 전체 탐색 (넓게 10cm·5° → 상위 5곳 정밀 2.5cm·1°), 대칭이면 경고
- 시뮬레이션 확인: AMCL 시작(초기 위치 자동, 스캔 일치 100%) → 엉뚱한 위치(1, 1, 90°)로 지정 후 자동 찾기 2.8초 만에 (-1.94, -0.48, 0°) 복구
  → AMCL 이동 성공 → SLAM 전환 → SLAM 에서 자동 찾기는 거절 → SLAM 이동 성공 → 전체 정지(웹 서버 유지)
- 실물 로봇: 아직 꺼져 있어 미확인 (켜지면 "실물 + AMCL" 로 시험)

### 시뮬레이션 월드 추가 + 자동 탐색으로 맵 처음부터 만들기 (2026-10-20 10:20)
- 월드: `maze_room` (실제 미로 방 복제, `tools/map_to_world.py` 로 실제 맵에서 생성, `worlds/`), AWS `no_roof_small_warehouse` (`~/sim_ws` 에 받아 둠)
- 자동 탐색 (`tools/explore.py`, 웹 "자동 탐색"/"탐색 중지", SLAM 모드)
  - maze_room: 3.2분, 이동 9회 모두 성공, 9.4 m² → `~/maps/sim_maze_room_clean.yaml` 저장
    실제 맵과 겹쳐 보면 시뮬 맵 벽 100% 가 실제 벽 10cm 이내
  - AWS 창고(21×14.5m): 시뮬 LiDAR 3.5m 라 SLAM 이 아는 공간이 좁아 30~40cm 씩만 전진 → 너무 느려 중단 (개선 여지)
- 고친 문제
  - AWS 창고 모델 메시가 상대 경로라 충돌 형상이 없어 LiDAR 가 통과 → GAZEBO_RESOURCE_PATH 추가
  - 맵이 없을 때(SLAM 시작 직후) 웹이 빈 화면 → 로봇 주변 8m 격자 표시, 빈 맵은 보내지/저장하지 않음
  - stop_ros.sh 가 같은 PC 다른 작업의 Gazebo(도메인 93)까지 끌 수 있던 문제 → 도메인 44/77 프로세스만
  - 다른 곳에서 실행된 spawn_entity 가 우리 sim(도메인 77)에 끼어들어 /odom 이 끊긴 일 있음 → sim 재시작으로 복구
- 배터리 %: 11.0V = 0% 로 변경 (`battery_empty_v`), 브리지 재시작으로 적용
- 현재: sim / maze_room / SLAM 실행 중, 웹 서버 실행 중

### 장애물·부딪힘·좌표 문제 시뮬레이션 시험과 수정 (2026-10-20 오후)

시험 환경: sim `maze_room`, 높은 상자(30cm, LiDAR 에 보임)와 낮은 상자(8cm, LiDAR 아래) 배치.
`tools/sim_watch.py` 로 정답 위치(Gazebo)와 비교해 위치 오차, 충돌, 벽 여유, 기울기를 기록.

| 문제 | 원인 | 수정 | 결과 |
|---|---|---|---|
| 새 장애물에 막히면 재시도해도 같은 자리에서 3번 막힘 | 재계획이 같은 정적 맵 사용 | A* 가 LiDAR 로 본 맵에 없는 장애물을 20초 기억 | 높은 상자 우회해서 도착 |
| SLAM 중 아직 맵에 없는 곳을 찍으면 "경로 없음" | 목표가 격자 밖 | SLAM 모드에서 격자를 미탐색 칸으로 넓힘 | 해결 |
| AMCL 이 멈춰 있어도 12cm 어긋남 | TurtleBot3 기본 AMCL 설정 (빔 60개, 25cm 마다 갱신) | `config/amcl_waffle_pi.yaml` (빔 180, sigma 0.1, 5cm 마다 갱신) | 주행 중 오차 2~5cm |
| 초기 위치 직후 20cm 넘게 튐, 벽이 가짜 장애물로 잡힘 | 초기 위치 불확실성 50cm | 자동 위치 5cm / 웹 지정 20cm, 지정 후 정지 보정 10회 | 해결 |
| 다른 월드에서 만든 맵으로 띄우면 좌표가 안 맞음 | 맵-월드 짝 정보 없음 | 저장 시 맵 yaml 에 `# world:` 기록, `run.sh` 가 그 월드로 실행 | 해결 |
| 자동 초기 위치가 틀린 기준점이면 그대로 지정 | 기준점 주변만 탐색 | 20cm 이내 80% 미만이면 맵 전체 탐색 (`--fallback-global`) | 틀린 기준점에서도 찾음 |
| SLAM 중 등록한 위치가 맵 저장 후 사라짐 | 위치는 `slam_live` 맵에 묶임 | 맵 저장 시 새 맵으로 복사 | B 흐름에서 4곳 복사 확인 |
| SLAM 중 웹 지도에서 로봇/위치가 흔들려 보임 | 화면 범위를 이미지 픽셀로 기억 → 맵 원점이 바뀌면 밀림 | 화면 범위를 월드 좌표로 기억 | 계산으로 확인 (브라우저 미확인) |
| 낮은 상자에 올라타 뒤집힘 (147°) | 넘어짐 감지 35° 뿐 | 6° 부딪힘 감지, 물러나기, 부딪힌 자리 기억 | 뒤집히지는 않음. 아래 "남은 문제" |
| 부딪혀 기운 스캔으로 AMCL 위치가 80cm 튐 | 기운 LiDAR 가 바닥을 벽으로 봄 | 수평 스캔만 AMCL/SLAM/A* 에 전달 (`scan_level`, 평소 기울기 기준, 3초 넘으면 다시 전달) | 17cm 이내 |
| 자동 탐색이 두 곳을 15분 동안 오감 | 도착해도 안 사라지는 프론티어 | 같은 곳에 두 번 도착하면 제외 | 해결 |
| SLAM 에서 가짜 장애물 998칸이 쌓여 통로가 막힘 | 커지는 중인 맵 밖 점을 장애물로 기억 | SLAM 모드는 LiDAR 장애물 기억 끔 (SLAM 맵이 반영) | 해결 |

벽·장애물 근처 목표 10곳 (AMCL 모드): 벽에서 8~22cm 인 목표는 18~21cm 떨어진 빈 곳에 멈추고 "도착" 처리 (충돌 없음).
폭 40cm 틈 안쪽 목표는 "경로 없음" (정상).

**B 흐름 전체 시험 (상자 없음)**: SLAM + 자동 탐색 2.4분, 9.3 m², 충돌 0 → 위치 4곳 등록 → 맵 저장 `~/maps/sim_maze_b_clean.yaml`
→ 기존 맵(AMCL) 모드 자동 전환 → 선반A → B → C → 홈 작업 85초에 완료, 충돌 0, AMCL 오차 최대 8cm.
- 자동 초기 위치의 스캔 일치가 87% (평소 98%) → 원인 미확인

**남은 문제**
- LiDAR 보다 낮은 물체: 자동으로 계속 피하게 하면 위치 흔들림·기억에 갇힘·재충돌이 번갈아 생겨서, "한 이동에서 두 번 부딪히면 멈추고 사람에게 알림" 으로 정함.
  sim 에서는 Waffle Pi 가 8cm 상자에 쉽게 올라타 걸림 (실물도 그런지 미확인)
- 벽에 바짝 붙은 목표는 실제로 멈춘 위치를 알려주지 않고 "도착" 으로만 표시
- 실물 로봇에서는 오늘 수정 전부 미확인. 특히 IMU 기본 기울기, 6° 부딪힘 감지 오작동 여부, 새 AMCL 설정
- 운영 권장: SLAM 으로 지도 완성 → 저장 → 기존 맵(AMCL) 모드에서 위치 등록·작업 (SLAM 은 루프 클로저로 지도가 다시 맞춰질 수 있음)

### 현재 상태 (2026-10-20 14:05)
- sim / maze_room / 기존 맵(AMCL) 모드 실행 중, 맵 `sim_maze_b` (위치: 홈, 선반A·B·C), 웹 서버 실행 중
- 실물 로봇(Pi): 오늘 사용 안 함
- 이동 시간 예측 + 방문 순서 최적화 (웹 "⇅ 순서 최적화"): 코드만 있음, 데이터로 검증 전 (`tools/collect_travel.py` 로 기록 모으기 예정)

### LED 배달 기능 (2026-10-20 15:30)
- 흐름: 관제 화면 "LED 요청" (LED 선택 + 지도에서 받을 자리) → LED 위치로 이동 → 싣기 대기 → 카메라 인식 또는 수동 확인 → 요청 자리로 배달 → 꺼냄 인식 → 완료 (→ 홈)
- 한 번에 한 건, 진행 중이거나 로봇에 물건이 실려 있으면 새 요청 거절
- 로봇 위 태블릿 화면 `/robot`, 카메라 인식 결과는 `cargo/detected` 토픽 (학습 모델은 같은 토픽으로 라벨만 내면 됨), 임시 색 인식 노드 `cargo_detector`
- sim 확인 (맵 `maze_led`, 카메라는 `tools/cargo_sim.py` 로 흉내): B 요청 → 중복 요청 거절 → B 도착 → A 실림 안내 → B 인식 후 출발 → 도착 → 꺼냄 인식 → 완료 (47초).
  수동 확인 경로, 물건이 실려 있을 때 거절도 확인
- 아직: 실물 카메라 (Pi 카메라 노드 실행, 아래로 꺾은 카메라 영상으로 HSV 범위/학습 모델 맞추기), 태블릿 화면 실기기 확인
- 지도 `maze_led` (A, B, C 위치, SLAM 으로 만든 maze_room sim 맵). 홈 미등록

### 강당 시뮬레이션 월드 추가 (2026-10-21)
- `worlds/lecture_hall.world` (`tools/make_hall_world.py` 로 생성), 웹 실행 모드 월드 목록에 "강당 (강단 + 복도 6×5, 책상 20·의자 40)"
- 배치: 위쪽 강단(공터 2.0m) + 가로 복도 6 × 세로 복도 5 (폭 0.9m), 가로 복도 사이 좌석 줄(1.0m)에 세로 복도 사이 칸(1.4m)마다 책상 1개(강단 쪽) + 의자 2개 = 책상 20 · 의자 40. 실내 10.1 × 12.4m
- 책상 1.15 × 0.4m, 높이 0.72m, 앞판(바닥 0.1m~) 이 LiDAR 에 막힌 선으로 보임
- 의자: 다리 4개(지름 5cm, LiDAR 에 점 4개) + 상판(높이 0.45m) + 등받이, 의자 사이 틈 0.25m (로봇 폭 0.31m 라 못 지나감)
- 원점 = 가로 6(맨 아래) / 세로 1(맨 왼쪽) 교차점 = 로봇 출발 위치 (출발 위치 기본값 (0, 0) 그대로)
- sim + SLAM + DWA 로 띄워 확인: Gazebo 로드, 브리지 연결, SLAM 맵 갱신, 웹에서 이동 명령 성공
- 이 PC 는 TurtleBot3 를 apt 로 설치해 `~/turtlebot3_ws` 가 없음 → `runner.py` 기본 월드 경로가 없으면 `/opt/ros/humble/share/turtlebot3_gazebo/worlds` 사용, `run.sh` 는 있을 때만 source

### 자동 탐색 개선: 방향 유지 + 도중 전환 + 막힌 구멍 무시 (2026-10-21)
- 문제 (강당 월드, 예전 방식): 0.3~0.5m 씩 기어가듯 전진, 같은 근처를 반복 선택. 책상 밑/앞판 뒤 작은 미탐색 구멍이 가장 가까운 목표로 뽑혀 가 봐도 안 보임
- `tools/explore.py` 바꾼 점
  - 점수 = 거리 + 0.6 m/rad × (로봇 방향과 목표 방향 차이) − 크기 보너스 → 가던 방향 유지 (`--turn-weight`)
  - 가는 도중 목표 프론티어가 이미 보이면(반경 0.6m 안 8칸 미만) 취소하고 바로 다음 목표 (`--no-early-switch` 로 끔)
  - 1 m² 미만이고 맵 가장자리에 닿지 않는 미탐색 영역(막힌 구멍)에만 붙은 프론티어는 무시 (`--pocket-min`)
  - 프론티어 최소 크기 25cm → 40cm, 0.5m 보다 가까운 목표는 고르지 않음, 도착 방향 = 가는 방향
- 결과 (sim 강당, SLAM + DWA): 15.9분에 탐색 완료, 이동 76회 (도착 61, 도중 전환 15), 실패 0, 121.7 m² (실내 125 m²). 목표 간 평균 1.0m
- 같은 시험 중 브리지가 "위치 정보가 갱신되지 않아" 로 자주 실패(44건 중 20건)했으나 브리지를 다시 띄운 뒤로는 0건. 바깥 노드로 잰 TF 나이는 그동안에도 최대 0.08초로 정상 → 원인 미확정. 실패 메시지와 로그에 TF 나이/기울기를 남기도록 함 (`위치 TF 끊김: ...`)

### 강당 sim 전체 흐름 시험 (2026-10-21, test-pc-lecture-hall 병합 후)
- SLAM + DWA 자동 탐색: 8.6분, 125.3 m² (실내 125 m²), 실패 0 → `~/maps/hall_clean.yaml` 저장 → 기존 맵(AMCL) + DWA 로 전환 (자동 초기 위치 스캔 일치 99%)
- 위치: 홈 (0, 0), A (2.39, 10.64), B (2.04, 1.84), C (6.94, 3.99). 받는 사람 자리 (4.44, 5.74)
- `tools/delivery_demo.py` (요청 → LED 위치 → 싣기(cargo_sim) → 배달 도중 원기둥 가로지름 → 꺼냄 → 홈)
  - B 요청: 150초 완료, 충돌 0 (원기둥은 도구 버그로 배달 경로가 아니라 B 출발 직후 옆을 지나감, 최소 여유 4.2cm) → 도구 수정
  - C 요청: 177초 완료, 배달 경로 앞을 원기둥(0.15 m/s)이 가로지름 → 로봇이 감속·대기 후 지나감, 최소 여유 15.4cm, 충돌 0
- 관찰: 원기둥이 출발한 뒤 약 5초 동안 추적 속도가 거의 0 (처음엔 멈춘 물체로 잡혀 칼만필터가 늦게 따라감). 원인 미확인

### 실물 SLAM 확인 (2026-10-21 오전)
- 실물 LDS-02 는 빔 개수가 스캔마다 238~244 개이고 angle_min/max 와도 안 맞음 → slam_toolbox 가 처음 등록한 개수와 다른 스캔을 전부 버려서 지도가 몇 m² 에서 멈추고, 로봇은 지도에 없는 벽 앞에서 '앞이 막힘' 반복
  → 브리지가 SLAM 용 스캔(scan_slam)을 1.5° 간격 240 칸으로 다시 나눠 보냄. 이후 개수 경고 0건
- 실물 LiDAR range_max = 100 (드라이버 값), 반사 없음은 NaN (241 빔 중 약 45)
- 추적: SLAM 중 아직 지도에 없는 벽이 전부 장애물로 잡힘(350건 중 345건, 속도 노이즈 중앙값 0.06 m/s) → 지도의 빈 칸에 있는 점만 추적, 움직이는 물체 기준 0.08 m/s, 웹에는 움직이는 것만 표시
- 브리지 TF 전용 노드가 launch 의 이름 바꾸기 때문에 /warehouse_bridge 로 보여 브리지가 둘인 것처럼 보였음 → /warehouse_bridge_tf
- 웹 수동 조작 패널 추가 (실물에서 회전 명령/자동 정지 확인)
- 남은 문제: 로봇(Pi) 재부팅·bringup 재시작이면 오도메트리가 0 부터 다시 시작해 SLAM 을 처음부터 해야 함 → '이어서 그리기'(slam_toolbox 포즈 그래프 저장/불러오기 + 스캔 매칭으로 위치 찾기) 제안 중

### 강당(학원) 대응: 계획용 지도 가공 · 금지 구역 · 강당용 AMCL · 실물형 sim (2026-10-21 오후, 브랜치 `hall-planning`)
배경: 실물 강당 SLAM 지도는 책상/의자 다리가 점으로 흩어지고, 먼 곳은 빔 사이 미탐색 줄무늬가 남음 (sim 은 책상 앞판 때문에 막힌 선으로 보여서 실물보다 쉬웠음)
- **계획용 지도 가공** (`plan_map.py`, A* 만. AMCL 은 원본 그대로): 빈 칸 사이 미탐색 줄무늬 메움 + 가로·세로 0.4m 이하 덩어리(다리 점)를 0.1m 부풀림 → 다리 사이 0.7m 미만 틈이 막힘. 닫힘 연산(점 잇기)도 해 봤으나 반경 0.35m 로도 0.5m 격자까지만 이어서 A* 반경(0.25m → 0.5m 미만 틈 이미 막힘)과 차이가 없어 뺌. 넓이 기준은 얇은 벽 조각까지 부풀려 문이 좁아져서 가로·세로 길이 기준으로. 다리 간격이 더 넓은 책상 밑은 금지 구역으로
- **금지 구역/가상 벽** (웹): Zone 모델(마이그레이션 0005) → 서버 → 브리지 → `astar/zones` → 플래너. 출발 위치가 들어 있는 구역은 그 계획에서 빼서 빠져나옴. 서버 이동 시간 예측도 같은 가공·구역 반영
- **강당용 AMCL** `config/amcl_hall.yaml` (`--amcl hall`, 웹 체크) + `tools/scan_check.py` (laser_max_range 를 늘려도 되는지 실물 측정)
- **실물형 sim**: `--scan-noise` (빔 238~244, 흔들림 1cm+1%, 2.5m~ 값 빠짐, 무응답 NaN), 월드 `lecture_hall_real` (앞판 없음, 좌우 끝 세로 복도 없음 = 끝 열이 벽에 붙음, 의자 무작위, 원점 = 가로 6/세로 2)
- mac 에서 확인한 것: 단위 테스트 43개 통과(plan_map 5, scan_noise 3 추가), Django check, 금지 구역 서비스/경로 예측(임시 sqlite), 합성 맵으로 plan_map.py, run.sh 옵션 오류 처리
- **아직 안 한 것 (서버에서)**: ROS 노드 실행, Gazebo 회귀 시험, 실물 측정 전부
- 계획용 가공은 기존 맵 모드에서 **항상 켜짐** (`--amcl hall` 과 달리 옵션 아님). 0.4m 이하 짧은 벽 조각/노이즈 점도 부풀려져 들쭉날쭉한 벽 옆 통로가 한쪽 최대 0.1m 좁아질 수 있음 → 첫 실물 주행 전에 `tools/plan_map.py <맵> --check <홈> <LED>` 로 LED 마다 확인. 끄기: params.yaml `cluster_inflate: 0`, `stripe_cells: 0` 후 브리지 재시작

서버에서 받기 (순서 중요: 웹 서버가 새 코드로 자동 재시작되면 migrate 전까지 Zone 테이블이 없어 웹 접속/맵 저장이 실패함):
```bash
scripts/stop_ros.sh --all                                # 웹 서버까지 끔
git fetch && git switch hall-planning && git pull
cd server && .venv/bin/python manage.py migrate          # Zone 테이블
cd ../ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install   # 새 모듈/launch 인자
```
회귀 시험 (sim):
1. 기존 그대로인지: `scripts/run.sh --robot sim --mode map --map ~/maps/hall_clean.yaml --controller dwa` → `python3 tools/delivery_demo.py --led C --to 4.44 5.74 --obstacle` (이전: 177초, 충돌 0). `logs/bridge.log` 에 "계획용 가공: 줄무늬 메움 N칸, 다리 점 부풀림 N칸" 확인
2. 실물형: `scripts/run.sh --robot sim --mode slam --world lecture_hall_real --scan-noise --controller dwa` → 웹 자동 탐색 → "맵 저장" (예: hall_real) → `python3 tools/plan_map.py ~/maps/hall_real_clean.yaml --check <홈> <LED>` 로 미리 보기 → `scripts/run.sh --robot sim --mode map --map ~/maps/hall_real_clean.yaml --scan-noise --amcl hall --controller dwa` → 위치 등록 후 delivery_demo
   - 주의: `save_map.sh` 의 clean_map 은 `--open 1` 로 가느다란 빈 칸(빔 줄기)을 지움. 줄무늬가 많은 맵은 원본(`hall_real.yaml`)과 비교해 볼 것
실물:
1. `python3 tools/scan_check.py 20` (로봇 정지, 강당 가운데) → 추천 최대 거리
2. `scripts/run.sh --robot real --mode map --map ~/maps/<강당맵>.yaml --amcl hall` → 웹 금지 구역으로 책상 밑/좌석 줄 막기 → 홈 ↔ LED 이동
