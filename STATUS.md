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

### 자동 탐색 + 금지 구역 문제 수정 (2026-10-21 저녁, 브랜치 `hall-planning`)
현상 (sim `lecture_hall_real` SLAM 자동 탐색 중 통로를 막는 금지 구역 추가): 탐색기가 구역 뒤 (−1.45, 1.88) 를 목표로 골라 A* 가 실패,
줄 중심선 점은 실패 목록을 안 봐서 같은 점을 0.5초마다 약 230번 보냄 (작업/로그가 쌓임)
1. 탐색기(`tools/explore.py`)가 금지 구역을 받아 구역 칸은 장애물로, 로봇에서 A* 처럼(반경 0.25m, 미탐색 통과) 갈 수 있는 칸에서만 목표/줄 중심선 점을 고름
2. 줄 중심선 점도 실패 목록을 지킴. 같은 곳 두 번째 실패/금지 구역 때문이면 1m 제외, 연속 5번 실패하면 탐색 중지
3. 가는 중 구역이 바뀌어 목표가 구역 안/뒤가 되면 멈추고 다시 고름
4. 플래너가 실패 이유를 `astar/fail_reason` 으로 발행 → 웹 메시지 "경로를 찾을 수 없습니다: 금지 구역에 막혀 갈 수 있는 경로가 없습니다" / "목표가 금지 구역 안에 있습니다"
5. 버그 수정: 출발 위치가 구역에서 0.35m 안이기만 해도 구역 전체를 계획에서 빼던 것 → 실제로 안(5cm)일 때만
6. 서버: 금지 구역 안 목표는 바로 거절, 같은 지점 30초 안 3번 실패면 잠시 거절, 같은 로그 문구는 10초에 한 번
7. 웹: 자동 탐색 중 금지 구역을 바꾸면 알림

확인 (서버 sim, 같은 자리 구역 A + 이동 중 목표 위에 구역 B): 240초 동안 목표 12개, 실패 0, 구역 A 뒤 목표 0. 구역 B 를 그리자 1.1초 만에 새 목표로 바꿈
(막힌 통로를 돌아가느라 다음 목표까지 78초). 구역 안 목표 → 즉시 거절, 구역 뒤 목표 → 실패 이유 표시. 단위 테스트 49개 통과
- 남은 것: 루프 때 쌓인 실패 작업/로그 약 230건은 지우지 않음

### 강당 실물형: 좌석 줄 사이를 로봇이 못 지나가게 (2026-10-21)
- `make_hall_world.py --row-gap 0.25` (`--real` 기본): 좌석 줄 사이 빈 틈(의자 뒤 다리 ~ 다음 줄 책상 앞 다리) 0.25m, 맨 앞(강단 쪽)/맨 뒤(출발) 가로 복도와 세로 복도는 0.9m 그대로 → 실내 8.3 x 9.4m (전에는 12.4m)
- 생성한 월드에서 잰 실제 틈: 줄마다 0.09~0.27m (의자 무작위 이동/회전 포함) < 로봇 폭 0.31m. 의자가 남쪽으로 밀려도 다음 줄 책상과 겹치지 않게 제한
- 이 월드로 만든 예전 SLAM 지도/위치는 좌표가 맞지 않음 (실내 크기가 바뀜)

### SLAM 화면 방향 + 지도 즉시 갱신 (2026-10-21)
- (되돌림: 사용자 요청으로 화면 회전은 원복) 맵 만들기(SLAM)를 새로 시작하면 첫 로봇 위치의 방향이 웹 화면 위쪽이 되도록 맵을 돌려 보여 줌 (`Map.view_yaw = 90° - yaw`, 마이그레이션 0006). 좌표·A*·AMCL 은 그대로, 웹 보기만 돌림. 맵 저장 시 각도도 복사
- SLAM 맵을 브리지가 받자마자 서버로 보냄 (예전 3초 묶음 → slam_toolbox 갱신 2초 그대로, 최소 간격 0.5초)
- 서버 sim 확인: 15초 동안 웹 지도 갱신 7회(간격 2.0초), view_yaw 90° + 로봇 yaw 0° = 화면 위쪽. 브라우저 화면은 아직 직접 확인 못 함

### 라이다 점 → 선 (2026-10-21)
- 웹에 보이던 점선은 SLAM 지도의 벽: LDS-02 빔 간격 1.5° 가 1.9m 부터 칸(5cm)보다 넓고, slam_toolbox 가 0.5m/29° 마다만 스캔을 지도에 넣어서
- slam_toolbox `minimum_travel_distance/heading` 0.5 → 0.2 (더 자주 넣어 메움)
- 2026-10-22: 0.2 → 0.1 (10cm / 약 5.7°). 맵 발행은 그대로 2초마다지만 한 번에 반영되는 스캔이 늘어 자동 탐색의 다음 경로가 더 최신 맵을 참조. 0.1 m/s 직진이면 약 1초마다 (minimum_time_interval 0.5초가 한도)
- 웹 실시간 LiDAR 선 표시 (청록): 브리지가 스캔마다(5Hz) 지도 좌표 점을 보내고 웹이 이웃 점 거리 ≤ 0.1m + 빔 간격 x 2.5 면 선으로 이음. 서버 sim: 5.1개/초, 유효 점 281 중 227 구간이 선, 메시지 약 5.4KB

### 원점 x 축 = 로봇 정면, 웹은 x 위쪽 / y 왼쪽 (2026-10-21)
- 원인: SLAM 원점 = 오도메트리 원점(bringup 을 켠 자리). 실측 오도메트리 x −4.73, y 1.21, −19° 인 채로 SLAM 시작 → 원점이 로봇에서 4.9m, x 축이 19° 틀어짐
- `run.sh --robot real --mode slam`(웹 맵 만들기 포함)이 SLAM 직전에 Pi 에서 `/reset_odometry` 호출 → 원점 = 시작 때 로봇 자리, x 축 = 로봇 정면.
  PC(Cyclone)에서 부르면 요청은 가는데 응답이 안 와서(20초 시간 초과) SSH 로 Pi 에서 부름 (4초)
- 실물 확인: 초기화 후 오도메트리 0, SLAM 지도 속 로봇 방향 0.3°
- 웹 지도 표시: 고정 90° 회전으로 x 축이 화면 위쪽, y 축이 왼쪽 (좌표는 그대로, 보기만). 첫 번째 시도(SLAM 마다 각도 저장)는 되돌림


### 기본 경로 추종기를 DWA 로 (2026-10-22)
- `run.sh` / `start_bridge.sh` / launch / `params.yaml` / 브리지 기본값 `controller` 를 `builtin`(pure pursuit) → `dwa` 로. 웹 실행 모드로 시작해도 DWA (runner 는 `--controller` 를 넘기지 않음)
- 예전처럼 쓰려면 `scripts/run.sh ... --controller builtin`
- DWA 는 앞쪽 ±25° 정지 규칙을 쓰지 않음 (스캔 점 + 추적한 움직이는 장애물로 직접 피함). 기울어짐/TF 끊김 정지는 그대로
- **실물 미시험**: DWA 시험 기록은 sim 강당뿐. 실물(LDS-02 빔 개수 변동, NaN) + 좁은 미로 맵(`maze_map`)에서 여기로 이동/배송 확인 필요. 이상하면 `--controller builtin`

### DWA 단독 → hybrid (pure pursuit + 장애물 가까울 때만 DWA), 속도 낮춤 (2026-10-22)
- 실물 `realmap_1` 에서 DWA 단독: 홈(-0.08, -0.04)으로 가다 '진행이 없습니다 → 뒤로 15cm 물러난 뒤 재계획' 반복, 취소. 사용자 판단 '너무 안 됨', 속도도 빠름
- 새 기본 `controller: hybrid`: 평소 pure pursuit (앞쪽 정지 규칙 그대로). 추적 track(지도 빈 칸의 물체: 상자, 사람)이
  로봇에서 1.0 m 안 + 앞쪽 경로 1.5 m 에서 0.4 m 안이면, 또는 움직이는 track 이 1.5 m 안이면 DWA 로 전환.
  2초 동안 그런 track 이 없고 경로에서 0.25 m 안으로 돌아오면 pure pursuit 로 복귀 (경로 진행 index 를 서로 넘겨줌). 파라미터 `hybrid.*`
- 지도에 있는 벽만 있는 곳은 항상 pure pursuit. track 은 5번 연속 보여야 확정이라 처음 보인 뒤 약 1초 뒤 전환
- 속도: pure pursuit `max_speed` 0.15 → 0.1 m/s, `max_turn_speed` 1.0 → 0.6 rad/s, DWA `max_w` 1.0 → 0.6 (`max_v` 는 0.1 그대로)
- 같이 커밋: 정규화된 DWA 비용항(`dwa.py`, `w_path` 0.75 / `w_goal` 0.86), 움직이는 물체 앞 제동 궤적 검사, 옆 차선 비키기
- 확인: 단위 테스트 49개 통과, 전환 판단을 가짜 track 으로 확인. **실물 주행 미확인**

### 지도 정리 1칸 직선/직각 (2026-10-22)
- `save_map.sh` 기본 정리본(`_clean`)을 `straighten_map.py --auto` 로: 기울기 보정(0.2° 미만은 안 돌림) + 벽 1칸 직선 + 직각 모서리 + 물체 1칸 테두리, 방 밖 벽 조각 제거, 나란히 겹친 짧은 직선 제거
- 벽 점이 직선에 85% 미만으로 맞으면 회전 없이 `clean_map.py` 정리 (새 7단계: 1칸 외곽선)
- 돌린 맵은 yaml `# rotation_deg`, 웹 "맵 저장" 이 SLAM 중 등록한 위치/금지 구역을 같은 각도로 돌려 복사
- 서버 맵 `map.yaml`(직선 비율 96%, 0.05° → 안 돌림), `maze_map.yaml`(97%, 2.40° 보정)으로 확인. 웹 저장 → 위치 복사 흐름은 실물 미시험

### 기본 경로 추종기를 MPPI 로 (2026-10-22)
- 새 `mppi.py` (ROS 없음): `DWAController` 를 상속해 입력(정적 스캔 점, 추적 장애물 `[x, y, vx, vy, r]`), 차체 직사각형 여유, 움직이는 물체 예측을 그대로 씀.
  후보 궤적만 다름: 직전 해(한 칸 민 것) + 가우시안 잡음 500개 + 정지 + 원호 36개 → 시점마다 다른 (v, w) 순서를 굴려 비용 → exp(-J/λ) 가중 평균.
  평균 궤적이 안전하지 않으면(좌/우 우회가 섞여 정면으로 가는 경우) 안전한 샘플 중 최소 비용, 없으면 막힘. 움직이는 물체 앞에서는 '멈추면 안전' 으로 보지 않음 (감속 궤적 검사)
- `controller: mppi` 기본 (`run.sh` / `start_bridge.sh` / launch / `params.yaml`). 속도/가속/외곽/여유/predict_time 은 `dwa.*` 를 같이 쓰고 `mppi.*` 는 샘플링/비용 계수. 주행 중 변경 가능(웹 '주행 파라미터' 카드에 MPPI 묶음). 웹 보라 궤적 글자가 'MPPI ...'
- 막힘/회복/진행 없음 처리, 출발 전 돌아서기, 도착 방향 맞추기는 DWA 와 같은 경로 (브리지의 `f['dwa']` 자리에 `MPPIFollower`)
- 비교 (`tools/planner_compare.py`, 운동학 sim + LiDAR 광선 + A* 장애물 기억 + 브리지 재시도 흉내, 9개 시나리오, params.yaml 값 max_v 0.13 / predict_time 3.5 / A* 반경 0.20):
  MPPI 만 8/9 (지금 기본으로 쓰는 방식), DWA 만 6/9 (마주 오는/가로지르는 사람 충돌, predict_time 3.5 에서 목표 앞 기어감), hybrid PP↔DWA 7/9.
  재계획을 1초마다 하면 MPPI 8/9 (상자 70초대 → 45초), hybrid PP↔MPPI + 1초 재계획 9/9 (가장 빠름) — 아직 브리지에 1초 재계획은 없음
- **한계 (sim 확인)**: 경로를 정면으로 막은 상자는 MPPI 도 혼자 못 돌아감 (0.13 m/s x 2.5~5초 = 앞 0.3~0.65 m 만 봄, 가중치/잡음/예측시간 바꿔도 같음).
  15초 진행 없음 → 뒤로 15cm → LiDAR 장애물을 넣은 A* 재계획으로 돌아감. 주기당 계산 35~57 ms (PC)
- 같이 커밋: 사용자가 바꿔 둔 `params.yaml` 값 (A* `robot_radius` 0.20, `dwa.max_v` 0.13, `predict_time` 3.5, `w_obstacle` 1.8, `lookahead` 0.3) — 비교 sim 이 이 값으로 돌았음
- 확인: 단위 테스트 57개 통과 (`test_mppi.py` 3개 추가), 브리지가 다른 도메인에서 `controller=mppi` 로 뜨고 `mppi.*` 변경/거절 동작. **Gazebo/실물 주행 미확인**

### 기본을 다시 hybrid 로 + DWA 비용 함수 다시 정의 (2026-10-22)
- 실물 MPPI 시험: 느리고(최고 0.13 m/s 를 dwa.max_v 에서 같이 씀, 잡음 평균으로 실제 평균 0.09) 상자를 못 피함 (15초 정지 → 후진 → 재계획 → 막힘 → 취소).
  sim 에서도 MPPI 지역 계획만으로 경로에 걸친 상자를 지나간 계수 조합이 없었음 (32개 조합 모두 정지). 사용자 결정: hybrid 로
- 기본 `controller: hybrid` 로 되돌림 (run.sh / start_bridge.sh / launch / params.yaml / 브리지). MPPI 는 `--controller mppi` 로 남김
- DWA 비용 함수 (hybrid 에서 장애물 피할 때):
  - 경로 복귀 비용 `w_path` 0.75 → **0** (피하는 동안 경로 = 장애물 쪽으로 끌어당기지 않음)
  - **목표점을 장애물 너머로** (`dwa_follow.target_clear` 0.233 / `target_max` 2.0): 앞쪽 경로가 지도에 없는 물체(브리지 `scan_unmapped`: 지도 빈 칸의 스캔 점)에서
    0.233 m(차체 반폭 + 여유) 안을 지나면, 목표를 그 너머의 첫 경로점으로. 벽은 빼서 좁은 통로/모서리에서는 목표가 튀지 않음. 움직이는 물체도 뺌(옆 차선 비키기가 맡음)
  - 남은 항: 목표 방향 + 목표 거리 + 장애물 근접(`obstacle_influence` 0.5 → 0.3, `w_obstacle` 1.8 → 1.0) + 느림 + 여유 침범 + (진행 0)
  - `min_v` -0.05 → -0.1: 마주 오는 사람(0.15 m/s)보다 느리게 물러나다 부딪혔음
- 전환: `hybrid.path_width` 0.4 → 0.25 — A* 가 이미 피해 낸 경로는 pure pursuit 가 정확히 따라감 (0.4 면 DWA 로 바뀌어 좁은 틈 앞에서 섰음)
- A* `robot_radius` 0.20 → 0.24: 0.20 이면 A* 가 장애물 옆 0.40 m 틈으로 경로를 내는데 DWA(반폭 0.153 + 여유 0.08)는 못 들어가, 재계획해도 같은 길에서 멈춤
- 비교 (`tools/planner_compare.py`, 21개 시나리오: 기존 9 + 슬라럼/좁은 틈/ㄷ자 함정/갑자기 나타나는 물체/사람 3명+상자/마주 오는 사람+상자 + 실물 map2 지도 무작위 상자 6개):
  이전 hybrid 값 12/21 충돌 2, 평균 0.068 m/s → **새 기본 18/21 충돌 0, 평균 0.095 m/s**, 공통 성공 12개 시간 760 → 591초
  - 하나씩 바꿔 본 결과: w_path 0 만 0/6, + 목표 너머 0/6 (목표점 판정 버그 수정 전), + 장애물 범위 0.3 4/6, + A* 0.24 6/8, + 전환 폭 0.25 17/21, + 후진 0.1 18/21
  - predict_time 2.0, DWA max_v 0.2, 비키는 차선만 w_path, 동적 여유 줄이기는 더 나빠서 안 씀
- 남은 실패 3개: 좁은 틈(통로 1.0 m 에 0.5 m 틈 — 로봇 중심이 들어갈 폭이 3.4 cm), 1.2 m 통로에서 마주 오는 사람 + 그 뒤 상자, 실물 지도 1개 (진행 없음 반복)
- 확인: 단위 테스트 58개, 다른 도메인에서 브리지 기동(controller=hybrid, dwa_follow.target_clear 실행 중 변경). **실물 미확인**

### hybrid 전환 조건을 '로봇 앞 사각형' 으로 (2026-10-22)
- 사용자 요청: 경로 기준 대신 로봇 기준 앞쪽(+뒤 조금)에 장애물이 보이면 DWA
- `bridge_node.update_box()` (스캔마다): 지도에 없는 스캔 점(`scan_unmapped`, 지도 빈 칸 + 벽에서 0.15m 밖)을 로봇 좌표로 →
  x: 차체 뒤 끝 -0.197 에서 0.2m 뒤 ~ 차체 앞 끝 0.069 에서 1.0m 앞, y: ±0.25m 사각형 안에 3개 이상이면 연속 스캔 수 +1.
  `hybrid_obstacle()` 이 2번 연속이면 '앞쪽 장애물 0.53 m (점 4개) → DWA 로 피함'. 움직이는 track 1.5m 조건은 그대로, 복귀 조건(2초 + 경로 0.25m 안)도 그대로
- 파라미터 `hybrid.box_trigger`(true, false = 예전 경로 기준 track 조건), `box_front` 1.0, `box_back` 0.2, `box_half_width` 0.25, `box_min_points` 3, `box_min_scans` 2. 웹 '주행 파라미터' 카드에서 바로 변경
- track 을 거치지 않으므로 지름 0.8m 넘는 물체도 잡고, track 확정(5번 연속) 지연이 없음
- sim (21개): 경로 기준 18/21 (시간합 817초) / 사각형 반폭 0.33 17/21 (882초) / **반폭 0.25 18/21 (837초)** / 앞 0.7 16~17/21 / 앞 1.3 17/21. 충돌은 모두 0.
  sim 은 track 의 크기 제한/확정 지연을 흉내 내지 않아 사각형 쪽 이점(큰 물체, 지연 없음)은 숫자에 안 나옴
- 확인: 단위 테스트 58개, `update_box` 기하(앞 0.6m 점 → 전환, 옆 0.5m → 안 함, 뒤 0.1m → 셈) 직접 호출로 확인, 브리지 기동. **실물 미확인**
- 웹 표시 (같은 날): 브리지가 `hybrid_box`(사각형 map 꼭짓점, 안의 점 수, 전환 여부)를 0.2초마다 보내고 서버는 저장 없이 중계,
  웹은 로봇 둘레 점선 사각형 — 하늘색: 비어 있음 / 주황: 지도에 없는 점 있음 / 빨강 + 채움: 전환 조건 넘음('장애물 점 N → DWA'). 위치(AMCL)가 잡혀야 보임
- 사용자 params.yaml 변경 같이 반영: `dwa.max_acc_w` 1.5 → 4.0, `hybrid.box_front` 1.0 → 0.5 (sim 비교는 1.0 기준이었음)
- 웹 표시 확장: `hybrid_box` 에 실제로 센 점(사각형 안 지도에 없는 점, 빨간 점) / 사각형 밖 2m 안의 지도에 없는 점(회색 점),
  움직이는 물체 전환 반경(moving_distance 1.5m, 보라 점선 원), 지금 상태 글자('PP 주행' / 'DWA: 앞쪽 장애물 0.42 m (점 5개)' /
  'DWA → PP 대기: 장애물 없음 1.2/2.0s, 경로 0.31/0.25m') 추가. 지도 벽에서 0.15m 안의 점은 벽으로 보고 세지 않음(static_match_dist)

### (브랜치 hybrid-predict-box) 움직이는 물체 전환을 '1.5m 원' → '2초 안에 사각형에 들어오는지' 로 (2026-10-22)
- `bridge_node.moving_into_box()`: 움직이는 track 과 로봇을 지금 속도로 `hybrid.moving_predict`(2.0초)까지 옮겨 사각형과 겹치는 첫 시각.
  겹치면 'DWA: 움직이는 장애물 1.2초 뒤 사각형 (0.90 m)'. `box_trigger=false` 면 예전 moving_distance 원
- 웹: 움직이는 track 마다 지금 → 2초 뒤 예측 점선 (사각형에 들어오면 빨강 + 'n초 뒤 사각형'), 원은 box_trigger=false 일 때만
- sim (21개, 지금 params.yaml = PP 0.18 / box_front 0.5 / max_acc_w 4.0): 원 13/21 충돌 3, 사각형만 14/21 충돌 2, **예측 2초 14/21 충돌 2 (가장 빠름)**, 예측 1초/3초 14/21
- 예전 값(PP 0.26 / box_front 1.0 / max_acc_w 1.5)에서는 원 18/21 충돌 0 (사람 6개 중 5/6 충돌 0, 220초) vs 예측 18/21 충돌 1 (5/6 충돌 1, 282초) → **어느 쪽이 낫다고 하기 어려움**
- 같이 확인: sim 에서 성공을 가장 많이 떨어뜨린 건 `dwa.max_acc_w` 4.0 (1.5 로 두면 14 → 17/21). box_front 0.5 → 1.0 은 14 → 15/21. 실물에서는 지금 값이 잘 된다고 해서 바꾸지 않음

### (브랜치 hybrid-predict-box) 안 쓰는 기능/파일 정리 (2026-10-22)
이 브랜치는 주행을 **A* + hybrid (평소 pure pursuit, 사각형/예측 조건이면 DWA)** 하나로 고정하고 나머지를 뺐다. 다른 방식이 필요하면 `hall-planning` / `hybrid-box-stable` 브랜치.
- 지운 파일: `warehouse_robot/mppi.py`, `test/test_mppi.py` (MPPI), `config/nav2_waffle_pi.yaml` (Nav2 주행 실험 설정), `system_flow.html` (Nav2 follow_path 시절 흐름 설명, 아무 데서도 안 씀)
- 브리지(`bridge_node.py`): `controller` (mppi/dwa/builtin/nav2) / `planner` (nav2) 선택, Nav2 `NavigateToPose`·`FollowPath` 액션과 결과 처리(`on_goal_accepted`, `on_result`, `goal_error`, `path_in_frame`, `on_nav2_plan`, `plan` 토픽 구독), `follow_frame`, `arrive_tolerance`, `mppi.*` 파라미터,
  예전 경로 기준 전환(`hybrid.box_trigger` false, `trigger_distance`, `path_width`, `path_ahead`)과 움직이는 물체 1.5m 원(`moving_distance`) 제거. `on_plan` 은 항상 pp + dwa
- 스크립트: `run.sh` 의 `--nav2` / `--planner` / `--controller` 와 run_mode.env 의 `NAV2` / `PLANNER` / `CONTROLLER`, `start_bridge.sh` 의 planner/controller 인자,
  `start_nav2.sh` 의 Nav2 주행 노드(`--with-navigation`, `--navigation-only`) 제거 → map_server + AMCL 만. launch 의 planner/controller 인자, package.xml 의 nav2_msgs 의존 제거
- 웹: MPPI 파라미터 묶음, 궤적 글자의 MPPI 이름, 1.5m 원 표시 제거. 서버 dwa 중계에서 name 필드 제거
- `tools/planner_compare.py`: MPPI / DWA 단독 / 1초 재계획 / 경로 기준 전환 제거, hybrid 변형(예측 0/1/3초, box_front 1.0, max_acc_w 1.5, 예전 값 3개)만
- 문서: CLAUDE.md / README / RUN_GUIDE / user_guide 에서 위 옵션 설명 제거, README 기본값 표(robot_radius 0.24, max_speed 0.18) 갱신
- 남긴 것: `tools/dwa_tune.py` (DWA 계수 튜닝, hybrid 의 DWA 가 그대로 씀), Nav2 의 map_server/AMCL (`start_nav2.sh`, tf2 0.25.24 주의는 AMCL 에도 해당)
- 확인: 단위 테스트 55개, pyflakes/vulture 깨끗, bash -n, 이 브랜치 코드로 브리지를 다른 도메인에서 기동(hybrid.* 8개만, moving_predict 실행 중 변경), planner_compare / dwa_tune 실행. **실물 미확인**
- 2차 정리 (같은 브랜치): 시뮬레이션 LiDAR 노이즈 기능 제거 — `scan_noise.py`, `test_scan_noise.py`, 브리지 `sim_scan.*`, launch `scan_noise` 인자,
  `run.sh --scan-noise` / run_mode.env `SCAN_NOISE`, `start_bridge.sh` 전달, 웹 실행 모드 'LiDAR 를 실물처럼' 체크(`index.html`, `app.js`, `runner.py`, `consumers.py`).
  `dwa.py` 의 `last_terms`(쓰기만 하고 읽는 곳 없음), 안 쓰는 import 3개(`dwa_tune.py` asdict, `sim_watch.py` re, `test_tracker_dwa.py` transform_points),
  어디서도 안 쓰는 sim 전용 도구 `tools/sim_record.py`, `tools/track_eval.py` 제거. `tools/slam_drift.py`(SLAM map->odom 기록, 어디서도 안 씀), 테스트에서만 쓰던 `tracker.Track.velocity` 속성, `tools/explore.py` 의 부르는 곳 없는 `pick_goal` 도 제거.
  vulture 가 더 잡은 것(발행 메시지 필드 angle_max, TF 수신 유지용 tf_listener, 콜백 인자, Django admin/모델 필드, websocket 콜백 인자)은 아래 확인대로 처리
- 확인: 단위 테스트 52개(scan_noise 3개 빠짐), pyflakes 깨끗, vulture 에 남은 2개도 실제로 쓰임을 확인: LaserScan `angle_max` 는 `scan_slam` 을 받는 slam_toolbox(libtoolbox_common)가 읽음, `_on_close` 의 `code` 는 websocket-client 가 (ws, code, reason) 으로 호출. `tf_listener` 속성은 참조 없이도 TF 를 받는 것을 실험으로 확인해 대입을 뺌, bash -n, 이 브랜치 코드로 브리지 기동. **실물 미확인**
- 전체 점검 (마지막): 추적 파일마다 다른 곳에서 언급되는지 세어 봄 → 빈 파일 `bbackdaegari.txt`, DB 계정 메모 `db_info.md`(같은 값이 `server/.env` 에 있음) 제거.
  나머지 미언급 파일은 문서(AUTO_MAP_FLOW.md 등), pytest 가 찾는 테스트, Django apps.py, README 에 적힌 측정 도구(delivery_demo/latency_test/step_test)라 남김.
  params.yaml 값은 전부 코드가 읽음, 코드가 선언한 파라미터도 전부 읽음(hybrid.box_* 는 p('box_front') 처럼 이름 조합으로 읽음),
  app.js / robot.js 에 부르지 않는 함수 없음, 브리지 ↔ 서버 메시지 종류 일치(_connected/_relocalized 는 브리지 내부 메시지).
  문서에 지운 기능 이름이 남은 곳은 PROJECT_PLAN.md (처음 계획, 기록이라 그대로)뿐

### (브랜치 hybrid-predict-box) PP 앞쪽 정지 → DWA 전환, 경로 복귀 비용은 장애물 없을 때만, progress 항 삭제 (2026-10-23)
- pure pursuit 의 '앞쪽 ±25° 0.35m 안이면 서서 기다리다 5초 뒤 재계획' 삭제 → `hybrid_obstacle` 의 세 번째 전환 조건('바로 앞 0.xx m'):
  지도 벽/점이 적은 가는 물체/갑자기 나타난 물체처럼 사각형이 못 잡는 것도 DWA 가 피하거나 멈출지 정함
- 비상 정지 `estop_distance` 0.2 m (LiDAR 기준, 차체 앞 끝에서 약 7cm): 모드와 상관없이 전진 금지, 회전/후진은 허용
- DWA 경로이탈 비용: `DWAFollower.compute(obstacle_near=)` — 전환 이유가 있는 동안 0, 없으면 `dwa.w_path` 0.6 으로 경로에 다시 붙음
  (예전: 늘 0 이라 목표점만 따라 대각선으로 붙음). 쓰지 않던 `w_progress` 항과 계산 삭제
- sim (`tools/planner_compare.py` 21개, 지금 params): 이전 14/21 충돌 2 → **15/21 충돌 1**. 같은 비용 함수로 DWA 만 쓰면 10/21 충돌 2 (평균 0.053 m/s, hybrid 0.077)
  - 시험했지만 안 넣음: goal 을 목표점 거리로 정규화(14/21, 느려짐), obstacle_influence 0.45(14/21), safety_margin 0.1(13/21), PP 복귀 0.35(15/21, 여유 0.016)
  - `dwa.max_acc_w` 4.0 → 1.5 면 **18/21** (실물에서 정한 값이라 바꾸지 않음)
- 남은 문제: 최소 여유 0.02 m 는 뒤에 붙인 8cm 부착물(제자리 회전 때). DWA/도착 회전 검사는 차체 사각형만 봄 (차체만이면 0.048 m)
- 확인: 단위 테스트 53개(경로이탈 켜고 끄기 1개 추가), pyflakes, 이 브랜치 코드로 브리지 기동(estop 0.2, w_path 0.6). **실물 미확인**

### (브랜치 hybrid-predict-box) 주행 기록 + Frenet 로컬 플래너를 기본으로 (2026-10-23)
- 실물 시험(정지 물체만): `dwa.predict_time` 3.5 → 2.0, `w_obstacle` 1.0 → 0.3 으로도 소화기 앞에서 거의 멈춤
- **주행 기록**: 목표마다 `logs/records/<시각>_<목표>.jsonl.gz` (0.1초마다 위치/명령/hybrid 상태/트래킹/스캔 + 로컬 플래너 입력과 후보별 판정·비용, `record.enabled`).
  분석 `tools/dwa_replay.py` (요약, `--at T [--plot]` 그 순간 설명/그림, `--set 이름=값` 같은 입력으로 다시 계산). 기록 파일은 커밋하지 않음
- 기록으로 본 원인 (`20261023_150655_step-2592`): 경로 4cm 옆 소화기(트래킹 지름 13cm, 정지 물체로 맞게 분류). DWA 는
  ① 목표점이 소화기 바로 뒤 줄 위라 비키는 원호가 손해 ② 원호 하나 2초 유지라 '비켰다 펴기' 후보가 없어 0.5m 틈 아래 벽에 걸림 → 직진하며 감속
  ③ 9.5cm 앞에서 0.007 m/s '가짜 전진'(실제 0)을 안전한 전진으로 골라 후진/회전이 빠짐 → 17초 정지. 거리 인식은 맞았음
- **Frenet 로컬 플래너 (`frenet.py`, `drive.controller: frenet` 기본)**: 핵심 개념은 jeongseonarm/prediction-planning-control (MIT).
  A* 경로 → 스무딩 스플라인 기준선 → 옆 오프셋(±0.3, 0.025) x 옆 이동 거리(0.4/0.6) x 유지(0/0.3/계속, 그 뒤 경로 복귀) x 속도(0.15/0.10/0.05)
  약 450 후보, 매 주기 지금 상태에서. 터틀봇에 맞게: 옆 3차 d(s), 속도는 가/감속 프로필, 정적 장애물은 모양만으로 거리 1m 검사(속도와 무관 →
  느린 후보 편향 없음), 움직이는 물체는 등속 예측 + 여유(시간), 여유 0.06 은 제약/0.15 까지 비용, 비용 = 경로 벗어남/오프셋 바뀜/꺾는 기울기/느림/여유.
  추종은 계획 앞부분 그대로 (v, w = v·κ). 출발 전 돌아서기/도착 방향은 제자리 회전. 다 막히면 'blocked' → 기존 대기/물러나기/A* 재계획
- 실제 기록의 스캔/경로로 폐루프: 소화기 틈을 0.15 m/s 그대로 지나 목표 도착 (최소 여유 0.07, 여유 0.08 이면 경로가 꺾여 비스듬히 지나는 틈을 못 지나가 0.06)
- 계산 5~12 ms. hybrid (PP + DWA) 는 `drive.controller: hybrid` 로 그대로 쓸 수 있음
- 확인: 단위 테스트 61개(Frenet 7개 추가), pyflakes, 브리지를 다른 도메인에서 기동(frenet.* 선언, 실행 중 변경, 목록 값은 재시작), 기록→분석 도구 왕복. **실물 미확인**
- 실물 1차 (Frenet, 2026-10-23 15:53~): 웹에 'DWA' 로 보인 것은 웹 표시 수정 전에 띄운 브리지라서(궤적은 Frenet 이었음).
  ① step-2594 성공(소화기 옆 감속 없음, 최소 0.09 m) 이지만 실제 속도 0.23~0.25 m/s: 경로 방향 속도만 0.15 로 제한, 옆으로/커브 바깥에서 더 빨라짐
  → 속도 프로필을 실제 속도 기준으로(ṡ = V / 비율), 명령도 max_speed 이하. ② step-2593 출발 못 함: 구석(앞 벽 13 cm, 옆 9 cm), 경로가 130° 뒤.
  원 4개 차체 모델이 앞뒤 12 cm 커서 다 막힘 → 정적 검사를 직사각형 둘레 점 + 뒤 부착물(footprint_extra) 로. 그래도 제자리 회전은 양쪽 다 닿음
  → 가까운 쪽이 막히면 먼 쪽으로, 둘 다 막히면 1초 앞/뒤 짧은 이동으로 회전 중심 둘레를 넓힌 뒤 돎(make_room), 여유 안에서는 2 cm 까지 가까워져도 됨(1 cm 아래 금지)
  → 두 기록의 스캔으로 폐루프: 둘 다 목표 도착. 테스트 63개. 계산 13~24 ms
- 실물 2차 (Frenet, 16:11~, 3번 모두 도착): 2595 매끄러움(0.15 m/s, 최소 0.095). 2596/2597 은 막힘이 여러 번:
  ① 경로와 64~68° 틀어진 출발에서 후보 450개가 전부 '운동 한계' — 실제 속도 검사가 프로필에 너무 딱 맞춤 → max_speed 기준으로
  ② 돌던 방향이 막히면 바로 포기해 돌기/막힘 반복 → 반대 방향, 돌 자리 만들기까지. 플래너가 막혔는데 그 자리 경로 방향과 30° 넘게
  틀어져 있으면(움직이는 물체가 가까워 돌아서기를 안 한 경우 포함) 제자리 회전 시도
  ③ 상자(트랙 반경 0.15)와 비스듬한 벽 사이 0.54 m 틈: 가장 나은 후보 여유 0.051 < 0.06 → 5초 뒤 물러나 재계획해서 통과.
  safety_margin 0.05 면 바로 통과 (실제 최소 0.050) — 기본은 0.06 그대로
  ④ 사람이 가까이 지나갈 때(트랙 0.1~0.3 m/s) 1초 정도 '움직이는 물체' 로 막힘: 의도한 동작
  → 기록 스캔으로 폐루프: 2596 이 49초 → 29초, 2593/2592 그대로 도착. 테스트 63개
- 실물 2차 나머지 (2598, 2599 도착, 막힘 많음): ① 정적으로 막힘 — A* 경로가 아직 몰랐던 물체 옆 좁은 틈으로 남(2599 최대 여유 0.032),
  상자 옆 틈 0.057 < 0.06(2598) → 5초 대기+물러나기+재계획에 매번 약 8초. **정적 장애물만으로 막히면 1초 뒤 바로 A* 재계획** (목표당 4번)
  ② 사람이 지나갈 때 '움직이는 물체' 로 잠깐 막힘(의도) ③ odom 속도가 한 샘플씩 0.87~1.1 m/s 로 튐 (명령은 0 또는 0.18): 측정 튐/미끄러짐
- 인지 범위: 정적 격자를 후보가 지나가는 곳 + 차체 + clear_pref 로(뒤는 차체 뒤 끝 근처까지), 회전은 둘레 0.6 m, 움직이는 물체는
  horizon 안에 닿을 수 있는 것만 + **차체 뒤 끝보다 뒤에 있는 순간은 무시** (뒤에서 따라오는 물체 때문에 서지 않게). 계산 시간은 비슷(13~23 ms):
  병목이 격자(0.6 ms)가 아니라 후보 계산 전체
- 기록 폴더: `logs/records/<브리지 켠 시각>_<frenet|hybrid>/<시각>_<목표>/record.jsonl.gz`, 분석 그림도 그 폴더(plot_<t>.png). 예전 기록도 옮김.
  `tools/dwa_replay.py --list` 로 세션/주행 목록. 테스트 64개
- (2026-10-23 저녁, 로봇 꺼진 뒤) 실물 2610 삐걱거림 원인: ① 경로가 급히 꺾이는 곳에서 '15 cm 앞 점' 기준 돌아서기가 주행 중 걸림
  ② 좁은 곳에서 오프셋이 주기마다 바뀜(통과 후보 10~20개, w -0.1 ↔ -0.8). 오늘 실물 기록 18개를 폐루프로 다시 달리는 `tools/frenet_bench.py`
  (기록마다 여러 주기에 보인 스캔 점 = 환경, A* 재계획 흉내) 로 비교하며 수정:
  - 돌아서기를 '그 자리 경로 방향' 기준으로, 달리는 중이면 100° 넘을 때만 → 주행 중 회전 8 → 2
  - 그러자 목표 옆을 9.7 cm 로 지나쳐 끝 너머로 계속 가는 경우가 드러남 → 경로 끝을 지나면 16 cm 안이면 도착, 아니면 목표 보고 천천히 접근
  - 오프셋 바뀜 비용 w_change 0.3 → 1.0, 경로로 돌아오는 쪽은 1/4 (대칭이면 장애물 지나서도 안 돌아옴) → 바뀜 181 → 107, 도착 수 같음
  - 시험했지만 안 넣음: 각속도 변화 제한(흔들림이 대부분 제자리 회전/도착 회전이라 효과 없음), lateral_dists 바꾸기([0.3,0.5]~[0.6,0.9] 차이 없음)
  - 경로 끝을 지난 상태에서 검사 구간이 비면 예외가 나던 버그 수정 (브리지가 죽을 수 있었음)
  - **실물 미확인** (다음 주 실험)

### (브랜치 frenet-only, hybrid-predict-box 에서 갈라짐) 안 쓰는 PP / DWA / hybrid 전부 제거 (2026-10-23)
- 주행은 A* + Frenet 로컬 플래너 하나. 예전 방식(PP + DWA hybrid)이 필요하면 `git checkout hybrid-predict-box`
- 지운 것: `follower.py`(PP), `dwa.py`(DWA), 브리지의 hybrid 전환(전환 사각형/움직이는 물체 사각형 예측/DWA 계산/PP 준비/`drive.controller`),
  설정 `dwa.*` `dwa_follow.*` `hybrid.*` `drive.*` 와 PP 전용 `max_speed` `max_turn_speed` `max_accel` `max_turn_accel`,
  테스트 `test_follower.py` `test_dwa_closed_loop.py` 와 `test_tracker_dwa.py` 의 DWA 부분, 도구 `dwa_tune.py` `planner_compare.py`,
  웹 hybrid 전환 사각형 그리기·중계, 주행 파라미터 카드의 DWA/hybrid 묶음
- 옮긴/바꾼 이름: `dwa_follow.moving_speed` → `tracking.moving_speed`, `dwa.footprint` → `footprint`, `transform_points` → frenet.py,
  웹 메시지 `dwa` → `local_plan`, 분석 도구 `tools/dwa_replay.py` → `tools/drive_replay.py`(Frenet 전용, 예전 PP/DWA 기록은 사건/표만),
  `test_tracker_dwa.py` → `test_tracker.py`, 기록 세션 폴더는 항상 `_frenet`
- `stop_distance` 는 남김: 기운 채 앞이 막힌 '걸침' 감지에 씀. 경로 점이 하나뿐인 목표(이미 목표 칸 안)는 5 cm 경로로 바로 도착 → 방향만 맞춤
- `tools/frenet_bench.py` 는 이제 기본이 지금 params.yaml 값 (`--recorded` 면 기록 당시 값)
- 문서: CLAUDE.md, README, RUN_GUIDE, user_guide, AUTO_MAP_FLOW, guide/architecture*.html, run.sh/start_nav2.sh 주석
- 확인: 단위 테스트 35개, pyflakes, 브리지를 다른 도메인에서 기동(파라미터 목록, 실행 중 변경), 브리지 기록 메서드 → drive_replay 왕복
  (다시 계산 = 기록), 실물 기록 폐루프 비교 도구. **실물 미확인**

### (브랜치 frenet-only) 주문 흐름: 사람 위치 + 물건별 개수 → 구역들 적재(YOLO 개수 확인) → 사람 → 홈 (2026-10-26)
- 시나리오: 홈에서 대기 → 사람이 자기 위치(예: 골방)와 `{led_red: 1, led_green: 2}` 주문 → 구역(물건 종류마다 하나)을 가장 빠른 순서로
  돌며 그곳 사람이 싣고 "적재 완료" → YOLO 종류·개수가 요구와 다르면 "적재 잘못" 표시, "다음 경유지로" 로 강제 진행 가능 →
  사람 위치에서 꺼내고 "꺼냈어요"(YOLO 에 안 보여야) 또는 "강제 보내기" → 홈
- 서버: `services.start_order` / `preview_order` / `force_next`, 판단은 `check_counts` 한 곳(지금: 주문 라벨만, 정확히 같아야),
  적재 기대값은 **누적**(카메라가 적재함 전체를 봄), YOLO 개수는 브리지가 이미 보내던 `yolo` 메시지를 최근 3초 쌓아 가장 많이 나온 값.
  주문 시작 때 YOLO 켜고(`yolo_set`) 끝/취소 때 끔. 순서는 기존 `planning.optimize` (마지막 = 사람 위치)
- DB: `TaskStep.items` (JSON, 비면 예전 작업과 같음), `Location.kind` 에 `dropoff`(사람 위치) — 마이그레이션 0006
- 웹: 관제 "주문" 카드(받을 곳, 물건별 개수, 예상 순서), 위치 종류 '사람 위치', 태블릿 적재/수령 화면(YOLO 개수 표, 버튼 2개)
- 같이 고침: 재연결 때 수령 대기 단계가 WAITING_LOAD 로 돌아가던 것
- 확인: 임시 sqlite DB 로 서버 흐름 전체(틀린 적재 → 대기, 맞음 → 다음, 누적 확인, 덜 꺼냄 → 대기, 강제 → 홈, 끝나면 YOLO 끔),
  JS 문법, 페이지 로드. **실물·브라우저 조작 미확인**. 알아 둘 것: 브리지/주행 코드는 안 바뀜.
  기존 LED 배달의 자동 확인(`cargo/detected`)은 실물에서 내는 노드가 없어 버튼으로만 넘어감 (주문과 무관)
- (같은 날) 섹션 방식으로 바꿈: 위치 하나(섹션)에 물건 여러 종류 (인식 라벨 칸에 쉼표, 길이 50 → 200, 마이그레이션 0007),
  주문은 섹션별로 묶임. 판단은 **그 섹션 물건만** (누적 아님 — 앞에서 덜 싣고 보내도 다음 섹션이 계속 틀리지 않게).
  "적재 완료/꺼냈어요" 가 YOLO 와 다르면 태블릿이 물어봄 → "확인했어요, 보내기"(`force_next confirmed`) / "다시 싣기".
  확인: 임시 DB 흐름(두 종류 섹션 불일치 → 확인 후 진행, 다음 섹션은 자기 물건만 봄, 꺼냄 → 홈), JS 문법. **실물·브라우저 조작 미확인**
- (같은 날) 관제 화면의 "LED 요청" 카드와 로봇 카드의 '카메라 인식(cargo/detected)' 줄 제거 (주문으로 대신, 섹션 라벨과도 안 맞음).
  서버 `start_delivery`/`handle_cargo`, 태블릿의 예전 LED 화면 분기, `cargo_detector`, `tools/cargo_sim.py` 는 남김
- (같은 날) 사용자 화면 `/order` (`order.html` / `order.js` / `order.css`): 받을 위치 → 물건 개수 −/+ → 예상 순서 → 주문,
  주문 뒤 진행 상황(단계별 ✓/▶, 남은 시간, YOLO 불일치 안내, 도착 알림), 주문 취소. `?place=<id>` 로 위치 미리 고름, 고른 위치는 기기에 기억.
  관제 화면 주문 카드에 '사용자 화면 ↗' 링크. 확인: 가짜 데이터로 입력/진행 화면 그림 확인, JS 문법. **실제 주문으로는 미확인**
- (같은 날) 겹친 물건 개수 판정 때문에 Pi 카메라 320x240 → 640x480 (`yolo/camera.sh` CAMERA_SIZE 기본), YOLO imgsz 320 → 640.
  **실물 미확인**: Pi 카메라 노드를 다시 띄워야 적용(이미 떠 있으면 run.sh 가 그대로 둠), 압축 영상 대역폭·Pi JPEG 부하·PC YOLO 계산 시간이 늘어남
- (같은 날) 안 쓰는 색 기반 LED 인식 노드 삭제: `cargo_detect.py`, `cargo_detector_node.py`, `test_cargo_detect.py`, setup.py 의
  `cargo_detector` 실행 이름. 물건 인식은 `yolo/` 의 YOLO 검출기만 씀
- (같은 날) Pi 카메라 기본 해상도를 다시 320x240 으로 (`yolo/camera.sh` CAMERA_SIZE 기본, Pi 의 `~/camera.sh` 와 같음). YOLO imgsz 는 640 그대로.
  640x480 은 압축 영상도 약 5~6 Mbit/s (프레임 55 KB) 라 Pi Wi-Fi(송신 7~26 Mbit/s)에 부담. 320x240 은 약 1.6 Mbit/s
- (같은 날) 주문 중 YOLO 는 섹션/사람 위치에 서 있을 때만 켬 (도착 → 켬, 적재 완료/꺼냄/확인 후 진행/건너뛰기로 출발 → 끔,
  대기 중 브리지 재연결 → 다시 켬). 주행 중 카메라 영상 Wi-Fi·PC CPU 를 주행에 양보. 확인: 임시 DB 로 주문 전체의 켜기/끄기 순서
- (같은 날 오후) **실물 통신 끊김 원인 조사 → Pi 카메라는 YOLO 를 켤 때만**: 오늘 '위치 TF 지연 정지'(0.51~0.55초) 9번, Pi 데이터 2초 끊김,
  그리고 명령이 9초 동안 안 들어가 도착 후 벽에 부딪힘(160957, 브리지는 v=0 을 보냈는데 실제 0.12~0.18 m/s 로 계속 감). 측정: Pi 안에서는
  오도메트리/LiDAR 가 규칙적(최대 64 ms) → 오는 길(Pi Wi-Fi)에서 끊김. PC 부하·시계·RAM·디스크·Wi-Fi 절전 아님. 배터리 16% 에서도 악화.
  새 배터리: 카메라 끔 3번 끊김 0, 카메라 켬(320x240, 구독자 0) 3번 중 끊김 2(충돌 포함), Wi-Fi 전송 실패 분당 ~5 → ~18.
  → `scripts/pi_camera.sh on|off|status` (켜기 ~5초, 끄기 ~2초), 브리지가 YOLO 켜기/끄기 때 같이 부름 (한 작업 스레드가 마지막 요청만 반영,
  시뮬레이션은 안 함), `run.sh` 는 기본으로 카메라를 끔 (`--camera` 면 계속 켬). 확인: 실제 Pi 로 켜기/끄기/상태, 가짜 스크립트로 빠른 반복 요청.
  남은 일: **Pi 쪽 명령 감시(통신 끊기면 로봇이 스스로 정지)** 는 아직 없음, 도착 후 방향 맞추기 회전에 충돌 검사 없음, 골방(-2.42,1.94)/led_green 위치가 벽에 너무 가까움
- (같은 날) **Pi 쪽 명령 감시 (통신 끊기면 로봇이 스스로 정지)**: `scripts/pi_cmd_watchdog.py` 를 실물 `run.sh` 가 Pi 에 복사해 bringup 뒤에 띄움
  (매번 다시 띄움, 로그 Pi `~/warehouse_cmd_watchdog.log`). 브리지는 실물이면 `cmd_vel_pc` 로 보내고(시뮬레이션은 `cmd_vel` 그대로),
  감시 노드가 `cmd_vel` 로 넘기다가 마지막 명령이 움직임인데 0.5초 넘게 새 명령이 없으면 정지(0)를 5번 보냄. 감시 노드가 없으면 로봇이
  안 움직임 (브리지 로그에 10초마다 경고). 확인: 실제 Pi 에서 이름을 바꾼 토픽으로 (로봇 안 움직이게) 0.1 m/s 명령 → 0.8초 끊김 → 다시 →
  끊김: 두 번 다 끊긴 뒤 0.53~0.54초에 정지, 다시 받으면 그대로 넘김. **실제 run.sh 로 띄운 주행은 아직 미확인**
- (같은 날) **도착 후 방향 맞추기 회전에 충돌 검사** (`FrenetFollower._align`): 회전 중 차체(+부착물)가 안전 여유(6 cm, 이미 안이면 지금보다
  가까워지지 않게) 안으로 들어가면 반대쪽으로, 둘 다 안 되면 지금 방향에서 끝냄(기록 debug `align: skipped`). 정한 방향은 유지 (예전: ±w 번갈아).
  예전에는 검사 없이 돌아 뒤 부착물이 벽 0~0.8 cm (2609, 2627, 2636 등). 실물 기록 7개 폐루프: 좁은 곳에 도착하는 주행의 최소 여유
  0.000~0.010 → 0.110~0.200 m (2606, 2626, 2635, 2662), 옛 골방 2609 0.000 → 0.014, 7/7 도착. 단위 테스트 3개 추가 (34개)
- (같은 날) 위치 이동 (DB): 골방 (-2.42, 1.94) → **(-2.42, 1.73)** (구석에서 21 cm, 6 cm 여유로 제자리 회전 가능한 가장 가까운 자리.
  옛 자리는 벽까지 약 0.30 m 라 회전 반경 0.28 m 에 2 cm 만 남음 → 안전 여유는 안 낮춤), led_green → (-2.47, -0.02). 둘 다 A* 경로 확인
- (같은 날) 주행 기록에 바퀴 엔코더(`wheels`, joint_states rad/s)와 IMU 회전 속도(`imu_wz`) 추가: 실물에서 **회전이 명령의 약 절반**
  (Frenet 주행 1.5초 구간 실제/명령 0.53, 제자리 회전 1.0 rad/s 명령 → 오도메트리 0.51 / AMCL 0.58, 직진 속도는 0.97 정상, 끌림 없음 확인).
  바퀴부터 덜 도는지(모터/펌웨어) 바퀴는 도는데 몸체가 덜 도는지 다음 실험 기록으로 가림. 명령→실제 각속도 지연 약 280 ms
- (같은 날 저녁) **'위치 TF 끊김' 원인 측정 → 브리지 수정**: 시나리오 6.5분 동안 PC·Pi 양쪽 동시 기록 (도착 간격, 스탬프, ping 20Hz,
  Pi 코어별 CPU 0.1초, Wi-Fi 카운터). Pi 안에서는 odom/tf/scan/엔코더/IMU 전부 규칙적(0.25초 넘는 끊김 0), PC 에서는 전부 같은 순간에 끊김
  (odom 최대 0.41초, scan 0.72초, **/tf 2.4초**), 그때마다 ping 4 → 150~654 ms → **네트워크 지연** (Pi CPU ≤47%, 카메라는 내내 꺼져 있었음,
  주행/정지 둘 다). 브리지의 TF 나이와 별도 측정 프로그램의 TF 나이가 같은 순간 같은 값 → 브리지 내부 지연 아님.
  /tf 는 신뢰성 전송이라 패킷이 빠지면 재전송까지 뒤 데이터도 멈춤: 0.5초 넘는 사건 /tf 3번(최대 2.4초) vs /odom 1번(최대 0.67초).
  → 브리지: 로봇 위치를 /odom 메시지로 (map→odom 만 TF, AMCL 은 1초 앞 스탬프), 위치가 0.5초 늦으면 서서 기다렸다가 다시 오면 이어 감,
  `pose_lost_timeout`(3초) 넘게 안 오면 그때 실패 (예전: 한 번 늦으면 바로 목표 실패). 기울었을 때 '위치 끊김 → 물러나기' 분기는 뺌
  (예전엔 기울면 AMCL 스캔이 멈춰 map TF 가 늦어졌지만 이제 나이는 odom 기준, 기울기 부딪힘은 bump_tilt_deg 가 처리).
  카메라 원인이라고 했던 앞의 판단은 틀림 (배터리·표본 수와 섞였음). 근본 원인(공용 공유기 혼잡/간섭 추정)은 전용 공유기로 확인 필요. **실물 미확인**
- (같은 날 저녁) 수정 후 실물 7개 주행: 위치 지연 실패 0 (전에는 9분에 4번), '서서 기다림' 13번 모두 0.0~0.4초 만에 자동 재개.
  **다음에 할 일**: ① 카메라 계속 켜 두기로 되돌리기 (`run.sh` 기본 켬, 브리지의 YOLO 켜기/끄기 때 카메라 켜고 끄기 빼기) — 측정상 카메라는
  끊김 원인이 아니었고(카메라 끈 6.5분에도 끊김 11번), 도착 때 켜면 실제로 10~15초 걸려 섹션에서 YOLO 를 못 씀 (YOLO 는 서 있을 때만 그대로).
  되돌린 뒤 카메라 켠 상태 끊김이 끈 상태보다 많은지 다시 비교. ② 오늘 기록의 바퀴 엔코더로 '회전이 명령의 절반' 원인 분석
  ③ 제자리 회전 시험 (0.5 rad/s 6.3초 → 반 바퀴인지) ④ 남은 짧은 멈춤은 공용 공유기 지연 → 전용 공유기 검토
- (2026-10-27) **Pi 카메라 계속 켜 두기로 되돌림**: `run.sh` 기본으로 켬 (`--no-camera` 로 빼기), 브리지의 YOLO 켜기/끄기 때 카메라 켜고 끄기 뺌.
  YOLO 검출기는 그대로 서 있을 때만 켜서 영상은 그때만 Wi-Fi 로 옴. `scripts/pi_camera.sh` 는 run.sh 와 수동용으로 남김
- (2026-10-27) YOLO 입력 기본을 모두 압축 영상(`/csi/image_raw/compressed`)으로: 실제로 쓰는 `infer_ros_switch.py` 는 이미 압축이었고,
  나머지 `yolo/yolo_training/infer_ros.py`(VS Code 실행기 `python_project/yolo_ros.py` 가 띄움), `zoom_camera.py` 도 `compressed:=true` 기본
  (false 면 예전처럼 원본). 모델은 JPEG 사진 + JPEG 압축(75~95)·해상도 축소 보강으로 추가 학습돼 압축 입력이 학습 조건과 같음
- (2026-10-27) **'회전이 명령의 절반' 원인 확정 → OpenCR 펌웨어를 Waffle 용으로 다시 올림**: 보드에서 읽은 model_information = 1 (Burger).
  펌웨어 모델표: 1 = Burger 바퀴 간격 0.160 m, 2 = Waffle 0.287 m → 회전 명령을 버거 간격으로 계산해 실제 회전이 0.160/0.287 = 0.557 배
  (측정: 바퀴 엔코더 0.55, IMU 0.53, 이틀 같음, 배터리 16%~91% 무관, 직진 속도는 0.96 정상). Pi 의 `~/opencr_update/update.sh /dev/ttyACM0
  waffle.opencr` (bringup 끈 채로) → model_information = 2 (Waffle), 펌웨어 V230127R1. 확인: 다음 주행 기록의 엔코더/명령 회전 비율이 약 1.0 인지.
  되돌리기: 같은 명령으로 burger.opencr. 제자리 회전이 이제 설정대로(예전 실제 0.55 rad/s → 1.0) 빨라짐, 그 뒤 곡선 감속(frenet.max_w) 조정 예정
- (2026-10-27) 목표가 끝날 때마다 그 기록 폴더에 `summary.txt`(요약) + 중요한 순간 그림 `plot_*.png` 자동 생성 (브리지가 `tools/drive_replay.py`
  를 nice 19 로 따로 띄움, 끄기 `record.plots: false`). 오늘 폴더 28개는 손으로 채움. 좁은 틈 진입 실패 2번 (114731, 121436: 지도 (-1.15, 1.97) 근처
  약 0.5 m 틈) 은 안전 여유 때문: 가장 나은 후보 0.050~0.054 m < 0.06, 그 순간을 0.05 로 다시 계산하면 둘 다 출발 (그때 적용된 값은 0.06)
- (2026-10-27) **브리지가 죽던 버그 수정**: 목표가 바로 옆이라 A* 경로가 15 cm 안 되면(3개 점) Frenet 기준선 `_polyline` 의 곡률 5점 평균이
  길이를 5 로 만들어 예외 → 1점 경로용 대비책(5 cm 경로)도 같은 예외 → 브리지 종료(웹 '오프라인'). 점이 5개 미만이면 평균을 안 함.
  브리지는 기준선을 못 만들면 죽지 않고 그 목표만 실패 처리. 테스트 추가 (35개)
- (2026-10-27) **정적 여유를 후보 속도별로** (`frenet.safety_margin` 0.06 = 0.15 m/s, `frenet.safety_margin_slow` 0.04 = 0.05 m/s, 사이는 비례):
  좁은 틈은 느린 후보만 통과해 천천히 비집고, 넓은 곳은 그대로 6 cm. 이미 여유 안(빠를 때 여유 기준)이면 가장 느린 속도만 2 cm 더 가까워져도 됨
  (빠른 속도에도 주면 좁은 틈 안에서 0.15 m/s 로 갔음, 가장 느린 여유 기준으로 하면 4.2 cm 에서 출발하는 2728 이 막혔음 → 둘 다 고침).
  확인: 오늘 막혔던 4 순간(2715, 2729, 2732, 2740) 모두 0.03~0.05 m/s 로 출발, 오늘 기록 34개 폐루프 도착 31/34 로 같음, 2715 모서리 막힘 없이 통과,
  2745 최소 여유 0.028 → 0.076. 단위 테스트 2개 추가 (37개). 실험 중 0.04 고정으로 달린 3번은 벽까지 최소 2.4/4.7/4.5 cm, 기울기·끼임 없음
- (2026-10-27) LiDAR 가 지도와 어긋나 보이는 건 빠른 제자리 회전 때만: 지도 벽 10 cm 안 점 비율 거의 안 돌 때 94%, |w|≥0.6 rad/s 때 85%(하위 10% 71%).
  펌웨어 수정 뒤 회전이 실제로 2배 빨라져 잘 보임. 줄이려면 `frenet.rotate_speed` 1.0 → 0.6 (도착 후 방향 맞추기는 아직 max_w 까지)
- (2026-10-27) 웹 LiDAR 선이 지도와 어긋나 보이던 것: 위치 추정은 대체로 맞음 (다시 띄운 뒤 5 주행, 지도 벽 10 cm 안 점 중앙 91~95%, 순간 최저 59~71%).
  웹 표시만 스캔을 '가장 최근 자세'로 지도에 놓아 회전 중에 더 돈 방향으로 그려졌음 → 스캔 찍은 시각의 TF 로 (없으면 최근 것). 표시만 바뀜.
  11:39~11:44 세션(113908~114344)은 지도 맞음 7~49% 로 위치 추정이 크게 틀어져 있었음 (초기 위치 잘못 지정으로 추정)

### 버전 고정: `stable-2026-10-27` (브랜치 frenet-only)
실물에서 잘 되는 것으로 확인한 상태. 되돌리기: `git checkout stable-2026-10-27` (그 뒤 colcon build --symlink-install, run.sh).
- 주행: A* + Frenet (속도별 정적 여유 0.06/0.04, 도착 회전 충돌 검사, 위치는 /odom + 늦으면 서서 기다림, Pi 명령 감시)
- 주문/YOLO: 섹션 단위 주문, 서 있을 때만 YOLO, 카메라 계속 켬, 압축 영상 입력, 가중치 `best_components7_finetuned.pt` (10-26 15:25 판)
- 기록: 목표마다 record.jsonl.gz + summary.txt + plot_*.png
- **git 밖에 있어서 같이 맞춰야 하는 것**: OpenCR 펌웨어 Waffle (model_information 2, V230127R1), Pi 카메라 320x240 (`yolo/camera.sh`
  를 run.sh 가 복사), Pi 명령 감시 (`scripts/pi_cmd_watchdog.py` 를 run.sh 가 복사), DB 위치 (map2: 섹션 led_yellow/led_red/led_green, 하역장, 홈)
- (2026-10-27) 소화기 실험(141220 세션) 분석:
  - 홈 복귀 실패(2760 두 번)는 소화기와 무관한 버그: 기준선 끝 너머 직선 연장선(1.5 m)이 지나온 길 옆을 지나가(홈에 서쪽을 보며 도착)
    로봇을 연장선 위로 투영 → '경로 끝을 지남' 으로 보고 1 m 앞에서 정지 (막힌 주기 121/121, 83/105). 투영을 s_end 까지로 제한 + 테스트.
  - 2761, 2771 막힘: 소화기 사이 틈이 A* 반경(0.24, 폭 0.48 m)보다 좁아 기억 장애물 포함 계획이 '경로 없음' → A* 가 맵만으로 다시
    계획해 소화기 4 cm 옆을 지나는 경로를 냄 → Frenet 은 가장 나은 후보 2.2~3.6 cm < 4 cm 라 못 감 → 재계획해도 같은 경로 반복.
    즉 'A* 가 낸 경로는 Frenet 이 갈 수 있다' 는 A* 가 장애물을 알고 계획했을 때만 성립 (중심 0.24 → 옆 8.7 cm). 맵만으로 다시 계획한 경로는 보장 없음.
  - 도착 회전: 실제 2.0~2.4 rad/s 로 150~176° 를 2초에. 지도 위 위치 흔들림 2~5 cm(작음), 방향은 오도메트리와 AMCL 이 0.6~8° 차이(회전마다 보정),
    회전 끝 LiDAR-지도 맞음 67~80% 가 11번 중 6번, 다음 출발 때도 64~75% 가 10번 중 3번 (회전 전 80~100%).
- (2026-10-27) 소화기 실험 결과 반영 (실물 확인 전):
  - A* 기억 장애물 때문에 경로가 없으면 맵만으로 다시 계획하던 것 → ① 반경 `fallback_radius` 0.20 ② 최근 `scan_recent` 3초에 다시 본
    장애물만 ③ 그래도 없으면 '지나갈 틈이 없습니다' 로 실패 (막던 물체를 최근에 못 봤으면 맵만으로). 오늘 32번 중 11번이 맵만으로 → 소화기 4 cm 옆.
  - A* lethal 안 출발 시 빠져나가기는 장애물에서 멀어지는 칸으로만 (예전: lethal 띠를 따라 좁은 틈 통과 → 2771 에서 '피해서 계획' 도 소화기 4 cm 옆). 테스트 추가.
  - 계획마다 기억 장애물 상태 로그 (칸 수/나이, 비었으면 지움/만료/아직 없음).
  - 도착 회전을 rotate_speed(1.0) 까지로 (예전 max_w 1.82 → 실제 2.0~2.4 rad/s, 회전마다 방향 최대 8° 보정).
  - 오늘 배치의 소화기 옆 틈은 맵 벽 + 맵에 없는 북쪽 물체 사이 약 0.26~0.40 m (차체 0.306) → 새 코드로는 이 틈을 '못 지나감' 으로 판단해야 맞음.

### 버전 고정: `stable-2026-10-27-2` (브랜치 frenet-only)
`stable-2026-10-27` 이후: Frenet 기준선 투영 수정(경로 끝 오판), A* 기억 장애물로 막히면 반경 0.20 → 최근 3초 → 실패(맵만으로 안 감),
lethal 빠져나가기는 멀어지는 쪽으로만, 계획마다 기억 상태 로그, 도착 회전 rotate_speed(1.0) 까지.
- 소화기 통로 가운데(통로 0.95 m) 실험 151830: A* 반경 0.20 단계 14번 모두 소화기 옆 틈으로 경로, 맵만으로 계획 0번.
  Frenet 최선 후보 3.0~3.6 cm < 4 cm 로 막힘 (틈 ≈ 0.37 m), 통과한 경우 최소 2.9~5.5 cm, 부딪힘 없음. 8번 중 6번 성공.
- 결정: 시나리오는 쉬운 배치로 (소화기를 한쪽으로, 넓은 쪽 틈 50 cm 이상). 여유를 3 cm 로 줄이는 건 부딪힐 위험이 있어 안 함.
- 도착 회전 1.0 제한: 방향 보정 최대 7.6° → 4.4°, 회전 끝 지도 맞음 80% 미만 3/8 → 1/4 (회전 4번뿐이라 더 모아 볼 것).
- 나중 업그레이드 후보: A* 경로를 직사각형 차체로 검사, 주기적 A* 재계획 (경로 교체 흔들림 방지 장치와 함께).

### 시뮬레이션 SLAM 맵이 무너지던 문제 → 시뮬레이션만 예전 방식으로 (2026-10-28)
- 현상: sim turtlebot3_house 자동 탐색 중 벽이 두꺼워지고 두 겹, 방마다 다른 각도로 돌아가며 맵 전체 기울기가 바뀜 (LiDAR 와 맵이 완전히 다름)
- 측정 (탐색 중 4분): map→odom 회전이 직진 중에 한 번에 2~8° 씩 튀고 되돌아감 (−2.6 ~ +13.4°). 처리 지연 아님 (slam_toolbox CPU 4.8%)
- 시뮬레이션 오도메트리는 원인 아님: /odom 과 Gazebo 실제 위치(/get_entity_state) 199번 비교, 차이 최대 2.4 cm / 1.6° (평균 0.38°), 쌓이지 않음
- 마지막 정상 확인 10-21 10:21 (강당 8.6분 탐색) 이후 시뮬레이션 SLAM 에 영향 준 변경 두 가지:
  ① `minimum_travel_distance/heading` 0.5 → 0.2 → 0.1 (244ff6e, a894acb): slam_toolbox 의 `scan_buffer_size`(10), `loop_match_minimum_chain_size`(15) 는
     스캔 개수 라 간격을 줄이면 맞춤 기준 약 5 m → 1 m, 루프 클로저 인정 구간 약 7.5 m → 1.5 m 로 약해짐 → LiDAR 3.5 m, 반복되는 넓은 house 에서 잘못된 맞춤/루프 클로저
  ② SLAM 스캔 240 칸 재배열 (6d119cf, 실물 LDS-02 빔 개수 변동용): 시뮬레이션 1° 360 빔을 1.5° 로 줄여 벽 점 최대 4.5 cm, 각도 ±0.5° 어긋남
- 실물은 같은 설정으로 정상: map2 (10-22, 0.1 간격 + 재배열), map1027 (10-27, 와플 펌웨어 후) 모두 벽 한 겹, 틀어짐 없음 (작은 방 + LiDAR 5 m)
- 수정 (실물은 그대로): `start_slam.sh` 시뮬레이션은 스캔 반영 간격 0.5 m / 0.5 rad, 브리지는 시뮬레이션(use_sim_time)이면 빔 재배열 없이 inf 변환만 (예전 방식)
- 확정 실험(같은 house 에서 현재 / 0.5 / 루프 클로저 끔 비교)은 아직 안 함. 실물로 강당처럼 넓은 곳을 SLAM 하면 같은 문제가 날 수 있음 (강당 sim 은 0.5 에서만 확인)
- (같은 날) 간격 0.5 로 되돌리니 벽이 0.5 m 움직일 때마다만 들어가 가까이 가야 맵 벽이 됨 (실물에서 0.1 로 줄였던 이유와 같음) →
  시뮬레이션은 간격 0.1 그대로, 개수 기준 두 값을 5배: `scan_buffer_size` 10 → 50 (맞춤 기준 약 5 m), `loop_match_minimum_chain_size` 15 → 75
  (루프 클로저 인정 약 7.5 m) → 거리 기준이 0.5 시절과 같음. `start_slam.sh` 가 시뮬레이션 실행용 사본에만 적용, 실물(config/slam_toolbox.yaml)은 그대로.
  빔 재배열 안 함(시뮬레이션)은 유지. **house 에서 맵이 안 무너지는지 아직 확인 안 함**

### 시뮬레이션을 실물과 같은 사양·같은 처리 경로로 (2026-10-28)
시뮬레이션만 SLAM 설정을 따로 두면 실물과 차이가 커져서, 설정이 아니라 시뮬레이션 로봇을 실물에 맞춤. 시뮬레이션 전용 분기는 모두 뺌.
- 로봇 모델 `worlds/models/turtlebot3_waffle_pi_real/model.sdf` (turtlebot3_gazebo 2.3.8 Waffle Pi 사본): LiDAR 를 실물 LDS-02 측정값(real_scan_capture 187 스캔:
  빔 245~248, 약 1.45°, 약 9.3 Hz, 0.159 m 부터, 10 m 넘는 응답 없음)으로 247 빔 / 9.3 Hz / 0.16~10 m (원본은 LDS-01: 360 빔, 5 Hz, 0.12~3.5 m). Gazebo 오도메트리 끔
- `scripts/sim_real_io.py` (start_sim.sh 가 띄움): 스캔을 실물 드라이버 형식으로 (안 맞은 빔 NaN, range_max 100), 오도메트리는 실물 turtlebot3_node 처럼
  바퀴 엔코더(이동) + IMU(회전)로 /odom 과 TF (원점 = 출발 자리, 예전엔 Gazebo 실제 위치 그대로라 오차 0)
- 명령: 시뮬레이션도 `pi_cmd_watchdog.py` (start_sim.sh 가 PC 에 띄움)를 거침 → 브리지는 항상 `cmd_vel_pc`
- 같은 날 넣었던 시뮬레이션 전용 값 제거: start_slam.sh 의 max_laser_range 3.4 / scan_buffer_size 50 / chain 75, 브리지의 시뮬레이션 빔 재배열 생략
- 확인 (mac Docker, house, SLAM + 자동 탐색 13.9분, 실제 이동 76 m, 목표 21개 중 도착 실패 3 = 에뮬레이션이라 느려 90초 초과):
  오도메트리 오차 위치 최대 2.3 cm, map→odom 회전 −1.7 ~ +0.5° (한 번에 1° 넘게 바뀐 것 1번), SLAM 위치 오차 최대 0.13 m / 마지막 0.02 m,
  맵 벽 한 겹, 방들이 직각으로 맞음 (수정 전 같은 월드: map→odom −2.6 ~ +13.4°, 2~8° 씩 튐, 맵 무너짐). PC(실제 속도)와 lecture_hall_real 은 아직
- 남은 차이: 실물 LDS-02 빔 개수 변동(245~248)은 흉내 내지 않음(247 고정, 브리지 재배열은 같이 거침), Gazebo 바퀴는 거의 미끄러지지 않아 오도메트리가 실물보다 정확함
- (2026-10-29) Frenet 옆 모양 3차 → 5차 (Werling 형 경계 조건, 유지/복귀 구조와 종방향은 그대로). 실물 기록 57개 폐루프 재주행 (이전 vs 5차):
  도착 41/57 같음(재주행 도구 한계로 둘 다 같은 곳에서 막힘), 총 시간 1236 → 1228 s, 막힌 시간 107 → 108 s,
  오프셋 갈아탈 때 |Δw| 99% 0.41 → 0.16, 최대 0.53 → 0.30, >0.5 rad/s 0.3% → 0%. 같은 계획 이어 갈 때 99% 0.129 → 0.117.
  대가: 차체 최소 여유 중앙 8.0 → 7.1 cm (7개 기록에서 1 cm 넘게 줄었지만 모두 5 cm 이상). 좁은 틈/모서리/소화기 옆 6순간은 같은 속도로 출발.
  실물 확인 전 (소화기 통로 가운데 배치로 실험 예정). 다음: 비용 항목(장애물 근처 선호 여유 등)을 기존 방식들과 비교.
- (2026-10-29 오후) Frenet 옆 후보를 '다항식 하나' 로: 유지/복귀(사다리꼴) 제거, 6차 d(s) (시작 d, d', d'' / 중간 지점 d / 끝 (오프셋, 0, 0)),
  후보 길이 1.0 m, 중간 오프셋 2.5 cm x 끝 오프셋 10 cm x 속도 3 = 525개 (한 주기 약 32 ms). 비용 항목도 바꿈: 평균 |d| → 끝 오프셋² (w_end),
  최대 |d'| → ∫ d'''² ds (w_jerk) (Werling), 오프셋 바뀜은 '직전 계획의 같은 지점 위치' 와 비교, 장애물 근처 여유는 유지(빼면 나빠짐).
  실물 기록 57개 재주행 (기준 = 5차 + 유지/복귀): 도착 41 → 43, 막힘 108 → 96 s, 오프셋 갈아타기 314 → 208, 갈아탈 때 |Δw| 최대 0.30 → 0.24,
  최소 여유 중앙 7.1 같음. 나빠진 것: 주행 중 돌아서기 4 → 8 (2 개는 기준과 같은 기록, 1 개는 재주행 도구 A* 탓, 좁은 90° 모서리 2734 에서
  45° 틀어진 채 막힘 → 돌기 반복). 거친 것들: 5차 한 구간(S 0.4~1.0 여러 조합)은 좁은 곳에서 막힘/돌아서기 10~13, 길이 2개 섞은 6차는 갈아타기 2.5배.
  실물 확인 전.
- (2026-10-29 오후 2) 후보 길이 1.0 → 0.8 m, 끝 오프셋 10 → 5 cm. 실물(122034, 소화기 가운데): 길이 1.0 은 소화기 0.7~0.9 m 앞에서 미리 멈춤
  (후보가 틈까지 닿아 최선 2.2~3.6 cm), 같은 순간 0.8 이면 모두 지나감(5.0~15 cm). 재주행 57개: 길이 0.8 = 도착 42, 막힘 102 s,
  갈아탈 때 |Δw| 최대 0.22, 돌아서기 6 (기준 41 / 108 / 0.30 / 4). 0.7 은 돌아서기 12, 속도 x 시간(4, 5 s) 길이는 도착 37 로 나빠서 안 씀.
  한 주기 약 57 ms (후보 975개, 예전 450개 24 ms). 스캔/위치 추정 정상 (지도 맞음 97%), tracker 는 안 바꿈.
