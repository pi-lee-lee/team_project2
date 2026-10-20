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
- 0.22m(로봇 반경) 이내 통과 불가, 0.22~0.55m는 벽에 가까울수록 비용 증가 → 통로 가운데로 경로
- 8방향 이동(직선 1, 대각선 √2 × 비용 배율), octile 휴리스틱, 장애물 모서리 대각선 통과 금지
- 목표가 벽에 붙어 있으면 0.3m 안의 가장 가까운 빈 칸으로 이동
- 이동 평균으로 경로 다듬기, 마지막 점에 도착 방향 지정

### 경로 추종: Pure Pursuit (`follower.py`)
- 0.3m 앞 경로점을 목표로 곡률 2·sin(α)/거리 로 각속도 결정
- 방향 차이 > 0.8 rad 이면 제자리 회전
- 최고 0.15 m/s, 방향 차이·목표 거리에 따라 감속
- 목표 8cm 이내 → 정지 후 도착 방향(±0.1 rad) 맞춤

### 위치 추정: Nav2 AMCL (그대로 사용)
- 초기 위치는 웹 "초기 위치 지정" 또는 `tools/localize.py` (스캔-맵 전수 탐색)

### 안전장치 (`bridge_node.py`)
- 앞쪽 ±25° 0.35m 안 물체 → 정지 후 대기, 10초 넘으면 재계획
- 경로 이탈 0.5m / 15초 진행 없음 → 재계획 (최대 2회) → 실패
- 위치 TF 0.5초 이상 갱신 없음 → 즉시 정지

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
| A* 맵이 정적 맵이라 새 장애물을 피해 돌아가지 못함 | 정지 후 대기, 오래 막히면 재계획(같은 경로가 나올 수 있음) | LiDAR 장애물을 맵에 반영하는 기능 필요 |
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
