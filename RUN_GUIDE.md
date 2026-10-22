# 구동 및 중지 가이드

`scripts/` 분석 기준 (커밋 5c294f1). 옵션 전체와 웹 사용법은 [README.md](README.md), 현재 상태는 [STATUS.md](STATUS.md) 참고.

> **어디서 실행하나:** 모든 스크립트는 **Ubuntu PC (192.168.0.188)** 에서 실행합니다.
> `setsid`, `flock`, `/proc/<pid>/environ`, `hostname -I`, `/opt/ros/humble` 을 쓰므로 mac 에서는 동작하지 않습니다 (mac 은 코드 분석/편집용).

---

## 1. 한눈에 보기

| 하고 싶은 것 | 명령 (PC, 저장소 폴더에서) |
|---|---|
| 실물 + 기존 맵으로 운영 | `scripts/run.sh --robot real --mode map --map ~/maps/<맵>.yaml` |
| 실물로 맵 새로 만들기 | `scripts/run.sh --robot real --mode slam` |
| 시뮬레이션 (강당) 맵 만들기 | `scripts/run.sh --robot sim --mode slam --world lecture_hall --gui` |
| 시뮬레이션 + 저장한 맵 | `scripts/run.sh --robot sim --mode map --map ~/maps/<맵>_clean.yaml` |
| 만든 맵 저장 | 웹 "맵 저장" 또는 `scripts/save_map.sh <이름>` |
| 로봇 노드만 끄기 (웹 서버 유지) | `scripts/stop_ros.sh` |
| 전부 끄기 (PC) | `scripts/stop_ros.sh --all` |
| Pi bringup 까지 끄기 | 5.3 참고 (스크립트 없음, 직접) |

웹: `http://192.168.0.188:8000/` — 웹 "실행 모드" 카드에서도 같은 `run.sh` / `stop_ros.sh` 를 실행할 수 있습니다 (`server/warehouse/runner.py`).

---

## 2. 스크립트 구조

```
run.sh ── (잠금 logs/run.lock, 동시 실행 거절)
  ├─ stop_ros.sh            이미 떠 있는 노드/Gazebo 먼저 정리 (서버는 유지)
  ├─ logs/run_mode.env 기록  → env.sh 가 읽음 (도메인, sim 시간, 맵 이름, planner/controller)
  ├─ 1) start_server.sh      Django :8000  (이미 응답하면 건너뜀)
  ├─ 2) real: ssh Pi → turtlebot3_bringup robot.launch.py   (이미 실행 중이면 건너뜀)
  │     sim : start_sim.sh   Gazebo + 로봇 생성
  ├─ 3) slam: start_slam.sh  slam_toolbox | cartographer  (+ --nav2 면 start_nav2.sh --navigation-only)
  │     map : start_nav2.sh  map_server + AMCL           (+ --nav2 면 주행 노드)
  ├─ 4) start_bridge.sh      A* 플래너 + 브리지 (warehouse.launch.py)
  └─ 5) sim+map: tools/localize.py 로 초기 위치 자동 지정 / real+map: 웹에서 직접
```

| 파일 | 역할 | 비고 |
|---|---|---|
| `env.sh` | ROS 환경 + 도메인 + RMW 설정 (`source` 로 사용) | real=도메인 44, sim=77. PC 는 Cyclone DDS |
| `run.sh` | 전체 실행 (백그라운드, 로그 `logs/*.log`) | 끝나도 노드는 계속 돎 |
| `start_server.sh` | `manage.py runserver 0.0.0.0:8000` | |
| `start_sim.sh` | gzserver(+gzclient), robot_state_publisher, 로봇 생성 | `gazebo_ros_state` 플러그인 없으면 넣은 사본 `logs/world_*.world` 사용 |
| `start_slam.sh` | SLAM. `config/slam_toolbox.yaml` (`/scan_slam` 사용) | **AMCL 과 같이 띄우면 안 됨** |
| `start_nav2.sh` | AMCL(`config/amcl_waffle_pi.yaml`) + map_server, 선택적으로 Nav2 주행 노드 | 주행 노드는 map→odom TF 가 나온 뒤 시작 |
| `start_bridge.sh` | A* + 브리지. SLAM 모드는 맵 이름 `slam_live`, 미탐색 통과 허용 | |
| `save_map.sh` | `/map` 저장 → 기울기 보정 + 1칸 직선 정리 (`straighten_map.py --auto`, 직선이 아닌 공간은 자동으로 `clean_map.py`) | `~/maps/<이름>.yaml` + `_clean.yaml` |
| `reset_robot.sh` | 정지 명령 1초 (sim 은 월드 초기화 + 위치 재지정) | 웹 "상태 초기화" 가 브리지를 통해 실행 |
| `stop_ros.sh` | PC 쪽 노드/Gazebo 정리 | Pi bringup 은 건드리지 않음 |

---

## 3. 구동 절차

### 3.1 공통 사전 점검 (실물)

1. 로봇 배터리 충전 상태 확인 (11V 이상 권장), 전원 ON.
2. PC 에서 Pi 접속 확인: `ssh test@192.168.0.181 true` — 비밀번호 없이 바로 끝나야 합니다 (`run.sh` 는 `BatchMode` 로 접속).
3. MySQL 실행 중인지 확인 (`server/.env`). 안 되면 `DB_ENGINE=sqlite` 로 임시 사용.
4. 처음이거나 ROS 패키지를 바꿨으면 빌드:
   ```bash
   cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
   ```
   (`--symlink-install` 이라 python/yaml 수정만이면 빌드 없이 노드 재시작으로 반영)

### 3.2 실물 + 기존 맵 (평소 운영)

```bash
cd ~/webserver                       # 서버 PC 의 저장소 위치 (ros2_ws/install 링크가 가리키는 곳)
scripts/run.sh --robot real --mode map --map ~/maps/<맵>_clean.yaml [--name <DB맵이름>]
```

- `--name` 생략 시 파일 이름에서 `_clean` / `_straight` 를 뺀 것이 DB 맵 이름 → 등록 위치(홈, LED 등)가 이 이름에 묶입니다. 맵을 다시 만들어도 이름을 같게 하면 위치가 이어지지만 좌표계가 다르면 위치가 어긋납니다.
- 끝나면 **웹에서 "초기 위치 지정"** (지도에서 로봇 위치를 찍고 방향 드래그), 또는 실행 모드 카드의 자동 위치 찾기(`localize.py --global`), 또는
  ```bash
  source scripts/env.sh && python3 tools/localize.py <x> <y> <방향°>
  ```
- 웹에 빨간 원(로봇)이 나타나고 LiDAR 점이 벽에 겹치면 준비 완료.

### 3.3 실물로 맵 만들기 → 저장 → 사용

```bash
scripts/run.sh --robot real --mode slam
```
1. 웹 "여기로 이동" 또는 수동 조작 패널로 천천히 돌며 탐색 (SLAM 모드는 미탐색 칸도 지나가게 계획).
2. 웹 **"맵 저장"** 또는 `source scripts/env.sh && scripts/save_map.sh <이름>`
   - 이름은 영문/숫자/`_`/`-` 만, 이미 있으면 거절.
   - 기본 `_clean` 은 벽을 1칸 직선/직각으로 반듯하게 하고, 직선이 아닌 공간(강당 책상·의자 등)은 자동으로 회전 없이 정리만 합니다. 강제로 고르려면 `--plain` / `--straighten`.
3. 기존 맵 모드로 다시 실행: `scripts/run.sh --robot real --mode map --map ~/maps/<이름>_clean.yaml`
   (`run.sh` 가 이전 노드를 먼저 정리하므로 stop 없이 바로 실행해도 됩니다)
4. 홈/LED 위치 등록은 **저장 후 기존 맵 모드에서** 합니다 (SLAM 중 등록한 위치는 루프 클로저 후 어긋날 수 있음).

### 3.4 시뮬레이션

```bash
scripts/run.sh --robot sim --mode slam --world lecture_hall --gui     # 강당 맵 만들기
scripts/run.sh --robot sim --mode map --map ~/maps/<이름>_clean.yaml   # 저장한 맵 (월드는 yaml 의 '# world:' 로 자동)
```
- 도메인 77 이라 Pi 가 켜져 있어도 섞이지 않습니다.
- 기존 맵 모드는 출발 위치를 알고 있어 초기 위치를 자동 지정합니다 (5단계).
- 화면 없이 돌리면(`--gui` 생략) gzserver 만 뜹니다.

### 3.5 주행 방식 옵션

| 옵션 | 의미 |
|---|---|
| (기본) `--planner astar --controller mppi` | 직접 구현한 A* + 항상 MPPI (정적 LiDAR 점 + 추적한 움직이는 장애물을 피하며 경로 추종) |
| `--controller hybrid` | 평소 pure pursuit, 경로 앞 1m 안에 지도에 없는 장애물(상자, 사람)이 오면 DWA 로 피하고 지나가면 pure pursuit 로 복귀 |
| `--controller dwa` | 항상 DWA (추적한 움직이는 장애물의 미래 위치를 피함) |
| `--controller builtin` | 항상 pure pursuit (앞쪽 ±25° 장애물이면 정지 후 대기) |
| `--controller nav2` / `--planner nav2` | Nav2 controller_server / NavigateToPose 사용 (`--nav2` 자동). **tf2 ≥ 0.25.24 필요** |
| `--nav2` | Nav2 주행 노드만 추가로 띄움 |
| `--slam cartographer` | SLAM 엔진 변경 (기본 slam_toolbox) |
| `--amcl hall` | 강당/교실용 위치 추정 설정 (`config/amcl_hall.yaml`). 웹 실행 모드의 체크와 같음 |
| `--scan-noise` | (sim) LiDAR 를 실물 LDS-02 처럼 (빔 개수 변동, 흔들림, 값 빠짐) |
| `--world lecture_hall_real` | (sim) 실물형 강당: 책상 앞판 없음, 좌우 끝 열이 벽에 붙음, 좌석 줄 사이 0.25m(로봇이 못 지나감), 의자 무작위 |

### 3.6 정상 구동 확인

```bash
source scripts/env.sh
cat logs/run_mode.env                     # 현재 모드
ros2 node list --no-daemon                # 노드 목록 (데몬 캐시 무시)
ros2 topic hz /scan                       # LiDAR 가 들어오는지
python3 tools/tf_rates.py                 # TF 갱신 주기
tail -f logs/bridge.log                   # 브리지 (서버 연결, 목표 진행)
ssh test@192.168.0.181 tail -n 30 ~/warehouse_bringup.log   # Pi bringup ("Run!" 이 있어야 정상)
```

| 로그 | 내용 |
|---|---|
| `logs/server.log` | Django |
| `logs/sim.log` | Gazebo |
| `logs/slam.log` | SLAM |
| `logs/nav2.log` | map_server + AMCL (+ Nav2 주행 노드) |
| `logs/bridge.log` | A* 플래너 + 브리지 |

---

## 4. 실행 중 자주 쓰는 동작

| 상황 | 방법 |
|---|---|
| 로봇 즉시 정지 | 웹 정지 버튼 (CmdMux 에서 정지가 최우선) |
| 장애물 기억/오류 상태 지우기 | 웹 "상태 초기화" → `reset_robot.sh` + A* 기억 삭제 |
| 모드 바꾸기 (예: SLAM → 맵) | 그냥 다른 옵션으로 `run.sh` 다시 실행 (기존 노드 자동 정리, 서버/Pi 는 유지) |
| 브리지만 재시작 (python 수정 후) | `scripts/stop_ros.sh` 후 `run.sh` 재실행 권장. 개별 kill 은 피하기 |

---

## 5. 중지 절차

### 5.1 로봇 노드만 끄기 (웹 서버 유지)

```bash
scripts/stop_ros.sh
```
- 대상: `start_*.sh`(server 제외), `ros2 launch`, Nav2/AMCL/map_server, slam_toolbox/cartographer, 브리지·A*(warehouse_robot), Gazebo, robot_state_publisher, map_saver.
- 순서: SIGINT → (5초 대기) → SIGTERM → (5초) → SIGKILL. 남은 프로세스가 있으면 목록을 보여 주고 종료 코드 1.
- **도메인 44/77 또는 도메인 미설정 프로세스만** 끕니다 (다른 도메인의 다른 프로젝트는 유지).
- 로봇이 움직이는 중이라면 먼저 웹에서 정지한 뒤 끄세요. 브리지가 꺼지면 `/cmd_vel` 을 내는 노드가 없어지지만, 마지막 명령이 남는 경우를 피하기 위함입니다.

### 5.2 PC 전부 끄기

```bash
scripts/stop_ros.sh --all      # = --server --daemon
```
- 위 + Django 서버 + `ros2 daemon stop`.
- `--daemon` 만 따로: `ros2 node list` 에 죽은 노드가 남아 보일 때.
- 웹 "실행 모드" 카드의 정지는 `stop_ros.sh` (옵션 없음) 를 실행하므로 서버는 남습니다 (웹 서버가 자기 자신을 끄지 않게).

### 5.3 Pi bringup 끄기 (수동)

`stop_ros.sh` 는 Pi 를 건드리지 않습니다. 하루 운영을 마칠 때나 로봇을 옮길 때:

```bash
ssh test@192.168.0.181 'pkill -INT -f "ros2 launch turtlebot3_bringup"; sleep 3; pgrep -af turtlebot3 || echo 종료됨'
```
그 뒤 로봇 전원 OFF. Pi 까지 끌 때는 `ssh -t test@192.168.0.181 sudo poweroff` (sudo 비밀번호 필요할 수 있음) 후 LED 가 꺼지면 전원 스위치 OFF.

> 로봇을 들고 옮기다 OpenCR 통신 오류로 `turtlebot3_node` 가 죽은 적이 있습니다. 다시 쓸 때 `run.sh` 가 bringup 이 없으면 새로 띄우지만,
> 프로세스가 반쯤 살아 있으면 "이미 실행 중"으로 건너뛰니 위 명령으로 끄고 다시 실행하세요.

### 5.4 종료 확인

```bash
pgrep -af "ros2|gzserver|warehouse_robot|manage.py"     # PC: 아무것도 없어야 함 (--all 기준)
ssh test@192.168.0.181 pgrep -af turtlebot3             # Pi
```

---

## 6. 주의 사항 / 자주 겪는 문제

| 증상 | 원인 / 조치 |
|---|---|
| "다른 run.sh 가 실행 중입니다" | 웹 실행 모드와 터미널에서 동시에 실행. 앞의 것이 끝난 뒤 다시 (SLAM 이 두 개 뜨면 지도가 망가져서 막아 둠) |
| "Pi 에 접속할 수 없습니다" | 로봇 전원/Wi-Fi, SSH 키. `ssh test@192.168.0.181` 로 직접 확인 |
| "bringup 확인 필요" | Pi `~/warehouse_bringup.log` 확인. OpenCR USB 연결, 배터리 |
| `ros2 topic list` 에 `/parameter_events`, `/rosout` 만 | 도메인 불일치. `source scripts/env.sh` 후 `ros2 daemon stop` |
| map_server 는 떴는데 맵이 안 보임 | 맵 파일 경로 오류 → `start_nav2.sh` 가 바로 종료하므로 `logs/nav2.log` 확인 |
| 로컬화/SLAM 이 스캔을 못 받음 | AMCL·SLAM 은 브리지가 내는 `scan_level` / `scan_slam` 을 씀 → 브리지가 떠 있어야 함 |
| 노드를 `kill` 로 하나씩 껐더니 amcl 이 남음 | 항상 `stop_ros.sh` 사용 |
| SLAM 과 AMCL 이 같이 뜸 | map→odom TF 를 둘이 냄. `run.sh` 로만 실행하면 생기지 않음 |
