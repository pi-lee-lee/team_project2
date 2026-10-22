# scripts 사용 안내

서버 PC(`abc@192.168.0.188`, 저장소 `~/webserver`)에서 로봇 시스템을 켜고 끄는 방법과 `scripts/` 안 스크립트별 설명.
웹 사용법은 [../README.md](../README.md), 지금 상태와 시험 기록은 [../STATUS.md](../STATUS.md) 참고.

> 모든 스크립트는 **Ubuntu 서버 PC** 에서 실행합니다 (`setsid`, `flock`, `/proc`, `/opt/ros/humble` 사용). mac 에서는 동작하지 않습니다.
> 아래 명령은 모두 저장소 폴더(`cd ~/webserver`)에서 실행한다고 가정합니다.

---

## 1. 빠른 시작

| 하고 싶은 것 | 명령 |
|---|---|
| 실물 로봇, 저장된 지도로 운영 | `scripts/run.sh --robot real --mode map --map ~/maps/<맵>_clean.yaml` |
| 실물 로봇, 강당/교실에서 운영 | 위 명령 + `--amcl hall` |
| 실물 로봇으로 지도 만들기 | `scripts/run.sh --robot real --mode slam` |
| 시뮬레이션 강당(실물형)에서 지도 만들기 | `scripts/run.sh --robot sim --mode slam --world lecture_hall_real` |
| 시뮬레이션, 실물 같은 LiDAR | 위 명령 + `--scan-noise` |
| 만든 지도 저장 | 웹 "맵 저장" 또는 `source scripts/env.sh && scripts/save_map.sh <이름>` |
| ROS 노드만 끄기 (웹 서버 유지) | `scripts/stop_ros.sh` |
| 전부 끄기 | `scripts/stop_ros.sh --all` |
| 도움말 | `scripts/run.sh --help` |

웹: **http://192.168.0.188:8000/** — 웹의 "실행 모드" 카드에서도 같은 `run.sh` / `stop_ros.sh` 를 실행할 수 있습니다
(로봇·맵·월드 선택, "강당/교실용 위치 추정", "LiDAR 를 실물처럼" 체크).

---

## 2. 서버 구동 방법

### 2.1 처음 한 번 / 코드를 받은 뒤

```bash
cd ~/webserver
scripts/stop_ros.sh --all                     # 웹 서버까지 끔 (DB 구조가 바뀌는 코드면 migrate 전에 꼭)
git pull
cd server && .venv/bin/python manage.py migrate && cd ..
cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && cd ..
```
- `--symlink-install` 이라 python/yaml 만 바뀐 경우는 빌드 없이 노드 재시작으로 반영되지만, 새 파일이 생기면 빌드하세요.
- 웹 서버가 새 코드로 자동 재시작되면 `migrate` 전까지 DB 오류가 나므로 **끄고 → 받고 → migrate** 순서.

### 2.2 실물 로봇 사전 점검

1. 로봇 배터리(11V 이상)와 전원, Wi-Fi.
2. `ssh test@192.168.0.181 true` 가 비밀번호 없이 끝나야 함 (`run.sh` 가 SSH 로 Pi bringup 을 켬).
3. MySQL 실행 중 (`server/.env`).

### 2.3 실물: 저장된 지도로 운영

```bash
scripts/run.sh --robot real --mode map --map ~/maps/<맵>_clean.yaml [--name <DB맵이름>] [--amcl hall] [--controller dwa]
```
1. 끝나면 웹 **"초기 위치 지정"** (지도에서 로봇 자리를 찍고 정면 방향으로 끌기) 또는 "위치 자동 찾기".
2. 웹에 빨간 원(로봇)이 나오고 청록색 실시간 LiDAR 선이 지도의 벽과 겹치면 준비 완료.
- `--name` 을 빼면 파일 이름에서 `_clean` / `_straight` 를 뺀 이름이 DB 맵 이름 (등록 위치·금지 구역이 이 이름에 묶임).

### 2.4 실물: 지도 만들기 → 저장 → 사용

```bash
scripts/run.sh --robot real --mode slam
```
1. 시작할 때 **오도메트리를 0 으로 초기화**합니다 (Pi 에서 `/reset_odometry`). → 지도 원점 = 지금 로봇 자리, 원점 x 축 = 로봇 정면.
   **로봇을 지도의 x 축으로 삼을 방향으로 놓고** 시작하세요. 웹 지도는 x 축이 위쪽, y 축이 왼쪽으로 보입니다.
2. 웹 "여기로 이동" / 수동 조작 / "자동 탐색" 으로 돌아다니며 지도를 만듦 (지도는 2초마다 웹에 바로 반영).
3. 웹 **"맵 저장"** 또는 `source scripts/env.sh && scripts/save_map.sh <이름>` → `~/maps/<이름>.yaml`(원본) + `<이름>_clean.yaml`(정리본).
4. `scripts/run.sh --robot real --mode map --map ~/maps/<이름>_clean.yaml` 로 다시 실행 (기존 노드는 자동 정리).
5. 홈/LED 위치, 금지 구역은 이 **기존 맵 모드에서 등록**하는 것을 권장 (SLAM 은 루프 클로저 뒤 좌표가 조금 바뀔 수 있음).

### 2.5 시뮬레이션

```bash
scripts/run.sh --robot sim --mode slam --world lecture_hall_real [--scan-noise] [--gui]
scripts/run.sh --robot sim --mode map --map ~/maps/<이름>_clean.yaml       # 월드는 맵 yaml 의 '# world:' 로 자동
```
- 도메인 77 이라 실물(44)과 섞이지 않습니다. 기존 맵 모드는 출발 위치를 알아서 초기 위치를 자동 지정합니다.
- `--gui` 를 빼면 화면 없이 gzserver 만 뜹니다.

### 2.6 정상 동작 확인

```bash
source scripts/env.sh                        # 지금 모드(logs/run_mode.env)의 도메인으로 맞춤
cat logs/run_mode.env                        # 지금 실행 모드
ros2 node list --no-daemon                   # 노드 목록 (데몬 캐시 무시)
ros2 topic hz /scan                          # LiDAR 가 들어오는지
python3 tools/tf_rates.py                    # TF 갱신 주기
tail -f logs/bridge.log                      # 브리지·A* (서버 연결, 목표, 경로 계획 성공/실패 이유)
ssh test@192.168.0.181 tail -n 30 ~/warehouse_bringup.log    # Pi bringup ("Run!" 이 있어야 정상)
```

| 로그 (`logs/`) | 내용 |
|---|---|
| `server.log` | 웹 서버 (Django) |
| `sim.log` | Gazebo |
| `slam.log` | SLAM |
| `nav2.log` | map_server + AMCL (+ Nav2 주행 노드) |
| `bridge.log` | A* 플래너 + 브리지 |
| `run_mode.env` | `run.sh` 가 기록한 지금 실행 모드 (아래 4절) |

---

## 3. 중지 방법

| 명령 | 끄는 것 |
|---|---|
| `scripts/stop_ros.sh` | ROS 노드(SLAM, AMCL, Nav2, A*, 브리지)와 Gazebo. **웹 서버는 유지** (웹 "전체 정지" 와 같음) |
| `scripts/stop_ros.sh --all` | 위 + 웹 서버 + ROS 2 데몬 |
| `scripts/stop_ros.sh --daemon` | ROS 2 데몬만 추가로 (`ros2 node list` 에 죽은 노드가 남아 보일 때) |

- 노드를 `kill` 로 하나씩 끄지 마세요. lifecycle_manager/amcl 같은 짝이 남습니다. `stop_ros.sh` 는 SIGINT → SIGTERM → SIGKILL 순서로 끄고 남은 프로세스를 확인합니다.
- 도메인 44/77 이거나 도메인이 없는 이 프로젝트 프로세스만 끕니다.
- **Pi bringup 은 끄지 않습니다.** 하루를 마칠 때나 로봇을 옮길 때:
  ```bash
  ssh test@192.168.0.181 'pkill -INT -f "ros2 launch turtlebot3_bringup"'
  ```
  (OpenCR 통신 오류로 `turtlebot3_node` 만 죽어 있으면 `run.sh` 가 "이미 실행 중" 으로 건너뛰므로 위 명령으로 끈 뒤 다시 실행)

---

## 4. 스크립트별 설명

### `run.sh` — 전체 실행 (가장 많이 씀)

```
scripts/run.sh --robot real|sim --mode slam|map [옵션]
```

| 옵션 | 설명 |
|---|---|
| `--robot real` / `sim` | 실물(도메인 44, Pi bringup 을 SSH 로) / Gazebo(도메인 77) |
| `--mode slam` / `map` | 지도 만들기(SLAM) / 저장된 지도 사용(map_server + AMCL) |
| `--map FILE` | (map) 지도 yaml. sim + turtlebot3_world 는 생략 시 TurtleBot3 기본 지도 |
| `--name NAME` | (map) 서버 DB 맵 이름. 기본: 파일 이름에서 `_clean`/`_straight` 뺀 것 |
| `--slam ENGINE` | `slam_toolbox`(기본) / `cartographer` |
| `--world W` | (sim) `lecture_hall_real`(실물형 강당), `lecture_hall`, `maze_room`, `turtlebot3_world`(기본), `turtlebot3_house`, `empty_world` 등. map 모드에서 빼면 지도 yaml 의 `# world:` |
| `--gui` | (sim) Gazebo 화면 표시 |
| `--amcl A` | (map) 위치 추정 설정: `default`(작은 방) / `hall`(강당·교실, `config/amcl_hall.yaml`) / yaml 경로 |
| `--scan-noise` | (sim) LiDAR 를 실물 LDS-02 처럼 (빔 개수 238~244 변동, 거리 흔들림, 먼 곳 값 빠짐) |
| `--controller C` | `hybrid`(기본, pure pursuit + 장애물 가까우면 DWA) / `mppi`(항상 MPPI) / `dwa`(항상 DWA) / `builtin`(항상 pure pursuit) / `nav2` |
| `--planner P` | `astar`(기본) / `nav2` |
| `--nav2` | Nav2 주행 노드도 실행 (planner/controller 를 nav2 로 주면 자동) |

실행 순서:
1. 다른 `run.sh` 가 실행 중이면 거절 (`logs/run.lock`, 웹과 터미널 동시 실행 방지)
2. `stop_ros.sh` 로 남은 노드/Gazebo 정리 (웹 서버는 유지)
3. `logs/run_mode.env` 기록 → `env.sh` 가 읽음
4. 웹 서버 (`start_server.sh`, 이미 떠 있으면 건너뜀)
5. 실물: SSH 로 Pi bringup (이미 떠 있으면 건너뜀) / sim: `start_sim.sh`
6. 실물 + SLAM: Pi 에서 `/reset_odometry` (원점 = 로봇 자리, x 축 = 로봇 정면)
7. SLAM(`start_slam.sh`) 또는 map_server + AMCL(`start_nav2.sh`)
8. A* 플래너 + 브리지 (`start_bridge.sh`)
9. sim + map: `tools/localize.py` 로 초기 위치 자동 지정 / 실물 + map: 웹에서 지정

`logs/run_mode.env` 항목: `WAREHOUSE_ROBOT`, `MODE`, `MAP`, `MAP_NAME`, `WORLD`, `NAV2`, `SLAM_ENGINE`, `SPAWN_X/Y`, `PLANNER`, `CONTROLLER`, `AMCL`, `SCAN_NOISE` (앞에 `WAREHOUSE_`).

### `stop_ros.sh` — 정리
`scripts/stop_ros.sh [--server] [--daemon] [--all]` — 3절 참고.

### `env.sh` — 환경 설정 (`source` 로 사용)
```bash
source scripts/env.sh
```
- ROS 2 Humble, TurtleBot3, 이 저장소 `ros2_ws` 환경을 읽고 `logs/run_mode.env` 의 모드를 따름
- 실물 `ROS_DOMAIN_ID=44`, sim `77` + `use_sim_time`. PC 는 Cyclone DDS (`rmw_cyclonedds_cpp`), Pi 는 Fast DDS
- `ros2` 명령이나 `tools/*.py` 를 쓰기 전에 항상 먼저. `ros2 topic list` 에 `/parameter_events`, `/rosout` 만 보이면 도메인이 다른 것 → `source scripts/env.sh` 후 `ros2 daemon stop`

### `start_server.sh` — 웹 서버
Django(HTTP + WebSocket) 를 `0.0.0.0:8000` 으로. 보통 `run.sh` 가 부름. 웹 "실행 모드" 카드는 이 서버가 `run.sh` 를 실행하므로 웹 서버는 먼저 떠 있어야 함.

### `start_sim.sh` — Gazebo 시뮬레이션
`scripts/start_sim.sh [월드] [--gui] [--x X --y Y]` — 월드는 `worlds/`(이 저장소) → AWS 창고 → TurtleBot3 기본 순으로 찾음. 모델 위치를 바꾸는 플러그인이 없으면 넣은 사본(`logs/world_<월드>.world`)을 씀.

### `start_slam.sh` — SLAM
`scripts/start_slam.sh [slam_toolbox|cartographer]` — 설정 `config/slam_toolbox.yaml` (브리지가 내는 `/scan_slam` 사용, 0.2m / 약 11° 마다 스캔을 지도에 넣음, 2초마다 지도 갱신). **AMCL 과 같이 띄우면 안 됨** (`run.sh` 로만 실행하면 생기지 않음).

### `start_nav2.sh` — 위치 추정 (map_server + AMCL)
`scripts/start_nav2.sh <map.yaml> [--with-navigation]` / `--navigation-only`(SLAM 과 같이, 주행 노드만). AMCL 설정은 `run.sh --amcl` 이 정한 파일(기본 `config/amcl_waffle_pi.yaml`). 맵 파일이 없으면 바로 오류로 끝남.

### `start_bridge.sh` — A* 플래너 + 브리지
`scripts/start_bridge.sh [맵이름] [서버 URL]` — `warehouse.launch.py` 실행. SLAM 모드는 맵 이름 `slam_live`, 미탐색 칸 통과 허용. `run_mode.env` 의 planner/controller/LiDAR 노이즈를 따름.
- 브리지: 웹 서버와 WebSocket 으로 연결, 목표 주행, 안전 정지, 웹에 위치·지도·실시간 LiDAR 전송
- A* 플래너: 기존 맵 모드에서 지도를 계획용으로 가공(미탐색 줄무늬 메움, 책상/의자 다리 점 부풀림)하고 웹의 금지 구역을 피함 (`ros2_ws/src/warehouse_robot/config/params.yaml`)

### `save_map.sh` — 지도 저장
```bash
source scripts/env.sh && scripts/save_map.sh <이름> [--plain | --straighten]
```
- `/map` 을 `~/maps/<이름>.yaml/.pgm` 으로 저장 (`free_thresh 0.196`: 미탐색이 빈 칸으로 읽히지 않게) 후 `<이름>_clean` 정리본:
  기울기 보정 + 벽 1칸 직선 + 직각 모서리 + 물체 1칸 테두리 (`tools/straighten_map.py --auto`).
  벽 점이 가로/세로 직선에 85% 미만으로 맞으면(대각선 벽, 원기둥, 흩어진 책상·의자) 자동으로 회전 없이 1칸 외곽선 정리만
- 돌렸으면 yaml 에 `# rotation_deg` 가 남고, 웹 "맵 저장" 은 SLAM 중 등록한 위치/금지 구역도 같은 각도로 돌려 복사
- `--plain`: 회전/직선화 없이 정리만 (`tools/clean_map.py`). `--straighten`: 직선 비율과 상관없이 직선화 (`<이름>_straight`)
- 이름은 영문/숫자/`_`/`-`, 이미 있으면 거절. 웹 "맵 저장" 도 브리지가 이 스크립트를 실행

### `reset_robot.sh` — 상태 초기화
웹 "상태 초기화" 가 브리지를 통해 실행 (직접 실행도 가능). 정지 명령 1초 → sim 은 Gazebo 월드 초기화 + 초기 위치 다시 지정, 실물은 정지까지만 (넘어졌으면 직접 세운 뒤 웹 "초기 위치 지정").

### `fastdds_udp.xml` — Fast DDS 설정 파일
Fast DDS 의 공유 메모리 전송을 끄고 UDP 만 쓰는 설정. 지금 PC 는 Cyclone DDS 라 쓰지 않지만, Fast DDS 로 되돌릴 때 `env.sh` 가 가리킴.

---

## 5. 자주 겪는 문제

| 증상 | 원인 / 조치 |
|---|---|
| "다른 run.sh 가 실행 중입니다" | 웹과 터미널에서 동시에 실행. 앞의 것이 끝난 뒤 다시 |
| "Pi 에 접속할 수 없습니다" | 로봇 전원/Wi-Fi/SSH 키. `ssh test@192.168.0.181` 로 확인 |
| "bringup 확인 필요" | Pi `~/warehouse_bringup.log` 확인 (OpenCR USB, 배터리) |
| "오도메트리 초기화 실패" | Pi bringup(`turtlebot3_node`)이 안 떠 있음. 지도 원점이 로봇과 어긋나므로 bringup 을 다시 켠 뒤 재실행 |
| 웹 접속 오류 / 맵 저장 오류 (코드 받은 직후) | `migrate` 안 함 → 2.1절 순서대로 |
| AMCL/SLAM 이 스캔을 못 받음 | AMCL·SLAM 은 브리지가 내는 `scan_level` / `scan_slam` 을 씀 → 브리지가 떠 있어야 함 |
| "경로를 찾을 수 없습니다: 금지 구역에 막혀 …" | 웹 금지 구역이 길을 막음. 구역을 지우거나 다른 목표 |
| "같은 지점이 30초 안에 3번 실패했습니다" | 같은 곳 반복 실패를 서버가 잠시 막음. 잠시 뒤 다시 또는 다른 곳 |
| `ros2 node list` 에 죽은 노드 | 데몬 캐시. `--no-daemon` 으로 확인, `scripts/stop_ros.sh --daemon` |
