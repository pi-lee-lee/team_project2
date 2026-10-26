# TurtleBot3 물류창고 로봇

웹 지도에서 위치를 찍으면 TurtleBot3 Waffle Pi가 직접 구현한 A* 경로로 이동하고,
물류 위치에서 적재 확인 → 다음 위치 → 홈 복귀까지 진행하는 프로젝트입니다.

- ROS 2 Humble, TurtleBot3 Waffle Pi (Raspberry Pi 4 + LDS-02)
- Django 4.2 + Channels(WebSocket) + MySQL
- 경로 계획: 직접 구현한 A* / 경로 추종: 직접 구현한 Frenet 로컬 플래너 (장애물을 옆으로 비켜 감)
- 위치 추정: Nav2 AMCL (map_server + amcl만 사용)

기획 내용은 [PROJECT_PLAN.md](PROJECT_PLAN.md), 현재 진행 상태는 [STATUS.md](STATUS.md) 참고.

---

## 1. 구성

```
[브라우저] ⇄ WebSocket(/ws/ui/) ⇄ [Django 서버 :8000] ⇄ WebSocket(/ws/robot/tb3/) ⇄ [브리지 노드]
                                         │                                              │
                                       MySQL                     ┌──────────────────────┼──────────────┐
                                                                  │                      │              │
                                                        [A* 플래너 노드]        [Nav2 AMCL/map_server]   cmd_vel
                                                       (astar/get_plan)          (map → odom TF)          │
                                                                                                          ▼
                                                                              [Raspberry Pi: turtlebot3_bringup]
                                                                               OpenCR(모터/odom) + LDS-02(/scan)
```

| 위치 | 실행되는 것 |
|---|---|
| Raspberry Pi (192.168.0.181) | `turtlebot3_bringup robot.launch.py` (모터, 오도메트리, LiDAR) |
| PC (192.168.0.188) | Django 서버, MySQL, Nav2 위치 추정(AMCL), A* 플래너, 브리지 |

PC와 Pi는 **ROS_DOMAIN_ID=44** 로 통신합니다.

## 2. 폴더 구조

```
webserver/
├── server/                      Django 서버
│   ├── config/                  설정 (settings.py: DB는 server/.env에서 읽음)
│   ├── warehouse/
│   │   ├── models.py            Map, Location, Robot, Task, TaskStep, EventLog
│   │   ├── services.py          작업 흐름(상태 머신) 로직
│   │   ├── consumers.py         WebSocket (/ws/ui/, /ws/robot/<name>/)
│   │   ├── mapimage.py          OccupancyGrid/PGM → PNG 변환
│   │   ├── management/commands/import_map.py   map.yaml 불러오기
│   │   ├── templates/, static/  웹 UI (지도 캔버스 + 조작 패널)
│   └── .env                     DB 접속 정보 (저장소에 올리지 말 것)
├── ros2_ws/src/warehouse_robot/ ROS 2 패키지
│   ├── warehouse_robot/
│   │   ├── astar.py             A* (numpy, ROS 의존성 없음)
│   │   ├── astar_planner_node.py  A*를 nav_msgs/GetPlan 서비스로 제공
│   │   ├── frenet.py            Frenet 로컬 플래너 + 궤적 추종 (ROS 의존성 없음)
│   │   ├── tracker.py           움직이는 장애물 추적 (칼만 필터)
│   │   └── bridge_node.py       서버 ↔ ROS 브리지, 경로 추종/안전장치/주행 기록
│   ├── config/params.yaml       노드 파라미터
│   ├── launch/warehouse.launch.py
│   └── test/                    A*, Frenet, 추적 단위 테스트
├── scripts/                     실행 스크립트 (env.sh, start_*.sh)
├── tools/                       점검 도구 (위치 맞추기, TF 측정, 이동 시험)
└── logs/                        실행 로그
```

## 3. 처음 설치

### PC

```bash
# Django 서버
cd ~/webserver/server
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# MySQL DB/계정 (한 번만)
sudo mysql -e "CREATE DATABASE IF NOT EXISTS warehouse CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
CREATE USER IF NOT EXISTS 'warehouse'@'localhost' IDENTIFIED BY '1234';
CREATE USER IF NOT EXISTS 'warehouse'@'127.0.0.1' IDENTIFIED BY '1234';
GRANT ALL PRIVILEGES ON warehouse.* TO 'warehouse'@'localhost';
GRANT ALL PRIVILEGES ON warehouse.* TO 'warehouse'@'127.0.0.1';
FLUSH PRIVILEGES;"

# server/.env 예시
#   DB_NAME=warehouse
#   DB_USER=warehouse
#   DB_PASSWORD=1234
#   DB_HOST=127.0.0.1
#   DB_PORT=3306
.venv/bin/python manage.py migrate
.venv/bin/python manage.py import_map ~/maps/maze_map.yaml --name maze_map

# ROS 2 패키지
pip3 install --user "websocket-client>=1.2"
cd ~/webserver/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

PC의 `~/.bashrc` 에 `export ROS_DOMAIN_ID=44` (이미 반영됨).

### Raspberry Pi

TurtleBot3 기본 설정 그대로 사용합니다. `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=44
export LDS_MODEL=LDS-02
export TURTLEBOT3_MODEL=waffle_pi
```

## 4. 실행 (모드 선택)

`scripts/run.sh` 하나로 로봇 종류와 모드를 골라 전부 띄웁니다 (백그라운드, 로그는 `logs/`).

| | `--mode slam` (맵 만들기) | `--mode map` (기존 맵 사용) |
|---|---|---|
| **`--robot real`** (실물, 도메인 44) | Pi bringup + SLAM | Pi bringup + map_server + AMCL |
| **`--robot sim`** (Gazebo, 도메인 77) | Gazebo + SLAM | Gazebo + map_server + AMCL (초기 위치 자동) |

```bash
# 시뮬레이션에서 맵 만들기 (Gazebo 화면 표시)
scripts/run.sh --robot sim --mode slam --world turtlebot3_house --gui
# 시뮬레이션 + TurtleBot3 기본 맵 (--map 생략 시 turtlebot3_world 기본 맵)
scripts/run.sh --robot sim --mode map
# 실물 로봇 맵 만들기
scripts/run.sh --robot real --mode slam
# 실물 로봇 + 기존 맵 (--name: 서버 DB 맵 이름, 생략 시 파일 이름에서 _clean/_straight 뺀 것)
scripts/run.sh --robot real --mode map --map ~/maps/backup/backup2/maze_map_straight.yaml
# 전부 끄기
scripts/stop_ros.sh --all
```

| 옵션 | 설명 |
|---|---|
| `--slam slam_toolbox` / `cartographer` | SLAM 엔진 (기본 slam_toolbox). 좌표 원점: slam_toolbox = 오도메트리 원점, cartographer = 로봇 출발 위치 |
| `--world` | 시뮬레이션 월드: turtlebot3_world(기본), turtlebot3_house, empty_world 등 |
| `--gui` | Gazebo 화면 표시 |
| `--amcl default` / `hall` / 파일 | (map) 위치 추정 설정. `hall` = `config/amcl_hall.yaml`: 책상/의자가 흩어진 강당·교실용 (지도에 없는 빔에 너그럽게). 웹 실행 모드의 "강당/교실용 위치 추정" 체크와 같음 |
| `--world lecture_hall_real` | (sim) 실제 학원과 비슷한 강당: 책상 앞판 없음(다리만 보임), 좌우 끝 열이 벽에 붙음, 좌석 줄 사이 빈 틈 0.25m(로봇 폭 0.31m 보다 좁아 세로 복도와 맨 앞/뒤 가로 복도로만 다님), 의자 무작위 배치 |

실행 순서: 웹 서버 → 로봇(Pi bringup 은 SSH 로 자동 실행 / Gazebo) → SLAM 또는 map_server+AMCL → A* + 브리지 → (sim+map) 초기 위치 자동 지정.
`logs/run_mode.env` 에 현재 모드가 기록되고, `source scripts/env.sh` 를 하면 도구들도 같은 도메인/모드를 씁니다.

### 맵 만들기 → 저장 → 사용

1. `scripts/run.sh --robot real|sim --mode slam`
2. 웹 "여기로 이동" 으로 로봇을 움직이며 탐색 (SLAM 모드에서는 A*가 미탐색 칸도 지나갈 수 있음, 장애물은 LiDAR 안전장치가 막음)
3. 웹 **"맵 저장"** (SLAM 모드에서만 보임) 또는 `scripts/save_map.sh <이름>`
   → `~/maps/<이름>.yaml` (원본) + `~/maps/<이름>_clean.yaml` (정리본). 정리 기준점은 저장할 때 로봇 위치
   → 정리본은 기울기 보정 + 벽 1칸 직선 + 직각 모서리 + 물체 1칸 테두리 (`straighten_map.py --auto`).
     벽이 가로/세로 직선에 잘 안 맞는 곳(대각선 벽, 원기둥)은 자동으로 회전 없이 1칸 외곽선 정리만
4. `scripts/run.sh --robot ... --mode map --map ~/maps/<이름>_clean.yaml`

SLAM 모드에서는 서버에 맵을 **`slam_live`** 라는 별도 이름으로 보내므로, 저장된 맵과 등록 위치(홈 등)를 덮어쓰지 않습니다.
SLAM 맵의 원점(0,0)은 **SLAM 을 시작한 순간의 로봇 위치**라 실행할 때마다(월드, 출발 위치) 좌표계가 다릅니다.
그래서 SLAM 을 새로 시작하면 이전 `slam_live` 맵과 그 위의 위치는 지워집니다 (브리지 재연결만 한 경우는 유지).
맵을 저장하면 SLAM 중 등록한 위치가 새 맵(같은 이름)으로 복사됩니다. 다만 SLAM 은 한 바퀴 돌아오면 지도를 다시 맞추므로(루프 클로저)
**위치 등록은 저장 후 기존 맵(AMCL) 모드에서 하는 것을 권장**합니다.
시뮬레이션에서 저장한 맵에는 만든 월드가 `# world: <이름>` 으로 기록되고, `--world` 없이 그 맵을 쓰면 같은 월드로 실행됩니다
(다른 월드에서 쓰면 AMCL 좌표가 맞지 않음).
웹 상단에 모드("맵 만들기 (SLAM)" / "맵 사용 (AMCL)")가 표시되고, SLAM 모드에서는 "초기 위치 지정"이 비활성화됩니다.

### 개별 실행 (필요할 때)

`run.sh` 가 아래를 순서대로 부릅니다. 직접 쓸 때는 `logs/run_mode.env` 의 모드를 따릅니다.

| 스크립트 | 내용 |
|---|---|
| `start_server.sh` | Django 서버 |
| `start_sim.sh [월드] [--gui]` | Gazebo + 로봇 생성 |
| `start_slam.sh [slam_toolbox\|cartographer]` | SLAM |
| `start_nav2.sh <map.yaml> [--with-navigation]` | map_server + AMCL (맵 파일이 없으면 바로 오류로 종료) |
| `start_bridge.sh [맵 이름]` | A* 플래너 + 브리지 |
| `save_map.sh <이름> [--plain \| --straighten]` | SLAM 맵 저장 + 정리 (기본: 자동 직선화, `--plain`: 회전 없이 정리만) |
| `stop_ros.sh [--server] [--daemon] [--all]` | 정리 (Gazebo 포함, Pi bringup 은 건드리지 않음) |

초기 위치 (기존 맵 모드): 웹 "초기 위치 지정" 또는 `source scripts/env.sh && python3 tools/localize.py <x> <y> <방향°>`

### 끄기

```bash
~/webserver/scripts/stop_ros.sh          # Nav2, SLAM, A*, 브리지, Gazebo
~/webserver/scripts/stop_ros.sh --all    # + Django 서버, ROS 2 데몬 (전부)
```

노드를 하나씩 `kill` 하면 lifecycle_manager/amcl 같은 짝이 부모 없이 남기 쉬우므로 이 스크립트를 쓰세요.
SIGINT → SIGTERM → SIGKILL 순서로 보내고 남은 프로세스가 없는지 확인합니다. Pi bringup은 건드리지 않습니다.
(PC 에서 다른 프로젝트의 Gazebo/robot_state_publisher 를 같이 쓰고 있다면 그것도 꺼지니 주의)
`ros2 node list` 에 죽은 노드가 남아 보이면 데몬 캐시 때문이니 `--no-daemon` 으로 확인하세요.

브라우저에서 `http://localhost:8000/` (다른 PC에서는 `http://192.168.0.188:8000/`).

## 5. 웹 사용법

| 기능 | 방법 |
|---|---|
| **실행 모드** | "실행 모드" 카드에서 로봇(시뮬레이션/실물), 맵, (시뮬레이션이면) 월드 선택. 맵을 고르면 그 맵을 만든 월드가 자동 선택되고, 다른 월드면 경고 → "맵 사용 (AMCL)" (저장된 맵으로 위치 추정) / "맵 만들기 (SLAM)" / "전체 정지". 서버가 `scripts/run.sh` / `stop_ros.sh` 를 실행 (ROS 가 꺼져 있어도 가능, 수십 초 소요, 주행 중이면 먼저 비상정지). 그동안 **대기 화면**이 화면 전체를 덮고 단계(1) 웹 서버 → 2) Gazebo/bringup → 3) SLAM/AMCL → 4) 브리지 …)별 걸린 시간, 전체 시간, 지난번 걸린 시간(이 브라우저 기준)을 보여 줌. 성공하면 3초 뒤 닫히고 실패는 이유를 띄운 채 남음. "화면 숨기기" 로 닫아도 실행은 계속 (끝나면 알림). 자동 탐색 / 위치 자동 찾기는 지도를 봐야 하므로 덮지 않음 |
| **위치 자동 찾기** | AMCL 모드에서 로봇 위치를 모를 때. 맵 전체에서 LiDAR 스캔과 가장 잘 맞는 위치를 찾아 초기 위치로 지정 (`tools/localize.py --global`, 수 초). 대칭인 방이면 "비슷한 다른 위치" 경고 |
| **여기로 이동** | 버튼 → 지도에서 지점 클릭 (드래그하면 도착 후 바라볼 방향 지정, 클릭만 하면 진행 방향) |
| 위치 추가 | "위치 추가" → 지도 클릭+드래그 → 이름/종류(물류 위치·홈) 저장 |
| **금지 구역 / 가상 벽** | "금지 구역" → 지도를 끌어서 사각형, "가상 벽" → 끌어서 선(두께 0.1m). A* 가 그 안/선을 지나는 경로를 만들지 않음 (위치 추정·수동 조작에는 영향 없음). 맵별로 저장, 오른쪽 "금지 구역" 목록에서 삭제. 다리 간격이 넓은 책상 밑, 지나가면 안 되는 좌석 줄에 사용. 로봇이 구역 안에 있으면 그 구역은 빼고 계획해서 빠져나옴 |
| 작업 실행 | 위치 목록에서 체크 → "▶ 작업 시작" (목록 순서대로 방문, 마지막에 홈 복귀 선택). 이동 중에는 "약 N초 남음" 표시 |
| 순서 최적화 (실험) | 위치를 체크한 뒤 "⇅ 순서 최적화" → 이동 시간 합이 가장 작은 순서와 예상 시간. 이후 "작업 시작" 은 그 순서로 방문. 이동 기록이 쌓이면 시간 모델이 학습됨 (`server/warehouse/planning.py`) |
| 적재 확인 | 물류 위치 도착 시 "적재 완료" → 다음 위치로 이동 |
| 홈 지정 | 로봇을 홈에 두고 "현재 위치를 홈으로" (홈이 없으면 "홈 복귀" 버튼이 비활성화됨) |
| **수동 조작** | "수동 조작" 카드의 방향 버튼(▲ ◤ ◥ ⟲ ⟳ ▼)을 누르고 있는 동안만 이동, 손을 떼면(또는 0.4초 명령이 없으면) 정지. 속도 느리게/보통/빠르게(0.08/0.15/0.2 m/s), '키보드 사용' 을 켜면 방향키·WASD. 앞 0.25m / 뒤 0.3m 안에 물체가 있으면 그쪽으로는 안 감(회전은 됨). 작업 중에는 안 됨. SLAM 에서 직접 맵을 만들 때 사용. 출발/정지는 가속 제한으로 부드럽게 |
| 상태 초기화 | 작업 취소 + 정지 + A* 가 기억한 장애물(LiDAR/부딪힌 자리) 삭제. 시뮬레이션이면 로봇을 출발 위치에 바로 세움(넘어졌을 때). `scripts/reset_robot.sh` 와 같음 |
| 실패 시 | "재시도" / "건너뛰기" / "작업 취소" / "홈 복귀" |
| 비상정지 | "비상정지" → 즉시 정지, 작업 취소. "오류 해제"로 대기 상태로 |

### 속도 명령 (`/cmd_vel`) 은 브리지만 내보냄

- 브리지 안의 선택기(`cmd_mux.py`)가 **정지(비상정지) > 수동 조작 > 자동 이동** 순으로 하나만 골라 20Hz 로 내보냄
- 키보드로 조작하려면 **`cmd_vel_teleop`** 으로 보내기 (`/cmd_vel` 로 직접 보내면 브리지와 번갈아 내보내 끊기듯 움직임):
  `ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop`
  - 앞뒤 막힘 확인, 부드러운 가속/감속이 웹 수동 조작과 같이 적용됨. 자동 이동 중에 키보드로 움직이면 자동 이동은 "수동 조작(키보드)으로 중단"
- 실물 도메인(44)은 같은 공유기의 어느 PC 에서 띄운 노드든 로봇에 닿음 (다른 PC 의 teleop_keyboard 가 /cmd_vel 을 내던 일 있었음)

### 카메라 부품 검출 (YOLO)

- `yolo/` = [team_project2](https://github.com/obloodobladeo-spec/team_project2) 를 합친 것 (YOLO11n 부품 검출기, 가중치, Colab 학습 노트북). 자세한 설명은 `yolo/README.md`
- 검출기(`/component_detector_switch`)는 **브리지와 같이 자동 실행**됩니다 (`start_bridge.sh` → `scripts/start_yolo.sh`, 출력은 `logs/bridge.log` 의 `[yolo-3]`). 빼려면 `WAREHOUSE_YOLO=0`
- 카메라: 실물 `run.sh` 가 bringup 다음에 Pi 에서 `yolo/camera.sh` 를 띄웁니다 (Pi 의 `~/warehouse_camera.sh` 로 복사, 로그 `~/warehouse_camera.log`, 이미 떠 있으면 그대로). 빼려면 `--no-camera`.
  `stop_ros.sh` 는 카메라를 끄지 않음 (끄기: Pi 에서 `pkill -f v4l2_camera_node`). 직접 띄울 때는 Pi 에서 `bash camera.sh /dev/video0` (장치는 `v4l2-ctl --list-devices`)
- 입력 영상: 검출기는 **`/csi/image_raw/compressed` (JPEG, 약 1.6 Mbit/s)** 를 받습니다 (`compressed:=false` 면 원본 `/csi/image_raw`).
  원본 320x240 rgb8 15fps 는 약 28 Mbit/s 라 Pi Wi-Fi 가 가득 차서 LiDAR/오도메트리가 2초 넘게 늦어지고 웹에 "로봇(Pi) 응답 없음" ↔ "연결됨" 이 반복됐음
- `enabled` (카메라 영상 구독 + 추론 + 검출 영상 `/yolo/image_raw` + 개수 `/yolo/counts` 발행) **기본값 false**. false 면 카메라 영상 구독도 끊어 추론하지 않음 (모델은 메모리에 남아 다시 시작이 빠름). 웹 **"카메라 검출 (YOLO)"** 카드:
  - **▶ YOLO 시작 / ■ YOLO 정지** = `enabled` true / false (브리지가 검출기 파라미터 서비스로 바꿈)
  - **📷 장면 캡처** = `capture_once` (다음 검출 장면을 `~/camera_dataset/captures/` 에 PNG 로)
  - 상태 배지 (검출기 없음 / 정지 / 실행 중 · 영상 대기 / 실행 중), 버튼 위 클래스별 개수 칸, 개수 요약 (예: `검출 4개 · led_red 2, usb 2`), `/yolo/counts` 원문 글자
- 검출 대상: DHT11, bluetooth, led_green, led_red, led_yellow, usb (rgb 는 제외). 아직 LED 배달(`cargo/detected`)과는 연결하지 않았습니다

### 로봇 여러 대 (코드만 추가, 실물 2대로는 아직 시험 안 함)

- 로봇마다 **이름**(서버 접속 `/ws/robot/<이름>/`)과 **ROS 네임스페이스**를 다르게 해서 브리지를 띄웁니다:
  `WAREHOUSE_ROBOT_NAME=tb3_1 WAREHOUSE_NAMESPACE=tb3_1 scripts/start_bridge.sh <맵이름>`
  - 노드/토픽/서비스가 `/tb3_1/` 아래로 (`cmd_vel`, `scan`, `odom`, `astar/...`, YOLO `component_detector_switch`, `csi/image_raw` → `yolo/counts`)
  - TF 는 `/tb3_1/tf`, `/tb3_1/tf_static` 로 나눔 (프레임 이름은 map/odom/base_footprint 그대로)
  - `params.yaml` 은 launch 가 `/tb3_1/<노드>` 키로 바꾼 사본을 만들어 씀. 이름·네임스페이스를 안 주면 지금처럼 `tb3`, `/`
- 웹: 로봇이 두 대 이상 등록되면 위쪽 막대에 **로봇 선택** 칸이 생깁니다 (이름 · 상태). 고른 로봇의 위치/경로/상태만 보이고
  이동·정지·위치 지정·YOLO 등 모든 요청이 그 로봇으로 갑니다. 화면마다 따로 고를 수 있고 마지막 선택을 기억합니다.
  주소로도 고를 수 있음: `/?robot=tb3_2`, 로봇에 단 화면은 `/robot?robot=tb3_2`. 기록에는 `[로봇이름]` 이 붙습니다
- 아직 안 된 것 (실제로 2대를 돌리려면 필요):
  - Pi bringup / 카메라를 로봇마다 그 네임스페이스로 띄우기 (`robot.launch.py`, `yolo/camera.sh`)
  - 위치 추정(AMCL)·SLAM 을 로봇마다 네임스페이스로 (`start_nav2.sh`, `start_slam.sh` 는 지금 한 대용). 맵(map_server)은 같이 써도 됨
  - `run.sh` 는 시작할 때 전부 끄므로 두 번째 로봇의 브리지는 위처럼 `start_bridge.sh` 로 따로 띄워야 함. 웹 '실행 모드',
    자동 탐색, 위치 자동 찾기, `tools/*.py` 는 한 대(기본 `tb3`) 기준
  - 로봇끼리 충돌 회피는 없음 (서로를 LiDAR 장애물로만 봄)

### LED 배달 (서빙 로봇처럼)

1. **받는 사람** (관제 화면, 휴대폰 가능): "LED 요청" 카드에서 LED(A/B/C)를 고르고 → 지도에서 받을 자리를 클릭
2. 로봇이 그 LED 가 놓인 위치로 이동 → **로봇 위 태블릿** 에 "B 을(를) 실어 주세요"
3. 사람이 LED 를 실으면 카메라가 인식(`cargo/detected` = `B`) → 바로 요청한 자리로 출발. 다른 LED 를 실으면 "요청한 것은 B 입니다" 안내
4. 도착하면 "B 을(를) 꺼내 주세요" → 꺼내서 카메라에 아무것도 안 보이면(`''`) 완료 → 홈이 있으면 홈으로
5. 카메라가 20초 안에 인식하지 못하거나 인식 노드가 없으면 태블릿에 "실었어요" / "꺼냈어요" 버튼 (관제 화면 "적재 완료"/"수령 완료" 와 같음)

- **한 번에 한 건만** 받습니다. 진행 중이면 새 요청은 "다른 요청을 처리 중입니다" 로 거절, 로봇에 물건이 실려 있어도 거절
- **로봇 위 화면**: 같은 와이파이에서 태블릿/휴대폰으로 `http://192.168.0.188:8000/robot` (큰 안내, LED 색, 수동 확인 버튼, 정지 버튼)
- 위치마다 **카메라 인식 라벨** 을 정할 수 있음 (위치 추가/수정 폼, 비우면 위치 이름). 라벨이 `red`/`green`/`blue`/`빨강` 등이면 화면에 그 색으로 표시
- 카메라 인식 결과 연결 방법 (학습 모델로 바꿀 때도 같음):
  - 인식 노드가 **`cargo/detected` (`std_msgs/String`)** 에 지금 보이는 라벨을 발행 (`''` = 비어 있음). 라벨은 위치의 인식 라벨과 같게 (대소문자 무시)
  - 브리지가 1초(`cargo_stable_sec`) 동안 같은 값일 때만 서버에 알림 → 서버가 적재/수령을 자동 확인
  - 임시 인식 노드: `ros2 run warehouse_robot cargo_detector --ros-args -p image_topic:=camera/image_raw/compressed`
    (색 덩어리로 판단, 기본 A 빨강 / B 초록 / C 파랑, `hsv_<라벨>` 로 HSV 범위, `roi` 로 적재 칸만, `publish_debug:=true` 면 `cargo/debug` 영상)
  - 카메라 없이 시험: `python3 tools/cargo_sim.py B` (실림), `python3 tools/cargo_sim.py ''` (꺼냄)

지도 표시: 빨간 원 = 로봇, 초록 선 = A* 경로, 주황 화살표 = 이동 목표, 빨강/초록 축 = 맵 원점(0,0)

- 지도는 방 범위에 맞춰 **고정** (확대/축소/끌어서 이동 없음, 창 크기가 바뀌면 다시 맞춤).
  SLAM 중 맵이 커져도 화면 범위는 실제 좌표로 유지되어 로봇/위치가 밀리지 않음 (보이는 범위를 벗어날 때만 넓힘)
- 로봇 카드의 상태 점: 브리지 / LiDAR / 오도메트리 / 위치 추정 (초록 = 정상)
- 로봇이 사라지면 화면에서도 사라짐: 브리지 종료 → "연결 끊김", Pi 꺼짐(LiDAR·오도메트리 2초 이상 없음) → "로봇(Pi) 응답 없음",
  위치가 2초 넘게 안 오면 로봇 표시와 경로를 지움
- 배터리 %: 브리지가 최근 30초 전압 중앙값으로 계산. **11.0V = 0%** (TurtleBot3 는 11V 아래에서 충전 경고), 12.6V = 100%, 그 사이는 LiPo 3셀 방전 곡선 모양 (12.0V≈75%, 11.7V≈60%, 11.4V≈34%, 11.2V≈13%). 20% 미만이면 빨간색. 0% 기준은 `battery_empty_v`

## 6. 동작 확인 결과 (2026-10-19, 실제 로봇)

| 항목 | 결과 |
|---|---|
| PC ↔ Pi 통신 (도메인 44) | `/scan` 9Hz, `/odom` 20Hz 수신 |
| cmd_vel 직접 주행 | 0.05 m/s 10cm 전진/후진: 이동 10.4cm, 복귀 오차 0.3cm |
| | 0.1 m/s 80cm 전진/후진: 이동 80.1cm / 80.2cm, 복귀 오차 0.4cm |
| 위치 추정 (AMCL, maze_map) | 초기 위치 지정 후 스캔 점 96~100%가 맵 벽과 20cm 이내 |
| A* 경로 계획 | maze_map(104×134)에서 계획 1회 약 0.01초 |
| **웹 "여기로 이동" (A* + 내장 추종기)** | (0.18, -0.05) → (0.60, 0.00): 4.9초, 목표 오차 9cm |
| | (0.57, -0.09) → (0.15, 0.00): 제자리 180° 회전 → 직진 → 방향 맞춤, 14초, 목표 오차 7cm |
| 단위 테스트 | `cd ros2_ws/src/warehouse_robot && python3 -m pytest test` → 12개 통과 |

### 시뮬레이션 모드 확인 (2026-10-19, Gazebo turtlebot3_world)

| 항목 | 결과 |
|---|---|
| sim + SLAM(slam_toolbox) | 웹에 `slam_live` 맵, 모드 표시, 이동 4회 성공 (오차 5~6cm), 기존 `maze_map`/등록 위치 보존 |
| 웹 "맵 저장" | `~/maps/sim_world_test.yaml` + `_clean.yaml` 생성 |
| sim + 기존 맵(저장한 맵) | 초기 위치 자동 지정, 이동 3회 성공 (오차 5~10cm), 미탐색 목표는 "경로 없음"으로 실패 후 바로 대기 |
| sim + 기존 맵(--map 생략) | TurtleBot3 기본 맵, 스캔 일치 100% |
| sim + SLAM(cartographer) | 이동 성공 (오차 6cm) |

아직 확인하지 않은 것: 여러 위치 작업(적재 확인 → 다음 위치 → 홈 복귀) 전체 흐름의 실제 로봇 시험,
주행 중 장애물 정지, 긴 거리 이동.

## 7. 주요 설정 (`ros2_ws/src/warehouse_robot/config/params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `astar_planner.robot_radius` | 0.24 | 차체 반폭 0.153 + 여유보다 크게. 이 안쪽은 통과 불가 (A* 가 낸 틈을 로컬 플래너도 들어갈 수 있게) |
| `astar_planner.inflation_radius` | 0.55 | 이 거리까지 벽에서 멀어지도록 비용 부여 |
| `warehouse_bridge.frenet.*` | | 주행은 A* + Frenet 로컬 플래너: A* 경로를 기준선으로 '옆 오프셋(±`max_offset`, `offset_step`) x 옆 이동 거리(`lateral_dists`) x 유지 거리(`hold_dists`) x 속도(`target_speed` 부터 `speed_step` 씩)' 후보 중 안전하고 비용이 가장 낮은 궤적을 0.1초마다 골라 그 앞부분을 실행 |
| `warehouse_bridge.frenet.target_speed` / `max_speed` | 0.15 / 0.18 | 목표 속도 / 최고 속도 (m/s) |
| `warehouse_bridge.frenet.safety_margin` / `clear_pref` | 0.06 / 0.15 | 차체(+뒤 부착물)와 장애물 최소 여유(제약) / 이보다 가까우면 비용 (m) |
| `warehouse_bridge.frenet.rotate_speed` | 1.0 | 출발 전 돌아서기 각속도 (rad/s). SLAM 모드는 0.6 |
| `warehouse_bridge.tracking.moving_speed` | 0.08 | 이보다 빠른 트랙만 움직이는 물체로 예측 (m/s) |
| `warehouse_bridge.stop_distance` | 0.35 | 차체가 기운 채 앞쪽 ±25° 이 거리(LiDAR 기준) 안이 막히면 장애물에 걸친 것으로 보고 물러남 |
| `warehouse_bridge.estop_distance` | 0.2 | 앞쪽 ±25° 안에 이 거리 이내면 전진 금지 (비상 정지) |
| `astar_planner.scan_obstacles` / `scan_memory` | true / 20.0 | AMCL 모드: LiDAR 로 본, 맵에 없는 장애물을 이 시간(초) 기억해 피함 (SLAM 모드는 SLAM 맵이 대신함) |
| `astar_planner.bump_memory` / `bump_radius` | 120.0 / 0.15 | 부딪힌 자리를 장애물로 기억하는 시간(초)과 반경(m) |
| `astar_planner.stripe_cells` | 2 | (기존 맵 모드) 빈 칸 사이 폭 4칸 미만 미탐색 줄무늬를 빈 칸으로 (먼 곳 LiDAR 빔 사이). 0 이면 끔 |
| `astar_planner.cluster_inflate` / `cluster_max_size` | 0.1 / 0.4 | (기존 맵 모드) 가로·세로 0.4m 이하 장애물 덩어리(책상/의자 다리 점)를 0.1m 부풀림 → 다리 사이 0.7m 미만 틈을 막음. 벽은 그대로. 0 이면 끔. AMCL 은 원본 맵 그대로 |
| `astar_planner.start_clear_radius` | 0.35 | 출발 위치 주변은 부풀린 점/금지 구역에서 빼고 계획 (그 안에 서 있어도 빠져나오게) |
| `warehouse_bridge.blocked_timeout` | 5.0 | 막힌 채로 이 시간이 지나면 장애물을 피하는 경로로 재계획 |
| `warehouse_bridge.bump_tilt_deg` | 6.0 | 평소보다 이만큼 더 기울면 부딪힘(낮은 물체에 올라탐)으로 보고 정지 후 빠져나옴 |
| `warehouse_bridge.stall_timeout` / `backoff_distance` | 2.5 / 0.15 | 앞으로 가는데 이 시간 동안 못 나아가면 부딪힘 / 빠져나오는 거리(m) |
| `warehouse_bridge.progress_timeout` | 15.0 | 이 시간 동안 5cm도 못 가면 실패 |
| `warehouse_bridge.tf_timeout` | 0.5 | 위치 TF가 이보다 오래되면 즉시 정지 |
| `warehouse_bridge.max_retries` | 2 | 실패 시 현재 위치에서 다시 계획하는 횟수 |

`colcon build --symlink-install` 로 빌드했으므로 yaml 수정 후 브리지만 다시 실행하면 적용됩니다.

위치 추정/SLAM 설정은 이 프로젝트 파일을 씁니다.
- `config/amcl_waffle_pi.yaml` (`start_nav2.sh`): TurtleBot3 기본값보다 빔 수↑(180), 측정 모델을 날카롭게(sigma 0.1), 5cm 마다 갱신. `NAV2_PARAMS` 로 바꿀 수 있음
- `config/amcl_hall.yaml` (`run.sh --amcl hall`): 위에서 z_hit/z_rand 0.75/0.15, sigma_hit 0.15, likelihood_max_dist 1.5 (강당/교실: 옮겨진 의자·사람 다리 빔에 너그럽게)
- A* 는 맵을 받을 때 계획용으로 가공해서 씀 (`plan_map.py`, 위 표). 실제로 쓰는 격자는 `astar/plan_map` 토픽 (RViz 확인용)
- `config/slam_toolbox.yaml` (`start_slam.sh`): 기본값에서 `scan_topic` 만 `/scan_level`
- AMCL/slam_toolbox/A* 는 브리지가 발행하는 **`scan_level`** (차체가 평소보다 3° 넘게 기울지 않았을 때의 스캔. 3초 넘게 기울어 있으면 다시 전달)을 씁니다. **브리지 없이 AMCL/SLAM 만 띄우면 스캔을 못 받으니** 그때는 `scan_topic` 을 `scan` 으로 바꾸세요.

## 8. 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `ros2 topic list` 에 `/parameter_events`, `/rosout` 만 보임 | ROS_DOMAIN_ID 불일치. `export ROS_DOMAIN_ID=44` 후 `ros2 daemon stop` |
| 웹에 로봇(빨간 원)이 안 보임 | 초기 위치 미지정. 웹 "초기 위치 지정" 또는 `tools/localize.py` |
| Pi 로그에 `Failed to read ... no status packet`, `stack smashing detected` 후 `turtlebot3_node` 종료 | OpenCR 통신 끊김. 로봇을 들거나 옮길 때 발생한 적 있음. bringup 재실행 |
| 웹에 "로봇(Pi) 응답 없음" | Pi bringup 종료, 전원 꺼짐, Wi-Fi 끊김, 배터리 방전. `ssh test@192.168.0.181` 로 접속 확인 |
| "로봇 위치 정보가 갱신되지 않아 정지했습니다" | 위치 TF(Pi의 odom 또는 AMCL)가 끊김. Pi bringup과 `start_nav2.sh` 상태 확인, `tools/tf_rates.py` 로 측정 |
| "앞이 막혀 진행할 수 없습니다" | 앞쪽 0.35m 안 장애물이 5초 넘게 있고, 피해 가는 재계획(2회)도 막힘 |
| "부딪힘 감지: ... 두 번 부딪혀 멈췄습니다" | LiDAR(높이 약 13cm)에 안 보이는 낮은 물체에 같은 이동에서 두 번 부딪힘. 바닥을 확인하고 치운 뒤 다시 지시. 기억한 자리는 "상태 초기화" 로 지움 |
| "차체가 N도 기운 채 장애물에 걸려 있습니다" | 로봇이 물체에 걸쳐 있어 출발하지 않음. 로봇을 내려놓은 뒤 다시 지시 |
| "경로를 찾을 수 없습니다" (벽 근처 목표) | 목표가 벽에서 0.25m 안이면 0.3m 안의 가장 가까운 빈 곳으로 감. 그보다 좁은 틈(폭 약 0.5m 미만)은 갈 수 없음. 장애물을 기억 중이면 "상태 초기화" 후 다시 시도. 원인 확인은 `logs/astar_fail.npz` |
| AMCL 위치가 맞지 않음 (sim) | 맵을 만든 월드와 다른 월드로 실행했을 수 있음. 맵 yaml 의 `# world:` 확인 |
| A*가 방 밖(회색 영역)으로 경로를 만듦 | 맵 yaml의 `free_thresh` 가 0.196보다 크면(예: 0.25) map_server가 미탐색(205)을 빈 칸으로 읽음. `tools/clean_map.py` 로 만든 맵은 0.196 사용 |
| 시뮬레이션에서 로봇이 생성되지 않음 (`Service /spawn_entity unavailable`) | Gazebo 기본 모델 경로 누락 → 인터넷 모델 저장소 대기. `scripts/env.sh` 가 `/usr/share/gazebo/setup.sh` 를 읽도록 수정됨 |
| SLAM 중 "경로를 찾을 수 없습니다" (바로 앞인데) | SLAM 초반 맵은 대부분 미탐색. SLAM 모드는 `allow_unknown:=true` 로 실행됨 (`start_bridge.sh`) |
| "홈 위치가 등록되어 있지 않습니다" | 로봇을 홈에 두고 웹에서 "현재 위치를 홈으로" |
| `ssh-copy-id: ERROR: No identities found` | 다른 계정에서 실행함. `abc` 계정에서 `ssh-copy-id -i ~/.ssh/id_ed25519.pub test@192.168.0.181` |

### Nav2 노드(AMCL) 안에서 TF 가 멈추던 문제 (원인 확인, 해결)

증상: Nav2 controller_server/AMCL 이 몇 초~몇 분 뒤 TF 를 더 이상 받지 못함 → 로봇이 움직이는데 "진행 없음",
목표를 지나쳐 계속 주행(실물에서 0.62m 목표를 지나 2.8m), 움직이지 않았는데 "Reached the goal", AMCL 위치 갱신 멈춤.

원인: **tf2 / tf2_ros 0.25.23 의 ABBA 교착**. TF 수신 스레드(`setTransform → testTransformableRequests`)와
LiDAR 를 받는 MessageFilter(`waitForTransform → addTransformableRequest`)가 서로의 잠금을 기다리며 멈춤 (gdb 로 확인).
DDS 설정과는 무관.

해결: **0.25.24 로 업데이트** (변경 기록: "Fix ABBA deadlock between waitForTransform and testTransformableRequests")

```bash
sudo apt update && sudo apt install --only-upgrade ros-humble-tf2 ros-humble-tf2-ros
dpkg -l ros-humble-tf2 ros-humble-tf2-ros | grep ^ii   # 0.25.24 확인
```

이 문제 때문에 경로 추종을 브리지의 내장 추종기(지금은 `frenet.py` Frenet 로컬 플래너)로 만들었습니다.
지금은 Nav2 의 map_server + AMCL 만 쓰고 Nav2 주행 노드(planner/controller_server)는 쓰지 않습니다.

## 9. 점검 도구 (`tools/`)

먼저 `source ~/webserver/scripts/env.sh`.

| 명령 | 용도 |
|---|---|
| `python3 tools/localize.py [x y 방향°] [--dry-run]` | 스캔으로 위치를 찾아 초기 위치 지정 (`--dry-run`: 찾기만) |
| `python3 tools/localize.py --global` | 맵 전체에서 찾기 (10cm·5° 로 넓게 → 상위 5곳을 2.5cm·1° 로 정밀). 웹 "위치 자동 찾기" 와 같음 |
| `python3 tools/tf_rates.py [초]` | TF별 수신 빈도와 지연 측정 |
| `python3 tools/sim_watch.py --world worlds/maze_room.world [--world 상자.sdf] [--seconds N]` | (sim) Gazebo 정답 위치와 비교한 위치 오차, 충돌, 벽 여유, 기울기 기록. Ctrl-C 로 끝내도 요약 출력 |
| `python3 tools/make_hall_world.py` | 강당 월드 `worlds/lecture_hall.world` 다시 만들기 (강단 + 가로 복도 6 × 세로 복도 5, 책상 20개 · 의자 40개, 치수는 파일 위쪽 상수) |
| `python3 tools/make_hall_world.py --real` | 실물형 강당 `worlds/lecture_hall_real.world` (책상 앞판 없음, 좌우 끝 세로 복도 없음, 좌석 줄 사이 0.25m `--row-gap`, 의자 ±0.12m/±15° 무작위, `--seed` 로 배치 바꿈) |
| `python3 tools/plan_map.py ~/maps/<맵>.yaml [--check X1 Y1 X2 Y2]...` | A* 가 쓰는 계획용 가공(줄무늬 메움 + 다리 점 부풀림)을 미리 보기 → `<맵>_plan_preview.png`. `--check` 로 홈 ↔ LED 위치 같은 두 점 사이 경로가 가공 전/후 있는지 확인 |
| `python3 tools/scan_check.py [초]` | (실물) 로봇을 세워 두고 LiDAR 거리대별 응답률/흔들림 측정 → `laser_max_range` 를 3.5m 보다 늘려도 되는지 추천 |
| `python3 tools/explore.py` | (SLAM) 프론티어 자동 탐색. 웹 "자동 탐색" 과 같음. 가던 방향 유지, 가는 도중 목표가 보이면 바로 다음 목표, 막힌 작은 미탐색 구멍(1 m² 미만) 무시. `--turn-weight 0 --no-early-switch --pocket-min 0` 이면 예전 방식 |
| `python3 tools/latency_test.py` | 명령 → 실제 속도 지연 측정: cmd_vel 계단 명령에 대한 odom/IMU 반응을 순수 지연 L + 1차 지연 τ 로 맞춰 출발/정지, 직진/회전별 중앙값 출력 (MPPI 지연 보상용). **로봇이 움직임** (앞뒤 약 0.26 m 왕복, 좌우 회전), LiDAR 로 앞/뒤 여유 확인. 기록은 `logs/latency_*.csv`, `--analyze <csv>` 로 다시 계산 |
| `python3 tools/cargo_sim.py <라벨>` | 카메라 없이 LED 인식 결과 흉내 (`''` = 비어 있음) |
| `python3 tools/collect_travel.py [--count 30]` | 빈 곳으로 무작위 이동을 반복해 이동 시간 학습 기록 모으기 |
| `python3 tools/ui_goto.py x y [yaw]` / `--home` | 웹과 같은 방식으로 이동(또는 홈 복귀) 명령을 보내고 진행 출력 (시간 초과 시 비상정지) |
| `python3 tools/clean_map.py ~/maps/maze_map.yaml` | 맵 정리 → `maze_map_clean.yaml/.pgm` + 비교 이미지 `_preview.png` (원본 유지) |
| `python3 tools/straighten_map.py ~/maps/maze_map.yaml` | 정리 + 기울기 보정 + 벽 직선화 → `maze_map_straight.yaml/.pgm` (원본 유지) |
| `python manage.py rotate_locations <각도>` (server 폴더) | 맵을 회전했을 때 등록된 위치도 같은 각도로 회전 |

### 맵 정리 (`tools/clean_map.py`)

SLAM 맵에서 방 밖 노이즈를 지우고, 로봇이 다니는 공간만 남깁니다.
1. `--seed` 위치(기본 원점)가 속한 빈 공간만 남기고 떨어진 빈 칸 → 미탐색
2. 그 공간에서 2칸보다 먼 장애물 점 → 미탐색
3. 공간 안에 홀로 있는 2칸 이하 장애물 점 → 빈 칸 (`--speck`)
4. 공간에 둘러싸인 30칸 이하 미탐색 구멍 → 빈 칸 (`--hole`)
5. 같은 행/열에서 벽 사이 3칸(15cm) 이하 틈 이음 (`--gap`, 로봇이 못 지나가는 폭이라 주행 공간은 그대로)
6. 경계 봉합: 빈 공간과 맞닿은(대각선 포함) 미탐색 칸을 벽으로 → 바깥 벽이 끊김 없이 이어짐 (`--no-seal` 로 끔)
7. 1칸 외곽선: 벽 = 빈 공간을 둘러싼 1칸 테두리만, 두꺼운 벽 바깥쪽/물체 안쪽은 미탐색 (`--thick` 로 끔)
8. `free_thresh: 0.196` 으로 저장 (미탐색이 빈 칸으로 읽히지 않게)

결과는 `_preview.png` 로 확인. 원본 maze_map 기준 로봇 이동 가능 칸 2,139 → 정리본 2,138 (차이 없음).

### 맵 반듯하게 (`tools/straighten_map.py`)

1. `clean_map.py` 와 같은 정리
2. 기울기 추정: 벽 점을 -15°~+15° 로 돌려 보며 가로/세로 줄에 가장 잘 모이는 각도 → maze_map 은 **-2.40°** 기울어져 있었음
3. 원점 (0,0) 기준으로 반대로 회전 (맵 크기도 방에 맞게 잘라 73×91)
4. 가로/세로 벽 구간(30cm 이상)을 1칸 두께 직선으로 다시 그림, 모서리 끝점 맞물림, 물체 뒤처럼 못 본 곳의 끊긴 바깥 벽은 이어 그림.
   두꺼운 벽에서 나란히 겹쳐 나온 짧은 직선은 버림
5. 방 안에 갇힌 미탐색 영역(물체 안쪽)은 1칸 사각형 테두리, 바깥 경계 봉합, 방과 맞닿지 않은 벽 조각 지움
- 0.2° 미만으로 기울었으면 돌리지 않음 (좌표 그대로)
- `--auto` (`save_map.sh` 기본, 출력 `<이름>_clean`): 벽 점 중 직선에 맞는 비율이 85%(`--min-coverage`) 미만이면 회전/직선화 없이 `clean_map.py` 정리
- 돌린 맵 yaml 에는 `# rotation_deg: <각도>` 가 남고, 웹 "맵 저장" 은 SLAM 중 등록한 위치/금지 구역을 같은 각도로 돌려 복사 (`services.carry_over_locations`)

**이미 위치를 등록한 맵을 나중에 돌렸다면 위치 좌표도 돌려야 함**: 출력되는 `rotation_deg` 로 `python manage.py rotate_locations 2.40` (maze_map 적용 완료).
초기 위치를 지정할 때도 새 좌표 기준 (홈 = (0.20, 0.01, 2.4°)).

현재 사용 맵: `~/maps/maze_map_straight.yaml` (`scripts/start_nav2.sh` 기본값). 서버 DB 맵 이름은 `maze_map` 그대로라 등록한 위치가 유지됩니다.
