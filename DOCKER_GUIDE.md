# Docker 실행 가이드 (시뮬레이션)

시뮬레이션 전체를 **컨테이너 하나**로 실행합니다. 들어 있는 것: Django 서버, Gazebo(TurtleBot3 Waffle Pi, LiDAR·오도메트리·명령 감시를 실물과 같은 사양·경로로), SLAM/AMCL, A* 플래너, 브리지.

- **이미지 = 실행 환경만** (ROS 2 Humble, Gazebo, 서버용 파이썬). 소스는 이미지에 넣지 않습니다.
- **소스 = 프로젝트 폴더**: 실행할 때 프로젝트 폴더를 컨테이너의 `/app` 으로 연결하므로, 고친 코드가 다시 빌드하지 않아도 반영됩니다.
- **로그 = 프로젝트 폴더의 `wsim/logs/`**: 컨테이너의 `/app/logs` 가 여기로 연결됩니다. PC 에서 직접 실행할 때의 `logs/` 와 섞이지 않습니다. `wsim/` 은 git 에 올라가지 않습니다.

파일: [compose.yaml](compose.yaml), [Dockerfile](Dockerfile), [docker/entrypoint.sh](docker/entrypoint.sh), [.dockerignore](.dockerignore).
옵션 전체는 [README.md](README.md), 도커 없이 실행은 [RUN_GUIDE.md](RUN_GUIDE.md) 참고.

---

## 1. 한눈에 보기

모든 명령은 **프로젝트 폴더**(`team_project2/`)에서 실행합니다.

| 하고 싶은 것 | 명령 |
|---|---|
| 처음 시작 (이미지 빌드 + 실행) | `docker compose up --build` |
| 시작 (기존 맵, turtlebot3_world) | `docker compose up` (백그라운드: `docker compose up -d`) |
| 맵 새로 만들기 (SLAM) | `WSIM_ARGS="--mode slam --world turtlebot3_house" docker compose up` |
| 저장한 맵으로 주행 | `WSIM_ARGS="--mode map --map /root/maps/<이름>_clean.yaml" docker compose up` |
| 서버만 띄우기 | `WSIM_ARGS=none docker compose up` (웹 "실행 모드" 카드에서 시작) |
| 고친 코드 반영 | 파이썬(ROS 노드): 웹 "실행 모드" 에서 다시 시작. 서버: 자동. 자세한 것은 [5절](#5-코드-수정과-반영) |
| 로그 보기 | `tail -f wsim/logs/bridge.log` (프로젝트 폴더에서 바로) |
| 끄기 | 실행한 터미널에서 `Ctrl+C`, 또는 `docker compose down` |
| 웹 | http://localhost:8000/ (다른 PC 에서는 `http://<도커 PC IP>:8000/`) |

---

## 2. 준비

- Docker 설치 (Ubuntu: Docker Engine + compose 플러그인, Mac/Windows: Docker Desktop). 디스크 약 6 GB (이미지 약 5 GB).
  Windows 는 [9. Windows 에서 실행](#9-windows-에서-실행) 도 확인.
- **CPU 종류**: 이미지는 `linux/amd64` 고정입니다. Gazebo classic 의 ROS 2 Humble 패키지(`gazebo_ros`, `turtlebot3_gazebo`)가 amd64 만 있기 때문입니다.
  - Ubuntu / Windows PC (Intel/AMD): 그대로 빠르게 동작
  - Apple Silicon Mac: 에뮬레이션으로 동작하지만 느림 (`platform (linux/amd64) does not match` 경고는 정상)
- 포트 8000 이 비어 있어야 합니다. 같은 PC 에서 `scripts/run.sh` 로 띄운 서버가 있으면 먼저 `scripts/stop_ros.sh --all`.

---

## 3. 구성 (무엇이 어디에 있나)

| 컨테이너 안 | 실제 위치 | 내용 |
|---|---|---|
| `/app` | 프로젝트 폴더 | 소스 전체 (scripts, server, ros2_ws/src, tools, config, worlds) |
| `/app/logs` | 프로젝트 폴더 `wsim/logs/` | 노드 로그, `run_mode.env`, 주행 기록 `records/`, `colcon_build.log` |
| `/opt/warehouse/ros2_ws` | 컨테이너 안 | ROS 패키지 빌드 결과. 시작할 때마다 다시 빌드 |
| `/opt/warehouse/.venv` | 이미지 | Django 서버 가상환경 |
| `/root/maps` | 볼륨 `warehouse-maps` | SLAM 으로 저장한 맵 (`~/maps`) |
| `/data` | 볼륨 `warehouse-data` | DB `db.sqlite3` (맵, 등록 위치, 작업 기록) |

왜 빌드 결과와 서버 가상환경은 프로젝트 폴더 것을 안 쓰나:
저장소에 커밋된 `ros2_ws/build|install` 은 Ubuntu PC 경로(`/home/abc/webserver/...`)를 가리키고, `server/.venv` 도 그 PC 용입니다.
그래서 컨테이너는 환경 변수로 자기 것을 씁니다 (`WAREHOUSE_ROS_INSTALL` → `scripts/env.sh`, `WAREHOUSE_SERVER_PY` → `scripts/start_server.sh`).
PC 에서 직접 실행할 때는 이 변수가 없으므로 지금과 같습니다.

### 컨테이너 시작 순서 (`docker/entrypoint.sh`)

1. ROS 패키지 빌드: `/app/ros2_ws/src` → `/opt/warehouse/ros2_ws` (`--symlink-install`, 로그 `wsim/logs/colcon_build.log`)
2. DB 준비: `/data/db.sqlite3` 가 없으면 저장소의 `server/db.sqlite3` 를 복사 → `migrate`
3. 웹 서버 시작 (`wsim/logs/server.log`)
4. `scripts/run.sh --robot sim <WSIM_ARGS>`: Gazebo(+ `scripts/sim_real_io.py`: 실물 형식 /scan·/odom, `scripts/pi_cmd_watchdog.py`: 명령 감시) → AMCL 또는 SLAM → A* + 브리지 → (map 모드) 초기 위치 자동 지정
5. 웹 서버가 살아 있는 동안 유지. 끄라는 신호(`Ctrl+C`, `docker compose down`)를 받으면 `scripts/stop_ros.sh --all` 로 노드 정리 후 종료

정상 시작이면 마지막에 이렇게 나옵니다:

```
5) 초기 위치 자동 지정 (-2.0, -0.5)
   가장 잘 맞는 위치: x=-1.975 y=-0.475 yaw=0.0°  (스캔 점 중 벽과 10cm 이내 100%, ...)
== 완료. 웹: http://172.18.0.2:8000/   끄기: scripts/stop_ros.sh --all
[docker] 웹: http://localhost:8000/   로그: 프로젝트 폴더 wsim/logs/
```

`172.18.0.2` 는 컨테이너 내부 주소입니다. 브라우저에서는 **http://localhost:8000/** 로 접속합니다.

---

## 4. 실행

```bash
cd team_project2
docker compose up --build          # 처음 한 번 (이미지 빌드). 이후엔 docker compose up
```

`WSIM_ARGS` 는 `scripts/run.sh --robot sim` 뒤에 그대로 붙습니다 (생략하면 `--mode map`).

```bash
docker compose up                                                              # 기존 맵: turtlebot3_world + TurtleBot3 기본 맵
WSIM_ARGS="--mode slam" docker compose up                                      # SLAM, turtlebot3_world
WSIM_ARGS="--mode slam --world turtlebot3_house" docker compose up             # SLAM, 집 월드
WSIM_ARGS="--mode slam --world lecture_hall" docker compose up                 # 저장소 worlds/ 의 월드
WSIM_ARGS="--mode map --map /root/maps/house_clean.yaml" docker compose up     # 저장한 맵 (맵에 기록된 월드로 자동 실행)
WSIM_ARGS=none docker compose up                                               # 서버만
```

- `-d` 를 붙이면 백그라운드로 돌고, 진행 상황은 `docker compose logs -f` 로 봅니다.
- 컨테이너를 끄지 않고 웹 "실행 모드" 카드에서 모드/맵/월드를 바꿔도 됩니다 (서버가 같은 컨테이너 안에서 `run.sh` 를 실행).

### compose 없이 `docker run` 으로

```bash
docker build -t warehouse-sim .
docker run --rm -it --name wsim -p 8000:8000 \
  -v "$PWD":/app -v "$PWD/wsim/logs":/app/logs \
  -v warehouse-maps:/root/maps -v warehouse-data:/data \
  warehouse-sim --mode slam --world turtlebot3_house
```

`-v "$PWD":/app` 이 빠지면 entrypoint 가 "프로젝트 폴더가 연결되지 않았습니다" 를 출력하고 끝납니다.

---

## 5. 코드 수정과 반영

프로젝트 폴더의 파일을 고치면 컨테이너 안에서도 바로 같은 파일입니다. **이미지를 다시 빌드할 필요가 없습니다.**

| 고친 것 | 반영 방법 |
|---|---|
| `ros2_ws/src/.../*.py` (브리지, A*, Frenet) | 노드 재시작: 웹 "실행 모드" 에서 다시 시작, 또는 `docker exec wsim /app/scripts/run.sh --robot sim --mode map` |
| `config/params.yaml`, `config/*.yaml` | 위와 같이 노드 재시작 (`frenet.*` 등 일부는 웹 "주행 파라미터" 로 실행 중 변경 가능) |
| `scripts/*` (`*.sh`, `sim_real_io.py` 등), `tools/*.py`, `worlds/*` | 다음 실행부터 바로 |
| `server/` (Django 파이썬, 템플릿, `app.js`) | 서버가 파일 변경을 감지해 자동 재시작. 브라우저 새로고침 |
| `setup.py`, `launch/*`, 파이썬 파일 추가/삭제 | 컨테이너 재시작 (`docker compose restart`): 시작할 때 ROS 패키지를 다시 빌드 |
| `Dockerfile`, `server/requirements.txt` | 이미지 다시 빌드: `docker compose up --build` |

---

## 6. 로그와 컨테이너 안 확인

로그는 프로젝트 폴더에서 바로 봅니다.

```bash
ls wsim/logs                       # server, sim, nav2 | slam, bridge, colcon_build, run_mode.env, records/
tail -f wsim/logs/bridge.log
ls wsim/logs/records               # 주행 기록 (분석은 아래 docker exec 로 drive_replay.py)
```

ROS 명령은 컨테이너 안에서, 반드시 `source scripts/env.sh` 뒤에 실행합니다 (도메인 77, Cyclone DDS, 시뮬레이션 시간).

```bash
docker exec -it wsim bash                                                                       # 컨테이너 셸 (/app = 프로젝트)
docker exec wsim bash -c 'cd /app && source scripts/env.sh && ros2 topic hz /scan'              # 약 9 Hz 면 정상 (실물 LDS-02 와 같은 9.3 Hz)
docker exec wsim bash -c 'cd /app && source scripts/env.sh && python3 tools/ui_goto.py -1.6 0.6'   # 웹처럼 목표 보내기
docker exec wsim bash -c 'cd /app && source scripts/env.sh && python3 tools/drive_replay.py --list'
```

맵 꺼내기 / 넣기 (볼륨이라 `docker cp` 사용):

```bash
docker cp wsim:/root/maps ./maps_from_docker
docker cp ~/maps/my_map.yaml wsim:/root/maps/ && docker cp ~/maps/my_map.pgm wsim:/root/maps/
```

**리눅스 PC 주의**: 컨테이너는 root 로 돌아 `wsim/logs` 의 새 파일이 root 소유로 생깁니다. 끌 때(`Ctrl+C`, `docker compose down`)
entrypoint 가 프로젝트 폴더 주인에게 소유권을 돌려줍니다. 컨테이너를 강제로 지워 남은 경우: `sudo chown -R $USER: wsim`.

---

## 7. 맵 만들기 → 저장 → 그 맵으로 주행

```bash
# 1) SLAM 으로 시작
WSIM_ARGS="--mode slam --world turtlebot3_house" docker compose up
# 2) 웹에서 지도를 클릭해 로봇을 돌아다니게 하며 맵을 채움 (또는 자동 탐색)
# 3) 웹 "맵 저장" (이름 예: house) → /root/maps/house.yaml, house_clean.yaml (볼륨에 저장, 월드 이름도 기록됨)
# 4) Ctrl+C 로 끄고, 저장한 맵으로 다시 시작 (또는 끄지 않고 웹 "실행 모드" 에서 맵 선택)
WSIM_ARGS="--mode map --map /root/maps/house_clean.yaml" docker compose up
```

---

## 8. 넣지 않은 것 / 문제 해결

| 넣지 않은 것 | 이유 |
|---|---|
| 실물 로봇 (`--robot real`) | Pi 에 SSH 로 bringup, DDS 가 Wi-Fi 로 Pi 와 통신해야 함. 실물은 PC 에서 `scripts/run.sh` 로 |
| YOLO 검출기 | torch 가 크고, 시뮬레이션엔 Pi 카메라(`/csi/image_raw/compressed`)가 없음 (`WAREHOUSE_YOLO=0`) |
| Gazebo 화면 (`--gui`) | 화면 없는 gzserver 만. 화면이 필요하면 도커 없이 실행 |
| MySQL | SQLite 사용 (`DB_ENGINE=sqlite`, `SQLITE_PATH=/data/db.sqlite3`) |
| cartographer, AWS 창고 월드 | 설치하지 않음 (`--slam cartographer`, `--world small_warehouse` 는 실패) |

| 증상 | 원인 / 해결 |
|---|---|
| `open Dockerfile: no such file or directory`, `no configuration file provided` | 프로젝트 폴더가 아닌 곳에서 실행. `cd team_project2` 후 다시 |
| `프로젝트 폴더가 연결되지 않았습니다` | `docker run` 에 `-v "$PWD":/app` 이 빠짐. `docker compose up` 사용 권장 |
| `빌드 실패: wsim/logs/colcon_build.log` | `ros2_ws/src` 의 `setup.py`/`package.xml` 오류. 로그 확인 후 고치고 `docker compose restart` |
| 빌드 중 `Unable to locate package ros-humble-gazebo-ros-pkgs` | arm64 로 빌드된 경우. `Dockerfile` 의 `FROM --platform=linux/amd64` 확인 |
| `port is already allocated` / 웹이 안 열림 | 8000 포트 사용 중. PC 의 서버를 `scripts/stop_ros.sh --all` 로 끄거나 `compose.yaml` 의 포트를 `"8080:8000"` 으로 |
| `container name "/wsim" is already in use` | 이전 컨테이너가 남음: `docker rm -f wsim` |
| 웹에 로봇이 안 보임 (위치 없음) | `wsim/logs/nav2.log` (AMCL), `sim.log` (Gazebo, `/scan`·`/odom` 을 내는 `sim_real_io.py`). map 모드는 웹 "초기 위치 지정" 또는 "위치 자동 찾기" |
| 경로는 나오는데 로봇이 안 움직임 | 브리지는 `cmd_vel_pc` 로만 내고 명령 감시(`scripts/pi_cmd_watchdog.py`, `start_sim.sh` 가 띄움)가 `cmd_vel` 로 넘김. `sim.log` 에 `명령 감시 시작` 이 있는지 확인 |
| `sim.log` 의 `ALSA lib ... error` | 컨테이너에 사운드 장치가 없어서 나는 경고. 무시 |
| `run.sh 실패` 후 서버만 떠 있음 | 컨테이너는 계속 실행됨. 로그 확인 후 웹 "실행 모드" 에서 다시 시작 |
| DB 를 처음 상태로 | `docker compose down` 후 `docker volume rm warehouse-data` (맵 파일은 `warehouse-maps` 에 그대로) |
| Mac 에서 로봇이 느리게/끊기며 움직임 | amd64 에뮬레이션 때문. 시뮬레이션 시간 기준이라 결과는 맞지만 느림. Ubuntu/Windows PC 권장 |

---

## 9. Windows 에서 실행

Docker Desktop 이 WSL2 안의 리눅스에서 컨테이너를 돌리므로 동작은 같습니다. 일반 Windows PC 는 amd64 라 에뮬레이션 없이 돕니다.
(아직 Windows 실물에서 시험하지는 않았습니다. 문제가 있으면 이 절에 추가해 주세요.)

### 준비

- Docker Desktop 설정 → General → **Use the WSL 2 based engine** 켜기 (기본값)
- 메모리: Gazebo + AMCL + 서버가 같이 돌므로 8 GB 이상 권장. WSL2 는 기본으로 PC 메모리의 절반까지 씀
  (바꾸려면 `%USERPROFILE%\.wslconfig` 의 `[wsl2]` 아래 `memory=8GB` 후 `wsl --shutdown`)
- Git for Windows 로 저장소 받기

### 줄바꿈 (CRLF) 주의

Git for Windows 는 기본(`core.autocrlf=true`)으로 받을 때 줄바꿈을 CRLF 로 바꿉니다. 컨테이너는 **프로젝트 폴더의 스크립트를 그대로** 실행하므로,
CRLF 면 `/bin/bash^M: bad interpreter` 로 시작부터 멈춥니다.

- [.gitattributes](.gitattributes) 가 `*.sh`, `*.py`, `*.yaml`, `Dockerfile` 등을 Windows 에서 받아도 LF 로 유지합니다.
- `.gitattributes` 가 들어오기 전에 받은 저장소라면 한 번 다시 받아 LF 로 맞춥니다 (고친 파일이 없을 때만):

```powershell
git rm --cached -r -q .
git reset --hard
```

### 명령 (PowerShell)

`docker compose` 명령은 같습니다. 환경 변수 지정 방법만 다릅니다.

```powershell
cd C:\work\team_project2                     # 프로젝트 폴더
docker compose up --build                    # 처음
docker compose up                            # 기존 맵

$env:WSIM_ARGS="--mode slam --world turtlebot3_house"; docker compose up    # SLAM
Remove-Item Env:WSIM_ARGS                    # 기본(--mode map)으로 되돌리기 (같은 창에서 계속 남아 있음)

Get-Content wsim\logs\bridge.log -Wait       # 로그 보기 (tail -f)
docker exec wsim bash -c 'cd /app && source scripts/env.sh && ros2 topic hz /scan'
```

| 가이드 (bash) | PowerShell | cmd.exe |
|---|---|---|
| `WSIM_ARGS="..." docker compose up` | `$env:WSIM_ARGS="..."; docker compose up` | `set WSIM_ARGS=...` 다음 줄에 `docker compose up` |
| 줄 끝 `\` 로 이어 쓰기 | 줄 끝 `` ` `` (백틱) 또는 한 줄로 | 줄 끝 `^` 또는 한 줄로 |
| `-v "$PWD":/app` (`docker run`) | `-v "${PWD}:/app"` | `-v "%cd%:/app"` |
| `bash -c '...'` (작은따옴표) | 그대로 | `bash -c "..."` (큰따옴표로) |
| `tail -f wsim/logs/x.log` | `Get-Content wsim\logs\x.log -Wait` | (없음, 메모장 등으로 열기) |

웹은 http://localhost:8000/ 입니다. 다른 PC 에서 접속하려면 Windows 방화벽에서 8000 포트(Docker Desktop)를 허용해야 합니다.

### Windows 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `/bin/bash^M: bad interpreter` 또는 `$'\r': command not found` | 스크립트가 CRLF. 위 "줄바꿈 주의" 대로 다시 받기 (이미지 다시 빌드는 필요 없음) |
| `docker: error during connect ... pipe/docker_engine` | Docker Desktop 이 꺼져 있음. 실행 후 고래 아이콘이 "running" 이 될 때까지 대기 |
| 매우 느림, 메모리 부족으로 노드가 죽음 | WSL2 메모리 부족. `.wslconfig` 로 늘리고 `wsl --shutdown` 후 Docker Desktop 재시작 |
| 다른 PC 에서 웹 접속 안 됨 | Windows 방화벽에서 8000 포트 허용 |
