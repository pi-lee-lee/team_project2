# Docker 실행 가이드 (시뮬레이션)

시뮬레이션 전체를 **컨테이너 하나**로 실행합니다. 들어 있는 것: Django 서버, Gazebo(TurtleBot3 Waffle Pi), SLAM/AMCL, A* 플래너, 브리지.
파일: [Dockerfile](Dockerfile), [.dockerignore](.dockerignore), [docker/entrypoint.sh](docker/entrypoint.sh).
옵션 전체는 [README.md](README.md), 직접 실행(도커 없이)은 [RUN_GUIDE.md](RUN_GUIDE.md) 참고.

---

## 1. 한눈에 보기

| 하고 싶은 것 | 명령 (저장소 폴더에서) |
|---|---|
| 이미지 빌드 | `docker build -t warehouse-sim .` |
| 기존 맵으로 주행 (turtlebot3_world) | `docker run --rm -it --name wsim -p 8000:8000 -v warehouse-maps:/root/maps -v warehouse-data:/data warehouse-sim` |
| 맵 새로 만들기 (SLAM) | 위 명령 뒤에 `--mode slam --world turtlebot3_house` |
| 저장한 맵으로 주행 | 위 명령 뒤에 `--mode map --map /root/maps/<이름>_clean.yaml` |
| 서버만 띄우기 | 위 명령 뒤에 `none` (웹 "실행 모드" 카드에서 시작) |
| 웹 | http://localhost:8000/ (다른 PC 에서는 `http://<도커 PC IP>:8000/`) |
| 끄기 | 실행한 터미널에서 `Ctrl+C`, 또는 `docker stop wsim` |

---

## 2. 준비

- Docker 설치 (Ubuntu: Docker Engine, Mac/Windows: Docker Desktop). 디스크 약 6 GB (이미지 약 5 GB).
  Windows 는 [9. Windows 에서 실행](#9-windows-에서-실행) 도 확인.
- **CPU 종류**: 이미지는 `linux/amd64` 고정입니다. Gazebo classic 의 ROS 2 Humble 패키지(`gazebo_ros`, `turtlebot3_gazebo`)가 amd64 만 있기 때문입니다.
  - Ubuntu PC (Intel/AMD): 그대로 빠르게 동작
  - Windows PC (Intel/AMD): Docker Desktop(WSL2) 에서 그대로 동작
  - Apple Silicon Mac: 에뮬레이션으로 동작하지만 느림 (실행 때 `platform (linux/amd64) does not match` 경고는 정상)
- 포트 8000 이 비어 있어야 합니다. 같은 PC 에서 `scripts/run.sh` 로 띄운 서버가 있으면 먼저 `scripts/stop_ros.sh --all`.

---

## 3. 빌드

```bash
cd team_project2          # 저장소 폴더
docker build -t warehouse-sim .
```

이미지 안에서 하는 일:
1. ROS 2 Humble + Gazebo + turtlebot3 + map_server/AMCL + slam_toolbox + Cyclone DDS 설치
2. `server/.venv` 새로 만들기 (Django, channels, daphne). DB 는 SQLite 라 `mysqlclient` 는 빼고 설치
3. `ros2_ws` 를 `colcon build --symlink-install`

저장소에 커밋된 `server/.venv`, `ros2_ws/build|install` 은 PC 경로로 만들어진 것이라 `.dockerignore` 로 빼고 이미지 안에서 새로 만듭니다.
**코드를 고치면 다시 빌드해야** 반영됩니다 (변경된 단계부터만 다시 실행).

---

## 4. 실행

```bash
docker run --rm -it --name wsim -p 8000:8000 \
  -v warehouse-maps:/root/maps \
  -v warehouse-data:/data \
  warehouse-sim [run.sh 인자 | none]
```

| 옵션 | 뜻 |
|---|---|
| `--rm` | 끄면 컨테이너 삭제 (맵/DB 는 볼륨에 남음) |
| `-it` | 터미널에 진행 로그 표시, `Ctrl+C` 로 정지 |
| `-d` (`-it` 대신) | 백그라운드 실행. 로그는 `docker logs -f wsim` |
| `--name wsim` | 컨테이너 이름 (아래 `docker exec` 에서 사용) |
| `-p 8000:8000` | 웹 포트 |
| `-v warehouse-maps:/root/maps` | SLAM 으로 저장한 맵 (`~/maps`) |
| `-v warehouse-data:/data` | DB (`/data/db.sqlite3`: 맵, 등록 위치, 작업 기록) |

뒤에 붙이는 인자는 `scripts/run.sh --robot sim` 에 그대로 넘어갑니다 (생략하면 `--mode map`).

```bash
warehouse-sim                                          # 기존 맵: turtlebot3_world + TurtleBot3 기본 맵, 초기 위치 자동 지정
warehouse-sim --mode slam                              # SLAM, turtlebot3_world
warehouse-sim --mode slam --world turtlebot3_house     # SLAM, 집 월드
warehouse-sim --mode slam --world lecture_hall         # 저장소 worlds/ 의 월드
warehouse-sim --mode map --map /root/maps/house_clean.yaml   # 저장한 맵 (맵에 기록된 월드로 자동 실행)
warehouse-sim none                                     # 서버만
```

### 컨테이너 시작 순서 (`docker/entrypoint.sh`)

1. DB 준비: `/data/db.sqlite3` 가 없으면 저장소의 `server/db.sqlite3` 를 복사 → `migrate`
2. 웹 서버 시작 (로그 `logs/server.log`)
3. `scripts/run.sh --robot sim <인자>`: Gazebo → AMCL 또는 SLAM → A* + 브리지 → (map 모드) 초기 위치 자동 지정
4. 웹 서버가 살아 있는 동안 유지. 끄라는 신호(`Ctrl+C`, `docker stop`)를 받으면 `scripts/stop_ros.sh --all` 로 노드를 정리하고 종료

정상 시작이면 마지막에 이렇게 나옵니다 (Ubuntu PC 기준 약 40초, Mac 에뮬레이션은 더 오래):

```
5) 초기 위치 자동 지정 (-2.0, -0.5)
   가장 잘 맞는 위치: x=-1.975 y=-0.500 yaw=0.0°  (스캔 점 중 벽과 10cm 이내 100%, ...)
== 완료. 웹: http://172.17.0.2:8000/   끄기: scripts/stop_ros.sh --all
[docker] 웹: http://localhost:8000/   ...
```

`172.17.0.2` 는 컨테이너 내부 주소입니다. 브라우저에서는 **http://localhost:8000/** 로 접속합니다.

---

## 5. 맵 만들기 → 저장 → 그 맵으로 주행

```bash
# 1) SLAM 으로 시작
docker run --rm -it --name wsim -p 8000:8000 -v warehouse-maps:/root/maps -v warehouse-data:/data \
  warehouse-sim --mode slam --world turtlebot3_house
# 2) 웹에서 지도를 클릭해 로봇을 돌아다니게 하며 맵을 채움 (또는 자동 탐색)
# 3) 웹 "맵 저장" (이름 예: house) → /root/maps/house.yaml, house_clean.yaml (볼륨에 저장, 월드 이름도 기록됨)
# 4) Ctrl+C 로 끄고, 저장한 맵으로 다시 시작
docker run --rm -it --name wsim -p 8000:8000 -v warehouse-maps:/root/maps -v warehouse-data:/data \
  warehouse-sim --mode map --map /root/maps/house_clean.yaml
```

컨테이너를 끄지 않고 웹 "실행 모드" 카드에서 모드/맵을 바꿔도 됩니다 (서버가 같은 컨테이너 안에서 `run.sh` 를 실행).

---

## 6. 컨테이너 안에서 확인하기

```bash
docker exec -it wsim bash                                   # 컨테이너 셸 (/app = 저장소)
docker exec wsim tail -f /app/logs/bridge.log               # 노드 로그: server, sim, nav2 | slam, bridge
docker exec wsim bash -c 'cd /app && source scripts/env.sh && ros2 topic hz /scan'           # ROS 명령 (도메인 77)
docker exec wsim bash -c 'cd /app && source scripts/env.sh && python3 tools/ui_goto.py -1.6 0.6'   # 웹처럼 목표 보내기
```

ROS 명령은 반드시 `source scripts/env.sh` 뒤에 실행합니다 (도메인 77, Cyclone DDS, 시뮬레이션 시간).
`/scan` 이 약 5 Hz 로 나오고, `ui_goto.py` 가 "도착" 으로 끝나면 정상입니다.

### 파일 꺼내기 / 넣기

```bash
docker cp wsim:/root/maps ./maps_from_docker                # 저장한 맵
docker cp wsim:/app/logs ./logs_from_docker                 # 로그, 주행 기록(logs/records)
docker cp ~/maps/my_map.yaml wsim:/root/maps/ && docker cp ~/maps/my_map.pgm wsim:/root/maps/   # 맵 넣기 (yaml + pgm)
```

---

## 7. 넣지 않은 것

| 항목 | 이유 |
|---|---|
| 실물 로봇 (`--robot real`) | Pi 에 SSH 로 bringup, DDS 가 Wi-Fi 로 Pi 와 통신해야 함. 실물은 PC 에서 `scripts/run.sh` 로 |
| YOLO 검출기 | torch 가 크고, 시뮬레이션엔 Pi 카메라(`/csi/image_raw/compressed`)가 없음 (`WAREHOUSE_YOLO=0`) |
| Gazebo 화면 (`--gui`) | 화면 없는 gzserver 만. 화면이 필요하면 도커 없이 실행 |
| MySQL | SQLite 사용 (`DB_ENGINE=sqlite`, `SQLITE_PATH=/data/db.sqlite3`) |
| cartographer, AWS 창고 월드 | 설치하지 않음 (`--slam cartographer`, `--world small_warehouse` 는 실패) |

---

## 8. 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| 빌드 중 `Unable to locate package ros-humble-gazebo-ros-pkgs` | arm64 로 빌드된 경우. `Dockerfile` 의 `FROM --platform=linux/amd64` 가 지워지지 않았는지 확인 |
| `port is already allocated` / 웹이 안 열림 | 8000 포트 사용 중. PC 의 서버를 `scripts/stop_ros.sh --all` 로 끄거나 `-p 8080:8000` 으로 바꿔 http://localhost:8080/ |
| `Conflict. The container name "/wsim" is already in use` | 이전 컨테이너가 남음: `docker rm -f wsim` |
| 웹에 로봇이 안 보임 (위치 없음) | `docker exec wsim tail -50 /app/logs/nav2.log` (AMCL), `.../sim.log` (Gazebo). map 모드는 웹 "초기 위치 지정" 또는 "위치 자동 찾기" |
| `sim.log` 의 `ALSA lib ... error` | 컨테이너에 사운드 장치가 없어서 나는 경고. 무시 |
| `run.sh 실패` 후 서버만 떠 있음 | 컨테이너는 계속 실행됨. 로그 확인 후 웹 "실행 모드" 에서 다시 시작 |
| DB 를 처음 상태로 | `docker volume rm warehouse-data` (맵 파일은 `warehouse-maps` 에 그대로) |
| Mac 에서 로봇이 느리게/끊기며 움직임 | amd64 에뮬레이션 때문. 시뮬레이션 시간 기준이라 결과는 맞지만 느림. Ubuntu PC 에서 실행 권장 |

---

## 9. Windows 에서 실행

Docker Desktop 이 WSL2 안의 리눅스에서 컨테이너를 돌리므로 이미지와 동작은 같습니다. 일반 Windows PC 는 amd64 라 에뮬레이션 없이 돕니다.
(아직 Windows 실물에서 시험하지는 않았습니다. 문제가 있으면 이 절에 추가해 주세요.)

### 준비

- Docker Desktop 설정 → General → **Use the WSL 2 based engine** 켜기 (기본값)
- 메모리: Gazebo + AMCL + 서버가 같이 돌므로 8 GB 이상 권장. WSL2 는 기본으로 PC 메모리의 절반까지 씀
  (줄이거나 늘리려면 `%USERPROFILE%\.wslconfig` 의 `[wsl2]` 아래 `memory=8GB` 후 `wsl --shutdown`)
- Git for Windows 로 저장소 받기

### 줄바꿈 (CRLF) 주의

Git for Windows 는 기본(`core.autocrlf=true`)으로 받을 때 줄바꿈을 CRLF 로 바꿉니다. 셸 스크립트가 CRLF 면 리눅스에서
`/bin/bash^M: bad interpreter` 로 실행이 안 됩니다. 이를 두 겹으로 막아 두었습니다.

- [.gitattributes](.gitattributes): `*.sh`, `*.py`, `*.yaml`, `Dockerfile` 등은 Windows 에서 받아도 LF 로 유지
- `Dockerfile`: 이미지 안에서 `*.sh` 의 CR 을 한 번 더 지우고 실행 권한을 줌

`.gitattributes` 가 들어오기 전에 받은 저장소라면 한 번 다시 받아 LF 로 맞춥니다 (고친 파일이 없을 때만):

```powershell
git rm --cached -r -q .
git reset --hard
```

### 명령 (PowerShell)

가이드의 명령은 대부분 그대로 쓰면 됩니다. 다른 점:

| 가이드 (bash) | PowerShell | cmd.exe |
|---|---|---|
| 줄 끝 `\` 로 이어 쓰기 | 줄 끝 `` ` `` (백틱) 또는 한 줄로 | 줄 끝 `^` 또는 한 줄로 |
| `bash -c '...'` (작은따옴표) | 그대로 | `bash -c "..."` (큰따옴표로) |
| `~/maps/...` | 그대로 (`~` = 사용자 폴더) | `%USERPROFILE%\maps\...` |

```powershell
cd C:\work\team_project2          # 저장소 폴더
docker build -t warehouse-sim .

# 실행 (한 줄)
docker run --rm -it --name wsim -p 8000:8000 -v warehouse-maps:/root/maps -v warehouse-data:/data warehouse-sim

# 실행 (여러 줄: 백틱)
docker run --rm -it --name wsim -p 8000:8000 `
  -v warehouse-maps:/root/maps -v warehouse-data:/data `
  warehouse-sim --mode slam --world turtlebot3_house

# 컨테이너 안 확인
docker exec wsim bash -c 'cd /app && source scripts/env.sh && ros2 topic hz /scan'
docker cp wsim:/root/maps .\maps_from_docker
```

웹은 http://localhost:8000/ 입니다. 다른 PC 에서 접속하려면 Windows 방화벽에서 8000 포트(Docker Desktop)를 허용해야 합니다.

### Windows 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `/bin/bash^M: bad interpreter` 또는 `$'\r': command not found` | 스크립트가 CRLF. 위 "줄바꿈 주의" 대로 다시 받고 다시 빌드 |
| `docker: error during connect ... pipe/docker_engine` | Docker Desktop 이 꺼져 있음. 실행 후 고래 아이콘이 "running" 이 될 때까지 대기 |
| 빌드/실행이 매우 느림, 메모리 부족으로 노드가 죽음 | WSL2 메모리 부족. `.wslconfig` 로 늘리고 `wsl --shutdown` 후 Docker Desktop 재시작 |
| 다른 PC 에서 웹 접속 안 됨 | Windows 방화벽에서 8000 포트 허용 |
