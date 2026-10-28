#!/bin/bash
# 컨테이너 시작: ROS 패키지 빌드 → DB 준비 → Django 서버 → scripts/run.sh --robot sim <인자> → 서버가 살아 있는 동안 유지
#   /app = 연결한 프로젝트 폴더 (compose.yaml), /app/logs = 프로젝트의 wsim/logs
#   인자: run.sh 옵션 (기본 --mode map). "none" 이면 서버만 띄우고 웹 "실행 모드" 카드에서 시작
cd /app
[ -f docker/entrypoint.sh ] && [ -d ros2_ws/src ] || {
  echo "[docker] /app 에 프로젝트 폴더가 연결되지 않았습니다. 프로젝트 폴더에서 docker compose up 으로 실행하세요 (DOCKER_GUIDE.md)"
  exit 1
}
mkdir -p logs /root/maps "$(dirname "$SQLITE_PATH")"

# 프로젝트의 ros2_ws/build|install 은 Ubuntu PC 경로라 쓸 수 없어서 컨테이너 안에 빌드한다.
# --symlink-install 이라 파이썬 소스는 /app/ros2_ws/src 를 그대로 가리킴 (고친 코드는 노드 재시작만으로 반영)
echo "[docker] ROS 패키지 빌드 → ${WAREHOUSE_ROS_INSTALL%/install}"
mkdir -p "${WAREHOUSE_ROS_INSTALL%/install}"
( cd "${WAREHOUSE_ROS_INSTALL%/install}" && source /opt/ros/humble/setup.bash \
  && colcon build --symlink-install --base-paths /app/ros2_ws/src --event-handlers console_cohesion- summary+ ) \
  > logs/colcon_build.log 2>&1 || { echo "[docker] 빌드 실패: wsim/logs/colcon_build.log"; tail -20 logs/colcon_build.log; exit 1; }

# 처음이면 저장소의 db.sqlite3(등록 위치 등)를 볼륨으로 복사해 시작
[ -f "$SQLITE_PATH" ] || cp server/db.sqlite3 "$SQLITE_PATH"
"$WAREHOUSE_SERVER_PY" server/manage.py migrate --noinput

# 컨테이너는 root 로 돌기 때문에 리눅스 PC 에서는 wsim/logs 파일이 root 소유가 된다 → 끌 때 프로젝트 폴더 주인에게 돌려줌
give_back_logs() { chown -R "$(stat -c %u:%g /app)" logs 2>/dev/null; }

stop() {
  echo "[docker] 정지: scripts/stop_ros.sh --all"
  scripts/stop_ros.sh --all
  give_back_logs
  exit 0
}
trap stop INT TERM

echo "[docker] 웹 서버 시작 (wsim/logs/server.log)"
scripts/start_server.sh > logs/server.log 2>&1 &
SERVER_PID=$!
for _ in $(seq 30); do
  curl -s -o /dev/null --max-time 2 http://localhost:8000/ && break
  sleep 1
done

if [ "$1" != none ]; then
  scripts/run.sh --robot sim "$@" || echo "[docker] run.sh 실패 (wsim/logs/*.log 확인). 서버는 계속 실행: 웹 '실행 모드' 카드에서 다시 시작"
fi
echo "[docker] 웹: http://localhost:8000/   로그: 프로젝트 폴더 wsim/logs/"

# 서버가 끝나면 컨테이너도 끝 (wait 는 신호를 받으면 바로 돌아와 trap 이 실행된다)
while kill -0 "$SERVER_PID" 2>/dev/null; do
  wait "$SERVER_PID"
done
echo "[docker] 웹 서버가 종료됨 (wsim/logs/server.log)"
scripts/stop_ros.sh --all
give_back_logs
