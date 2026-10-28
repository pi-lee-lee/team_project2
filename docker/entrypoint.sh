#!/bin/bash
# 컨테이너 시작: DB 준비 → Django 서버 → scripts/run.sh --robot sim <인자> → 서버가 살아 있는 동안 유지
#   인자: run.sh 옵션 (기본 --mode map). "none" 이면 서버만 띄우고 웹 "실행 모드" 카드에서 시작
cd /app
mkdir -p logs /root/maps "$(dirname "$SQLITE_PATH")"

# 처음이면 저장소의 db.sqlite3(등록 위치 등)를 볼륨으로 복사해 시작
[ -f "$SQLITE_PATH" ] || cp server/db.sqlite3 "$SQLITE_PATH"
server/.venv/bin/python server/manage.py migrate --noinput

stop() {
  echo "[docker] 정지: scripts/stop_ros.sh --all"
  scripts/stop_ros.sh --all
  exit 0
}
trap stop INT TERM

echo "[docker] 웹 서버 시작 (logs/server.log)"
scripts/start_server.sh > logs/server.log 2>&1 &
SERVER_PID=$!
for _ in $(seq 30); do
  curl -s -o /dev/null --max-time 2 http://localhost:8000/ && break
  sleep 1
done

if [ "$1" != none ]; then
  scripts/run.sh --robot sim "$@" || echo "[docker] run.sh 실패 (logs/*.log 확인). 서버는 계속 실행: 웹 '실행 모드' 카드에서 다시 시작"
fi
echo "[docker] 웹: http://localhost:8000/   로그: docker exec <컨테이너> tail -f /app/logs/<이름>.log"

# 서버가 끝나면 컨테이너도 끝 (wait 는 신호를 받으면 바로 돌아와 trap 이 실행된다)
while kill -0 "$SERVER_PID" 2>/dev/null; do
  wait "$SERVER_PID"
done
echo "[docker] 웹 서버가 종료됨 (logs/server.log)"
scripts/stop_ros.sh --all
