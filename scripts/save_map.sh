#!/bin/bash
# SLAM 으로 만든 맵을 저장하고 정리한다.
#   사용: scripts/save_map.sh <이름> [--straighten]
# 결과: ~/maps/<이름>.yaml/.pgm (원본) + ~/maps/<이름>_clean.yaml/.pgm (정리, 기본 사용)
#   --straighten: 정리 대신 기울기 보정 + 벽 직선화 (<이름>_straight). 벽이 가로/세로로만 된
#                 창고·방에서만 쓰세요. 대각선 벽이나 원기둥이 있는 곳에서는 맵이 망가집니다.
# 이후: scripts/run.sh --robot ... --mode map --map ~/maps/<이름>_clean.yaml
source "$(dirname "$0")/env.sh"
NAME="$1"
STRAIGHTEN=0
[ "$2" = "--straighten" ] && STRAIGHTEN=1
if [ -z "$NAME" ] || [[ ! "$NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "[save_map] 이름은 영문/숫자/_/- 만 쓸 수 있습니다: scripts/save_map.sh office_1f"
  exit 1
fi
MAP_DIR="$HOME/maps"
mkdir -p "$MAP_DIR"
if [ -e "$MAP_DIR/$NAME.yaml" ]; then
  echo "[save_map] 이미 있는 이름입니다: $MAP_DIR/$NAME.yaml"
  exit 1
fi

echo "[save_map] /map 저장 중 → $MAP_DIR/$NAME (도메인 $ROS_DOMAIN_ID)"
# free_thresh 0.196: 미탐색(205)이 빈 칸으로 읽히지 않게 (0.25 로 저장하면 방 밖이 빈 칸이 됨)
if ! timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$NAME" \
     --occ 0.65 --free 0.196 --ros-args -p use_sim_time:="$WAREHOUSE_SIM_TIME" -p save_map_timeout:=10.0; then
  echo "[save_map] 저장 실패: /map 을 받지 못했습니다 (SLAM 이 실행 중인지 확인)"
  exit 1
fi
[ -f "$MAP_DIR/$NAME.yaml" ] || { echo "[save_map] 저장 실패: 파일이 만들어지지 않았습니다"; exit 1; }

# 로봇이 다니던 공간을 기준으로 정리 (SLAM 원점 = 로봇 시작 위치 근처)
SEED_ARGS=()
if [ -n "$SEED_X" ]; then SEED_ARGS=(--seed "$SEED_X" "$SEED_Y"); fi
if [ "$STRAIGHTEN" = 1 ]; then
  TOOL=straighten_map.py; FINAL="$MAP_DIR/${NAME}_straight.yaml"
else
  TOOL=clean_map.py; FINAL="$MAP_DIR/${NAME}_clean.yaml"
fi
if ! python3 "$WAREHOUSE_HOME/tools/$TOOL" "$MAP_DIR/$NAME.yaml" "${SEED_ARGS[@]}"; then
  echo "[save_map] 정리 실패, 원본 맵을 사용하세요: $MAP_DIR/$NAME.yaml"
  FINAL="$MAP_DIR/$NAME.yaml"
fi
# 시뮬레이션에서 만든 맵은 어느 월드에서 만들었는지 남긴다 (기존 맵 모드에서 같은 월드로 띄우기 위함)
if [ "$WAREHOUSE_ROBOT" = sim ] && [ -n "$WAREHOUSE_WORLD" ]; then
  for f in "$MAP_DIR/$NAME.yaml" "$FINAL"; do
    [ -f "$f" ] || continue
    # 파일 끝에 줄바꿈이 없으면 붙어서 마지막 값이 망가지므로 먼저 줄바꿈을 보장
    [ -n "$(tail -c1 "$f")" ] && echo >> "$f"
    grep -q '^# world:' "$f" || echo "# world: $WAREHOUSE_WORLD" >> "$f"
  done
fi
echo "[save_map] 완료: $FINAL"
echo "[save_map] 이 맵으로 실행: scripts/run.sh --robot $WAREHOUSE_ROBOT --mode map --map $FINAL"
