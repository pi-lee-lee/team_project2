#!/bin/bash
# Save and clean up a map built with SLAM.
#   Usage: scripts/save_map.sh <name> [--plain | --straighten]
# Output: ~/maps/<name>.yaml/.pgm (raw) + ~/maps/<name>_clean.yaml/.pgm (cleaned, used by default)
#   default: tilt correction + 1-cell straight walls + square corners + 1-cell object outlines (tools/straighten_map.py --auto).
#            If wall points don't fit horizontal/vertical lines well (diagonal walls, round pillars), only clean up without rotating (1-cell outline).
#            If rotated, '# rotation_deg' is left in the yaml, and the web '맵 저장' also rotates locations registered during SLAM
#   --plain:      clean up only, no rotation/straightening (clean_map.py, 1-cell outline)
#   --straighten: always straighten regardless of the straight-line ratio (<name>_straight)
# Then: scripts/run.sh --robot ... --mode map --map ~/maps/<name>_clean.yaml
source "$(dirname "$0")/env.sh"
NAME="$1"
STYLE=auto
[ "$2" = "--straighten" ] && STYLE=straighten
[ "$2" = "--plain" ] && STYLE=plain
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
# free_thresh 0.196: so unknown (205) is not read as free (a higher value makes the area outside the room free)
if ! timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$NAME" \
     --occ 0.65 --free 0.196 --ros-args -p use_sim_time:="$WAREHOUSE_SIM_TIME" -p save_map_timeout:=10.0; then
  echo "[save_map] 저장 실패: /map 을 받지 못했습니다 (SLAM 이 실행 중인지 확인)"
  exit 1
fi
[ -f "$MAP_DIR/$NAME.yaml" ] || { echo "[save_map] 저장 실패: 파일이 만들어지지 않았습니다"; exit 1; }

# Clean up based on the space the robot drove in (SLAM origin = near the robot's start pose)
SEED_ARGS=()
if [ -n "$SEED_X" ]; then SEED_ARGS=(--seed "$SEED_X" "$SEED_Y"); fi
case "$STYLE" in
  straighten) TOOL=(straighten_map.py); FINAL="$MAP_DIR/${NAME}_straight.yaml" ;;
  plain)      TOOL=(clean_map.py); FINAL="$MAP_DIR/${NAME}_clean.yaml" ;;
  *)          TOOL=(straighten_map.py --auto -o "$MAP_DIR/${NAME}_clean"); FINAL="$MAP_DIR/${NAME}_clean.yaml" ;;
esac
if ! python3 "$WAREHOUSE_HOME/tools/${TOOL[0]}" "$MAP_DIR/$NAME.yaml" "${TOOL[@]:1}" "${SEED_ARGS[@]}"; then
  echo "[save_map] 정리 실패, 원본 맵을 사용하세요: $MAP_DIR/$NAME.yaml"
  FINAL="$MAP_DIR/$NAME.yaml"
fi
# Record which world a sim map was built in (so existing-map mode launches the same world)
if [ "$WAREHOUSE_ROBOT" = sim ] && [ -n "$WAREHOUSE_WORLD" ]; then
  for f in "$MAP_DIR/$NAME.yaml" "$FINAL"; do
    [ -f "$f" ] || continue
    # Ensure a trailing newline first, otherwise the line would be appended to the last value and break it
    [ -n "$(tail -c1 "$f")" ] && echo >> "$f"
    grep -q '^# world:' "$f" || echo "# world: $WAREHOUSE_WORLD" >> "$f"
  done
fi
echo "[save_map] 완료: $FINAL"
echo "[save_map] 이 맵으로 실행: scripts/run.sh --robot $WAREHOUSE_ROBOT --mode map --map $FINAL"
