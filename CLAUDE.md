# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

TurtleBot3 Waffle Pi warehouse robot: click a point/locations on a web map → the robot drives there with a hand-written A* planner + Frenet local planner, with a load-confirm → next location → return-home task flow. ROS 2 Humble, Django 4.2 + Channels, MySQL.

**Read `STATUS.md` first** — it is the running log of what works, what's been tested on the real robot, known issues and what's running right now. `README.md` is the user manual (setup, run modes, web UI, troubleshooting); `PROJECT_PLAN.md` is the original plan. All docs, comments, log messages and UI text are in Korean — keep new ones in Korean, and save `.md` files as UTF-8 with BOM (existing ones have it).

## Git workflow (applies to all work)

- Commit after each completed unit of work (one logical change per commit) — don't batch unrelated changes or leave finished work uncommitted.
- Every commit is immediately followed by `git push` to the current branch's remote. This is standing authorization; no need to ask first.
- Commit messages in Korean, matching the existing history.

## Commands

```bash
# Everything (background, logs in logs/). Records mode in logs/run_mode.env
scripts/run.sh --robot real|sim --mode slam|map [--map FILE] [--name DBNAME] [--slam slam_toolbox|cartographer] \
               [--world W] [--gui] [--amcl default|hall|FILE]
scripts/stop_ros.sh            # stop ROS nodes + Gazebo (never touches Pi bringup)
scripts/stop_ros.sh --all      # + Django server and ROS daemon
# Always stop via stop_ros.sh — killing nodes individually leaves orphan lifecycle_manager/amcl.

# Before any ros2 / tools/*.py command (sets domain, RMW, sim time from logs/run_mode.env)
source scripts/env.sh

# Build ROS package (symlink-install: yaml/python edits only need a node restart)
cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Unit tests (pure-python A* and follower; no ROS needed)
cd ros2_ws/src/warehouse_robot && python3 -m pytest test
python3 -m pytest test/test_astar.py -k <name>     # single test

# Django (uses server/.venv; DB creds from server/.env, DB_ENGINE=sqlite to fall back to db.sqlite3)
cd server && .venv/bin/python manage.py migrate
.venv/bin/python manage.py import_map ~/maps/<map>.yaml --name <name>
.venv/bin/python manage.py rotate_locations <deg>   # after rotating a map with straighten_map.py
```

Web UI: http://localhost:8000/ (PC 192.168.0.188). Pi: `ssh test@192.168.0.181`, bringup log `~/warehouse_bringup.log`. There are no Django tests.

## Architecture

```
browser ⇄ /ws/ui/ ⇄ Django (daphne, :8000) ⇄ /ws/robot/tb3/ ⇄ bridge_node ⇄ ROS 2 (A* service, AMCL/SLAM, cmd_vel) ⇄ Pi bringup / Gazebo
```

**Django server (`server/warehouse/`)**
- `consumers.py`: two JSON WebSocket consumers. `RobotConsumer` (bridge) and `UIConsumer` (browsers). High-rate data (`pose`, `path`, `health`) is kept only in the in-memory `LIVE` dict and relayed to the UI group, never written to DB. Everything that changes state calls into `services`.
- `services.py`: the task state machine. Every function is sync, runs via `database_sync_to_async`, mutates DB and returns an `Outbox` of messages (`to_robot` / `to_ui`); the consumer's `dispatch()` actually sends them. Raise `ServiceError` for errors that go back only to the requesting UI. Robot states: OFFLINE / IDLE / MOVING / WAITING_LOAD / ERROR. Tasks are `Task` + ordered `TaskStep`s, each step has a `goal_id` matched against the bridge's `nav_status` replies.
- Channel layer is `InMemoryChannelLayer` → assumes a single server process.
- Multiple robots: each bridge connects as `/ws/robot/<name>/` (launch args `robot_name` / `namespace`, env `WAREHOUSE_ROBOT_NAME` / `WAREHOUSE_NAMESPACE` in `start_bridge.sh`). Each UI connection has a selected robot (`?robot=` or `select_robot` action, default `tb3`); `UIConsumer.send_msg` drops other robots' messages (except `state` → `robot_summary`), and requests go to the selected robot unless `robot_name` is given (`robot` in runner actions means real/sim). AMCL/SLAM/Pi bringup are still single-robot.
- Maps are stored per name; `Location`s (incl. home) are tied to a map name. SLAM mode always uses map name `slam_live` so saved maps/locations aren't overwritten.
- Frontend is a single page: `templates/warehouse/index.html` + `static/warehouse/app.js` (canvas map, fixed fit-to-room, no zoom/pan).
- Orders (`services.start_order`): dropoff `Location` (kind `dropoff`) + `{label: count}` → one `load` step per section (`Location.cargo` = comma-separated labels; order from `planning.optimize`, dropoff last) → `unload` at the dropoff → home. Each step's `TaskStep.items` holds its counts; `confirm_load`/`confirm_unload` compare the YOLO counts (last 3 s mode of the bridge's `yolo` text, that section's items only) via `check_counts` and stay waiting on mismatch (the tablet then asks the person); `force_next` (`confirmed` = person checked) advances without checking. YOLO is switched on (`yolo_set`) only while stopped at an order step (on arrival) and off when leaving. Empty `items` = old behaviour.
- LED delivery (`services.start_delivery`, server side only — its web card was removed in favour of orders): one Task = `load` step at the LED location (step.cargo = expected label) → `unload` step at the requested point → home. Robot state `WAITING_UNLOAD`. `handle_cargo` auto-confirms when the camera label matches / disappears; `confirm_load` / `confirm_unload` are the manual fallbacks. New requests are rejected while a task runs. Robot-mounted tablet page: `/robot` (`robot.html` / `robot.js`). Orderer's page: `/order` (`order.html` / `order.js`, same `/ws/ui/`).
- `runner.py`: the web "실행 모드" card. The server itself runs `scripts/run.sh` / `stop_ros.sh` / `tools/localize.py --global` in a background thread (one job at a time) and broadcasts `{'type': 'runner', ...}` to the UI, so modes can be switched from the browser even when no ROS node is running. UI actions: `runner_start`, `runner_stop`, `auto_localize`, `list_maps`.

**Message protocol** (field `type`): bridge → server: `hello`, `pose`, `pose_lost`, `health`, `path`, `map`, `battery`, `nav_status`, `map_saved`, `reset_done`, `cargo` (camera LED label from ROS topic `cargo/detected`, debounced), `yolo` (YOLO detector state + `/yolo/counts` text), `params` / `param_result` (runtime-tunable bridge params for the web "주행 파라미터" card), `local_plan` (Frenet's chosen trajectory, display only), `obstacles` (tracks). Server → bridge: `goto`, `cancel`, `stop`, `initial_pose`, `save_map`, `reset`, `yolo_set` (detector `enabled` / `capture_once` via its parameter service), `param_set` (set a bridge ROS parameter; same path as `ros2 param set /warehouse_bridge ...`).

**YOLO (`yolo/`)**: merged from obloodobladeo-spec/team_project2 (component detector, weights, Colab notebooks). `warehouse.launch.py` starts `scripts/start_yolo.sh` with the bridge (`WAREHOUSE_YOLO=0` to skip); node `/component_detector_switch`, input `/csi/image_raw/compressed` (JPEG; raw over Wi-Fi saturated the Pi link and made health flap) from Pi `yolo/camera.sh`, which `run.sh --robot real` copies to the Pi and starts after bringup (`--no-camera` to skip; `stop_ros.sh` leaves it running), `enabled` defaults to false and the web "카메라 검출 (YOLO)" card toggles it. Adding a message means touching `bridge_node.py`, `consumers.py` (+ `services.py`) and usually `app.js`.

**ROS 2 package (`ros2_ws/src/warehouse_robot/`)**
- `astar.py` / `frenet.py` / `tracker.py`: pure numpy/math (frenet uses scipy), no ROS imports — this is what the unit tests cover. Keep them ROS-free.
- `astar_planner_node.py`: exposes `GridPlanner` as a `nav_msgs/GetPlan` service (`astar/get_plan`). Occupancy ≥65 and unknown are obstacles unless `allow_unknown` (set true in SLAM mode).
- `bridge_node.py`: WebSocket client to the server (`websocket-client` in a thread, inbox queue drained by a timer), goal execution, and safety (forward stop within `estop_distance`, tilt-while-blocked within `stop_distance` → back off, TF staleness stop, stall/tip detection, progress/blocked recovery). Driving is A* + the Frenet local planner only (PP/DWA/hybrid were removed on branch `frenet-only`; `hybrid-predict-box` still has them): `frenet.py` `FrenetFollower` turns the A* path into a smoothed spline reference and every 0.1 s picks the cheapest safe candidate from lateral offset x shift/hold distance x speed (static obstacles checked by distance along the path, moving tracks (`tracking.moving_speed`) by constant-velocity prediction over time and ignored while fully behind the robot, `frenet.safety_margin` as a hard constraint), then executes the start of that plan (v, w = v·κ). Turning in place (start, >60° off the path tangent while stopped / >100° while moving, make-room moves in corners), approach past the path end and goal-heading alignment are handled outside the candidates. Blocked by static obstacles alone → immediate A* replan after 1 s (up to 4 per goal, A* remembers the obstacle), otherwise `blocked` → wait `blocked_timeout` → back off/turn → A* replan. The chosen trajectory is sent as `local_plan` and drawn on the web map. Map save/reset are done by the bridge running `scripts/save_map.sh` / `scripts/reset_robot.sh`.
- Parameters: `config/params.yaml`; scalar `frenet.*`, `tracking.moving_speed` and safety/recovery values can be changed while running (`BridgeNode.on_set_params`, memory only — restart reverts to yaml); others are rejected as restart-only; launch args (map_name, mode, use_sim_time, allow_unknown, server_url) come from `scripts/start_bridge.sh`.

**Scripts / environment**
- `run.sh` orchestrates `start_server.sh` → Pi bringup over SSH or `start_sim.sh` → `start_slam.sh` or `start_nav2.sh` (map_server+AMCL; `config/amcl_waffle_pi.yaml`) → `start_bridge.sh`.
- `env.sh`: real robot = `ROS_DOMAIN_ID=44`, sim = 77 (so they never mix). PC uses Cyclone DDS (`rmw_cyclonedds_cpp`); the Pi stays on Fast DDS.
- `tools/`: standalone diagnostics — `frenet_bench.py` (re-drives the current FrenetFollower closed-loop on each real drive record's accumulated static scan points with A* replans, prints arrival/time/blocked/mid-route turns/w jitter/offset changes/min clearance; uses the current params.yaml unless `--recorded`; use it to compare before/after a planner change or `--set frenet.x=v`), `drive_replay.py` (reads the bridge's per-goal drive record `logs/records/<bridge start>_<controller>/<time>_<goal>/record.jsonl.gz`, written when `record.enabled`; `--list` lists sessions/runs: pose/cmd/hybrid state/tracks/scan every 0.1 s plus Frenet inputs and per-candidate filter/cost results; summarizes, explains a moment with `--at T [--plot]`, re-runs the planner on the recorded inputs with `--set frenet.x=v`), `localize.py` (scan-matching initial pose), `tf_rates.py`, `ui_goto.py` (drive via the server WebSocket like the UI), `clean_map.py` / `straighten_map.py` (map cleanup; output saved with `free_thresh: 0.196` so map_server doesn't read unknown as free).
- Maps live outside the repo in `~/maps/` (current: `~/maps/backup/backup2/maze_map_straight.yaml`, DB name `maze_map`).

## Gotchas

- Nav2 nodes (AMCL) stalling on TF was a **tf2/tf2_ros 0.25.23 ABBA deadlock**, fixed in 0.25.24 — not a DDS issue (`dpkg -l ros-humble-tf2 ros-humble-tf2-ros`). Only Nav2's map_server + AMCL are used (`start_nav2.sh`); Nav2 planner/controller were removed on this branch.
- `ros2 topic list` showing only `/parameter_events`, `/rosout` → domain mismatch; `source scripts/env.sh` and `ros2 daemon stop`. Stale nodes in `ros2 node list` → check with `--no-daemon`.
- Map yaml `free_thresh` > 0.196 makes map_server treat unknown (205) as free, letting A* plan outside the room.
- AMCL, slam_toolbox (`config/slam_toolbox.yaml`) and the A* obstacle memory subscribe to **`scan_level`**, which `bridge_node` republishes from `/scan` only while the robot isn't tilted (relative to its usual tilt; passes again after 3 s). Without the bridge running, localization/SLAM get no scans.
- A* planner remembers LiDAR obstacles not in the map (AMCL mode only, 20 s) and bump points from the bridge (`astar/obstacle`, 120 s); `astar/clear_obstacles` (called by web "상태 초기화") clears them. Diagnose planning failures with `logs/astar_fail.npz`.
- Sim maps record their world as `# world: <name>` in the yaml; `run.sh` uses it when `--world` is omitted.
- `ros2_ws/build|install|log`, `server/.venv`, `logs/` and `server/.env` are committed (no `.gitignore`); don't hand-edit build/install outputs.
