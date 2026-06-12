# IGVI Robot

IGVI Robot is the integrated system for the WildBot / IGVI competition robot. It is built on **ROS 2 Jazzy** at its core, uses **Docker Compose** to orchestrate the subsystems (base, sensing, SLAM, navigation, perception, missions), and exposes a **FastAPI host agent** + **PySide6 desktop UI** as the local control center.

Day-to-day operation is UI-driven: toggle services on the `Docker` page, run missions on the `Robot` / `Door` pages — no need to type `docker compose` by hand every time.

## Table of Contents

- [IGVI Robot](#igvi-robot)
  - [Table of Contents](#table-of-contents)
  - [System Architecture](#system-architecture)
  - [Project Layout](#project-layout)
  - [Quick Start](#quick-start)
    - [1. One-time Setup](#1-one-time-setup)
    - [2. Install Python Dependencies](#2-install-python-dependencies)
    - [3. Launch host + UI](#3-launch-host--ui)
  - [Docker Services \& Profiles](#docker-services--profiles)
  - [Operating via the UI](#operating-via-the-ui)
    - [Docker Page](#docker-page)
    - [Robot Page](#robot-page)
    - [Door Page](#door-page)
    - [Settings Page](#settings-page)
  - [Missions](#missions)
    - [Grasp Stack (`wildbot_grasp` service)](#grasp-stack-wildbot_grasp-service)
    - [Starting a Mission](#starting-a-mission)
  - [Sensing, Localization \& Navigation Pipeline](#sensing-localization--navigation-pipeline)
  - [Perception: YOLO Detection \& Semantic Memory](#perception-yolo-detection--semantic-memory)
  - [Host Agent API](#host-agent-api)
  - [Configuration \& Environment Variables](#configuration--environment-variables)
  - [Building the ROS Workspace](#building-the-ros-workspace)
  - [Tests](#tests)
  - [Debugging via CLI](#debugging-via-cli)

## System Architecture

```text
Hardware (Kinect / LiDAR / wheels / arm)
   │  USB
   ▼
Docker Compose services (ROS 2 nodes)
   │  ROS 2 topic / action network (FastDDS)
   ▼
igvi_bridge  ── ROS ↔ HTTP bridge (port 8771)
   │  HTTP
   ▼
igvi_host    ── FastAPI host agent (port 8770), also drives the Docker Engine
   │  HTTP (localhost)
   ▼
igvi_ui      ── PySide6 desktop control center
```

Layers:

- **ROS 2 layer** — all real-time computation runs in ROS 2 nodes inside Docker containers, interoperating through a shared FastDDS profile (`robot_ws/configs/fastdds_shm.xml`).
- **Bridge layer** — `igvi_bridge` wraps map, pose, navigation, missions, images, arm temperatures, etc. as HTTP `/api/*` so non-ROS clients can drive the robot.
- **Host layer** — `igvi_host` is the only component that touches the Docker Engine API directly, and it also proxies the ROS bridge; the UI talks to it over localhost HTTP only.
- **UI layer** — `igvi_ui` provides Docker management, teleoperation, mission control, and calibration settings.

## Project Layout

```text
IGVI_Robot/
├── apps/                          # Local Python tools (managed by uv)
│   ├── igvi_host/                 # FastAPI host agent (8770): bridges Docker + ROS bridge → UI
│   ├── igvi_ui/                   # PySide6 desktop control center
│   │   ├── pages/                 # docker / robot / door / settings / capture
│   │   ├── widgets/               # arm, navigation, waypoint, search/bridge retrieve, map, image...
│   │   └── clients/host_client.py # HTTP client for the host agent
│   ├── tests/                     # pytest (service registry / config)
│   └── main.py
├── robot_ws/
│   ├── src/
│   │   ├── igvi_bridge/           # ROS ↔ HTTP bridge node (8771) + door_mission + open_door_proxy
│   │   ├── motion_arbiter/        # sole /cmd_vel authority: pure-pursuit + manual override + estop
│   │   ├── igvi_imu/              # online IMU gyro-bias calibration
│   │   ├── eto_eye/               # YOLO detector_node + semantic_memory_node
│   │   └── wildbot_grasp/         # arm/gripper + all high-level mission action servers
│   ├── configs/                   # calibration / controllers / ekf / nav2 / task_speeds / urdf ...
│   ├── launch/                    # kinect_bringup / slam_fusion / slam_localization / nav2 / lidar
│   ├── scripts/                   # lidar_transformer_node.py, etc.
│   └── patches/                   # Azure Kinect driver patches
├── docker/
│   ├── images/                    # Per-service Dockerfiles (base / bridge / motion / fusion / grasp / kinect / foxglove / eto_eye_*)
│   └── compose/
│       ├── compose.yaml           # Entry point that includes all services/*.yaml
│       ├── .env(.example)         # ROS_DOMAIN_ID / IGVI_DATA_ROOT / mission params
│       └── services/              # One yaml per service
├── tools/legacy_scripts/          # Old bash scripts (kept for transition)
├── tuner_output/                  # open_door pose tuning output
├── .docs/ , docs/                 # Planning docs and hardware docs
├── Makefile
└── CLAUDE.md
```

Each package has its own README with deeper detail:

- `apps/README.md` — host agent / UI startup and Qt prerequisites
- `robot_ws/src/wildbot_grasp/README.md` — grasp stack, arm poses, distance/centering calibration
- `docs/README_kros_car.md` — KROS car base notes

## Quick Start

### 1. One-time Setup

Run once from the repo root to create `docker/compose/.env` and the shared data directories:

```bash
make install
# or with a custom shared data path:
make install DATA_ROOT=/your/shared/path
```

This will:

- Generate `docker/compose/.env` from `.env.example` (`.env` is gitignored).
- Write `IGVI_DATA_ROOT` into `.env` (default `~/igvi_robot`).
- Create the shared directories `slam/`, `maps/`, `models/eto_eye/`, `migraphx_cache/eto_eye/`.

Run `make install` once per worktree on the same machine — they all point to the same absolute path, so map/model data is shared. Before running on a shared network, set a unique `ROS_DOMAIN_ID` in `.env`.

### 2. Install Python Dependencies

```bash
make sync          # equivalent to: cd apps && uv sync
```

Qt prerequisite on Linux:

```bash
sudo apt update && sudo apt install -y libxcb-cursor0
```

`igvi-ui` must run inside a graphical desktop session (`$DISPLAY` or `$WAYLAND_DISPLAY` must be set).

### 3. Launch host + UI

```bash
make run              # start igvi-host in the background, then open igvi-ui
make host             # host agent only (loopback)
make host lan         # bind host agent to 0.0.0.0 so other LAN machines can connect
make ui               # UI only
make run 192.168.1.5  # point the UI at a remote host (http://192.168.1.5:8770)
```

Ports: host agent `8770`, igvi_bridge `8771`, Foxglove `8765`.

## Docker Services & Profiles

`docker/compose/compose.yaml` pulls in `services/*.yaml` via `include`, and each service belongs to a **profile bundle**. The UI's service registry automatically discovers every included service.

| Profile | Services | Purpose |
|---|---|---|
| `robot` | `kros_car`, `camera_kinect`, `motion_arbiter`, `lidar`, `wildbot_grasp` | Base + arm controllers, Azure Kinect, LiDAR, velocity arbitration, grasp stack |
| `application` | `eto_eye_gpu` (or `eto_eye_cpu`), `nav2` | YOLO detection + semantic memory, planning-only Nav2 |
| `communication` | `igvi_bridge`, `foxglove_bridge` | ROS↔HTTP bridge (8771), Foxglove WS (8765) |
| `slam_system` | `slam_fusion`, `slam_localization` | RTAB-Map mapping / localization |
| `task_server` | `search_retrieve_server`, `arena_mission_server`, `bridge_retrieve`, `bridge_traverse`, `open_door_server` | High-level mission action servers (share the `wildbot_grasp:latest` image) |

Service roles:

- **kros_car** — KROS car bringup; starts the base and arm controllers (`/dev/usb_wheel`, `/dev/usb_robot_arm`).
- **camera_kinect** — Azure Kinect driver publishing `/rgb/image_raw`, `/depth_to_rgb/image_raw`, IMU and camera info; also starts the `igvi_imu` gyro-bias calibrator.
- **motion_arbiter** — the sole publisher of `/cmd_vel`. Combines pure-pursuit tracking of the Nav2 `/plan`, manual override via `/motion/cmd` and `/motion/operator_cmd`, and `/estop` emergency stop.
- **lidar** — oradar driver + NaN filter + timestamp re-stamp → `/scan` (`/dev/oradar`).
- **wildbot_grasp** — grasp stack (see the Missions section).
- **eto_eye_gpu / eto_eye_cpu** — `detector_node` (ONNX YOLO) + `semantic_memory_node`. The GPU variant runs ROCm / MIGraphX.
- **nav2** — planning-only Nav2 (`planner_server` + `bt_navigator` + a custom plan-only BT; the costmap subscribes directly to `/map`). **No controller** — paths are executed by motion_arbiter.
- **igvi_bridge** — ROS↔HTTP bridge (8771), hosting the door-mission coordinator and open_door proxy.
- **foxglove_bridge** — Foxglove Studio WebSocket.
- **slam_fusion / slam_localization** — RTAB-Map mapping / localization against an existing `arena_map.db`.

> By default `compose.yaml` includes only the GPU detector. To use the CPU variant, uncomment `- services/eto_eye_cpu.yaml` in `compose.yaml`, and **do not** run `eto_eye_cpu` and `eto_eye_gpu` at the same time (both publish `/detections_json` and would conflict). `services/` also holds non-default services such as `camera_gemini`, `unity_*`, `urdf_viz`, and `rosbridge` — include them when needed.

Starting a single profile from the CLI (the UI is preferred day-to-day):

```bash
cd docker/compose
docker compose -f compose.yaml --profile robot up -d
docker compose -f compose.yaml --profile slam_system up -d
docker compose -f compose.yaml --profile application up -d
```

## Operating via the UI

UI tabs on the left rail: **Docker**, **Robot**, **Door**, **Settings**. The top status bar shows Sensors / Host Agent / Docker / Compose / UI Bridge / Battery and an **EMERGENCY STOP** button.

### Docker Page

Select services, then act with the buttons; on first run, `Build` before `Start`.

| Button | When to use |
|---|---|
| `Start` | Normal startup of the selected services. Most common. |
| `Stop` | Pause services but keep the container; `Start` directly next time. |
| `Restart` | Service is stuck, or you changed Python/config files mounted into the container. |
| `Build` | First-time image build, or when image contents changed. |
| `Rebuild` | Dockerfile, apt/pip deps, or a ROS action/msg interface changed, or the build cache is broken (slower). |
| `Remove` / `Down All` | Remove the selected container / tear down the whole compose project. Only for architecture changes or a messy state. |

A typical grasp run (GPU) starts: `kros_car`, `motion_arbiter`, `camera_kinect`, `lidar`, `eto_eye_gpu`, `wildbot_grasp`, `igvi_bridge`, `foxglove_bridge`. For navigation/localization, add `slam_localization` (or `slam_fusion` to map) plus `nav2`, and start the relevant `task_server` services for your mission.

### Robot Page

The left side shows a 2D map (`Map2DView`), the camera image (`ImageView`), and the current pose; the right side has the control tabs:

- **Drive** — keyboard/button teleop (publishes `/motion/operator_cmd`).
- **Arm** — arm and gripper pose control, joint temperatures.
- **Navigation** — set goal / initial pose, clear costmap, view nav status.
- **Waypoints** — save / delete / go to named waypoints.
- **Search & Retrieve** — start the search-and-retrieve mission.
- **Bridge Mission** — start the bridge-retrieve / bridge-traverse missions.

### Door Page

Dedicated to the door task: vision alignment to the red push-bar, approach, press, and push — optionally chained with navigation to a door waypoint (door mission).

### Settings Page

Reads/writes the calibration and tuning parameters under `robot_ws/configs/` (camera extrinsics, IMU bias, EKF, Kinect image params, face point, wheel geometry, task speeds). After saving, `Restart` the relevant service to apply.

## Missions

All high-level missions are ROS 2 **action servers**, wrapped as HTTP by `igvi_bridge` and driven from the UI. The `task_server` family shares one `wildbot_grasp:latest` image and inherits from `retrieve_base.py` (shared navigation, visual servoing, and arm-control infrastructure).

| Mission | Service / node | Action interface | Behavior summary |
|---|---|---|---|
| Grasp | `wildbot_grasp` → `grab_object_server` | `GrabObject` `/grab_object` | Move to grasp pose, open/close gripper, verify the catch, move to carry pose |
| Bear grasp (visual servo) | `wildbot_grasp` → `bear_grasp_task_node` | (reads `/detections_json`, then calls `/grab_object`) | Approach + center on the YOLO bbox + depth, grasp once in range |
| Search & Retrieve | `search_retrieve_server` | `SearchAndRetrieve` `/search_retrieve` | Find `target_id` in semantic memory → Nav2 approach → visual servo → grasp → return home |
| Arena mission | `arena_mission_server` | (patrols and calls `/search_retrieve`) | Patrol waypoints from `/maps/waypoints.yaml`, retrieve each, exclusion radius/blacklist to avoid repeats |
| Bridge retrieve | `bridge_retrieve_server` | `BridgeRetrieve` `/bridge_retrieve` | Navigate to a bridge-center waypoint → find `xiong_qiao` (semantic memory or in-place scan) → grasp → return over the bridge via the return path to home → release |
| Bridge traverse | `bridge_traverse_server` | `BridgeTraverse` `/bridge_traverse` | Go to bridge center → return via the return path to home (no grasp, no arm motion) |
| Open door | `open_door_server` | `OpenDoor` `/open_door` | Vision FSM: align to red push-bar → approach → press the handle → push the door |

### Grasp Stack (`wildbot_grasp` service)

Inside the container, `grasp_stack` launches four nodes at once:

```text
grab_object_server      # /grab_object action server
arm_safeguard_node      # safety proxy for high-level arm commands + post-motion relax to cool down
camera_masker_node      # mask the arm out of RGB/depth images when not at home pose
bear_grasp_task_node    # reads /detections_json, approaches + centers on X, sends /grab_object in range
```

Bear-grasp flow (brief; full version in `robot_ws/src/wildbot_grasp/README.md`):

```text
Kinect + YOLO detects a bear
-> bear_grasp_task_node reads bbox.center_x and depth_m
-> distance_m > threshold: approach via /motion/cmd with X-axis centering assist
-> distance_m <= threshold: stop and immediately call /grab_object
-> arm_safeguard_node forwards the arm trajectory; after catching, move to carry pose and hold
-> on a failed grasp, back up briefly, re-approach, and retry
```

Centering calibration notes: the Kinect RGB is `1280x720`, so the image center is ≈ 640, but the gripper centerline is offset laterally from the camera, so the aim point is shifted right. The relevant params are `BEAR_TARGET_CENTER_X_PX` / `RETRIEVE_IMAGE_CENTER_X` (default ≈ 690–710) and the distance gate `BEAR_TARGET_MAX_DISTANCE_M` (default 0.21). After editing `.env`, just `Restart` the relevant service — no full restart needed.

### Starting a Mission

The usual path is the UI's Robot / Door tabs; under the hood these go through the host API (which forwards to igvi_bridge):

```bash
# Search & Retrieve
curl -X POST localhost:8770/api/ros/search_retrieve/start \
  -H 'content-type: application/json' \
  -d '{"target_id":"xiong_1","home_pose_x":0,"home_pose_y":0,"home_pose_yaw":0}'

# Open door
curl -X POST localhost:8770/api/ros/open_door/start \
  -H 'content-type: application/json' -d '{"ready_distance_m":0.5}'

# Arena mission / bridge retrieve / bridge traverse
curl -X POST localhost:8770/api/ros/arena_mission/start  -d '{}'
curl -X POST localhost:8770/api/ros/bridge_retrieve/start -d '{...}'
curl -X POST localhost:8770/api/ros/bridge_traverse/start -d '{...}'
```

The bear task node can also be driven directly via a ROS service (inside the `wildbot_grasp` container):

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec wildbot_grasp bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 service call /bear_grasp_task_node/start std_srvs/srv/Trigger '{}'"
```

## Sensing, Localization & Navigation Pipeline

```text
IMU bias calibration (igvi_imu) ─→ /imu/calibrated
   └→ Madgwick filter            ─→ /imu/filtered
EKF (robot_localization)         ─→ /odometry/filtered   (fuses wheel odometry + IMU yaw)
RGBD odometry (rtabmap_odom)     ─→ visual odometry
RTAB-Map (slam_fusion/localization) ─→ map building/localization, publishes map→odom TF
Nav2 planner ─→ /plan ─→ motion_arbiter ─→ /cmd_vel
```

- **TF tree**: `map → odom (RTAB-Map) → base_link (EKF) → camera_base (static, calibrated extrinsic)`.
- **Nav2 is planning-only**: it only computes paths; the actual drive is executed by `motion_arbiter` via pure pursuit, which is the sole authority over `/cmd_vel`.
- **motion_arbiter state machine**: `IDLE / PATH_TRACKING / ALIGNING / OVERRIDE / MANUAL / ESTOP`. Manual commands (`/motion/cmd`, `/motion/operator_cmd`) temporarily override path tracking; `/estop` is a latched emergency stop that zeroes the output.
- **Mapping vs localization**: `slam_fusion` wipes the DB on start to build a fresh map; `slam_localization` localizes against `${IGVI_DATA_ROOT}/slam/arena_map.db`. Maps can be saved from the UI or via `/api/ros/map/save`.

## Perception: YOLO Detection & Semantic Memory

The `eto_eye` image launches `eto_eye.launch.py`, which runs two nodes together:

- **detector_node** — reads `/rgb/image_raw` (+ aligned depth), runs the ONNX YOLO model, and publishes:
  - `/detections_json` — includes `image_width/height` and per-detection `class_name / score / bbox.center_x/center_y / depth_m`
  - `/eto_eye/annotated_image/compressed` — overlay image (for Foxglove / UI)
  - It keeps only target classes via `CLASS_FILTER` (default `"4,6"`, i.e. the competition targets `xiong` / `xiong_qiao`).
- **semantic_memory_node** — projects detections into the `map` frame, deduplicates (`merge_radius_m`), applies hit-count thresholds (`publish_min_hits` / `xiong_memory_min_hits`), maintains a semantic memory for the search/retrieve missions to query, and publishes markers.

The model lives on the host at `${IGVI_DATA_ROOT}/models/eto_eye/best.onnx` (seen as `/models/best.onnx` in the container). The GPU path uses ROCm 7.2.x + the MIGraphX EP; the compiled model cache is stored under `${IGVI_DATA_ROOT}/migraphx_cache/eto_eye`.

There are also two image pre-processing nodes (in the grasp stack / camera flow): `camera_masker_node` (masks the arm out of the image when it leaves home pose, so it isn't mis-detected into semantic memory) and `camera_router_node` (switches between raw / filtered images for SLAM).

## Host Agent API

`igvi_host` (FastAPI, default `http://127.0.0.1:8770`) is the single entry point for the UI and external tools. Endpoint groups:

- **Health/settings**: `/api/health`, `/api/settings`, `/api/calibration`, `/api/task_speeds`
- **Docker/Compose**: `/api/compose/registry`, `/api/compose/services`, `/api/compose/actions/{build,rebuild,start,stop,restart,remove,build_start,stop_all,remove_all}`, `/api/compose/progress`, `.../logs`
- **ROS core**: `/api/ros/{connection,cmd_vel,stop,estop,goal_pose,initial_pose,map,costmap,pose,plan,approach_pose}`, `/api/ros/image/{topics,frame}`, `/api/ros/params/set`
- **Arm/sensing**: `/api/ros/arm/{temperatures,trajectory}`, `/api/ros/imu/calibration[/start]`, `/api/host/battery`
- **Navigation/waypoints**: `/api/ros/nav/{goal,cancel,status}`, `/api/ros/waypoints[/save,/delete,/goto]`, `/api/ros/costmap/clear`, `/api/ros/map/save`
- **Missions**: `/api/ros/{search_retrieve,bridge_retrieve,bridge_traverse,arena_mission,open_door,door_mission}/{start,cancel,status}`
- **Semantic memory**: `/api/ros/semantic_memory[/clear]`
- **UI bridge**: `/api/ui-bridge/health`

`igvi_bridge` itself also serves a parallel `/api/*` on `8771` (the host agent forwards to it).

## Configuration & Environment Variables

Key settings under `robot_ws/configs/` (most are readable/writable from the UI Settings page):

| File | Contents |
|---|---|
| `calibration.yaml` | Camera extrinsics, IMU gyro bias, EKF frequency, Kinect image params, face point |
| `controllers.yaml` | base_controller (wheel separation/radius) and arm controller |
| `ekf_*.yaml` | EKF fusion params (wheel+imu / fusion / no-imu variants) |
| `nav2_params.yaml` | Planning-only Nav2 params |
| `task_speeds.yaml` | Per-task linear/angular speed and acceleration limits (search/bridge retrieve, bridge traverse, open door, igvi_bridge door transit) |
| `target_filter.yaml` | Detection confidence threshold, confirm frames, semantic-memory min hits |
| `hardware.yaml` | kros_car hardware (wheel/arm ports, gripper overheat & stall protection) |
| `kros_car.xacro` / `v6_unity.urdf` | Robot URDF |
| `fastdds_shm.xml` | Shared FastDDS profile; must be identical across all containers |

Common variables in `docker/compose/.env` (generated by `make install` from `.env.example`):

- `ROS_DOMAIN_ID`, `IGVI_DATA_ROOT`, `FASTRTPS_DEFAULT_PROFILES_FILE`
- Bear grasp: `BEAR_TARGET_MAX_DISTANCE_M`, `BEAR_TARGET_CENTER_X_PX`, `BEAR_MIN_CONFIDENCE`, `BEAR_TARGET_CONFIRM_FRAMES`, `BEAR_MAX_TASK_RETRIES`
- Visual-servo retrieve: `RETRIEVE_IMAGE_CENTER_X`, `RETRIEVE_*` (standoff / approach / arrival tolerance, etc.)
- Semantic memory: `SEMANTIC_MERGE_RADIUS_M`, `SEMANTIC_XIONG_MEMORY_MIN_HITS`
- Arena / bridge / arm masking / gripper detection: `ARENA_*`, `BRIDGE_*`, `ARM_*`, `GRASP_*`

Shared data under `IGVI_DATA_ROOT`: `slam/` (RTAB-Map DB), `maps/` (maps and waypoints), `models/eto_eye/` (ONNX model), `migraphx_cache/eto_eye/` (GPU compile cache).

## Building the ROS Workspace

Services normally run via Docker; to build manually on the host:

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

`wildbot_grasp` is an `ament_cmake` package (it generates the action interfaces); `igvi_bridge`, `motion_arbiter`, `igvi_imu`, and `eto_eye` are `ament_python`. The compose services mount the Python source back into the container, so pure-Python changes usually take effect with a `Restart`; only interface (action/msg) changes require a `Rebuild`.

## Tests

```bash
cd apps
uv run pytest                              # all tests
uv run pytest tests/test_config.py         # a single file
uv run pytest tests/test_service_registry.py::test_name   # a single test
```

(pytest currently covers `apps`, configured in `apps/pyproject.toml`.)

## Debugging via CLI

Use the UI day-to-day; the following are for debugging only (all take `--env-file docker/compose/.env -f docker/compose/compose.yaml`):

```bash
# Container status
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml ps

# Logs for a service
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml logs --tail=120 wildbot_grasp

# Confirm YOLO is publishing
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec eto_eye_gpu bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /detections_json --once"

# Verify the arm safety path (expect publisher=arm_safeguard_node, subscriber=arm_controller)
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic info -v /arm_controller/joint_trajectory"

# Watch the base nudge commands
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /motion/cmd"
```

The `wildbot_grasp` package also ships a combined CLI: `python3 robot_ws/src/wildbot_grasp/tools/grasp.py {start,stop,check,state,logs,recreate}` (see that package's README).

