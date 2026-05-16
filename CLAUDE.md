# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

IGVI Robot is a WildBot competition robot system with SLAM, navigation, perception, and a web-based control UI. Built on ROS 2 Jazzy with Docker Compose orchestration.

## Build & Run Commands

### Python apps (host agent + UI)
```bash
make sync              # Install/sync Python dependencies (uv sync)
make host              # Run igvi-host API server (loopback)
make host lan          # Run igvi-host bound to 0.0.0.0 (LAN-exposed)
make ui                # Run igvi-ui desktop app
make run               # Run igvi-host + igvi-ui together
make run <ip>          # Connect UI to remote host at http://<ip>:8770
```

### Docker Compose services
```bash
# From docker/compose/ directory:
docker compose -f compose.yaml --profile core up -d        # KROS car bringup
docker compose -f compose.yaml --profile sensors up -d      # Kinect, Gemini, LiDAR
docker compose -f compose.yaml --profile slam up -d         # RTAB-Map SLAM
docker compose -f compose.yaml --profile localization up -d # RTAB-Map localization mode
docker compose -f compose.yaml --profile navigation up -d   # Nav2 planning
docker compose -f compose.yaml --profile monitoring up -d   # Foxglove bridge

# Single service:
docker compose -f services/kros_car.yaml --profile core up -d
```

### ROS 2 workspace
```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

### Tests
```bash
cd apps && uv run pytest              # Run all tests
cd apps && uv run pytest tests/test_foo.py       # Single test file
cd apps && uv run pytest tests/test_foo.py::test_bar  # Single test
```

## Architecture

### Data Flow
```
Hardware (Kinect/LiDAR/Motors) → Docker Compose Services (ROS 2 nodes)
    → ROS 2 topic network (FastDDS) → igvi_host (FastAPI) → igvi_ui (PySide6 + VisPy)
```

### Key Directories
- `apps/igvi_host/` — FastAPI backend bridging Docker/Compose/UI control (entry: `server.py`)
- `apps/igvi_ui/` — PySide6 desktop control center (entry: `app.py`)
- `robot_ws/src/` — Custom ROS 2 packages: `igvi_imu`, `igvi_bridge`, `motion_arbiter`
- `robot_ws/launch/` — ROS 2 launch files (slam_fusion, slam_localization, nav2_planning_only)
- `robot_ws/configs/` — All robot/sensor/nav configuration YAML files
- `docker/compose/services/` — Individual service compose files (one per service)
- `docker/compose/compose.yaml` — Main compose entrypoint (includes all service files)

### SLAM/Localization Pipeline
1. **IMU calibration** (`igvi_imu`) → online gyro bias correction → `/imu/calibrated`
2. **Madgwick filter** → orientation estimation → `/imu/filtered`
3. **EKF** (`robot_localization`) → fuses wheel odometry + IMU yaw → `/odometry/filtered` (50 Hz)
4. **RGBD odometry** (`rtabmap_odom`) → visual odometry → `/odom_visual`
5. **RTAB-Map** → builds/localizes on map, publishes `map→odom` TF

### Navigation
- Planning-only Nav2 stack (no controller server) — custom BT in `plan_only_bt.xml`
- `motion_arbiter` executes paths via pure-pursuit + handles manual override

### TF Tree
```
map → odom (RTAB-Map) → base_link (EKF) → camera_base (static, calibrated extrinsic)
```

## Configuration

- **Environment**: `docker/compose/.env` — set unique `ROS_DOMAIN_ID` on shared networks
- **EKF tuning**: `robot_ws/configs/ekf_wheel_imu.yaml`
- **Nav2 params**: `robot_ws/configs/nav2_params.yaml`
- **Camera calibration**: `robot_ws/configs/calibration.yaml` (extrinsics + IMU bias)
- **Hardware params**: `robot_ws/configs/hardware.yaml`
- **Robot URDF**: `robot_ws/configs/kros_car.xacro`

## Tech Stack

- **ROS 2 Jazzy** with FastDDS (Zenoh migration planned — see `.docs/Implement.md`)
- **Python apps**: uv package manager, FastAPI + Uvicorn, PySide6, VisPy, Docker SDK
- **Robotics**: RTAB-Map (SLAM), robot_localization (EKF), Nav2, Azure Kinect driver (built from source)
- **Testing**: pytest (apps only, configured in `apps/pyproject.toml`)

## Planning Documents

- `.docs/Implement.md` — 7-phase implementation roadmap (compose fixes → Zenoh migration → UI bridge → Docker supervisor)
- `.docs/chat_planning.md` — Detailed v3 system architecture design
- `memo.md` — Calibration & tuning backlog (IMU, camera intrinsics, wheel odometry, EKF)
