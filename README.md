# IGVI Robot

WildBot competition robot code and system orchestration.

## Project Structure

```text
IGVI_Robot/
  apps/
    api/                  # Backend API for future UI/system control
    ui/                   # Future frontend UI
  robot_ws/
    src/
      wildbot_bringup/    # ROS launch files and runtime config
      wildbot_control/    # Robot control and task nodes
  docker/
    images/               # Dockerfiles grouped by image purpose
    compose/              # Integrated and per-service Docker Compose files
    scripts/              # Container-side helper scripts
  tools/
    legacy_scripts/       # Transitional host scripts
  docs/
  data/                   # Local runtime data, ignored by Git except .gitkeep
```

## Setup

Create a local compose environment file:

```bash
cp docker/compose/.env.example docker/compose/.env
```

Set a unique `ROS_DOMAIN_ID` in `docker/compose/.env` before running on a shared network.

## Docker Compose Profiles

The integrated compose entrypoint is `docker/compose/compose.yaml`. It includes the independent service compose files from `docker/compose/services/`, and services are grouped with profiles so the future API/UI can control them consistently:

```bash
docker compose -f docker/compose/compose.yaml --profile core up -d
docker compose -f docker/compose/compose.yaml --profile sensors up -d
docker compose -f docker/compose/compose.yaml --profile slam up -d
docker compose -f docker/compose/compose.yaml --profile monitoring up -d
```

Run one service compose directly when you only want that piece:

```bash
docker compose -f docker/compose/services/kros_car.yaml --profile core up -d
docker compose -f docker/compose/services/camera_kinect.yaml --profile kinect up -d
```

Useful profiles:

- `core`: car bringup
- `sensors`: Kinect, Gemini, LiDAR related services
- `kinect`: Kinect camera only
- `gemini`: Gemini camera only
- `lidar`: ORadar LiDAR and filter services
- `slam`: RTAB-Map SLAM
- `localization`: RTAB-Map localization
- `navigation`: Nav2
- `monitoring`: Foxglove bridge
- `bridge`: IGVI HTTP bridge
- `unity`: Unity/simulation helpers
- `debug`: URDF and TF visualization helpers

## ROS Workspace

ROS packages live in `robot_ws/src`.

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

Launch files and robot runtime configuration are intentionally kept in `robot_ws/src/wildbot_bringup`, not under `docker/`, so the ROS behavior is not tied to one container layout.

## Legacy Scripts

The old host scripts have been moved to `tools/legacy_scripts/`. They now call the unified compose profiles, but they are transitional tooling. The long-term path is:

```text
UI -> apps/api -> docker compose services -> ROS launch files
```

## Hardware Notes

Detailed KROS car notes are in `docs/README_kros_car.md`.



<!-- IGVI_Robot/
  apps/
    api/
      pyproject.toml
      uv.lock
      src/igvi_api/
        main.py
    ui/                  
  robot_ws/
    src/
      wildbot_bringup/
        launch/
        config/
        package.xml
        setup.py
      wildbot_control/
        wildbot_control/
        package.xml
        setup.py

  docker/
    images/
      base/Dockerfile
      fusion/Dockerfile
      kinect/Dockerfile
      foxglove/Dockerfile
    compose/
      compose.yaml
      compose.dev.yaml
      .env.example
      services/
        kros_car.yaml
        camera_kinect.yaml

  tools/
    legacy_scripts/      # 原本 scripts/*.sh 過渡期放這裡

  docs/
    README_kros_car.md
    architecture.md

  data/
    .gitkeep
    calibration/         # 若 calibration data 要保留，可放這裡

  README.md
  .gitignore -->
