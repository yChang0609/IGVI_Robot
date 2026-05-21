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

Run the one-time install step on each machine (or worktree, first time only):

```bash
make install
# or, to use a custom shared data path:
make install DATA_ROOT=/your/shared/path
```

This creates `docker/compose/.env` from the example template and ensures the shared data
directories (`slam/`, `maps/`, `models/eto_eye/`, `migraphx_cache/eto_eye/`) exist at `IGVI_DATA_ROOT`. Because `.env` is gitignored,
every worktree on the same machine should run `make install` once — they will all point
to the same absolute path, so robot data is shared regardless of which worktree is running.

Set a unique `ROS_DOMAIN_ID` in `docker/compose/.env` before running on a shared network.

Install Python dependencies:

```bash
make sync
```

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

## ETO Eye Detector

`eto_eye` is integrated as the YOLO detector for the Kinect RGB topic. The CPU and GPU variants are separate profiles so only one detector publishes `/detections_json` at a time.

Place the ONNX model at `${IGVI_DATA_ROOT}/models/eto_eye/best.onnx` on the host. With the default install path, that is `~/igvi_robot/models/eto_eye/best.onnx`; the detector sees it inside the container as `/models/best.onnx`.

```bash
# Kinect + Foxglove
docker compose -f docker/compose/compose.yaml --profile kinect --profile monitoring up -d

# CPU detector
docker compose -f docker/compose/compose.yaml --profile eto_eye_cpu up -d eto_eye_cpu

# AMD GPU detector, ROCm/MIGraphX
docker compose -f docker/compose/compose.yaml --profile eto_eye_gpu up -d eto_eye_gpu
```

Foxglove can connect to `ws://localhost:8765`. Useful topics are `/eto_eye/annotated_image/compressed` and `/detections_json`.

The GPU path expects the ASUS Vivobook AMD setup that was validated with kernel `6.17.0-29-generic`, Ubuntu inbox `amdgpu`, UMA set to 8GB, ROCm `7.2.1`, and `MIGraphXExecutionProvider`. MIGraphX compiled model cache is stored under `${IGVI_DATA_ROOT}/migraphx_cache/eto_eye`.
