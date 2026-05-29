# IGVI Robot

IGVI Robot 是 WildBot/IGVI 機器人的 ROS 2 + Docker Compose + 本機控制 UI 專案。現在主要用 Docker Compose profiles 啟動各子系統；ROS package 在 `robot_ws/src`，本機 host/UI 工具在 `apps`。

## 專案內容

```text
IGVI_Robot_merge_gpu_yolo/
├── apps/                         # 本機工具
│   ├── igvi_host/                # FastAPI host agent，管理 Docker/ROS bridge/API
│   ├── igvi_ui/                  # PySide6 桌面控制中心
│   └── tests/
├── robot_ws/
│   ├── src/
│   │   ├── wildbot_grasp/        # 抓熊/手臂/夾爪任務
│   │   ├── eto_eye/              # YOLO/ONNX 偵測節點
│   │   ├── igvi_bridge/          # ROS <-> UI/HTTP bridge
│   │   ├── motion_arbiter/       # /motion/cmd 與底盤速度仲裁
│   │   └── igvi_imu/             # IMU calibration
│   ├── configs/                  # URDF、controllers、EKF、Nav2、RTAB-Map 設定
│   ├── launch/                   # Kinect、SLAM、Localization、Nav2 launch files
│   └── models/                   # YOLO model，例如 eto_eye/best.onnx
├── docker/
│   ├── compose/                  # compose.yaml 與各 service yaml
│   └── images/                   # Dockerfiles
├── tools/legacy_scripts/         # 舊腳本，保留作過渡用途
└── docs/
```

## 一次性設定

在 repo 根目錄執行一次：

```bash
cd ~/workspace/IGVI_Robot_merge_gpu_yolo
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

`make install` 會建立 `docker/compose/.env`，並設定共享資料路徑 `IGVI_DATA_ROOT`。平常 Docker 都用 UI 開，不需要每次手打 `docker compose`，也不需要每次 restart。

## 用 UI 開 Docker

抓熊流程建議開這些服務。GPU 版：

```text
kros_car
motion_arbiter
camera_kinect
eto_eye_gpu
wildbot_grasp
igvi_bridge
foxglove_bridge
```

CPU 版把 `eto_eye_gpu` 換成：

```text
eto_eye_cpu
```

不要同時啟動 `eto_eye_cpu` 和 `eto_eye_gpu`，避免兩個 detector 同時發布 `/detections_json`。

### 第一次跑

1. 打開 UI，進入 `Docker` 頁。
2. 選取上面列出的服務。
3. 第一次 image 還沒建好時，先按 `Build`。
4. Build 完成後，保持同樣服務被選取，按 `Start`。
5. 服務狀態變成 `running` 後，就可以開始抓熊任務。

### 之後平常跑

1. 打開 UI，在 `Docker` 頁確認服務狀態。
2. 如果服務已經是 `running`，不用再按 `Start`，也不用 `Restart`。
3. 如果服務是 stopped/exited/not_created，選取需要的服務後按 `Start`。

### UI 按鈕怎麼用

| 按鈕 | 什麼時候用 |
|---|---|
| `Start` | 平常啟動選到的服務。最常用。 |
| `Stop` | 暫停選到的服務，但保留 container。下次可直接 `Start`。 |
| `Restart` | 服務卡住，或改了掛載進 container 的 Python/設定檔時才用。不要每次都按。 |
| `Build` | 第一次建 image，或 Docker image 內容有變時用。 |
| `Rebuild` | Dockerfile、apt/pip dependency、ROS action/msg/service 介面有變，或 build cache 壞掉時用。比較慢。 |
| `Down All` | 清掉整個 compose project。只在換架構、清舊 container、狀態混亂時用。平常不要按。 |
| `Up Profile` | 只啟動單一 profile 時用。完整抓熊流程需要多個服務，建議直接選服務後 `Start`。 |

## 抓熊任務流程

目前完整可測流程是：

```text
Kinect + YOLO 看到熊
-> bear_grasp_task_node 讀 bbox.center_x 與 depth_m
-> 靠近時同步用 /motion/cmd 做短距離視覺伺服
-> 距離到達 <= 0.24 m
-> 立即呼叫 /grab_object
-> arm_safeguard_node 轉發手臂 trajectory
-> 抓失敗就後退 0.8 秒，再重新靠近重抓
```

Navigation 尚未真的接 Nav2 goal；已預留 topic interface，後續可接。

### 開始任務

Docker 服務啟動後，抓熊任務本身用 ROS service 開始。這不需要 restart Docker。

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 service call /bear_grasp_task_node/start std_srvs/srv/Trigger '{}'"
```

### 停止任務

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 service call /bear_grasp_task_node/stop std_srvs/srv/Trigger '{}'"
```

### 看任務狀態

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /bear_grasp/state"
```

## 靠近時的置中輔助

YOLO detector 會發布 bbox 中心：

```text
detections[].bbox.center_x
detections[].bbox.center_y
```

目前底盤只做 X 軸置中，所以任務節點主要看：

```text
center_error_px = bbox.center_x - target_center_x_px
```

`bbox.center_x` 是熊在畫面裡的中心。`target_center_x_px` 是你希望熊最後落在哪一條垂直線上。

預設 `target_center_x_px < 0`，程式會自動用影像中心：

```text
target_center_x_px = image_width / 2
```

Kinect RGB 目前輸出是 `1280x720`，camera info 光心約為：

```text
x = 638.6
y = 365.6
```

但這只是影像中心，不一定等於夾爪真正的中心線。依目前 URDF 抓取姿態估算，Kinect RGB 相對夾爪中心線約有 +0.02 m 側向偏差；在 0.22~0.24 m 抓取距離下，約等於 51~56 px，所以目前建議先用 target_center_x_px ≈ 690.0。

校正方式：

1. 把熊放在夾爪正前方、你認為最適合抓的位置。
2. 看 `/detections_json` 裡該熊的 `bbox.center_x`。
3. 把這個值當成 `target_center_x_px`。
4. 之後靠近目標距離時，底盤會同步讓 `bbox.center_x` 靠近這個目標值；距離到達後不再等待置中穩定，會直接抓取。

可選設定方式是在 `docker/compose/.env` 加：

```bash
BEAR_TARGET_CENTER_X_PX=690.0
```

如果實測仍偏右，就再把數值加大；如果偏左，就把數值調小。例如：

```bash
BEAR_TARGET_CENTER_X_PX=670.0
```

改完 `.env` 後，在 UI 裡 `Restart` `wildbot_grasp` 即可，不需要整套 Docker 重啟。

如果 state 一直停在 `aligning` 且 detail 是 `approaching target distance`，代表目前 `depth_m` 還在抓取距離範圍外。目前抓取條件是不再等待置中穩定，距離到達上限就直接抓取；預設上限是：

```text
target_max_distance_m = 0.24
```

也可以在 `docker/compose/.env` 調整：

```bash
BEAR_TARGET_MIN_DISTANCE_M=0.22
BEAR_TARGET_MAX_DISTANCE_M=0.24
```

## 清掉舊容器

如果 UI 裡看到舊版 container，例如 `arm_safeguard`、`bear_grasp_task`、`bear_nav_interface`，代表以前的多 container 架構還殘留。這種情況才清一次：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  --profile core \
  --profile bridge \
  --profile motion \
  --profile grasp \
  --profile grasp_task \
  --profile kinect \
  --profile eto_eye_gpu \
  --profile monitoring \
  down --remove-orphans
```

清完後回到 UI，用 `Start` 開需要的服務。

## CLI 除錯指令

平常 Docker 用 UI 開就好。下面這些只在除錯時使用。

確認 container：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml ps
```

看 `wildbot_grasp` log：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml logs --tail=80 wildbot_grasp
```

確認 YOLO 有輸出：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /detections_json --once"
```

確認 safeguard 接到 arm controller：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic info -v /arm_controller/joint_trajectory"
```

正常應看到：

```text
Publisher count: 1
Node name: arm_safeguard_node
Subscription count: 1
Node name: arm_controller
```

看輪子微調命令：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /motion/cmd geometry_msgs/msg/Twist"
```

## 常用 Profiles

| Profile | 內容 |
|---|---|
| `core` | KROS car bringup，包含底盤與手臂 controller |
| `motion` | `motion_arbiter`，接 `/motion/cmd` 並輸出到底盤 |
| `kinect` | Kinect camera |
| `eto_eye_cpu` | CPU YOLO detector |
| `eto_eye_gpu` | AMD GPU YOLO detector |
| `grasp` | unified `wildbot_grasp` stack |
| `grasp_task` | 同一個 unified `wildbot_grasp` stack，保留任務語意 profile |
| `bridge` | IGVI ROS/UI bridge |
| `monitoring` | Foxglove bridge |
| `slam` | RTAB-Map SLAM |
| `localization` | RTAB-Map localization |
| `navigation` | Nav2 |

注意：目前 `slam` / `localization` profile 依賴的 `imu_calibrator` service 尚未在 compose 裡定義完整；主要抓熊流程先不要啟動這兩個 profile。

## 本機 UI 指令

啟動 host agent 和 UI：

```bash
make run
```

只啟動 host agent：

```bash
make host
```

只開 UI：

```bash
make ui
```

連遠端 host：

```bash
make run 192.168.1.5
```

## ROS Workspace 手動建置

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

## TODO

### 最高優先

- [ ] 實機測定 `backup_linear_x=-0.05`、`backup_duration_sec=0.8` 是否剛好，避免抓失敗後退太多或太少。
- [ ] 確認 YOLO label 實際名稱是否為 `bear`、`teddy bear`、`xiong` 或 `熊`，必要時更新 `target_labels`。
- [ ] 驗證 `0.22~0.24 m` 是否真的是最佳抓取距離區間。

### Navigation 接入

- [ ] 實作 Nav2 adapter：訂閱 `/bear_grasp/nav/request`，收到 `go_to_capture_area` 或 `return_home` 後送 Nav2 goal。
- [ ] Nav2 adapter 抵達後發布 `/bear_grasp/nav/status`。
- [ ] 設定 `home_pose` / `return_goal_pose` 的來源，可由 UI、參數或任務開始時記錄。
- [ ] 將 `enable_navigation` 和 `return_home_after_grasp` 改為實機任務可切換的設定。

### 可靠度與安全

- [ ] 在 UI 顯示 `bear_grasp_task_node` state、retry count、目前距離、中心偏差。
- [ ] 加入任務層級 emergency stop，停止 `/motion/cmd` 並取消抓取 action。
- [ ] 針對夾爪過熱時，在 UI/任務狀態明確提示等待降溫。
- [ ] 收斂 `/motion/cmd` 的靠近與轉向速度參數，避免接近目標時震盪。

### 文件與維護

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
