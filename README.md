# IGVI Robot

IGVI Robot 是 WildBot / IGVI 競賽機器人的整合系統：以 **ROS 2 Jazzy** 為核心，用 **Docker Compose** 編排各子系統（底盤、感測、SLAM、導航、感知、任務），再透過一個 **FastAPI host agent** + **PySide6 桌面 UI** 做為本機控制中心。

平常操作以 UI 為主：在 UI 的 `Docker` 頁開關服務，在 `Robot` / `Door` 頁執行任務，不需要每次手打 `docker compose`。

## 目錄

- [IGVI Robot](#igvi-robot)
  - [目錄](#目錄)
  - [系統架構](#系統架構)
  - [專案結構](#專案結構)
  - [快速開始](#快速開始)
    - [1. 一次性設定](#1-一次性設定)
    - [2. 安裝 Python 依賴](#2-安裝-python-依賴)
    - [3. 啟動 host + UI](#3-啟動-host--ui)
  - [Docker 服務與 Profiles](#docker-服務與-profiles)
  - [用 UI 操作](#用-ui-操作)
    - [Docker 頁](#docker-頁)
    - [Robot 頁](#robot-頁)
    - [Door 頁](#door-頁)
    - [Settings 頁](#settings-頁)
  - [任務 / Missions](#任務--missions)
    - [grasp stack（`wildbot_grasp` 服務）](#grasp-stackwildbot_grasp-服務)
    - [啟動任務](#啟動任務)
  - [感測、定位與導航 Pipeline](#感測定位與導航-pipeline)
  - [感知：YOLO 偵測與語意記憶](#感知yolo-偵測與語意記憶)
  - [Host Agent API](#host-agent-api)
  - [設定檔與環境變數](#設定檔與環境變數)
  - [ROS Workspace 手動建置](#ros-workspace-手動建置)
  - [測試](#測試)
  - [CLI 除錯指令](#cli-除錯指令)
  - [規劃文件](#規劃文件)
  - [TODO / Roadmap](#todo--roadmap)
    - [任務可靠度與安全](#任務可靠度與安全)
    - [感知與資料](#感知與資料)

## 系統架構

```text
硬體 (Kinect / LiDAR / 輪子 / 手臂)
   │  USB
   ▼
Docker Compose 服務 (ROS 2 nodes)
   │  ROS 2 topic / action 網路 (FastDDS)
   ▼
igvi_bridge  ── ROS ↔ HTTP bridge (port 8771)
   │  HTTP
   ▼
igvi_host    ── FastAPI host agent (port 8770)：同時控制 Docker Engine
   │  HTTP (localhost)
   ▼
igvi_ui      ── PySide6 桌面控制中心
```

分層說明：

- **ROS 2 層**：所有即時運算都在 Docker container 內的 ROS 2 節點，透過共用的 FastDDS profile (`robot_ws/configs/fastdds_shm.xml`) 互通。
- **Bridge 層**：`igvi_bridge` 把地圖、姿態、導航、任務、影像、手臂溫度等包成 HTTP `/api/*`，讓非 ROS 端也能操作。
- **Host 層**：`igvi_host` 是唯一直接碰 Docker Engine API 的元件，並轉發 ROS bridge；UI 只透過 localhost HTTP 與它溝通。
- **UI 層**：`igvi_ui` 提供 Docker 管理、機器人遙控、任務操作、校正設定。

## 專案結構

```text
IGVI_Robot/
├── apps/                          # 本機 Python 工具 (uv 管理)
│   ├── igvi_host/                 # FastAPI host agent (8770)：橋接 Docker + ROS bridge → UI
│   ├── igvi_ui/                   # PySide6 桌面控制中心
│   │   ├── pages/                 # docker / robot / door / settings / capture
│   │   ├── widgets/               # arm, navigation, waypoint, search/bridge retrieve, map, image...
│   │   └── clients/host_client.py # 呼叫 host agent 的 HTTP client
│   ├── tests/                     # pytest (service registry / config)
│   └── main.py
├── robot_ws/
│   ├── src/
│   │   ├── igvi_bridge/           # ROS ↔ HTTP bridge node (8771) + door_mission + open_door_proxy
│   │   ├── motion_arbiter/        # /cmd_vel 唯一輸出者：pure-pursuit + 手動 override + estop
│   │   ├── igvi_imu/              # IMU gyro bias 線上校正
│   │   ├── eto_eye/               # YOLO detector_node + semantic_memory_node
│   │   └── wildbot_grasp/         # 手臂/夾爪 + 所有高階任務 action server
│   ├── configs/                   # calibration / controllers / ekf / nav2 / task_speeds / urdf ...
│   ├── launch/                    # kinect_bringup / slam_fusion / slam_localization / nav2 / lidar
│   ├── scripts/                   # lidar_transformer_node.py 等
│   └── patches/                   # Azure Kinect 驅動 patch
├── docker/
│   ├── images/                    # 各服務 Dockerfile (base / bridge / motion / fusion / grasp / kinect / foxglove / eto_eye_*)
│   └── compose/
│       ├── compose.yaml           # include 所有 services/*.yaml 的進入點
│       ├── .env(.example)         # ROS_DOMAIN_ID / IGVI_DATA_ROOT / 任務參數
│       └── services/              # 每個服務一份 yaml
├── tools/legacy_scripts/          # 舊 bash 腳本 (過渡保留)
├── tuner_output/                  # open_door 姿態調校輸出
├── .docs/ , docs/                 # 規劃文件與硬體文件
├── Makefile
└── CLAUDE.md
```

各 package 另有自己的 README，包含更深入的細節：

- `apps/README.md` — host agent / UI 啟動與 Qt 需求
- `robot_ws/src/wildbot_grasp/README.md` — 抓熊 stack、手臂姿態、距離/置中校正
- `docs/README_kros_car.md` — KROS car 底盤說明

## 快速開始

### 1. 一次性設定

在 repo 根目錄執行一次，建立 `docker/compose/.env` 並建立共享資料目錄：

```bash
make install
# 或自訂共享資料路徑：
make install DATA_ROOT=/your/shared/path
```

這會：

- 從 `.env.example` 產生 `docker/compose/.env`（`.env` 已 gitignore）。
- 把 `IGVI_DATA_ROOT` 寫進 `.env`（預設 `~/igvi_robot`）。
- 建立 `slam/`、`maps/`、`models/eto_eye/`、`migraphx_cache/eto_eye/` 等共享目錄。

同一台機器上每個 worktree 都各跑一次 `make install`，它們會指向同一個絕對路徑，所以地圖/模型資料共享。在共用網路上跑之前，記得在 `.env` 設一個唯一的 `ROS_DOMAIN_ID`。

### 2. 安裝 Python 依賴

```bash
make sync          # 等同 cd apps && uv sync
```

Linux 上 Qt 需要：

```bash
sudo apt update && sudo apt install -y libxcb-cursor0
```

`igvi-ui` 需在圖形桌面 session 下執行（要有 `$DISPLAY` 或 `$WAYLAND_DISPLAY`）。

### 3. 啟動 host + UI

```bash
make run              # 背景啟動 igvi-host，再開 igvi-ui
make host             # 只開 host agent (loopback)
make host lan         # host agent 綁 0.0.0.0，讓 LAN 上其他機器連
make ui               # 只開 UI
make run 192.168.1.5  # UI 連遠端 host (http://192.168.1.5:8770)
```

服務埠：host agent `8770`、igvi_bridge `8771`、Foxglove `8765`。

## Docker 服務與 Profiles

`docker/compose/compose.yaml` 透過 `include` 匯入 `services/*.yaml`，每個服務歸屬一個 **profile bundle**。UI 的 service registry 會自動掃描所有被 include 的服務。

| Profile | 服務 | 用途 |
|---|---|---|
| `robot` | `kros_car`, `camera_kinect`, `motion_arbiter`, `lidar`, `wildbot_grasp` | 底盤+手臂 controller、Azure Kinect、LiDAR、速度仲裁、抓取 stack |
| `application` | `eto_eye_gpu`（或 `eto_eye_cpu`）, `nav2` | YOLO 偵測+語意記憶、planning-only Nav2 |
| `communication` | `igvi_bridge`, `foxglove_bridge` | ROS↔HTTP bridge (8771)、Foxglove WS (8765) |
| `slam_system` | `slam_fusion`, `slam_localization` | RTAB-Map 建圖 / 定位 |
| `task_server` | `search_retrieve_server`, `arena_mission_server`, `bridge_retrieve`, `bridge_traverse`, `open_door_server` | 高階任務 action server（共用 `wildbot_grasp:latest` image） |

各服務角色：

- **kros_car** — KROS car bringup，啟動底盤與手臂 controller（`/dev/usb_wheel`、`/dev/usb_robot_arm`）。
- **camera_kinect** — Azure Kinect 驅動，發布 `/rgb/image_raw`、`/depth_to_rgb/image_raw`、IMU 與 camera info；同時啟動 `igvi_imu` 的 gyro bias 校正。
- **motion_arbiter** — `/cmd_vel` 的唯一輸出者。整合 Nav2 `/plan` 的 pure-pursuit 路徑追蹤、`/motion/cmd` 與 `/motion/operator_cmd` 的手動 override、以及 `/estop` 急停。
- **lidar** — oradar 驅動 + NaN 過濾 + 時間戳重打 → `/scan`（`/dev/oradar`）。
- **wildbot_grasp** — 抓取 stack（見下方任務段）。
- **eto_eye_gpu / eto_eye_cpu** — `detector_node`（ONNX YOLO）+ `semantic_memory_node`。GPU 版走 ROCm / MIGraphX。
- **nav2** — planning-only Nav2（`planner_server` + `bt_navigator` + 客製 plan-only BT，costmap 直接吃 `/map`）；**不含 controller**，路徑由 motion_arbiter 執行。
- **igvi_bridge** — ROS↔HTTP bridge（8771），並內含 door mission 協調器與 open_door proxy。
- **foxglove_bridge** — Foxglove Studio WebSocket。
- **slam_fusion / slam_localization** — RTAB-Map 建圖 / 在既有 `arena_map.db` 上定位。

> 預設 `compose.yaml` 只 include GPU detector。要用 CPU 版，取消註解 `compose.yaml` 裡的 `- services/eto_eye_cpu.yaml`，且**不要同時**啟動 `eto_eye_cpu` 與 `eto_eye_gpu`（兩者都發布 `/detections_json` 會打架）。`services/` 下還有 `camera_gemini`、`unity_*`、`urdf_viz`、`rosbridge` 等非預設服務，需要時再 include。

直接用 CLI 啟動單一 profile（平常用 UI 即可）：

```bash
cd docker/compose
docker compose -f compose.yaml --profile robot up -d
docker compose -f compose.yaml --profile slam_system up -d
docker compose -f compose.yaml --profile application up -d
```

## 用 UI 操作

UI 左側分頁：**Docker**、**Robot**、**Door**、**Settings**。頂部狀態列顯示 Sensors / Host Agent / Docker / Compose / UI Bridge / Battery 與一顆 **EMERGENCY STOP**。

### Docker 頁

選取服務後用按鈕操作；第一次要先 `Build` 再 `Start`。

| 按鈕 | 什麼時候用 |
|---|---|
| `Start` | 平常啟動選到的服務。最常用。 |
| `Stop` | 暫停服務但保留 container，下次直接 `Start`。 |
| `Restart` | 服務卡住，或改了掛載進 container 的 Python/設定檔時才用。 |
| `Build` | 第一次建 image，或 image 內容有變時用。 |
| `Rebuild` | Dockerfile、apt/pip 依賴、ROS action/msg 介面有變，或 build cache 壞掉時用（較慢）。 |
| `Remove` / `Down All` | 移除選定 container / 清掉整個 compose project。只在換架構或狀態混亂時用。 |

典型抓熊流程要開的服務（GPU）：`kros_car`、`motion_arbiter`、`camera_kinect`、`lidar`、`eto_eye_gpu`、`wildbot_grasp`、`igvi_bridge`、`foxglove_bridge`。需要導航/定位時再加 `slam_localization`（或 `slam_fusion` 建圖）與 `nav2`，並依任務啟動對應 `task_server` 服務。

### Robot 頁

左側顯示 2D 地圖（`Map2DView`）與相機影像（`ImageView`）與目前 pose；右側為控制分頁：

- **Drive** — 鍵盤/按鈕遙控（送 `/motion/operator_cmd`）。
- **Arm** — 手臂與夾爪姿態控制、關節溫度。
- **Navigation** — 設定 goal / initial pose、清 costmap、看 nav 狀態。
- **Waypoints** — 儲存 / 刪除 / 前往命名 waypoint。
- **Search & Retrieve** — 啟動搜尋取回任務。
- **Bridge Mission** — 啟動過橋取回 / 過橋任務。

### Door 頁

開門任務專用：紅色推桿視覺對位、approach、按壓、推門，可串接導航到門口 waypoint（door mission）。

### Settings 頁

讀寫 `robot_ws/configs/` 下的校正與調校參數（相機外參、IMU bias、EKF、Kinect 影像參數、face point、輪子幾何、task speeds）。存檔後對應服務 `Restart` 即生效。

## 任務 / Missions

所有高階任務都是 ROS 2 **action server**，由 `igvi_bridge` 包成 HTTP，再由 UI 操作。`task_server` 系列共用同一個 `wildbot_grasp:latest` image，並繼承 `retrieve_base.py`（共用導航、視覺伺服、手臂控制基礎建設）。

| 任務 | 服務 / 節點 | Action 介面 | 行為摘要 |
|---|---|---|---|
| 抓取 | `wildbot_grasp` → `grab_object_server` | `GrabObject` `/grab_object` | 移到 grasp pose、開/關爪、確認是否夾到、移到 carry pose |
| 抓熊（視覺伺服） | `wildbot_grasp` → `bear_grasp_task_node` | （讀 `/detections_json` 後呼叫 `/grab_object`） | 直接看 YOLO bbox + depth 靠近、置中、到距離後抓 |
| 搜尋取回 | `search_retrieve_server` | `SearchAndRetrieve` `/search_retrieve` | 從語意記憶找 `target_id` → Nav2 靠近 → 視覺伺服 → 抓 → 回 home |
| 競技場任務 | `arena_mission_server` | （巡邏並呼叫 `/search_retrieve`） | 巡邏 `/maps/waypoints.yaml` 各點、逐一取回、排除半徑/黑名單避免重複 |
| 過橋取回 | `bridge_retrieve_server` | `BridgeRetrieve` `/bridge_retrieve` | 導航到橋中心 waypoint → 找 `xiong_qiao`（語意記憶或原地掃描）→ 抓 → 沿 return path 過橋回 home → 放 |
| 過橋通過 | `bridge_traverse_server` | `BridgeTraverse` `/bridge_traverse` | 到橋中心 → 沿 return path 回 home（不抓、不動手臂） |
| 開門 | `open_door_server` | `OpenDoor` `/open_door` | 視覺 FSM：對位紅色推桿 → approach → 按壓門把 → 推門 |

### grasp stack（`wildbot_grasp` 服務）

container 內由 `grasp_stack` 同時啟動四個節點：

```text
grab_object_server      # /grab_object action server
arm_safeguard_node      # 高階手臂指令安全 proxy + 動作後 relax 降溫
camera_masker_node      # 手臂不在 home pose 時，把手臂從 RGB/depth 影像遮掉
bear_grasp_task_node    # 讀 /detections_json，靠近 + X 軸置中 + 到距離後送 /grab_object
```

抓熊流程（簡述，完整版見 `robot_ws/src/wildbot_grasp/README.md`）：

```text
Kinect + YOLO 偵測到熊
-> bear_grasp_task_node 讀 bbox.center_x 與 depth_m
-> distance_m > 門檻：用 /motion/cmd 靠近，同步做 X 軸置中輔助
-> distance_m <= 門檻：停車並立即呼叫 /grab_object
-> arm_safeguard_node 轉發手臂 trajectory，夾到後移到 carry pose 抱著
-> 抓失敗就後退一小段，重新靠近重抓
```

置中校正重點：Kinect RGB 為 `1280x720`，影像中心 ≈ 640，但夾爪中心線相對相機有側向偏移，所以瞄準點要往右移。對應參數 `BEAR_TARGET_CENTER_X_PX` / `RETRIEVE_IMAGE_CENTER_X`（預設約 690~710），距離門檻 `BEAR_TARGET_MAX_DISTANCE_M`（預設 0.21）。改 `.env` 後 `Restart` 對應服務即可，不需整套重啟。

### 啟動任務

慣用路徑是 UI 的 Robot / Door 分頁；底層走 host API（再轉給 igvi_bridge）：

```bash
# 搜尋取回
curl -X POST localhost:8770/api/ros/search_retrieve/start \
  -H 'content-type: application/json' \
  -d '{"target_id":"xiong_1","home_pose_x":0,"home_pose_y":0,"home_pose_yaw":0}'

# 開門
curl -X POST localhost:8770/api/ros/open_door/start \
  -H 'content-type: application/json' -d '{"ready_distance_m":0.5}'

# 競技場任務 / 過橋任務 / 過橋通過
curl -X POST localhost:8770/api/ros/arena_mission/start  -d '{}'
curl -X POST localhost:8770/api/ros/bridge_retrieve/start -d '{...}'
curl -X POST localhost:8770/api/ros/bridge_traverse/start -d '{...}'
```

抓熊 task node 也可直接用 ROS service（在 `wildbot_grasp` container 內）：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec wildbot_grasp bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 service call /bear_grasp_task_node/start std_srvs/srv/Trigger '{}'"
```

## 感測、定位與導航 Pipeline

```text
IMU bias 校正 (igvi_imu)  ─→ /imu/calibrated
   └→ Madgwick filter      ─→ /imu/filtered
EKF (robot_localization)   ─→ /odometry/filtered   (融合輪速里程 + IMU yaw)
RGBD odometry (rtabmap_odom)─→ 視覺里程
RTAB-Map (slam_fusion/localization) ─→ 建圖/定位、發布 map→odom TF
Nav2 planner ─→ /plan ─→ motion_arbiter ─→ /cmd_vel
```

- **TF 樹**：`map → odom (RTAB-Map) → base_link (EKF) → camera_base (static, 外參校正)`。
- **Nav2** 為 planning-only：只算路徑，實際驅動由 `motion_arbiter` 以 pure-pursuit 執行，是 `/cmd_vel` 的唯一權威。
- **motion_arbiter 狀態機**：`IDLE / PATH_TRACKING / ALIGNING / OVERRIDE / MANUAL / ESTOP`。手動命令（`/motion/cmd`、`/motion/operator_cmd`）在路徑追蹤中會暫時 override；`/estop` 為 latched 急停，輸出歸零。
- **建圖 vs 定位**：`slam_fusion` 開機清空 DB 重新建圖；`slam_localization` 在 `${IGVI_DATA_ROOT}/slam/arena_map.db` 上定位。地圖可由 UI 或 `/api/ros/map/save` 存檔。

## 感知：YOLO 偵測與語意記憶

`eto_eye` 服務的 image 啟動 `eto_eye.launch.py`，同時跑兩個節點：

- **detector_node** — 讀 `/rgb/image_raw`（+ 對齊深度），跑 ONNX YOLO，發布：
  - `/detections_json` — 含 `image_width/height`、每個 `detections[].class_name / score / bbox.center_x/center_y / depth_m`
  - `/eto_eye/annotated_image/compressed` — 疊框影像（給 Foxglove / UI 看）
  - 以 `CLASS_FILTER`（預設 `"4,6"`，即競賽目標類別 `xiong` / `xiong_qiao`）只保留目標類別。
- **semantic_memory_node** — 把偵測投影到 `map` frame，做去重合併（`merge_radius_m`）、命中次數門檻（`publish_min_hits` / `xiong_memory_min_hits`），維護語意記憶供搜尋/取回任務查詢，並發布 marker。

模型放在 host 的 `${IGVI_DATA_ROOT}/models/eto_eye/best.onnx`（container 內為 `/models/best.onnx`）。GPU 路徑使用 ROCm 7.2.x + MIGraphX EP，編譯後的模型快取在 `${IGVI_DATA_ROOT}/migraphx_cache/eto_eye`。

另有兩個影像前處理節點（在 grasp stack / camera 流程中）：`camera_masker_node`（手臂離開 home pose 時把手臂從影像遮掉，避免誤偵測進語意記憶）與 `camera_router_node`（在 raw / filtered 影像間切換給 SLAM）。

## Host Agent API

`igvi_host`（FastAPI，預設 `http://127.0.0.1:8770`）是 UI 與外部工具的單一入口，主要端點群組：

- **健康/設定**：`/api/health`、`/api/settings`、`/api/calibration`、`/api/task_speeds`
- **Docker/Compose**：`/api/compose/registry`、`/api/compose/services`、`/api/compose/actions/{build,rebuild,start,stop,restart,remove,build_start,stop_all,remove_all}`、`/api/compose/progress`、`.../logs`
- **ROS 基礎**：`/api/ros/{connection,cmd_vel,stop,estop,goal_pose,initial_pose,map,costmap,pose,plan,approach_pose}`、`/api/ros/image/{topics,frame}`、`/api/ros/params/set`
- **手臂/感測**：`/api/ros/arm/{temperatures,trajectory}`、`/api/ros/imu/calibration[/start]`、`/api/host/battery`
- **導航/航點**：`/api/ros/nav/{goal,cancel,status}`、`/api/ros/waypoints[/save,/delete,/goto]`、`/api/ros/costmap/clear`、`/api/ros/map/save`
- **任務**：`/api/ros/{search_retrieve,bridge_retrieve,bridge_traverse,arena_mission,open_door,door_mission}/{start,cancel,status}`
- **語意記憶**：`/api/ros/semantic_memory[/clear]`
- **UI bridge**：`/api/ui-bridge/health`

`igvi_bridge` 自身也在 `8771` 提供平行的 `/api/*`（host agent 即轉發到此）。

## 設定檔與環境變數

`robot_ws/configs/` 內的主要設定（多數可由 UI Settings 頁讀寫）：

| 檔案 | 內容 |
|---|---|
| `calibration.yaml` | 相機外參、IMU gyro bias、EKF 頻率、Kinect 影像參數、face point |
| `controllers.yaml` | base_controller（輪距/輪徑）與手臂 controller |
| `ekf_*.yaml` | EKF 融合參數（wheel+imu / fusion / no-imu 變體） |
| `nav2_params.yaml` | planning-only Nav2 參數 |
| `task_speeds.yaml` | 各任務的線/角速度與加速度上限（search/bridge retrieve、bridge traverse、open door、igvi_bridge 過門） |
| `target_filter.yaml` | 偵測信心門檻、確認幀數、語意記憶最少命中次數 |
| `hardware.yaml` | kros_car 硬體（輪/手臂 port、夾爪過熱與失速保護） |
| `kros_car.xacro` / `v6_unity.urdf` | 機器人 URDF |
| `fastdds_shm.xml` | 共用 FastDDS profile，所有 container 必須一致 |

`docker/compose/.env`（由 `make install` 從 `.env.example` 產生）常用變數：

- `ROS_DOMAIN_ID`、`IGVI_DATA_ROOT`、`FASTRTPS_DEFAULT_PROFILES_FILE`
- 抓熊：`BEAR_TARGET_MAX_DISTANCE_M`、`BEAR_TARGET_CENTER_X_PX`、`BEAR_MIN_CONFIDENCE`、`BEAR_TARGET_CONFIRM_FRAMES`、`BEAR_MAX_TASK_RETRIES`
- 視覺伺服取回：`RETRIEVE_IMAGE_CENTER_X`、`RETRIEVE_*`（standoff / approach / arrival tolerance 等）
- 語意記憶：`SEMANTIC_MERGE_RADIUS_M`、`SEMANTIC_XIONG_MEMORY_MIN_HITS`
- 競技場 / 過橋 / 手臂遮罩 / 夾爪偵測：`ARENA_*`、`BRIDGE_*`、`ARM_*`、`GRASP_*`

`IGVI_DATA_ROOT` 下的共享資料：`slam/`（RTAB-Map DB）、`maps/`（地圖與 waypoints）、`models/eto_eye/`（ONNX 模型）、`migraphx_cache/eto_eye/`（GPU 編譯快取）。

## ROS Workspace 手動建置

平常服務用 Docker 跑；若要在 host 上手動 build：

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

`wildbot_grasp` 為 `ament_cmake` package（產生 action 介面）；`igvi_bridge`、`motion_arbiter`、`igvi_imu`、`eto_eye` 為 `ament_python`。compose 服務會把 Python 原始碼掛回 container，所以純 Python 改動多半 `Restart` 即可生效，介面（action/msg）有變才需 `Rebuild`。

## 測試

```bash
cd apps
uv run pytest                              # 全部測試
uv run pytest tests/test_config.py         # 單一檔案
uv run pytest tests/test_service_registry.py::test_name   # 單一測試
```

（pytest 目前涵蓋 `apps`，設定在 `apps/pyproject.toml`。）

## CLI 除錯指令

平常用 UI；以下只在除錯時使用（都帶 `--env-file docker/compose/.env -f docker/compose/compose.yaml`）：

```bash
# 看 container 狀態
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml ps

# 看某服務 log
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml logs --tail=120 wildbot_grasp

# 確認 YOLO 有輸出
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec eto_eye_gpu bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /detections_json --once"

# 確認手臂 safety path（應看到 publisher=arm_safeguard_node、subscriber=arm_controller）
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic info -v /arm_controller/joint_trajectory"

# 看底盤微調命令
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml \
  exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /motion/cmd"
```

`wildbot_grasp` package 另提供一支整合 CLI：`python3 robot_ws/src/wildbot_grasp/tools/grasp.py {start,stop,check,state,logs,recreate}`（詳見該 package README）。

## 規劃文件

- `.docs/Implement.md` — 多階段落地計畫（compose 修正 → Zenoh 遷移 → UI mmap bridge → Docker supervisor）。部分內容早於目前 repo 狀態，當作方向參考。
- `.docs/chat_planning.md` — v3 系統架構設計。
- `memo.md` — 校正與調校 backlog（IMU、相機內參、輪速里程、EKF）。

## TODO / Roadmap

### 任務可靠度與安全

- [ ] 在 UI 顯示各任務 state、retry 次數、目前距離與中心偏差。
- [ ] 收斂各任務 `/motion/cmd` 的靠近/轉向速度，避免接近目標時震盪。
- [ ] 夾爪過熱時，在 UI/任務狀態明確提示等待降溫。
- [ ] 驗證抓取距離區間與 `BEAR_TARGET_CENTER_X_PX` 是否讓夾爪中心線對準目標。

### 感知與資料

- [ ] 確認 YOLO 類別名稱（`xiong` / `xiong_qiao`）與 `CLASS_FILTER` 一致。
- [ ] 調整語意記憶去重半徑與命中門檻，平衡漏記與誤記。