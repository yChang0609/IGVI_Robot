# 物件抓取任務 UI MVP Runbook

這份文件是第一階段操作 SOP：不新增自動任務邏輯，先使用現有 UI 完成一次可重複的半手動流程。

目標是把以下流程跑通並記錄參數：

```text
home -> target_1_pregrasp -> 手動近距對準 -> 夾取 -> home -> 放下
```

完成後，下一步才把這些步驟寫進 `object_retrieval_manager`。

## 成功標準

第一階段完成條件：

- UI 可以看到 arena map。
- UI 可以看到穩定 robot pose。
- 可以儲存 `home` waypoint。
- 可以儲存 `target_1_pregrasp` waypoint。
- 可以從 `home` 導航到 `target_1_pregrasp`。
- 可以手動微調底盤靠近物件。
- 可以用 Arm tab 夾起物件。
- 可以從物件附近回到 `home`。
- 回到 `home` 後可以放下物件。

## 需要啟動的服務

在 Docker 頁面啟動以下 profile 或服務：

```text
core
sensors
lidar
localization
navigation
bridge
motion
monitoring
```

如果 UI 裡沒有 `motion` profile，請啟動 `motion_arbiter` service。

最低必要組合：

```text
kros_car
camera_kinect
oradarlidar
lidar_filter
lidar_transform
slam_localization
nav2
igvi_bridge
motion_arbiter
```

## 啟動檢查

在 UI 中確認：

- Docker 頁面中必要服務狀態為 running。
- Robot 頁面中 2D map 有顯示地圖。
- Robot pose 有更新，且不固定在 `(0, 0, 0)`。
- Navigation tab 顯示 Nav server ready。
- Drive tab 可以用低速手動前進/旋轉。
- Arm tab 可以控制手臂與夾爪。

若 pose 明顯錯誤：

1. 到 Navigation tab。
2. 按 `Set Initial Pose (click map)`。
3. 在地圖上點選機器人實際位置。
4. 拖曳設定車頭方向。
5. 等待 localization pose 穩定。

## 建立 Waypoints

### 1. 建立 home

1. 將機器人放在出發點。
2. 確認 UI pose 穩定。
3. 到 Waypoints tab。
4. 按 `Mark current position`。
5. 命名為：

```text
home
```

驗收：

- Waypoint list 中出現 `home`。
- 點選 `home` 後可看到座標。

### 2. 建立 target_1_pregrasp

`target_1_pregrasp` 是物件前方的導航停車點，不是物件本身的位置。

建議距離：

```text
物件前方 0.5 m 到 0.8 m
車頭朝向物件
```

建立方式二選一：

方式 A：手動開車建立

1. 用 Drive tab 慢速開到物件前方。
2. 調整車頭朝向物件。
3. 到 Waypoints tab。
4. 按 `Mark current position`。
5. 命名為：

```text
target_1_pregrasp
```

方式 B：地圖點選建立

1. 到 Waypoints tab。
2. 按 `Pick on map`。
3. 在地圖上點物件前方停車點。
4. 拖曳方向讓車頭朝向物件。
5. 命名為：

```text
target_1_pregrasp
```

驗收：

- Waypoint list 中出現 `target_1_pregrasp`。
- 按 `Go` 可以導航到該位置附近。
- 到達後車頭大致朝向物件。

## 手動 MVP 流程

### Step 1: 從 home 出發

1. 在 Waypoints tab 選擇 `home`。
2. 按 `Go`，確認機器人能回到出發點。
3. 若已在 home，可跳過。

### Step 2: 導航到物件前方

1. 在 Waypoints tab 選擇 `target_1_pregrasp`。
2. 按 `Go`。
3. 到 Navigation tab 觀察狀態。
4. 到達後確認車體沒有碰撞障礙物。

若導航失敗：

- 按 `Clear Costmap`。
- 確認 `/map` 和 pose 正常。
- 降低速度或重新建立 waypoint。

### Step 3: 手動近距對準

切到 Drive tab，使用低速控制。

建議速度：

```text
Linear: 3% 到 8%
Angular: 10% 到 20%
```

操作目標：

- 讓物件位於車體正前方。
- 讓夾爪中心線對準物件中心。
- 保持慢速，不要直接撞上物件。

建議停止距離：

```text
物件距離夾爪前端 5 cm 到 15 cm
```

### Step 4: 手動夾取

切到 Arm tab。

建議先用這個固定流程測試：

```text
Open
arm_1 到預備角度
arm_2 到預備角度
微調底盤前進
Close
抬高手臂
底盤後退 10 cm 到 20 cm
```

請記錄成功的角度：

```text
arm_1_pre:
arm_2_pre:
gripper_open:
gripper_close:
arm_1_lift:
arm_2_lift:
```

驗收：

- 物件被夾起。
- 車體後退後物件沒有掉落。
- 沒有撞到靜態障礙。

### Step 5: 回 home

1. 到 Waypoints tab。
2. 選擇 `home`。
3. 按 `Go`。
4. 到達後確認物件仍在夾爪中。

### Step 6: 放下物件

切到 Arm tab。

建議流程：

```text
降低手臂
Open
後退 10 cm
Home
```

驗收：

- 物件留在 home / 出發點區域。
- 車體沒有碰撞。
- 任務完成一次完整閉環。

## 參數紀錄表

每次測試後填寫。

```text
日期/場次:
地圖檔案: data/slam/arena_map.db

home:
  x:
  y:
  yaw:

target_1_pregrasp:
  x:
  y:
  yaw:
  到物件距離:

Drive:
  linear_slider:
  angular_slider:
  對準耗時:

Arm:
  arm_1_pre:
  arm_2_pre:
  gripper_open:
  gripper_close:
  arm_1_lift:
  arm_2_lift:

結果:
  導航成功: yes/no
  夾取成功: yes/no
  回 home 成功: yes/no
  有無碰撞:
  有無掉落:
  備註:
```

## 常見問題

### UI 有地圖但導航失敗

處理：

- 確認 `navigation` profile 已啟動。
- 按 `Probe Actions`，確認 bridge 看得到 `navigate_to_pose`。
- 按 `Clear Costmap`。
- 確認 robot pose 在地圖上不是錯位。

### Pose 跳動或位置錯

處理：

- 用 `Set Initial Pose` 重新給定位。
- 確認 Kinect、LiDAR、wheel odom 都正常。
- 確認 localization 載入的是正確的 `arena_map.db`。

### 到 pregrasp 後物件不在正前方

處理：

- 重新建立 `target_1_pregrasp`。
- 建立 waypoint 時要讓 yaw 朝向物件。
- pregrasp 不要太近，先停在 0.5 m 到 0.8 m。

### 夾取容易撞物件

處理：

- 降低 Drive tab linear slider。
- pregrasp 停更遠。
- 先打開夾爪再靠近。
- 把最後 10 cm 改成手動短按，不要長按前進。

### 物件回 home 前掉落

處理：

- 增加 gripper close 角度，但不要低於安全範圍。
- 回 home 速度調低。
- 抬高手臂後先原地停 1 秒確認穩定。

## 下一步自動化

當這份 runbook 能穩定成功後，下一步實作：

```text
object_retrieval_manager
```

第一版 manager 只需要自動執行：

```text
Go target_1_pregrasp
等待 Nav2 succeeded
執行固定 pick sequence
Go home
執行固定 place sequence
```

這樣可以先不依賴 perception，把導航、手臂、回 home 的主流程自動化。

## MVP 執行 Script

已提供一個主機端 script，可以透過現有 Host API 直接執行第一版 waypoint 任務：

```bash
python3 tools/object_retrieval_mvp.py --dry-run
```

確認 dry-run 顯示的流程沒問題後，才允許真實移動：

```bash
python3 tools/object_retrieval_mvp.py --yes
```

預設使用：

```text
home waypoint:   home
target waypoint: target_1_pregrasp
host API:        http://127.0.0.1:8770
```

第一次真車測試建議不要讓 script 自動前進靠近物件，保留預設：

```text
--approach-seconds 0.0
```

如果目前只想測「車可以導航到熊前面」，使用 nav-only 模式：

```bash
python3 tools/object_retrieval_mvp.py --dry-run --nav-only
python3 tools/object_retrieval_mvp.py --yes --nav-only
```

這個模式只會：

```text
Go target_1_pregrasp
等待 Nav2 succeeded
Stop
```

不會控制手臂、不會自動靠近、不會後退，也不會回 home。

### 不使用 waypoint：用 YOLO 偵測結果導航到熊前面

如果 YOLO 已經能輸出熊的位置，可以不要建立 `target_1_pregrasp` waypoint。

目前 `eto_eye_gpu` 會發布：

```text
/detections_json [std_msgs/msg/String]
frame_id: rgb_camera_link
```

因此 YOLO 原始輸出是相機 frame，不是 `map` 也不是 `base_link`。專案已加入
`detection_projector` service，會把：

```text
/detections_json
/depth_to_rgb/image_raw
/rgb/camera_info
/tf
```

轉成：

```text
/task/target_candidates [std_msgs/msg/String]
```

輸出內容會包含：

```json
{
  "class_name": "xiong_qiao",
  "confidence": 0.92,
  "x_base": 1.2,
  "y_base": 0.1,
  "x_map": 1.25,
  "y_map": -0.4
}
```

預設只接受以下類別，不會把 `men` 當作目標：

```text
xiong_qiao
bear
```

啟動或重建：

```bash
docker compose -f docker/compose/compose.yaml \
  --project-name igvi_robot \
  --project-directory /home/igvilab/IGVI_Robot/docker/compose \
  --profile bridge up -d --build detection_projector igvi_bridge
```

檢查轉換結果：

```bash
docker exec igvi_robot-detection_projector-1 bash -lc \
'source /opt/ros/jazzy/setup.bash; ros2 topic echo --once /task/target_candidates'
```

也可以透過 Host API 查最新 target：

```bash
curl http://127.0.0.1:8770/api/ros/task/target
```

若 UI host 還沒重啟，新的 Host API 路由可能尚未載入；bridge 端可先直接查：

```bash
curl http://127.0.0.1:8771/api/task/target
```

直接使用最新 YOLO target 導航：

```bash
python3 tools/object_retrieval_mvp.py \
  --dry-run \
  --nav-only \
  --use-latest-target

python3 tools/object_retrieval_mvp.py \
  --yes \
  --nav-only \
  --use-latest-target
```

script 支援兩種座標：

```text
map 座標:      x_map, y_map
base_link 座標: x_base, y_base
```

如果 YOLO 輸出的是 map 座標：

```bash
python3 tools/object_retrieval_mvp.py \
  --dry-run \
  --nav-only \
  --bear-map-x 1.25 \
  --bear-map-y -0.40

python3 tools/object_retrieval_mvp.py \
  --yes \
  --nav-only \
  --bear-map-x 1.25 \
  --bear-map-y -0.40
```

如果 YOLO 輸出的是相對車體的 `base_link` 座標：

```bash
python3 tools/object_retrieval_mvp.py \
  --yes \
  --nav-only \
  --bear-base-x 1.20 \
  --bear-base-y 0.10
```

也可以讀 JSON 檔：

```json
{
  "class_name": "bear",
  "confidence": 0.92,
  "x_map": 1.25,
  "y_map": -0.40
}
```

執行：

```bash
python3 tools/object_retrieval_mvp.py \
  --yes \
  --nav-only \
  --detection-json /tmp/latest_bear_detection.json
```

script 不會導航到熊的中心點，而是會自動算熊前方的停車點：

```text
pregrasp = 熊的位置 - 從機器人指向熊的方向 * standoff
```

預設停在熊前方：

```text
--standoff 0.65
```

可調整：

```bash
python3 tools/object_retrieval_mvp.py \
  --yes \
  --nav-only \
  --detection-json /tmp/latest_bear_detection.json \
  --standoff 0.80
```

注意：如果 YOLO 只有 bounding box，還不夠直接導航。需要再加 depth + camera TF，轉成 `x_base/y_base` 或 `x_map/y_map` 後才能送給 script。

等 arm 角度與停車位置都穩定後，再逐步增加：

```bash
python3 tools/object_retrieval_mvp.py \
  --yes \
  --approach-seconds 0.5 \
  --approach-linear 0.03
```

手臂角度可以用 Arm tab 調好後填入，例如：

```bash
python3 tools/object_retrieval_mvp.py \
  --yes \
  --arm1-pre 118 \
  --arm2-pre 132 \
  --arm1-lift 140 \
  --arm2-lift 96 \
  --gripper-close 168
```

安全設計：

- 沒有 `--yes` 時不會允許真實移動。
- `--dry-run` 只列印流程，不送任何 robot command。
- 中斷或失敗時會呼叫 `/api/ros/stop`。
- 自動靠近物件預設關閉，避免第一次執行就前撞。
