# wildbot_grasp

`wildbot_grasp` 是 WildBot/IGVI 的抓熊、手臂與夾爪任務 package。現在採用 **單一 Docker container** 架構：compose 只會啟動一個 `wildbot_grasp` service，container 內由 `grasp_stack` 同時啟動 action server、安全守門員、抓熊任務節點與 Navigation interface。

這份 README 只說明抓熊任務本身；不需要 UI，也不會透過 UI 控制車子或夾爪。

## 現況摘要

目前可執行的完整流程是「非導航版抓熊」：

```text
Kinect + YOLO 偵測熊
-> bear_grasp_task_node 讀 /detections_json
-> 靠近時用 /motion/cmd 控底盤做短距離視覺伺服
-> 距離到達 <= 0.24 m
-> 立即呼叫 /grab_object action
-> arm_safeguard_node 轉發給 /arm_controller/joint_trajectory
-> 抓失敗先後退 0.8 秒，再重新靠近重抓
```

Navigation 還沒有實際送 Nav2 goal，但已預留 topic contract：

```text
/bear_grasp/nav/request
/bear_grasp/nav/status
```

## 一行 Python 執行抓熊

先進 repo：

```bash
cd ~/workspace/IGVI_Robot_merge_gpu_yolo
```

啟動必要 container 並開始抓熊任務：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py start
```

`start` 會先把手臂定位到 home `[190.0, 0.0, 240.0]`，再開始抓熊任務。任務開始後會一直等待新鮮的熊偵測；看到熊後會對準、靠近、抓取。抓到熊後會沿用原本邏輯移到 place pose 放熊，放掉後最後才回到 home `[190.0, 0.0, 240.0]`。抓失敗會後退並重試；抓成功一次後會進入 `succeeded`，不會自動找下一隻。要重新抓下一次，先 `stop` 再 `start`。

這一行會啟動抓熊必要服務：

```text
kros_car
motion_arbiter
camera_kinect
eto_eye_gpu
wildbot_grasp
```

工具會帶 `--remove-orphans`，清掉舊版殘留 container，例如舊的 `arm_safeguard`。這很重要，否則 ROS graph 會出現兩個 `/arm_safeguard_node`，手臂 trajectory 可能被重複轉發。

它不會啟動 UI、`igvi_bridge` 或 `foxglove_bridge`。

如果 GPU detector 卡住或你想用 CPU detector：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py --cpu start
```

## 常用指令

改過 `.env`、compose、或需要重建 container 狀態時：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py recreate
```

如果 log 出現 `No executable found`，或 `wildbot_grasp` 一直 restarting，代表 Docker image 裡的 ROS install 還是舊的，沒有裝到新的 `grasp_stack` / bear task 腳本。這時先 rebuild 一次：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py --build recreate
python3 robot_ws/src/wildbot_grasp/tools/grasp.py start --no-up
```

只檢查目前連線與設定，不開始任務：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py check
```

觀察抓熊狀態：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py state --watch
```

看 `wildbot_grasp` 和 `kros_car` log：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py logs --tail 200
```

測試夾爪開合：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py gripper open
python3 robot_ws/src/wildbot_grasp/tools/grasp.py gripper close
```

停止抓熊任務：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py stop
```

如果你正在跑 `state --watch` 或 log 持續輸出，只是要離開畫面，用 `Ctrl+C`。`Ctrl+C` 只會中止目前這個 Python 指令，不等於停止機器人任務；真正停止抓熊請用 `python3 robot_ws/src/wildbot_grasp/tools/grasp.py stop`。

## 單一 Container 架構

`docker/compose/services/wildbot_grasp.yaml` 目前只定義一個 service：

```text
wildbot_grasp
```

`grasp` / `grasp_task` profile 都只會啟動這個 service。container 內執行 `grasp_stack`，同時啟動：

```text
grab_object_server          # /grab_object action server
arm_safeguard_node          # 高階手臂指令安全 proxy
bear_grasp_task_node        # YOLO 對中、距離控制、抓失敗後退重抓
bear_nav_interface_node     # 未來 Navigation topic contract / 測試輔助
```

因此現在不要再找這些舊 container：

```text
arm_safeguard
bear_grasp_task
bear_nav_interface
```

## 啟動後檢查重點

`python3 robot_ws/src/wildbot_grasp/tools/grasp.py check` 會檢查：

```text
/bear_grasp_task_node target_min_distance_m
/bear_grasp_task_node target_max_distance_m
/bear_grasp_task_node target_center_x_px
/arm_joint_temperatures
/controller_manager update_rate
/arm_controller/joint_trajectory publisher/subscriber
```

正常手臂 safety path 應該看到：

```text
Publisher: arm_safeguard_node
Subscriber: arm_controller
```

確認 YOLO JSON 時，重點看 `/detections_json` 是否包含：

```text
image_width
image_height
detections[].class_name
detections[].score
detections[].bbox.center_x
detections[].depth_m
```

如果偵測有熊但沒有 `depth_m`，任務會停在等待距離或對距離狀態，不會進入抓取。bear task 會優先選有有效 `depth_m` 的 `xiong`，避免畫面邊緣高分但無深度的 detection 卡住任務。

## 置中與距離控制

### 距離

抓取距離上限：

```text
target_max_distance_m = 0.24
```

行為：

```text
distance_m > 0.24  -> 往前靠近，同時做 X 軸置中輔助
distance_m <= 0.24 -> 停車並立即送出 /grab_object 抓取
```

### 置中

Kinect RGB 目前解析度是 `1280x720`，實測 camera info 光心約為：

```text
x = 638.6
y = 365.6
```

目前只做 X 軸置中：

```text
center_error_px = bbox.center_x - target_center_x_px
```

`bbox.center_x` 是 YOLO bbox 的中心；`target_center_x_px` 是你希望熊落在畫面中的目標 X 位置。預設 `target_center_x_px < 0` 時會用 `image_width / 2`。

模型估算 Kinect RGB 相機相對抓取姿態的夾爪中心線約有 `+0.02 m` 側向偏差；在 0.22~0.24 m 抓取距離下，約等於 `51~56 px`。因此目前起始校正值是：

```bash
BEAR_TARGET_CENTER_X_PX=690.0
```

改完後執行 `python3 robot_ws/src/wildbot_grasp/tools/grasp.py recreate`，讓 `.env` 重新進入 container。

任務節點只在靠近目標距離時發布 `/motion/cmd` 的 `angular.z` 作為置中輔助；一旦距離到達 `target_max_distance_m`，不再等待置中誤差或穩定時間，會停車並送出抓取 goal。`target_min_distance_m` 保留為舊參數，但目前不會阻擋抓取。

### 抓失敗與重試

重試分成兩層：

```text
/grab_object action 內層：max_grasp_attempts = 2
bear_grasp_task_node 外層：max_task_retries = 0
```

`max_grasp_attempts=2` 表示每一次 `/grab_object` action goal 裡面最多會試兩次夾取。`max_task_retries=0` 表示任務層沒有上限：action 回報失敗後，車子會後退、重新靠近、再送下一次 `/grab_object`。如果未來要限制外層重試次數，可以在 `.env` 設：

```bash
BEAR_MAX_TASK_RETRIES=3
```

每次失敗後會先後退：

```text
backup_linear_x = -0.05
backup_duration_sec = 0.8
```

流程：

```text
grasping
-> backing_up
-> realigning
-> waiting_for_bear
-> aligning
-> grasping
```

## /grab_object Action

`GrabObject.action`：

```text
# Goal
string object_label
float32 distance_m
---
bool success
bool object_grasped
string message
---
string stage
float32 progress
string detail
```

手動測 action：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 action send_goal /grab_object wildbot_grasp/action/GrabObject '{object_label: bear, distance_m: 0.23}' --feedback"
```

## 手臂姿態

角度以 degrees 設定，發布前轉成 radians。

| Pose | arm_1 | arm_2 | gripper | 用途 |
|---|---:|---:|---:|---|
| `grasp_pose_deg` | 167.0 | 75.0 | 170.6 | 原本夾取位置，關爪 |
| `place_pose_deg` | 120.0 | 75.0 | 239.0 | 原本放置位置，開爪/放開 |
| `home_pose_deg` | 190.0 | 0.0 | 240.0 | 啟動初始位置、放熊後最後回來的位置 |

`grasp_pose_deg` 和 `place_pose_deg` 是原本開/關爪核心姿態，不拿 home pose 取代。`grab_object_server` 啟動後會自動送一次 `home_pose_deg`，所以車子/抓取 stack 開起來時，手臂會先定位到 `[190.0, 0.0, 240.0]`。

完整姿態流程：

```python
initial_home       = [190.0, 0.0, 240.0]
open_at_grasp     = [167.0, 75.0, 239.0]
close_at_grasp    = [167.0, 75.0, 170.6]
place_holding     = [120.0, 75.0, 170.6]
release_at_place  = [120.0, 75.0, 239.0]
return_home_final = [190.0, 0.0, 240.0]
```

## Safety

### 高階溫度防護

`grab_object_server` 會讀 `/arm_joint_temperatures`。

```text
gripper_temperature_index = 2
gripper_max_start_temp_c = 68.0
gripper_resume_temp_c = 65.0
```

如果夾爪溫度大於等於 `68.0 C`，action 會 abort。

### 底層 safeguard

所有高階手臂 trajectory 先送：

```text
/arm_safeguard/target_trajectory
```

再由 `arm_safeguard_node` 轉發：

```text
/arm_controller/joint_trajectory
```

動作完成後 safeguard 會根據 controller feedback 發布短時間 relax trajectory，降低馬達持續出力與過熱。

## Navigation 接口

目前 Navigation 還沒真正接 Nav2，但 `bear_nav_interface_node` 已定義 topic contract。

任務節點發布 request：

```text
/bear_grasp/nav/request    std_msgs/String JSON
```

導航端未來回報 status：

```text
/bear_grasp/nav/status     std_msgs/String JSON
```

Request 範例：

```json
{
  "command": "go_to_capture_area",
  "home_frame_id": "map",
  "home_pose": [0.0, 0.0, 0.0]
}
```

回家 request：

```json
{
  "command": "return_home",
  "home_frame_id": "map",
  "home_pose": [0.0, 0.0, 0.0]
}
```

Status 範例：

```json
{
  "capture_area_reached": true,
  "home_reached": false,
  "state": "capture_area_reached"
}
```

手動模擬導航抵達：

```bash
docker compose --env-file docker/compose/.env -f docker/compose/compose.yaml exec wildbot_grasp bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 service call /bear_nav_interface_node/mark_capture_reached std_srvs/srv/Trigger '{}'"
```

## 主要檔案

```text
wildbot_grasp/
├── action/GrabObject.action
├── scripts/
│   ├── grasp_stack
│   ├── grab_object_server
│   ├── gripper_command
│   ├── arm_safeguard_node
│   ├── bear_grasp_task_node
│   └── bear_nav_interface_node
├── tools/
│   └── grasp.py
└── wildbot_grasp_nodes/
    ├── grab_object_server.py
    ├── motion.py
    ├── arm_safeguard_node.py
    ├── bear_grasp_task_node.py
    ├── bear_nav_interface_node.py
    └── gripper_command.py
```

## 已確認問題與處理方式

| 項目 | 處理方式 |
|---|---|
| `wildbot_grasp_nodes/__init__.py` 是空檔案 | 保持不改。這是正常 Python package marker；scripts 直接 import 各 module 的 `main()`，和 `ament_python_install_package(wildbot_grasp_nodes)` 相容。 |
| `max_grasp_attempts` 與任務重試容易混淆 | 已明確分層：`grab_object_server` 內層預設每次 action 試 2 次；`bear_grasp_task_node` 外層用 `max_task_retries` 控制，預設 0 代表無上限。 |
| timer callback 裡有 `time.sleep` | 目前保留，因為 sleep 時間短且 executor 是 multi-threaded；這不是造成「距離到但不夾」的主因。之後若要更乾淨，可改成非阻塞 timer state machine。 |
| Dockerfile CMD 與 compose command 不同 | 已在 `docker/images/Dockerfile.grasp` 註明：direct `docker run` 只啟 action server；正式抓熊一律走 compose，compose 會覆寫成 `grasp_stack`。 |
| scripts / tools / bear task module 還是 untracked | 這些是新功能檔案，測完要 commit 時記得 `git add robot_ws/src/wildbot_grasp/scripts robot_ws/src/wildbot_grasp/tools robot_ws/src/wildbot_grasp/wildbot_grasp_nodes/bear_*`。 |
| `__pycache__` | repo 根目錄 `.gitignore` 已有 `__pycache__/` 和 `*.py[cod]`，不要 commit 進去。 |
| `bear_nav_interface_node` 是 stub | 保持 stub；目前 `enable_navigation=false`，實機抓熊不走 Nav2。它只保留 topic contract 和手動測試 service。 |
| YOLO label | 預設接受 `bear`, `teddy bear`, `xiong`, `熊`；目前實機重點是 `xiong`。模型 class name 改掉時才調 `target_labels`。 |
| 電流/電壓抓取判斷 | 目前 kros_car 還沒有發布可用電流/電壓 feedback，所以 `/grab_object` message 會固定附註這件事；現在主要靠位置誤差與 close motion 判斷是否夾到。 |

## TODO

### 目前最優先

- [ ] 實機確認 `python3 robot_ws/src/wildbot_grasp/tools/grasp.py start` 可完成「找熊 -> 對準 -> 0.22~0.24 m -> 送 `/grab_object` -> open_at_grasp -> close_at_grasp -> place_holding -> release_at_place -> 回到 `[190, 0, 240]`」。
- [ ] 實機確認 `grab_object_server` 啟動後會先把手臂定位到 `[190, 0, 240]`。
- [ ] 確認 `/detections_json` 每次看到 `xiong` 時都有穩定的 `depth_m`；如果只有 bbox 沒有深度，任務不會進入抓取。
- [ ] 確認 ROS graph 裡只有一個 `/arm_safeguard_node`；若有兩個，先用 `python3 robot_ws/src/wildbot_grasp/tools/grasp.py recreate` 清掉 orphan。
- [ ] 確認 `grab_object_server` log 有出現 `accepted grasp goal`、`open_for_grasp`、`close_gripper`、`publishing close_gripper_attempt`。

### 實機校正

- [ ] 實測 `0.22~0.24 m` 是否為最佳抓取距離區間。
- [ ] 實測 `BEAR_TARGET_CENTER_X_PX=690.0` 是否讓夾爪中心線對準熊；偏右就調小，偏左就調大。
- [ ] 實測 `backup_linear_x=-0.05`、`backup_duration_sec=0.8` 是否合適。
- [ ] 確認 YOLO class name 是否固定為 `xiong`，必要時調整 `target_labels`。

### 偵測與硬體穩定性

- [ ] 確認 GPU detector 不會長時間卡在 MIGraphX compile；必要時改用 CPU detector 或建立 GPU cache。
- [ ] 確認 CPU detector 不會因記憶體不足退出；必要時關閉 annotated image 或降低輸出負載。
- [ ] 確認 `/arm_controller/joint_trajectory` 只有 `arm_safeguard_node` 發布，避免多來源搶手臂。
- [ ] 加入更明確的 detection timeout / target lost 狀態與停止策略。

### Navigation

- [ ] 實作 Nav2 adapter，訂閱 `/bear_grasp/nav/request`。
- [ ] Nav2 adapter 抵達任務區後發布 `/bear_grasp/nav/status`。
- [ ] 實作抓取成功後 `return_home` 的 Nav2 回家流程。
- [ ] 決定 `home_pose` 來源：固定參數或任務開始時記錄。

### 安全與可靠度

- [ ] 決定未來是否需要「抓成功後繼續找下一隻熊」的連續模式；目前預設是成功一次就停在 `succeeded`。
- [ ] 加入任務層級 emergency stop。
- [ ] 將 `/motion/cmd` 靠近與轉向速度參數實測調到不震盪。
- [ ] 把實機成功參數回填到本 README。
