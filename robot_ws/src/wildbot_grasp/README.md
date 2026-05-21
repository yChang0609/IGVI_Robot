# wildbot_grasp

`wildbot_grasp` 是 WildBot/IGVI 的抓熊、手臂與夾爪任務 package。現在採用單一 Docker service：`wildbot_grasp` container 內由 `grasp_stack` 同時啟動 action server、安全守門員與抓熊任務節點。

這份 package 目前不包含 Navigation/Nav2 adapter。等之後要接 Nav2 時，再新增正式節點即可。

## 現況流程

```text
Kinect + YOLO 偵測熊
-> bear_grasp_task_node 讀 /detections_json
-> 距離 > 0.24 m 時用 /motion/cmd 靠近，並同步做 X 軸置中輔助
-> 距離 <= 0.24 m 時停車
-> 立即呼叫 /grab_object action
-> grab_object_server 執行開爪、關爪、檢查
-> 夾到後移到 carry pose `[190.0, 0.0, 170.6]`，保持夾爪關閉等待導航
-> 抓失敗時後退 0.8 秒，再重新靠近重抓
```

成功抓取一次後，任務會停在 `succeeded`，手臂保持 carry pose 並夾著熊，不會自動找下一隻。要再抓一次，先 `stop` 再 `start`。

## 一行啟動

在 repo 根目錄：

```bash
cd ~/workspace/IGVI_Robot_merge_gpu_yolo
python3 robot_ws/src/wildbot_grasp/tools/grasp.py start
```

預設會啟動：

```text
kros_car
motion_arbiter
camera_kinect
eto_eye_gpu
wildbot_grasp
```

如果 GPU detector 不適合當下環境，可改 CPU：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py --cpu start
```

## 常用指令

只檢查目前連線與參數，不開始任務：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py check
```

觀察任務狀態：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py state --watch
```

看 `wildbot_grasp` 與 `kros_car` log：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py logs --tail 200
```

停止抓熊任務：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py stop
```

手動測 carry pose：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py gripper carry
```

改過 `.env`、compose、或需要清掉舊 container 狀態時：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py recreate
```

如果 image 裡的 ROS install 太舊，出現 `No executable found` 或 `wildbot_grasp` 一直 restarting，先 rebuild：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py --build recreate
python3 robot_ws/src/wildbot_grasp/tools/grasp.py start --no-up
```

## Container 內容

`docker/compose/services/wildbot_grasp.yaml` 只定義一個 service：

```text
wildbot_grasp
```

container 內執行 `grasp_stack`，目前啟動三個節點：

```text
grab_object_server      # /grab_object action server
arm_safeguard_node      # 高階手臂指令安全 proxy
bear_grasp_task_node    # YOLO 目標選擇、靠近、到距離後送抓取 goal
```

舊版或未來用途的 `bear_nav_interface_node` 已移除。目前實機抓熊不走 Nav2。

## 啟動後檢查重點

`python3 robot_ws/src/wildbot_grasp/tools/grasp.py check` 會檢查：

```text
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

如果偵測有熊但沒有有效 `depth_m`，任務會停在等待距離或靠近狀態，不會送抓取 goal。bear task 會優先選有有效深度的目標。

## 距離與置中

目前抓取只有一個距離門檻：

```text
target_max_distance_m = 0.24
```

行為：

```text
distance_m > 0.24  -> 往前靠近，同時做 X 軸置中輔助
distance_m <= 0.24 -> 停車並立即送出 /grab_object 抓取
```

X 軸置中只在靠近時輔助使用，不是抓取前的等待條件：

```text
center_error_px = bbox.center_x - target_center_x_px
```

`target_center_x_px < 0` 時會使用影像中心；目前 Kinect RGB 是 `1280x720`，起始校正值建議：

```bash
BEAR_TARGET_CENTER_X_PX=690.0
```

改 `.env` 後重新建立 container 狀態：

```bash
python3 robot_ws/src/wildbot_grasp/tools/grasp.py recreate
```

如果 state 一直停在 `aligning` 且 detail 是 `approaching target distance`，代表目前 `depth_m` 還大於 `target_max_distance_m`，車子還在靠近。

## 抓失敗與重試

重試分兩層：

```text
/grab_object action 內層：max_grasp_attempts = 2
bear_grasp_task_node 外層：max_task_retries = 0
```

`max_grasp_attempts=2` 表示每次 `/grab_object` action goal 裡最多試兩次夾取。`max_task_retries=0` 表示任務層沒有上限：action 失敗後，車子會後退、重新靠近，再送下一次 `/grab_object`。

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
| `grasp_pose_deg` | 167.0 | 75.0 | 170.6 | 夾取位置，關爪 |
| `carry_pose_deg` | 190.0 | 0.0 | 170.6 | 夾到後導航返回用的抱熊姿勢，保持關爪 |
| `place_pose_deg` | 120.0 | 75.0 | 239.0 | 放置位置，開爪/放開 |
| `home_pose_deg` | 190.0 | 0.0 | 240.0 | 啟動初始位置、放熊後最後回來的位置 |

完整姿態流程：

```python
initial_home       = [190.0, 0.0, 240.0]
open_at_grasp     = [167.0, 75.0, 239.0]
close_at_grasp    = [167.0, 75.0, 170.6]
carry_holding     = [190.0, 0.0, 170.6]
release_at_place  = [120.0, 75.0, 239.0]  # optional/manual release
return_home_final = [190.0, 0.0, 240.0]   # after release
```

`place_pose_deg` 保留為放熊/開爪位置，不拿來當導航抱熊姿勢。`release_after_grasp=false` 時，`/grab_object` 成功後會停在 `carry_pose_deg` 並保持夾爪關閉；如果之後需要恢復「抓到就放」的舊測試流程，可以把 `GRASP_RELEASE_AFTER_GRASP=true` 傳給 `grab_object_server`。

## Safety

`grab_object_server` 會讀 `/arm_joint_temperatures`，避免夾爪過熱時繼續開始抓取：

```text
gripper_temperature_index = 2
gripper_max_start_temp_c = 68.0
gripper_resume_temp_c = 65.0
```

所有高階手臂 trajectory 先送到：

```text
/arm_safeguard/target_trajectory
```

再由 `arm_safeguard_node` 轉發：

```text
/arm_controller/joint_trajectory
```

動作完成後 safeguard 會根據 controller feedback 發布短時間 relax trajectory，降低馬達持續出力與過熱。

## 主要檔案

```text
wildbot_grasp/
├── action/GrabObject.action
├── scripts/
│   ├── grasp_stack
│   ├── grab_object_server
│   ├── gripper_command
│   ├── arm_safeguard_node
│   └── bear_grasp_task_node
├── tools/
│   └── grasp.py
└── wildbot_grasp_nodes/
    ├── grab_object_server.py
    ├── motion.py
    ├── arm_safeguard_node.py
    ├── bear_grasp_task_node.py
    └── gripper_command.py
```

## TODO

- [ ] 實機確認 `target_max_distance_m=0.24` 是否是最佳抓取距離。
- [ ] 實測 `BEAR_TARGET_CENTER_X_PX=690.0` 是否讓夾爪中心線對準熊。
- [ ] 實測 `backup_linear_x=-0.05`、`backup_duration_sec=0.8` 是否合適。
- [ ] 確認 YOLO class name 是否固定為 `xiong`，必要時調整 `target_labels`。
- [ ] 加入任務層級 emergency stop，停止 `/motion/cmd` 並取消抓取 action。
- [ ] 之後若要接 Nav2，再新增正式 Navigation adapter 節點。
