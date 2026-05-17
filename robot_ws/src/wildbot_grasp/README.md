# wildbot_grasp

`wildbot_grasp` 是目前 Wildbot 的 rule-based 夾取 package。這一版先不做 YOLO 自動定位，而是先把「固定角度開爪、關爪、判斷有沒有夾到、移到放置姿態、再放開」這條流程穩定下來。

目前正確流程已經可以跑：夾爪會從 open 角度往 close 角度移動，根據 `/joint_states` 的 `gripper_joint` 角度判斷是否夾到物體。

## 架構

```text
使用者 / YOLO TODO
        |
        v
/grab_object action
        |
        v
grab_object_server
        |
        v
/arm_controller/joint_trajectory
        |
        v
kros_car arm_controller
        |
        v
實體手臂 / 夾爪
```

最小測試入口：

```text
ros2 run wildbot_grasp gripper_command open
ros2 run wildbot_grasp gripper_command close
```

主要檔案：

```text
wildbot_grasp/
├── action/GrabObject.action          # /grab_object action 格式
├── scripts/grab_object_server        # action server 入口
├── scripts/gripper_command           # open/close 測試入口
└── wildbot_grasp_nodes/
    ├── grab_object_server.py         # 夾取流程與夾取判斷
    ├── gripper_command.py            # 手動 open/close
    └── motion.py                     # 角度設定與 JointTrajectory publish
```

## 目前校正角度

角度用「度」寫在程式參數裡，送給 ROS controller 前會自動轉成 radians。

| Pose | arm_1 | arm_2 | gripper | 用途 |
|---|---:|---:|---:|---|
| `grasp_pose_deg` | 167 | 75 | 170.6 | 夾取位置，關爪 |
| `place_pose_deg` | 120 | 75 | 239.0 | 放置位置，開爪/放開 |

目前夾爪實測角度：

```text
close = 170.6 deg = 2.9775 rad
open  = 239.0 deg = 4.1713 rad
```

衍生姿態：

```text
open_at_grasp    = [167, 75, 239.0]   # 到夾取位置並打開夾爪
close_at_grasp   = [167, 75, 170.6]   # 在夾取位置關爪
place_holding    = [120, 75, 170.6]   # 移到放置位置，但保持關爪
release_at_place = [120, 75, 239.0]   # 在放置位置打開夾爪
```

重要：之前 `open` 用過 `219.2` 度，但實測 `/joint_states` 顯示開爪可能已經在 `236~238` 度。這時候送 `219.2` 反而是在往關爪方向推，容易觸發：

```text
Gripper stall detected ... blocking further close
```

所以現在 open/release 改成 `239.0` 度。

## 啟動方式

在 host 主機執行 compose 指令，不要在 container 裡執行。

```bash
cd ~/workspace/IGVI_Robot_kevin/docker/compose
```

啟動 `kros_car` 和 `wildbot_grasp`：

```bash
docker compose --profile core --profile grasp up -d --build kros_car wildbot_grasp
```

如果只要重建/重啟夾取 package：

```bash
docker compose --profile grasp up -d --build wildbot_grasp
```

夾爪測試時建議先停掉 `igvi_bridge`，避免它也 publish `/arm_controller/joint_trajectory` 搶控制：

```bash
docker compose stop igvi_bridge
```

確認目前只有 `grab_object_server` 發手臂命令、`arm_controller` 有訂閱：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 topic info -v /arm_controller/joint_trajectory"
```

正常重點：

```text
Subscription count: 1
Node name: arm_controller
```

## 最小硬體測試

先測 open：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 run wildbot_grasp gripper_command open"
```

預期 log 會看到：

```text
publishing open: deg=[120.0, 75.0, 239.0]
```

再測 close：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 run wildbot_grasp gripper_command close"
```

預期 log 會看到：

```text
publishing close: deg=[167.0, 75.0, 170.6]
```

看目前手臂回授：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 topic echo --once /joint_states"
```

## 正式夾取測試

把物體放在夾爪前方，執行：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 action send_goal /grab_object wildbot_grasp/action/GrabObject    '{object_label: bear, distance_m: 0.32}' --feedback"
```

`distance_m` 目前只是保留欄位，還沒有真的拿來算 IK 或 YOLO 距離。

## Action 流程

`/grab_object` 目前流程：

```text
1. safety check
2. open_for_grasp       -> [167, 75, 239.0]
3. close_gripper        -> [167, 75, 170.6]
4. check_grasp
5. 如果沒夾到，最多重試 2 次
6. 如果夾到，move_to_place_holding -> [120, 75, 170.6]
7. verify_at_place      -> 到 place 後確認還夾著
8. release_at_place     -> [120, 75, 239.0]
```

成功時會看到這些 feedback stage：

```text
open_for_grasp
gripper_angle
close_gripper
gripper_min_angle
gripper_angle
check_grasp
move_to_place_holding
gripper_angle
verify_at_place
release_at_place
```

`gripper_min_angle` 會記錄關爪過程中 gripper 角度最小的那一刻，用來確認夾爪真的有動。

## 夾取成功判斷

目前沒有 voltage/current/load topic，所以用 `/joint_states` 的 `gripper_joint` 做 rule-based 判斷。

關爪後要同時滿足兩個條件，才算有夾到：

```text
actual_gripper - close_target >= grasp_detect_min_error_deg
open_gripper - actual_gripper >= grasp_detect_min_close_motion_deg
```

目前預設：

```text
grasp_detect_settle_sec = 1.0
grasp_detect_min_error_deg = 3.0
grasp_detect_min_close_motion_deg = 5.0
max_grasp_attempts = 2
```

意思：

```text
條件 1：actual 比 close target 大至少 3 度
        表示夾爪沒有完全關到底，可能被物體卡住。

條件 2：open 到 actual 至少關了 5 度
        表示夾爪真的有往關閉方向移動，不是卡在 open 位置。
```

例子：

```text
open=236.6, close_target=170.6, actual=173.5
error=+2.9, threshold=3.0
=> 沒夾到，因為幾乎關到底。

open=235.7, close_target=170.6, actual=177.6
error=+7.0, threshold=3.0, close_motion=58.1
=> 判斷有夾到。
```

## 如何看 log

看 `wildbot_grasp` log：

```bash
docker compose logs -f wildbot_grasp
```

重點看：

```text
publishing open_for_grasp: deg=[167.0, 75.0, 239.0]
publishing close_gripper_attempt_1: deg=[167.0, 75.0, 170.6]
gripper_min_angle: ... first=... last=... min=... min_close_motion=...
detect_grasp variables: ... error_deg=... close_motion_deg=... grasped=...
```

健康的關爪 log 會長這樣：

```text
first=236.64deg
last=174.48deg
min=174.48deg
min_close_motion=62.16deg
```

這代表夾爪真的從 open 往 close 移動。

如果看到：

```text
first=237.8deg
last=237.8deg
min=237.8deg
min_close_motion=0.0deg
```

代表夾爪完全沒有動，通常是硬體 stall、serial 通訊、或控制被其他 node 蓋掉。

## 安全規則

開始夾取前會讀 `/arm_joint_temperatures`。

目前設定：

```text
gripper_temperature_index = 2
gripper_max_start_temp_c = 68.0
gripper_resume_temp_c = 65.0
```

查溫度：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 topic echo --once /arm_joint_temperatures"
```

如果溫度太高，action 會 abort，不會繼續開關爪：

```text
stage: safety_abort
success: false
object_grasped: false
Goal finished with status: ABORTED
```

## 常見問題

### 1. 指令有 publish，但手臂不動

確認 `kros_car` 是否在跑：

```bash
docker ps --format "table {{.Names}}	{{.Status}}"
```

確認 ROS topic 有接上：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 topic info -v /arm_controller/joint_trajectory"
```

正常要有：

```text
Subscription count: 1
Node name: arm_controller
```

### 2. arm1/arm2 會動，但 gripper 不動

看 kros_car log：

```bash
docker logs --tail 80 igvi_robot-kros_car-1
```

如果看到：

```text
Gripper stall detected ... blocking further close
```

先送 open：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc   "source /opt/ros/jazzy/setup.bash &&    source /workspace/install/setup.bash &&    ros2 run wildbot_grasp gripper_command open"
```

如果仍然不能解除，重啟 `kros_car`：

```bash
docker restart igvi_robot-kros_car-1
```

然後再測 open/close。

### 3. 出現 serial write error

看 `kros_car` log 如果有：

```text
Arm cmd tx slow/fail
Wheel serial write error
```

代表 ROS topic 可能是通的，但 kros_car 寫實體硬體失敗。檢查 USB、手臂電源、控制板，再重啟：

```bash
docker restart igvi_robot-kros_car-1
```

### 4. 有兩個 publisher 搶手臂 topic

檢查：

```bash
ros2 topic info -v /arm_controller/joint_trajectory
```

測 grasp 時建議停掉 bridge：

```bash
docker compose stop igvi_bridge
```

正常測試時最好只留下：

```text
Publisher: grab_object_server
Subscriber: arm_controller
```

## Action 定義

`action/GrabObject.action`：

```text
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

結果語意：

```text
object_grasped=true  -> success=true,  action status=SUCCEEDED
object_grasped=false -> success=false, action status=ABORTED
```

## TODO：YOLO + Kinect 自動夾取

未來要接 YOLO/Kinect 時，保留這條方向：

```text
YOLO bbox
-> Kinect depth 取得目標距離
-> camera_info + TF/calibration 轉成 3D camera 座標
-> Kinect frame 轉到 arm/base frame
-> 判斷目標是否在可夾範圍
-> 產生 grasp pose 或 IK
-> 呼叫 /grab_object
```

待補：

- YOLO detection topic 格式
- bbox center + depth 轉 3D camera 座標
- Kinect 到 base/arm 的外參或 TF
- 根據目標位置決定 grasp pose
- 自動呼叫 `/grab_object`
