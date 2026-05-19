# wildbot_grasp

`wildbot_grasp` 是目前 Wildbot 的 rule-based 夾取 package。這一版主要確立了「固定角度開爪、關爪、判斷有沒有夾到、移到放置姿態、再放開」的穩定流程，並導入了 **硬體安全守門員 (Safeguard Node)** 機制，徹底解決了馬達穩態誤差導致的過熱與爭奪控制權問題。

## 系統架構 (攔截器模式)

為了保護底層硬體，所有高階控制訊號（包含自動化 AI 與手動 UI 遙控）都不再直接對接底層馬達，而是統一發送至中間主題 `/arm_safeguard/target_trajectory`，由 `arm_safeguard_node` 統一審查、計時放鬆後，才發送給硬體。

```text
使用者 (igvi-ui) / YOLO (TODO)
        |
        v
/grab_object action 或 bridge 遙控訊號
        |
        v
grab_object_server / bridge_node
        |
        v [發布至: /arm_safeguard/target_trajectory]
        |
=================================================
🌟 arm_safeguard_node (硬體安全守門員)
  - 統整所有高階來源，加上 Mutex 鎖防止互搶
  - 執行「抵達定點後延遲蒐集純淨數據，並取平均值放鬆」
=================================================
        |
        v [發布至: /arm_controller/joint_trajectory]
        |
kros_car arm_controller
        |
        v
實體手臂 / 夾爪
```

最小測試入口：

```bash
ros2 run wildbot_grasp gripper_command open
ros2 run wildbot_grasp gripper_command close
```

主要檔案：

```text
wildbot_grasp/
├── action/GrabObject.action          # /grab_object action 格式
├── scripts/grab_object_server        # action server 入口
├── scripts/gripper_command           # open/close 測試入口
├── scripts/arm_safeguard_node        # 守門員節點入口
└── wildbot_grasp_nodes/
    ├── grab_object_server.py         # 夾取流程與夾取判斷
    ├── gripper_command.py            # 手動 open/close
    ├── motion.py                     # 角度設定與發布 (指向 safeguard)
    └── arm_safeguard_node.py         # 🌟 防過熱與穩態誤差消除機制
```

## 目前校正角度

角度用「度」寫在程式參數裡，送給 ROS controller 前會自動轉成 radians。

| Pose | arm_1 | arm_2 | gripper | 用途 |
|---|---|---|---|---|
| `grasp_pose_deg` | 167 | 75 | 170.6 | 夾取位置，關爪 |
| `place_pose_deg` | 120 | 75 | 239.0 | 放置位置，開爪/放開 |

衍生姿態：

```python
open_at_grasp    = [167, 75, 239.0]   # 到夾取位置並打開夾爪
close_at_grasp   = [167, 75, 170.6]   # 在夾取位置關爪
place_holding    = [120, 75, 170.6]   # 移到放置位置，但保持關爪
release_at_place = [120, 75, 239.0]   # 在放置位置打開夾爪
```

重要：之前 open 用過 219.2 度，但實測 `/joint_states` 顯示開爪可能已經在 236~238 度。這時候送 219.2 反而是在往關爪方向推，容易觸發 `Gripper stall detected`，所以現在 open/release 統一改為 239.0 度。

## 啟動方式

在 host 主機執行 compose 指令，不要在 container 裡執行。

```bash
cd ~/workspace/IGVI_Robot/docker/compose
```

啟動核心、夾取服務與通訊橋樑（這會同時帶起 Server 與 Safeguard 兩個容器）：

```bash
docker compose -f compose.yaml --profile core --profile grasp --profile bridge up -d
```

如果修改了 CMakeLists.txt 或新增了節點，請重新 Build 映像檔：

```bash
docker compose -f compose.yaml --profile grasp build wildbot_grasp
```

*(註：如果僅修改 Python 邏輯，無需重新 Build，直接 `docker restart igvi_robot-wildbot_grasp-1` 及對應容器即可生效。)*

確認架構是否正確接上守門員：

```bash
docker exec -it igvi_robot-wildbot_grasp-1 bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic info -v /arm_controller/joint_trajectory"
```

正常狀態下，實體硬體端應只有一個發布者：

```text
Publisher count: 1
Node name: arm_safeguard_node
Subscription count: 1
Node name: arm_controller
```

## 雙重安全防護機制 (Safety Rules)

本專案採用「高階邏輯 + 底層硬體」雙重防護，徹底解決過熱當機問題：

### 1. 高階防護 (Grab Object Server)

開始夾取前會讀取 `/arm_joint_temperatures`，若溫度超過設定值，Action 會直接中止 (ABORTED)，拒絕執行任務。

- `gripper_max_start_temp_c = 68.0`
- `gripper_resume_temp_c = 65.0`

### 2. 底層防護 (Arm Safeguard Node)

針對馬達夾住物體時產生的「穩態誤差 (Steady-State Error)」，Safeguard 節點會在背景全天候運行：

- 當手臂移動至定點後，延遲蒐集純淨的歷史角度數據 (deque)。
- 計算平均真實角度，並發布微調放鬆軌跡（Duration = 0.1s）。
- 放鬆後馬達將不再持續出力對抗應力，大幅降低電流與發熱，同時保持夾持狀態。

## 夾取成功判斷

目前沒有 force sensor topic，因此使用 `/joint_states` 的 `gripper_joint` 進行 Rule-based 判斷。關爪後需同時滿足兩個條件：

- **條件 1 (卡住檢查)**：`actual - close_target >= 3.0` 度
  表示夾爪沒有完全關到底，被物體卡住了。
- **條件 2 (移動檢查)**：`open - actual >= 5.0` 度
  表示夾爪確實有往關閉方向移動，而非一開始就卡死在原位。

## TODO：YOLO 自動夾取 (IPM 架構)

未來整合 YOLO 視覺辨識時，將捨棄容易產生盲區的 Kinect 深度圖，改採 平地假設 (IPM, Inverse Perspective Mapping) 與 視覺伺服 (Visual Servoing) 策略：

- **視覺測距**：YOLO 辨識到目標後，取 Bounding Box 的底部中心像素 (`cy = y2`)。
- **IPM 轉換**：利用相機內參與已知車體高度，透過射線追蹤純幾何轉換為 3D 地面絕對座標。
- **車體對準 (Visual Servoing)**：YOLO 節點發布 Twist 給 `/motion/cmd`，由 `motion_arbiter` 控制底盤，將車體精準移動至目標正前方指定距離。
- **觸發抓取**：車體停妥後，YOLO 作為 Action Client 發送 `GrabObject` Goal，交由 Server 執行固定的盲抓軌跡。

待補實作：

- [ ] YOLO NCNN 節點建立與相機內參校正。
- [ ] IPM 像素轉換地圖座標公式實作。
- [ ] 實作 YOLO Node 中的 GrabObject Action Client 觸發邏輯。
