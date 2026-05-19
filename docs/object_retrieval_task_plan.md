# 物件抓取任務實作規劃

## 任務目標

在比賽場地中，機器人需要自主尋找目標物件、移動到物件附近、夾取物件，並將物件帶回出發點。第一版目標是穩定完成「一顆物件」的完整流程；等單顆流程可靠後，再擴充成多物件策略。

評分重點：

- 成功取起並舉起一個目標物件：加 5 分。
- 成功將物件運回出發點：再加 5 分。
- 每碰撞一次靜態障礙物扣 1 分。
- 每掉落一次物件扣 1 分。
- 每成功取得決取物件加 1 分，每個物件最多 3 分。
- 撞到對手車直接取消比賽資格。

因此策略上應優先保守、穩定、低碰撞，而不是一開始追求高速多物件。

## 系統架構

建議把任務拆成五層：

```text
task_manager
  -> perception
  -> localization / navigation
  -> visual_servo
  -> arm_gripper_control
```

### 1. Localization

使用既有 RTAB-Map localization 模式載入 `data/slam/arena_map.db`，提供：

- `/map`
- `map -> odom`
- `/odometry/filtered`
- `/rtabmap/localization_pose`

用途：

- 知道機器人在 arena map 的位置。
- 讓 Nav2 在地圖上規劃路徑。
- 讓 UI 顯示目前位置與任務狀態。

啟動時必須包含：

- `core`：底盤與 `/base_controller/odom`
- `sensors` 或至少 `kinect`：RGB-D 與 IMU
- `lidar`：LiDAR scan，如果要用 ICP odometry 或避障
- `localization`
- `navigation`
- `bridge`
- `motion_arbiter`

### 2. Perception

第一版偵測方式建議由簡到難：

1. 如果目標物件顏色固定：使用 OpenCV HSV threshold + contour。
2. 如果物件外觀變化大：使用 YOLO / Eto Eye 偵測 bounding box。
3. 使用 depth image 估算物件距離。
4. 將物件位置由 camera frame 轉到 `base_link`，必要時再轉到 `map`。

Perception node 建議輸出：

```text
/task/target_candidates
```

資料內容：

- `id`
- `class_name`
- `confidence`
- `x_map`, `y_map`
- `x_base`, `y_base`
- `distance`
- `last_seen_time`

第一版只需要選最近且信心最高的目標。

### 3. Navigation

導航分成遠距與近距兩段。

遠距：

- 使用 Nav2 規劃到目標物件附近。
- 目標點不要直接設在物件上，應設在物件前方約 0.5 到 0.8 m。
- 由 `motion_arbiter` 負責實際速度輸出與安全限制。

近距：

- 切換成 visual servoing。
- 根據相機畫面讓物件保持在畫面中央。
- 慢速前進到夾取距離。
- 不使用 Nav2 做最後 30 到 50 cm 的對準。

### 4. Arm and Gripper

第一版使用固定夾取流程，不要一開始做複雜 motion planning。

建議流程：

```text
停車
確認物件在畫面中央
慢速前進到指定 depth
手臂移到預備姿態
打開夾爪
手臂下降或伸出
關閉夾爪
手臂抬起
車體後退
確認是否成功夾取
```

夾取成功判斷可以從簡單開始：

- 夾爪關閉角度沒有到完全閉合，表示中間可能夾到物體。
- 抬起後物件仍出現在相機近距離區域。
- 如果硬體支援馬達電流或 limit feedback，加入夾爪受力判斷。

### 5. Task Manager

建議建立一個高層 ROS node：`object_retrieval_manager`。

狀態機：

```text
IDLE
LOCALIZE
FIND_TARGET
SELECT_TARGET
PLAN_TO_TARGET
GO_TO_PREGRASP
VISUAL_ALIGN
PICK_OBJECT
VERIFY_PICK
RETURN_HOME
PLACE_OBJECT
DONE
RECOVERY
ABORT
```

狀態說明：

- `IDLE`：等待 UI 或 API 開始任務。
- `LOCALIZE`：確認 localization pose 已穩定。
- `FIND_TARGET`：旋轉或掃描，尋找目標物。
- `SELECT_TARGET`：選擇最近、最高信心、路徑風險最低的物件。
- `PLAN_TO_TARGET`：送 Nav2 goal 到 pre-grasp pose。
- `GO_TO_PREGRASP`：等待導航完成。
- `VISUAL_ALIGN`：用相機慢速對準物件。
- `PICK_OBJECT`：執行固定夾取序列。
- `VERIFY_PICK`：判斷是否夾取成功。
- `RETURN_HOME`：導航回出發點。
- `PLACE_OBJECT`：放下物件。
- `DONE`：任務完成。
- `RECOVERY`：處理找不到物件、導航失敗、夾取失敗。
- `ABORT`：安全中止。

## Safety Strategy

撞到對手車會直接失格，因此安全策略要比速度優先。

最低限度安全規則：

- 前方 LiDAR 距離小於門檻時立即停車。
- 偵測到動態障礙物時停車等待，不嘗試硬繞。
- 近距離 visual servoing 限制最大線速度與角速度。
- 夾取時底盤必須停住。
- 導航失敗超過指定次數就放棄該物件，重新選目標。

建議速度限制：

- 遠距導航：`0.2 ~ 0.35 m/s`
- 近距對準：`0.03 ~ 0.08 m/s`
- 旋轉搜尋：`0.2 ~ 0.4 rad/s`

## ROS Interfaces

### Task Manager Subscriptions

```text
/task/target_candidates
/odometry/filtered
/rtabmap/localization_pose
/map
/motion/state
/imu/calibration_state
```

### Task Manager Publishers

```text
/task/state
/task/selected_target
/motion/cmd
/arm_controller/joint_trajectory
/base_controller/cmd_vel
```

### Task Manager Actions / Services

```text
Action: /navigate_to_pose
Service: /global_costmap/clear_entirely_global_costmap
Service: /local_costmap/clear_entirely_local_costmap
```

## First MVP

第一版只做一顆物件：

1. 從 UI 按下開始任務。
2. 確認 localization pose 有效。
3. 搜尋最近目標物。
4. 導航到目標附近。
5. 視覺對準。
6. 夾取。
7. 確認成功。
8. 回 home pose。
9. 放下。
10. 任務結束。

MVP 不做：

- 多物件最佳化。
- 高速移動。
- 複雜手臂 motion planning。
- 主動與對手車搶路權。

## Milestones

### Milestone 1: Localization and Home

目標：

- 啟動 localization 後 UI 能看到穩定 robot pose。
- 設定 home pose。
- 機器人可以從任意位置回到 home 附近。

驗收：

- `/map` 正常。
- `/rtabmap/localization_pose` 或 UI pose 正常。
- Nav2 可以規劃回 home。

### Milestone 2: Target Detection

目標：

- 偵測目標物件。
- 輸出相對位置與信心分數。

驗收：

- 物件在 1 到 3 m 內可被穩定偵測。
- 偵測結果能轉成 `base_link` 座標。

### Milestone 3: Pre-Grasp Navigation

目標：

- 根據目標位置產生 pre-grasp pose。
- 導航到物件前方。

驗收：

- 車能停在物件前方 0.5 到 0.8 m。
- 車頭大致朝向物件。

### Milestone 4: Visual Servoing

目標：

- 讓物件在畫面中央。
- 慢速前進到夾取距離。

驗收：

- 物件中心誤差小於指定像素門檻。
- depth 到達夾取距離後停車。

### Milestone 5: Pick and Verify

目標：

- 執行固定夾取流程。
- 判斷是否成功。

驗收：

- 連續 5 次中至少 3 次成功夾起。
- 失敗時能回到 `RECOVERY`，不亂動。

### Milestone 6: Return and Place

目標：

- 夾到物件後回 home。
- 放下物件。

驗收：

- 單顆完整流程成功率達到 60% 以上。
- 過程中不碰撞靜態障礙物。

### Milestone 7: Multi-Object Strategy

目標：

- 根據距離、路徑風險、偵測信心選下一個物件。
- 每次只追一顆，成功回 home 後再選下一顆。

驗收：

- 能連續完成兩顆物件。
- 任務中可安全放棄高風險目標。

## Recommended Implementation Order

1. 修好 localization profile 的依賴，讓一鍵啟動包含 core、Kinect、LiDAR、localization。
2. 確認 `arena_map.db` 的保存與載入流程。
3. 實作 `object_retrieval_manager` skeleton，只輸出 state。
4. 接 Nav2 action，完成 home pose return。
5. 實作 perception MVP。
6. 實作 pre-grasp pose 計算。
7. 實作 visual servoing。
8. 實作 arm/gripper 固定流程。
9. 加上 verify pick。
10. 加 recovery 與安全停止。

第一階段的現場操作 SOP 見：

- `docs/object_retrieval_ui_mvp_runbook.md`

## Risk List

- RTAB-Map localization 找不到初始位置：需要人工 initial pose 或設計固定起點。
- RGB-D depth 延遲過大：已用較大的 sync queue 緩解，但 close-range servoing 仍需低速。
- 物件太小或反光：需要調整光源、偵測模型或改用顏色標記。
- 夾爪沒有 feedback：verify pick 會不穩，需要用相機或電流回授補強。
- LiDAR 只看到平面障礙：低矮或透明物件可能需要相機輔助避障。
- 對手車動態不可預測：策略上寧可停等，不要強行穿越。

## Success Criteria

第一階段成功標準：

- 能在已知地圖內定位。
- 能找到一個目標物。
- 能導航到物件前方。
- 能完成一次夾取。
- 能回到出發點。
- 全流程不撞障礙物。

比賽策略成功標準：

- 單顆任務穩定成功率高於 70%。
- 每次失敗能安全停止或重新嘗試。
- 只有在場地與對手車狀態安全時才嘗試第二顆以上。
