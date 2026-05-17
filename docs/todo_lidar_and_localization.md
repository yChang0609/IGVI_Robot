# TODO: LiDAR 整合 + Localization 修正

> 建立日期：2026-05-17
> 分支：`todo/lidar-and-localization-fixes`

---

## 1. 修正 Localization 更新頻率過低

### 問題
`slam_localization.launch.py` 沒有設定 `RTAB-Map/DetectionRate`，
導致定位模式預設只有 **1 Hz** 更新，而 SLAM 建圖是 5 Hz。

### 影響
- `map → odom` 校正每秒只發生一次
- 機器人快速移動時全局位置落後
- Nav2 Global Planner 使用的 `map` frame 更新過慢

### 修正方式
在 `slam_localization.launch.py` 的 `rtabmap_loc_node` 參數加入：

```python
'RTAB-Map/DetectionRate': '10.0',   # 定位模式運算量輕，可設更高
'RTAB-Map/TimeThr':       '0',
'Mem/STMSize':            '10',     # 定位不需要大的短期記憶體
'RGBD/LinearUpdate':      '0.05',   # 移動 5cm 就更新
'RGBD/AngularUpdate':     '0.02',   # 轉 1° 就更新
```

### 驗證方式
```bash
ros2 topic hz /rtabmap/localization_pose
# 預期看到接近 10 Hz
```

---

## 2. 加入 2D LiDAR (ORadar MS200) 融合至 RTAB-Map

### 目前狀況
- LiDAR 驅動已存在（`oradarlidar.launch.py`）
- `/scan` 話題已發布
- RTAB-Map 目前 `subscribe_scan: False`，完全未使用

### 待處理事項

#### 2.1 確認機構遮擋角度
- LiDAR 某段角度被機構擋住，需要用 RViz 視覺化 `/scan` 找出被擋的範圍
- 修改 `ms200_scan.launch.py` 的預設角度：

```python
# 目前
angle_min: 0.1
angle_max: 350.9

# 修改為（依實際遮擋角度填入）
angle_min: ??   # TODO: 量測後填入
angle_max: ??   # TODO: 量測後填入
```

#### 2.2 實作斜坡偵測器（Slope Detector Node）
- 機器人會爬坡，2D LiDAR 爬坡時會掃到地面造成 SLAM 誤判
- 需要從 `/imu/filtered` 的 pitch 角判斷是否在爬坡
- 新增節點：`robot_ws/src/wildbot_control/wildbot_control/slope_detector_node.py`

```
訂閱：/imu/filtered  (sensor_msgs/Imu)
發布：/robot/pitch_angle  (std_msgs/Float32)
發布：/robot/on_slope     (std_msgs/Bool)
參數：pitch_threshold (預設 10.0 度)
```

- 爬坡時動態停用 RTAB-Map 的 LiDAR 輸入（`subscribe_scan: False`）

#### 2.3 修改 slam_fusion.launch.py
```python
'subscribe_scan': True,
'Grid/Sensor':    '0',   # 改用 LiDAR 建地圖

# 加入 scan 話題映射
('scan', '/scan'),
```

#### 2.4 修改 slam_localization.launch.py
- 同步加入 LiDAR 掃描輸入（與 slam_fusion 一致）

### 建議測試順序
1. 先確認遮擋角度（RViz 看 /scan）
2. 在平地測試 LiDAR 融合效果
3. 測試斜坡偵測器的 pitch 閾值
4. 測試爬坡時 LiDAR 自動停用

---

## 3. IMU Filter 頻率調整（選做）

### 問題
- Madgwick filter 沒有設定 `constant_dt`，跟著 IMU 1200 Hz 跑
- EKF 只在 50 Hz 處理，造成不必要的 CPU 負載

### 修正方式
在 `imu_filter_kinect.yaml` 加入：
```yaml
constant_dt: 0.02   # 50 Hz，與 EKF 同步
```
