# Memo — calibration & tuning backlog

Context: low-latency localization was re-architected to use a robot_localization
EKF fusing wheel odometry + Kinect IMU yaw rate (publishes `odom -> base_link`
at 50 Hz). RTAB-Map was demoted to `map -> odom` only; RGBD visual odometry is
no longer the TF authority. This was done with **uncalibrated** IMU/cameras to
first prove real-time localization works. The items below are the deferred
calibration/tuning work to do once the basic real-time pose is confirmed.

## 1. IMU calibration (Kinect built-in IMU, `/imu`, ~1.2 kHz)
- [ ] Gyro bias / zero-rate offset: log `/imu` stationary for ~60 s, record
      mean angular velocity, subtract as bias (or use `imu_filter` /
      `imu_tools` allan variance).
- [ ] Accelerometer scale/bias (only needed once we fuse acceleration —
      currently NOT fused in `ekf_wheel_imu.yaml`).
- [ ] Decide whether to keep raw `/imu` gyro-only (current, robust) or move to
      a calibrated orientation. Note: `ekf_fusion.yaml` documents that the
      Madgwick `/imu/filtered` orientation leaks linear accel into yaw and
      "explodes the map" — do NOT fuse absolute orientation until this is
      understood/fixed.

## 2. Camera intrinsics / extrinsics
- [ ] RGB + depth intrinsics calibration (the relay currently truncates
      rational_polynomial → plumb_bob 5-coeff as an approximation; replace with
      a real calibration).
- [ ] Camera ↔ base_link extrinsic: `camera_mount_*` args are passed in
      docker-compose by hand (x=0.17, z=0.25, pitch=0.11). Measure/calibrate
      properly (e.g. against base_link via a target).
- [ ] Camera ↔ IMU temporal + spatial extrinsic if visual-inertial fusion is
      revisited.

## 3. Wheel odometry calibration (`/base_controller/odom`)
- [ ] Verify `wheel_separation` (0.274) and `wheel_separation_multiplier`
      (2.21 — unusually large, looks like a fudge factor) with a
      straight-line + 360° rotation test; replace the multiplier with a real
      measured separation.
- [ ] Verify `wheel_radius` (0.05035) via a measured straight-line distance.
- [ ] Re-check `pose/twist_covariance_diagonal` in `controllers.yaml` against
      observed slip (skid-steer 4-wheel slips in rotation — that is why yaw is
      taken from the IMU, not the wheels).

## 4. EKF tuning follow-ups (`ekf_wheel_imu.yaml`)
- [ ] Confirm `controller_manager.update_rate` 50 is actually sustained by the
      wheel serial bus (115200 on `/dev/usb_wheel`). Check
      `ros2 topic hz /base_controller/odom` ≈ 50; if it stalls/jitters, drop
      toward 30 (in `controllers.yaml`).
- [ ] After IMU calibration, consider fusing IMU linear acceleration (ax) for
      better translation between RTAB-Map corrections.
- [ ] Consider re-adding RGBD visual odometry (`/odom_visual`) as a second EKF
      source once camera/IMU are calibrated, for drift resistance.
- [ ] Tune EKF process noise if pose is too sluggish/jittery.

## 5. Optional camera-pipeline latency (deferred, lower priority now)
Odometry no longer depends on the camera, so this only affects map-correction
freshness, not real-time pose. Point cloud stays ENABLED (needed by future
vision features). If RTAB-Map correction lag becomes an issue, revisit:
- [ ] `depth_mode` NFOV_UNBINNED → NFOV_2X2BINNED.
- [ ] rgbd_odometry / rtabmap input queue sizing.

## Validation checklist (after rebuild)
- [ ] `ros2 topic hz /base_controller/odom` ≈ 50 Hz.
- [ ] `ros2 topic hz /odometry/filtered` ≈ 50 Hz, low delay.
- [ ] `ros2 run tf2_ros tf2_echo odom base_link` updates at ~50 Hz and tracks
      motion with no visible lag.
- [ ] Only ONE publisher of `odom -> base_link` (ekf_filter_node). Confirm
      rgbd_odometry no longer publishes TF and `base_controller` has
      `enable_odom_tf: false`.
- [ ] RTAB-Map still publishes `map -> odom`; map does not drift/explode.
- [ ] Continuous nav still works; localized pose keeps up with a fast move.




self.declare_parameter("max_linear_velocity", 0.546)
self.declare_parameter("max_angular_velocity", 0.9)
self.declare_parameter("accel_linear", 0.6)
self.declare_parameter("accel_angular", 4.0)