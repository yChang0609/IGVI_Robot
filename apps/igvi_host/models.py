from __future__ import annotations

from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    ok: bool
    service: str = "igvi-host"
    message: str = "ready"
    docker_available: bool = False
    compose_available: bool = False
    dev_mode: bool = False


class SettingsModel(BaseModel):
    repo_root: str
    project_name: str
    host: str
    port: int
    bridge_url: str
    shm_path: str
    dev_mode: bool


class DevModeRequest(BaseModel):
    enabled: bool


class EstopRequest(BaseModel):
    engaged: bool = True


class EstopStatusResponse(BaseModel):
    ok: bool = True
    engaged: bool = False
    action: str = "estop"
    message: str = ""


class ComposeActionRequest(BaseModel):
    service: str | None = None
    services: list[str] | None = None
    profile: str | None = None
    dev_mode: bool | None = None
    no_cache: bool = False


class ComposeActionResponse(BaseModel):
    ok: bool
    action: str
    service: str | None = None
    services: list[str] = Field(default_factory=list)
    backend: str
    started_at: datetime
    finished_at: datetime
    message: str
    events_tail: list[str] = Field(default_factory=list)


class ServiceDescriptor(BaseModel):
    service: str
    profile: str
    profiles: list[str] = Field(default_factory=list)
    description: str = ""


class ContainerStatus(BaseModel):
    service: str
    profile: str | None = None
    container_id: str | None = None
    container_name: str | None = None
    image: str | None = None
    status: str = "unknown"
    health: str | None = None
    state: str | None = None
    created: str | None = None
    ports: list[str] = Field(default_factory=list)


class LogsResponse(BaseModel):
    service: str
    logs: str


class RosConnectionResponse(BaseModel):
    ok: bool
    url: str
    message: str


class CmdVelRequest(BaseModel):
    linear_x: float = 0.0
    angular_z: float = 0.0


class Pose2DRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


class ClearCostmapRequest(BaseModel):
    target: Literal["local", "global"] = "local"


class RosActionResponse(BaseModel):
    ok: bool
    action: str
    message: str


class ParamsSetRequest(BaseModel):
    node: str
    # Scalars (HSV ints, debug bool) or arrays (arm poses like [167.0, 80.0, 170.6]).
    params: dict[str, Any]


class OpenDoorGoalRequest(BaseModel):
    ready_distance_m: float = 0.0


class OpenDoorStatusResponse(BaseModel):
    ok: bool = True
    available: bool = False
    state: str = "unavailable"
    stage: str = ""
    message: str = ""
    progress: float = 0.0


class DoorMissionStartRequest(BaseModel):
    # Integrated nav → open_door task: drive to the named waypoint, then run
    # the open_door action. Defaults match the standard door_approach setup.
    waypoint: str = "door_approach"
    ready_distance_m: float = 0.0


class DoorMissionStatusResponse(BaseModel):
    ok: bool = True
    active: bool = False
    # idle | navigating | driving | opening | succeeded | failed | canceled
    phase: str = "idle"
    waypoint: str = ""
    ready_distance_m: float = 0.0
    message: str = ""
    # Nested snapshots so a UI can render everything from one polled GET.
    nav: dict[str, Any] = Field(default_factory=dict)
    open_door: dict[str, Any] = Field(default_factory=dict)


class ComposeProgressResponse(BaseModel):
    action: str | None = None
    busy: bool = False
    started_at: datetime | None = None
    finished_at: datetime | None = None
    last_line: str = ""
    lines: list[str] = Field(default_factory=list)
    seq: int = 0


class UiBridgeHealth(BaseModel):
    ok: bool
    path: str
    stale: bool = True
    message: str
    payload: dict = Field(default_factory=dict)


class RobotMapResponse(BaseModel):
    width: int = 0
    height: int = 0
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    data: list[int] = Field(default_factory=list)
    ok: bool = False


class RobotPoseResponse(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    ok: bool = False


class ImageTopicsResponse(BaseModel):
    topics: list[str] = Field(default_factory=list)




class ArmTemperaturesResponse(BaseModel):
    ok: bool = False
    temperatures: list[float | None] = Field(default_factory=list)
    gripper_index: int = 2
    gripper_temperature: float | None = None
    stamp_sec: float | None = None


class BatteryStatusResponse(BaseModel):
    ok: bool = False
    percentage: float | None = None
    voltage: float | None = None
    current: float | None = None
    charge: float | None = None
    capacity: float | None = None
    power_supply_status: int | None = None
    status: str = "unknown"
    charging: bool = False
    present: bool = False
    stamp_sec: float | None = None
    source: str = "host"
    message: str = ""


class ArmTrajectoryRequest(BaseModel):
    positions: list[float]
    time_from_start: float = 0.3


class NavGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class SearchRetrieveRequest(BaseModel):
    target_id: str
    home_pose_x: float
    home_pose_y: float
    home_pose_yaw: float = 0.0


class SearchRetrieveStatusResponse(BaseModel):
    state: str = "idle"
    message: str = ""
    server_ready: bool = False
    goal: dict | None = None
    feedback: dict = Field(default_factory=dict)


class BridgeRetrieveRequest(BaseModel):
    target_class: str = "xiong_qiao"
    bridge_waypoint_name: str = "bridge_center"
    bridge_pose_x: float | None = None
    bridge_pose_y: float | None = None
    bridge_pose_yaw: float = 0.0


class BridgeRetrieveStatusResponse(BaseModel):
    state: str = "idle"
    message: str = ""
    server_ready: bool = False
    goal: dict | None = None
    feedback: dict = Field(default_factory=dict)


class BridgeTraverseRequest(BaseModel):
    bridge_waypoint_name: str = "bridge_center"
    bridge_pose_x: float | None = None
    bridge_pose_y: float | None = None
    bridge_pose_yaw: float = 0.0


class ArenaMissionStartRequest(BaseModel):
    start_patrol_idx: int = 0


class BridgeTraverseStatusResponse(BaseModel):
    state: str = "idle"
    message: str = ""
    server_ready: bool = False
    goal: dict | None = None
    feedback: dict = Field(default_factory=dict)


class ArenaMissionStatusResponse(BaseModel):
    state: str = "idle"
    message: str = ""
    server_ready: bool = False
    goal: dict | None = None
    feedback: dict = Field(default_factory=dict)


class SaveMapRequest(BaseModel):
    filename: str = "arena_map"


class WaypointSaveRequest(BaseModel):
    name: str
    # x/y omitted → bridge snapshots the robot's current map-frame pose.
    x: float | None = None
    y: float | None = None
    yaw: float = 0.0


class WaypointNameRequest(BaseModel):
    name: str


class WaypointListResponse(BaseModel):
    waypoints: dict[str, dict[str, float]] = Field(default_factory=dict)


class CalibrationModel(BaseModel):
    camera_x: float = 0.17
    camera_y: float = 0.0
    camera_z: float = 0.25
    camera_roll: float = 0.0
    camera_pitch: float = 0.48
    camera_yaw: float = 0.0
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0
    wheel_separation: float = 0.274
    wheel_separation_multiplier: float = 2.21
    wheel_radius: float = 0.05035
    ekf_frequency: int = 50
    ekf_sensor_timeout: float = 0.2
    kinect_color_resolution: str = "720P"
    kinect_depth_mode: str = "NFOV_UNBINNED"
    kinect_fps: int = 15
    kinect_exposure_time_absolute: int = -1
    kinect_gain: int = -1
    kinect_white_balance: int = -1
    kinect_brightness: int = 128
    kinect_contrast: int = 5
    kinect_saturation: int = 32
    kinect_sharpness: int = 2
    kinect_backlight_compensation: bool = False
    kinect_powerline_frequency: int = 60
    face_point_distance_m: float = 0.3
    face_point_ang_kp: float = 1.5
    face_point_ang_max: float = 0.45
    face_point_ang_floor: float = 0.30
    face_point_align_deg: float = 30.0
    face_point_reverse_speed: float = 1.0
    face_point_yaw_tol_deg: float = 8.0
    face_point_timeout_sec: float = 10.0


class SaveMapResponse(BaseModel):
    ok: bool
    message: str = ""
    path: str = ""


class NavStatusResponse(BaseModel):
    state: str = "idle"
    message: str = ""
    server_ready: bool = False
    goal: dict | None = None
    feedback: dict = Field(default_factory=dict)
    visible_actions: list[str] = Field(default_factory=list)
    # Per-source freshness flags for the EKF inputs (wheel, imu, lidar).
    # Empty when bridge hasn't reported them yet (older bridge build).
    fusion_sources: dict = Field(default_factory=dict)


class ImuCalibrationStatusResponse(BaseModel):
    ok: bool = False
    state: str = "unavailable"
    message: str = ""
    stationary: bool = False
    converged: bool = False
    online: bool = False
    manual_required: bool = False
    manual_active: bool = False
    calibration_active: bool = False
    manual_remaining_s: float = 0.0
    stationary_age_s: float = 0.0
    convergence_age_s: float = 0.0
    gyro_error_rad_s: float = 0.0
    gyro_bias: list[float] = Field(default_factory=list)
    raw: str = ""


DockerAction = Literal["build", "rebuild", "start", "stop", "restart", "down"]
