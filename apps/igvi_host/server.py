from __future__ import annotations

import os
import tempfile
from contextlib import suppress
from pathlib import Path
from typing import Callable

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response

from .battery import read_host_battery
from .config import HostSettings, load_settings, save_settings
from .docker_clients import (
    ComposeProjectClient,
    DockerComposeError,
    DockerContainerNotFoundError,
    DockerEngineClient,
    DockerUnavailableError,
)
from .models import (
    ArmTemperaturesResponse,
    ArmTrajectoryRequest,
    ArenaMissionStatusResponse,
    BatteryStatusResponse,
    BridgeRetrieveRequest,
    BridgeRetrieveStatusResponse,
    BridgeTraverseRequest,
    BridgeTraverseStatusResponse,
    ClearCostmapRequest,
    CalibrationModel,
    CmdVelRequest,
    ComposeActionRequest,
    ComposeActionResponse,
    ComposeProgressResponse,
    ContainerStatus,
    DevModeRequest,
    EstopRequest,
    EstopStatusResponse,
    HealthResponse,
    ImageTopicsResponse,
    ImuCalibrationStatusResponse,
    LogsResponse,
    NavGoalRequest,
    NavStatusResponse,
    DoorMissionStartRequest,
    DoorMissionStatusResponse,
    OpenDoorGoalRequest,
    OpenDoorStatusResponse,
    ParamsSetRequest,
    Pose2DRequest,
    RobotMapResponse,
    RobotPoseResponse,
    RosActionResponse,
    RosConnectionResponse,
    SaveMapRequest,
    SaveMapResponse,
    SearchRetrieveRequest,
    SearchRetrieveStatusResponse,
    ServiceDescriptor,
    SettingsModel,
    UiBridgeHealth,
    WaypointListResponse,
    WaypointNameRequest,
    WaypointSaveRequest,
)
from .progress import get_progress_buffer
from .ros_control import RobotBridgeClient
from .service_registry import ServiceRegistry
from .ui_bridge import read_ui_bridge_health


def create_app(settings: HostSettings | None = None) -> FastAPI:
    state_settings = settings or load_settings()
    app = FastAPI(title="IGVI Host Agent", version="0.2.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://127.0.0.1", "http://localhost"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    def current_settings() -> HostSettings:
        return state_settings

    def registry() -> ServiceRegistry:
        return ServiceRegistry(current_settings().compose_file)

    def docker_engine() -> DockerEngineClient:
        try:
            return DockerEngineClient(current_settings(), registry=registry())
        except DockerUnavailableError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    def compose_project() -> ComposeProjectClient:
        try:
            return ComposeProjectClient(current_settings(), registry=registry())
        except DockerUnavailableError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    def compose_or_http(callback):
        try:
            return callback()
        except DockerComposeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def target_services(request: ComposeActionRequest) -> list[str] | None:
        reg = registry()
        services = reg.validate_services(request.services)
        if services:
            return services
        service = reg.validate_service(request.service)
        return [service] if service else None

    async def run_ros(action: Callable[[RobotBridgeClient], object]) -> RosActionResponse:
        client = RobotBridgeClient(current_settings())
        try:
            result = action(client)
            if hasattr(result, "__await__"):
                return await result  # type: ignore[no-any-return]
            return result  # type: ignore[return-value]
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.get("/api/health", response_model=HealthResponse)
    def health() -> HealthResponse:
        docker_available = False
        compose_available = False
        with suppress(Exception):
            docker_available = DockerEngineClient(current_settings(), registry=registry()).ping()
        with suppress(Exception):
            compose_available = ComposeProjectClient(current_settings(), registry=registry()).ping()
        return HealthResponse(
            ok=True,
            docker_available=docker_available,
            compose_available=compose_available,
            dev_mode=current_settings().dev_mode,
        )

    @app.get("/api/settings", response_model=SettingsModel)
    def get_settings() -> SettingsModel:
        return SettingsModel(**current_settings().to_json_dict())

    @app.post("/api/settings", response_model=SettingsModel)
    def update_settings(request: SettingsModel) -> SettingsModel:
        nonlocal state_settings
        state_settings = HostSettings.from_json_dict(request.model_dump())
        save_settings(state_settings)
        return SettingsModel(**state_settings.to_json_dict())

    @app.post("/api/compose/dev-mode", response_model=SettingsModel)
    def set_dev_mode(request: DevModeRequest) -> SettingsModel:
        nonlocal state_settings
        state_settings.dev_mode = request.enabled
        save_settings(state_settings)
        return SettingsModel(**state_settings.to_json_dict())

    @app.get("/api/calibration", response_model=CalibrationModel)
    def get_calibration() -> CalibrationModel:
        configs = current_settings().repo_root / "robot_ws" / "configs"
        calib_file = configs / "calibration.yaml"
        ctrl_file = configs / "controllers.yaml"
        result: dict = {}
        if calib_file.exists():
            data = yaml.safe_load(calib_file.read_text()) or {}
            cam = data.get("camera_extrinsics", {})
            result.update(camera_x=cam.get("x", 0.17), camera_y=cam.get("y", 0.0),
                          camera_z=cam.get("z", 0.25), camera_roll=cam.get("roll", 0.0),
                          camera_pitch=cam.get("pitch", 0.48), camera_yaw=cam.get("yaw", 0.0))
            imu = data.get("imu", {})
            result.update(gyro_bias_x=imu.get("gyro_bias_x", 0.0),
                          gyro_bias_y=imu.get("gyro_bias_y", 0.0),
                          gyro_bias_z=imu.get("gyro_bias_z", 0.0))
            ekf = data.get("ekf", {})
            result.update(ekf_frequency=ekf.get("frequency", 50),
                          ekf_sensor_timeout=ekf.get("sensor_timeout", 0.2))
            kinect = data.get("kinect", {})
            result.update(
                kinect_color_resolution=kinect.get("color_resolution", "720P"),
                kinect_depth_mode=kinect.get("depth_mode", "NFOV_UNBINNED"),
                kinect_fps=kinect.get("fps", 15),
                kinect_exposure_time_absolute=kinect.get("exposure_time_absolute", -1),
                kinect_gain=kinect.get("gain", -1),
                kinect_white_balance=kinect.get("white_balance", -1),
                kinect_brightness=kinect.get("brightness", 128),
                kinect_contrast=kinect.get("contrast", 5),
                kinect_saturation=kinect.get("saturation", 32),
                kinect_sharpness=kinect.get("sharpness", 2),
                kinect_backlight_compensation=kinect.get("backlight_compensation", False),
                kinect_powerline_frequency=kinect.get("powerline_frequency", 60),
            )
        if ctrl_file.exists():
            ctrl = yaml.safe_load(ctrl_file.read_text()) or {}
            bc = ctrl.get("base_controller", {}).get("ros__parameters", {})
            result.update(wheel_separation=bc.get("wheel_separation", 0.274),
                          wheel_separation_multiplier=bc.get("wheel_separation_multiplier", 2.21),
                          wheel_radius=bc.get("wheel_radius", 0.05035))
        return CalibrationModel(**result)

    @app.post("/api/calibration", response_model=CalibrationModel)
    def set_calibration(request: CalibrationModel) -> CalibrationModel:
        configs = current_settings().repo_root / "robot_ws" / "configs"
        calib_file = configs / "calibration.yaml"
        ctrl_file = configs / "controllers.yaml"
        d = request.model_dump()
        calib_data = {
            "camera_extrinsics": {
                "x": d["camera_x"], "y": d["camera_y"], "z": d["camera_z"],
                "roll": d["camera_roll"], "pitch": d["camera_pitch"], "yaw": d["camera_yaw"],
            },
            "imu": {
                "gyro_bias_x": d["gyro_bias_x"],
                "gyro_bias_y": d["gyro_bias_y"],
                "gyro_bias_z": d["gyro_bias_z"],
            },
            "ekf": {"frequency": d["ekf_frequency"], "sensor_timeout": d["ekf_sensor_timeout"]},
            "kinect": {
                "color_resolution": d["kinect_color_resolution"],
                "depth_mode": d["kinect_depth_mode"],
                "fps": d["kinect_fps"],
                "exposure_time_absolute": d["kinect_exposure_time_absolute"],
                "gain": d["kinect_gain"],
                "white_balance": d["kinect_white_balance"],
                "brightness": d["kinect_brightness"],
                "contrast": d["kinect_contrast"],
                "saturation": d["kinect_saturation"],
                "sharpness": d["kinect_sharpness"],
                "backlight_compensation": d["kinect_backlight_compensation"],
                "powerline_frequency": d["kinect_powerline_frequency"],
            },
        }
        tmp_fd, tmp_path = tempfile.mkstemp(dir=str(configs), suffix=".yaml")
        try:
            os.write(tmp_fd, yaml.dump(calib_data, default_flow_style=False).encode())
            os.close(tmp_fd)
            os.replace(tmp_path, str(calib_file))
        except Exception:
            with suppress(OSError):
                os.close(tmp_fd)
            with suppress(OSError):
                os.unlink(tmp_path)
            raise
        if ctrl_file.exists():
            ctrl = yaml.safe_load(ctrl_file.read_text()) or {}
            bc = ctrl.setdefault("base_controller", {}).setdefault("ros__parameters", {})
            bc["wheel_separation"] = d["wheel_separation"]
            bc["wheel_separation_multiplier"] = d["wheel_separation_multiplier"]
            bc["wheel_radius"] = d["wheel_radius"]
            tmp_fd2, tmp_path2 = tempfile.mkstemp(dir=str(configs), suffix=".yaml")
            try:
                os.write(tmp_fd2, yaml.dump(ctrl, default_flow_style=False).encode())
                os.close(tmp_fd2)
                os.replace(tmp_path2, str(ctrl_file))
            except Exception:
                with suppress(OSError):
                    os.close(tmp_fd2)
                with suppress(OSError):
                    os.unlink(tmp_path2)
                raise
        return request

    @app.get("/api/compose/profiles", response_model=list[str])
    def list_profiles() -> list[str]:
        return sorted(registry().allowed_profiles())

    @app.get("/api/compose/registry", response_model=list[ServiceDescriptor])
    def list_registry() -> list[ServiceDescriptor]:
        return [
            ServiceDescriptor(
                service=service.name,
                profile=service.profile,
                profiles=list(service.profiles),
                description=service.description,
            )
            for service in registry().services()
        ]

    @app.get("/api/compose/services", response_model=list[ContainerStatus])
    def list_services() -> list[ContainerStatus]:
        try:
            return docker_engine().list_project_containers()
        except HTTPException:
            return [
                ContainerStatus(service=service.name, profile=service.profile, status="docker_unavailable")
                for service in registry().services()
            ]

    @app.get("/api/compose/services/{service}", response_model=ContainerStatus)
    def service_status(service: str) -> ContainerStatus:
        registry().validate_service(service)
        for item in list_services():
            if item.service == service:
                return item
        raise HTTPException(status_code=404, detail=f"Service not found: {service}")

    @app.get("/api/compose/services/{service}/logs", response_model=LogsResponse)
    def service_logs(service: str, tail: int = 200) -> LogsResponse:
        try:
            registry().validate_service(service)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        tail = min(max(tail, 1), 2000)
        try:
            logs = docker_engine().get_logs(service, tail=tail)
        except DockerContainerNotFoundError:
            logs = (
                f"{service} has no container yet.\n\n"
                "Start the service first, then refresh logs.\n"
            )
        except DockerUnavailableError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return LogsResponse(service=service, logs=logs)

    @app.post("/api/compose/actions/build", response_model=ComposeActionResponse)
    def build(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        return compose_or_http(lambda: compose_project().build(services=services, no_cache=request.no_cache))

    @app.post("/api/compose/actions/rebuild", response_model=ComposeActionResponse)
    def rebuild(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        return compose_or_http(
            lambda: compose_project().rebuild(services=services, profile=request.profile)
        )

    @app.post("/api/compose/actions/start", response_model=ComposeActionResponse)
    def start(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        profile = registry().validate_profile(request.profile)
        return compose_or_http(lambda: compose_project().up(services=services, profile=profile, detach=True))

    @app.post("/api/compose/actions/stop", response_model=ComposeActionResponse)
    def stop(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        if not services:
            raise HTTPException(status_code=400, detail="stop requires at least one service")
        return compose_or_http(lambda: compose_project().stop(services=services))
    
    @app.post("/api/compose/actions/remove", response_model=ComposeActionResponse)
    def remove(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        if not services:
            raise HTTPException(status_code=400, detail="remove requires at least one service")
        return compose_or_http(lambda: compose_project().remove(services=services))

    @app.post("/api/compose/actions/build_start", response_model=ComposeActionResponse)
    def build_start(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        return compose_or_http(
            lambda: compose_project().build_start(services=services, profile=request.profile)
        )

    @app.post("/api/compose/actions/restart", response_model=ComposeActionResponse)
    def restart(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        if not services:
            raise HTTPException(status_code=400, detail="restart requires at least one service")
        return compose_or_http(lambda: compose_project().restart(services=services))

    @app.post("/api/compose/actions/stop_all", response_model=ComposeActionResponse)
    def stop_all() -> ComposeActionResponse:
        return compose_or_http(lambda: compose_project().stop_all())

    @app.post("/api/compose/actions/remove_all", response_model=ComposeActionResponse)
    def remove_all() -> ComposeActionResponse:
        return compose_or_http(lambda: compose_project().remove_all())

    @app.get("/api/compose/progress", response_model=ComposeProgressResponse)
    def compose_progress(tail: int = 50, since_seq: int = 0) -> ComposeProgressResponse:
        tail = min(max(tail, 1), 500)
        snapshot = get_progress_buffer().snapshot()
        lines = snapshot.lines[-tail:] if tail else snapshot.lines
        return ComposeProgressResponse(
            action=snapshot.action,
            busy=snapshot.busy,
            started_at=snapshot.started_at,
            finished_at=snapshot.finished_at,
            last_line=snapshot.last_line,
            lines=lines,
            seq=snapshot.seq,
        )

    @app.get("/api/ros/connection", response_model=RosConnectionResponse)
    async def ros_connection() -> RosConnectionResponse:
        return await RobotBridgeClient(current_settings()).ping()

    @app.post("/api/ros/cmd_vel", response_model=RosActionResponse)
    async def cmd_vel(request: CmdVelRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_cmd_vel(request))

    @app.post("/api/ros/stop", response_model=RosActionResponse)
    async def ros_stop() -> RosActionResponse:
        return await run_ros(lambda client: client.stop())

    @app.post("/api/ros/estop", response_model=EstopStatusResponse)
    async def ros_estop(request: EstopRequest) -> EstopStatusResponse:
        client = RobotBridgeClient(current_settings())
        try:
            result = await client.set_estop(request.engaged)
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return EstopStatusResponse(
            ok=bool(result.get("ok", True)),
            engaged=bool(result.get("engaged", request.engaged)),
            message=str(result.get("message", "")),
        )

    @app.get("/api/ros/estop", response_model=EstopStatusResponse)
    async def ros_estop_status() -> EstopStatusResponse:
        client = RobotBridgeClient(current_settings())
        try:
            result = await client.get_estop()
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return EstopStatusResponse(
            ok=bool(result.get("ok", True)),
            engaged=bool(result.get("engaged", False)),
        )

    @app.post("/api/ros/goal_pose", response_model=RosActionResponse)
    async def goal_pose(request: Pose2DRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_goal_pose(request))

    @app.post("/api/ros/initial_pose", response_model=RosActionResponse)
    async def initial_pose(request: Pose2DRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_initial_pose(request))

    @app.get("/api/ros/map", response_model=RobotMapResponse)
    async def ros_map() -> RobotMapResponse:
        return await run_ros(lambda client: client.get_map())

    @app.get("/api/ros/costmap", response_model=RobotMapResponse)
    async def ros_costmap() -> RobotMapResponse:
        return await run_ros(lambda client: client.get_costmap())

    @app.get("/api/ros/pose", response_model=RobotPoseResponse)
    async def ros_pose() -> RobotPoseResponse:
        return await run_ros(lambda client: client.get_pose())

    @app.get("/api/ros/plan")
    async def ros_plan() -> dict:
        return await run_ros(lambda client: client.get_plan())

    @app.get("/api/ros/approach_pose")
    async def ros_approach_pose() -> dict:
        return await run_ros(lambda client: client.get_approach_pose())

    @app.get("/api/ros/image/topics", response_model=ImageTopicsResponse)
    async def ros_image_topics() -> ImageTopicsResponse:
        client = RobotBridgeClient(current_settings())
        try:
            return await client.list_image_topics()
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.get("/api/ros/image/frame")
    async def ros_image_frame(topic: str | None = None) -> Response:
        client = RobotBridgeClient(current_settings())
        try:
            payload, active = await client.fetch_image_frame(topic)
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        headers = {"X-Active-Topic": active or ""}
        if payload is None:
            return Response(status_code=204, headers=headers)
        return Response(content=payload, media_type="image/jpeg", headers=headers)


    @app.post("/api/ros/params/set", response_model=RosActionResponse)
    async def ros_params_set(request: ParamsSetRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.set_ros_parameters(request))

    @app.post("/api/ros/open_door/start", response_model=RosActionResponse)
    async def ros_open_door_start(request: OpenDoorGoalRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.open_door_start(request))

    @app.post("/api/ros/open_door/cancel", response_model=RosActionResponse)
    async def ros_open_door_cancel() -> RosActionResponse:
        return await run_ros(lambda client: client.open_door_cancel())

    @app.post("/api/ros/open_door/save_poses", response_model=RosActionResponse)
    async def ros_open_door_save_poses() -> RosActionResponse:
        return await run_ros(lambda client: client.open_door_save_poses())

    @app.post("/api/ros/open_door/step/{step}", response_model=RosActionResponse)
    async def ros_open_door_step(step: str) -> RosActionResponse:
        return await run_ros(lambda client: client.open_door_step(step))

    @app.get("/api/ros/open_door/status", response_model=OpenDoorStatusResponse)
    async def ros_open_door_status() -> OpenDoorStatusResponse:
        return await RobotBridgeClient(current_settings()).open_door_status()

    # Integrated door mission: drive to a waypoint, then run open_door.
    # Future UIs can drive the whole task with these three endpoints alone.
    @app.post("/api/ros/door_mission/start", response_model=RosActionResponse)
    async def ros_door_mission_start(request: DoorMissionStartRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.door_mission_start(request))

    @app.post("/api/ros/door_mission/cancel", response_model=RosActionResponse)
    async def ros_door_mission_cancel() -> RosActionResponse:
        return await run_ros(lambda client: client.door_mission_cancel())

    @app.get("/api/ros/door_mission/status", response_model=DoorMissionStatusResponse)
    async def ros_door_mission_status() -> DoorMissionStatusResponse:
        return await RobotBridgeClient(current_settings()).door_mission_status()

    @app.get("/api/ros/arm/temperatures", response_model=ArmTemperaturesResponse)
    async def ros_arm_temperatures() -> ArmTemperaturesResponse:
        return await run_ros(lambda client: client.get_arm_temperatures())

    @app.get("/api/host/battery", response_model=BatteryStatusResponse)
    def host_battery() -> BatteryStatusResponse:
        return read_host_battery()

    @app.post("/api/ros/arm/trajectory", response_model=RosActionResponse)
    async def ros_arm_trajectory(request: ArmTrajectoryRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_arm_trajectory(request))

    @app.post("/api/ros/nav/goal", response_model=RosActionResponse)
    async def ros_nav_goal(request: NavGoalRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.send_nav_goal(request))

    @app.post("/api/ros/nav/cancel", response_model=RosActionResponse)
    async def ros_nav_cancel() -> RosActionResponse:
        return await run_ros(lambda client: client.cancel_nav_goal())

    @app.post("/api/ros/search_retrieve/start", response_model=RosActionResponse)
    async def ros_search_retrieve_start(request: SearchRetrieveRequest) -> RosActionResponse:
        result = await run_ros(lambda client: client.start_search_retrieve(
            request.target_id, request.home_pose_x, request.home_pose_y, request.home_pose_yaw
        ))
        return RosActionResponse(
            ok=bool(result.get("ok", False)),
            action="search_retrieve_start",
            message=str(result.get("message", "search and retrieve task started"))
        )

    @app.post("/api/ros/search_retrieve/cancel", response_model=RosActionResponse)
    async def ros_search_retrieve_cancel() -> RosActionResponse:
        await run_ros(lambda client: client.cancel_search_retrieve())
        return RosActionResponse(ok=True, action="search_retrieve_cancel", message="search and retrieve task canceled")

    @app.get("/api/ros/search_retrieve/status", response_model=SearchRetrieveStatusResponse)
    async def ros_search_retrieve_status() -> SearchRetrieveStatusResponse:
        result = await run_ros(lambda client: client.get_search_retrieve_status())
        return SearchRetrieveStatusResponse(**result)

    @app.post("/api/ros/bridge_retrieve/start", response_model=RosActionResponse)
    async def ros_bridge_retrieve_start(request: BridgeRetrieveRequest) -> RosActionResponse:
        result = await run_ros(lambda client: client.start_bridge_retrieve(request))
        return RosActionResponse(
            ok=bool(result.get("ok")),
            action="bridge_retrieve_start",
            message=str(result.get("message", "bridge retrieve task started")),
        )

    @app.post("/api/ros/bridge_retrieve/cancel", response_model=RosActionResponse)
    async def ros_bridge_retrieve_cancel() -> RosActionResponse:
        result = await run_ros(lambda client: client.cancel_bridge_retrieve())
        return RosActionResponse(
            ok=bool(result.get("ok")),
            action="bridge_retrieve_cancel",
            message=str(result.get("message", "bridge retrieve task canceled")),
        )

    @app.get("/api/ros/bridge_retrieve/status", response_model=BridgeRetrieveStatusResponse)
    async def ros_bridge_retrieve_status() -> BridgeRetrieveStatusResponse:
        result = await run_ros(lambda client: client.get_bridge_retrieve_status())
        return BridgeRetrieveStatusResponse(**result)

    @app.post("/api/ros/bridge_traverse/start", response_model=RosActionResponse)
    async def ros_bridge_traverse_start(request: BridgeTraverseRequest) -> RosActionResponse:
        result = await run_ros(lambda client: client.start_bridge_traverse(request))
        return RosActionResponse(
            ok=bool(result.get("ok")),
            action="bridge_traverse_start",
            message=str(result.get("message", "bridge traverse task started")),
        )

    @app.post("/api/ros/bridge_traverse/cancel", response_model=RosActionResponse)
    async def ros_bridge_traverse_cancel() -> RosActionResponse:
        result = await run_ros(lambda client: client.cancel_bridge_traverse())
        return RosActionResponse(
            ok=bool(result.get("ok")),
            action="bridge_traverse_cancel",
            message=str(result.get("message", "bridge traverse task canceled")),
        )

    @app.get("/api/ros/bridge_traverse/status", response_model=BridgeTraverseStatusResponse)
    async def ros_bridge_traverse_status() -> BridgeTraverseStatusResponse:
        result = await run_ros(lambda client: client.get_bridge_traverse_status())
        return BridgeTraverseStatusResponse(**result)

    @app.post("/api/ros/arena_mission/start", response_model=RosActionResponse)
    async def ros_arena_mission_start() -> RosActionResponse:
        result = await run_ros(lambda client: client.start_arena_mission())
        return RosActionResponse(
            ok=bool(result.get("ok")),
            action="arena_mission_start",
            message=str(result.get("message", "arena mission started")),
        )

    @app.post("/api/ros/arena_mission/cancel", response_model=RosActionResponse)
    async def ros_arena_mission_cancel() -> RosActionResponse:
        result = await run_ros(lambda client: client.cancel_arena_mission())
        return RosActionResponse(
            ok=bool(result.get("ok")),
            action="arena_mission_cancel",
            message=str(result.get("message", "arena mission canceled")),
        )

    @app.get("/api/ros/arena_mission/status", response_model=ArenaMissionStatusResponse)
    async def ros_arena_mission_status() -> ArenaMissionStatusResponse:
        result = await run_ros(lambda client: client.get_arena_mission_status())
        return ArenaMissionStatusResponse(**result)

    @app.get("/api/ros/semantic_memory")
    async def ros_semantic_memory() -> dict:
        return await run_ros(lambda client: client.get_semantic_memory())

    @app.post("/api/ros/semantic_memory/clear")
    async def ros_semantic_memory_clear() -> dict:
        return await run_ros(lambda client: client.clear_semantic_memory())

    @app.post("/api/ros/costmap/clear", response_model=RosActionResponse)
    async def ros_clear_costmap(request: ClearCostmapRequest | None = None) -> RosActionResponse:
        target = request.target if request else "local"
        return await run_ros(lambda client: client.clear_costmap(target))

    @app.post("/api/ros/map/save", response_model=SaveMapResponse)
    async def ros_map_save(request: SaveMapRequest | None = None) -> SaveMapResponse:
        filename = request.filename if request else "arena_map"
        return await run_ros(lambda client: client.save_map(filename))

    @app.get("/api/ros/waypoints", response_model=WaypointListResponse)
    async def ros_waypoints() -> WaypointListResponse:
        client = RobotBridgeClient(current_settings())
        try:
            return await client.list_waypoints()
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post("/api/ros/waypoints/save", response_model=RosActionResponse)
    async def ros_waypoint_save(request: WaypointSaveRequest) -> RosActionResponse:
        return await run_ros(
            lambda client: client.save_waypoint(
                request.name, request.x, request.y, request.yaw
            )
        )

    @app.post("/api/ros/waypoints/delete", response_model=RosActionResponse)
    async def ros_waypoint_delete(request: WaypointNameRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.delete_waypoint(request.name))

    @app.post("/api/ros/waypoints/goto", response_model=RosActionResponse)
    async def ros_waypoint_goto(request: WaypointNameRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.goto_waypoint(request.name))

    @app.get("/api/ros/nav/status", response_model=NavStatusResponse)
    async def ros_nav_status() -> NavStatusResponse:
        client = RobotBridgeClient(current_settings())
        try:
            return await client.get_nav_status()
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.get("/api/ros/imu/calibration", response_model=ImuCalibrationStatusResponse)
    async def ros_imu_calibration() -> ImuCalibrationStatusResponse:
        client = RobotBridgeClient(current_settings())
        try:
            return await client.get_imu_calibration_status()
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post("/api/ros/imu/calibration/start", response_model=RosActionResponse)
    async def ros_imu_calibration_start() -> RosActionResponse:
        return await run_ros(lambda client: client.start_imu_calibration())

    @app.get("/api/ui-bridge/health", response_model=UiBridgeHealth)
    def ui_bridge_health() -> UiBridgeHealth:
        return read_ui_bridge_health(current_settings())

    return app


app = create_app()


def main() -> None:
    import os

    import uvicorn

    settings = load_settings()
    bind_host = os.environ.get("IGVI_HOST_BIND") or settings.host
    uvicorn.run(
        "igvi_host.server:create_app",
        host=bind_host,
        port=settings.port,
        factory=True,
        reload=False,
    )


if __name__ == "__main__":
    main()
