from __future__ import annotations

from contextlib import suppress
from typing import Callable

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response

from .config import HostSettings, load_settings, save_settings
from .docker_clients import (
    ComposeProjectClient,
    DockerComposeError,
    DockerContainerNotFoundError,
    DockerEngineClient,
    DockerUnavailableError,
)
from .models import (
    ArmTrajectoryRequest,
    CmdVelRequest,
    ComposeActionRequest,
    ComposeActionResponse,
    ComposeProgressResponse,
    ContainerStatus,
    DevModeRequest,
    HealthResponse,
    ImageTopicsResponse,
    LogsResponse,
    Pose2DRequest,
    RobotMapResponse,
    RobotPoseResponse,
    RosActionResponse,
    RosConnectionResponse,
    RosServiceCallRequest,
    ServiceDescriptor,
    SettingsModel,
    UiBridgeHealth,
)
from .progress import get_progress_buffer
from .ros_control import RosbridgeClient
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

    async def run_ros(action: Callable[[RosbridgeClient], object]) -> RosActionResponse:
        client = RosbridgeClient(current_settings())
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
        registry().validate_service(service)
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
        return compose_or_http(lambda: compose_project().build(services=services, no_cache=False))

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

    @app.post("/api/compose/actions/restart", response_model=ComposeActionResponse)
    def restart(request: ComposeActionRequest) -> ComposeActionResponse:
        services = target_services(request)
        if not services:
            raise HTTPException(status_code=400, detail="restart requires at least one service")
        return compose_or_http(lambda: compose_project().restart(services=services))

    @app.post("/api/compose/actions/down", response_model=ComposeActionResponse)
    def down() -> ComposeActionResponse:
        return compose_or_http(lambda: compose_project().down())

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
        return await RosbridgeClient(current_settings()).ping()

    @app.post("/api/ros/cmd_vel", response_model=RosActionResponse)
    async def cmd_vel(request: CmdVelRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_cmd_vel(request))

    @app.post("/api/ros/stop", response_model=RosActionResponse)
    async def ros_stop() -> RosActionResponse:
        return await run_ros(lambda client: client.stop())

    @app.post("/api/ros/goal_pose", response_model=RosActionResponse)
    async def goal_pose(request: Pose2DRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_goal_pose(request))

    @app.post("/api/ros/initial_pose", response_model=RosActionResponse)
    async def initial_pose(request: Pose2DRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_initial_pose(request))

    @app.post("/api/ros/service_call", response_model=RosActionResponse)
    async def service_call(request: RosServiceCallRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.call_service(request))

    @app.get("/api/ros/map", response_model=RobotMapResponse)
    async def ros_map() -> RobotMapResponse:
        return await run_ros(lambda client: client.get_map())

    @app.get("/api/ros/pose", response_model=RobotPoseResponse)
    async def ros_pose() -> RobotPoseResponse:
        return await run_ros(lambda client: client.get_pose())

    @app.get("/api/ros/image/topics", response_model=ImageTopicsResponse)
    async def ros_image_topics() -> ImageTopicsResponse:
        client = RosbridgeClient(current_settings())
        try:
            return await client.list_image_topics()
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.get("/api/ros/image/frame")
    async def ros_image_frame(topic: str | None = None) -> Response:
        client = RosbridgeClient(current_settings())
        try:
            payload, active = await client.fetch_image_frame(topic)
        except Exception as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        headers = {"X-Active-Topic": active or ""}
        if payload is None:
            return Response(status_code=204, headers=headers)
        return Response(content=payload, media_type="image/jpeg", headers=headers)

    @app.post("/api/ros/arm/trajectory", response_model=RosActionResponse)
    async def ros_arm_trajectory(request: ArmTrajectoryRequest) -> RosActionResponse:
        return await run_ros(lambda client: client.publish_arm_trajectory(request))

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
