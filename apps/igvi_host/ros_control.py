from __future__ import annotations

import asyncio
import json
import urllib.parse
import urllib.request
from typing import Any

from .config import HostSettings
from .models import (
    ArmTrajectoryRequest,
    CmdVelRequest,
    ImageTopicsResponse,
    ImuCalibrationStatusResponse,
    NavGoalRequest,
    NavStatusResponse,
    Pose2DRequest,
    RobotMapResponse,
    RobotPoseResponse,
    RosActionResponse,
    RosConnectionResponse,
    RosServiceCallRequest,
    SaveMapResponse,
)


class RosbridgeClient:
    def __init__(self, settings: HostSettings):
        self.settings = settings

    def _bridge_post(self, path: str, payload: dict[str, Any], timeout: float = 2.0) -> dict[str, Any]:
        url = self.settings.bridge_url.rstrip("/") + path
        data = json.dumps(payload).encode()
        req = urllib.request.Request(
            url, data=data, headers={"Content-Type": "application/json"}, method="POST"
        )
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    async def _send(self, payload: dict[str, Any]) -> None:
        try:
            import websockets  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Python package 'websockets' is not installed") from exc

        async with websockets.connect(self.settings.rosbridge_url, open_timeout=2) as websocket:
            await websocket.send(json.dumps(payload))

    async def _publish_empty_topic(self, topic: str) -> None:
        try:
            import websockets  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Python package 'websockets' is not installed") from exc

        async with websockets.connect(self.settings.rosbridge_url, open_timeout=2) as websocket:
            await websocket.send(json.dumps({"op": "advertise", "topic": topic, "type": "std_msgs/Empty"}))
            await websocket.send(json.dumps({"op": "publish", "topic": topic, "msg": {}}))
            await asyncio.sleep(0.05)
            await websocket.send(json.dumps({"op": "unadvertise", "topic": topic}))

    async def _receive_string_topic(self, topic: str, timeout: float = 2.0) -> str:
        try:
            import websockets  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Python package 'websockets' is not installed") from exc

        async with websockets.connect(self.settings.rosbridge_url, open_timeout=2) as websocket:
            await websocket.send(json.dumps({"op": "subscribe", "topic": topic, "type": "std_msgs/String"}))
            try:
                while True:
                    raw = await asyncio.wait_for(websocket.recv(), timeout=timeout)
                    data = json.loads(raw)
                    if data.get("op") != "publish" or data.get("topic") != topic:
                        continue
                    msg = data.get("msg") or {}
                    return str(msg.get("data", ""))
            finally:
                await websocket.send(json.dumps({"op": "unsubscribe", "topic": topic}))

    async def ping(self) -> RosConnectionResponse:
        def _check() -> dict[str, Any]:
            url = self.settings.bridge_url.rstrip("/") + "/api/health"
            with urllib.request.urlopen(url, timeout=2) as r:
                return json.loads(r.read())
        try:
            result = await asyncio.to_thread(_check)
            if result.get("ok"):
                return RosConnectionResponse(ok=True, url=self.settings.bridge_url, message="connected")
        except Exception as exc:
            return RosConnectionResponse(ok=False, url=self.settings.bridge_url, message=str(exc))
        return RosConnectionResponse(ok=False, url=self.settings.bridge_url, message="bridge not ready")

    async def publish_cmd_vel(self, request: CmdVelRequest) -> RosActionResponse:
        await asyncio.to_thread(
            self._bridge_post, "/api/cmd_vel",
            {"linear_x": request.linear_x, "angular_z": request.angular_z},
        )
        return RosActionResponse(ok=True, action="cmd_vel", message="velocity command published")

    async def stop(self) -> RosActionResponse:
        await asyncio.to_thread(self._bridge_post, "/api/stop", {})
        return RosActionResponse(ok=True, action="stop", message="robot stopped")

    async def publish_goal_pose(self, request: Pose2DRequest) -> RosActionResponse:
        await asyncio.to_thread(
            self._bridge_post, "/api/goal_pose",
            {"x": request.x, "y": request.y, "yaw": request.yaw, "frame_id": request.frame_id},
        )
        return RosActionResponse(ok=True, action="goal_pose", message="goal pose published")

    async def publish_initial_pose(self, request: Pose2DRequest) -> RosActionResponse:
        await asyncio.to_thread(
            self._bridge_post, "/api/initial_pose",
            {"x": request.x, "y": request.y, "yaw": request.yaw, "frame_id": request.frame_id},
        )
        return RosActionResponse(ok=True, action="initial_pose", message="initial pose published")

    async def call_service(self, request: RosServiceCallRequest) -> RosActionResponse:
        payload = {
            "op": "call_service",
            "service": request.service,
            "type": request.service_type,
            "args": request.args,
        }
        await self._send(payload)
        return RosActionResponse(ok=True, action="service_call", message=f"service called: {request.service}")

    async def get_map(self) -> RobotMapResponse:
        return await asyncio.to_thread(self._fetch_map)

    def _fetch_map(self) -> RobotMapResponse:
        url = self.settings.bridge_url.rstrip("/") + "/api/map"
        try:
            with urllib.request.urlopen(url, timeout=3) as r:
                data = json.loads(r.read())
            if not data.get("width"):
                return RobotMapResponse(ok=False)
            return RobotMapResponse(ok=True, **{k: data[k] for k in ("width", "height", "resolution", "origin_x", "origin_y", "data") if k in data})
        except Exception as exc:
            raise RuntimeError(f"Bridge unavailable: {exc}") from exc

    async def get_pose(self) -> RobotPoseResponse:
        return await asyncio.to_thread(self._fetch_pose)

    def _fetch_pose(self) -> RobotPoseResponse:
        url = self.settings.bridge_url.rstrip("/") + "/api/pose"
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                data = json.loads(r.read())
            return RobotPoseResponse(ok=True, x=data.get("x", 0.0), y=data.get("y", 0.0), yaw=data.get("yaw", 0.0))
        except Exception as exc:
            raise RuntimeError(f"Bridge unavailable: {exc}") from exc

    async def list_image_topics(self) -> ImageTopicsResponse:
        return await asyncio.to_thread(self._fetch_image_topics)

    def _fetch_image_topics(self) -> ImageTopicsResponse:
        url = self.settings.bridge_url.rstrip("/") + "/api/image/topics"
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                data = json.loads(r.read())
            return ImageTopicsResponse(topics=list(data.get("topics", [])))
        except Exception as exc:
            raise RuntimeError(f"Bridge unavailable: {exc}") from exc

    async def fetch_image_frame(self, topic: str | None) -> tuple[bytes | None, str | None]:
        return await asyncio.to_thread(self._fetch_image_frame, topic)

    def _fetch_image_frame(self, topic: str | None) -> tuple[bytes | None, str | None]:
        url = self.settings.bridge_url.rstrip("/") + "/api/image/frame"
        if topic:
            url += f"?topic={urllib.parse.quote(topic)}"
        try:
            with urllib.request.urlopen(url, timeout=3) as r:
                active = r.headers.get("X-Active-Topic") or None
                if r.status == 204:
                    return None, active
                return r.read(), active
        except Exception as exc:
            raise RuntimeError(f"Bridge unavailable: {exc}") from exc

    async def publish_arm_trajectory(self, request: ArmTrajectoryRequest) -> RosActionResponse:
        await asyncio.to_thread(
            self._bridge_post, "/api/arm/trajectory",
            {"positions": request.positions, "time_from_start": request.time_from_start},
        )
        return RosActionResponse(ok=True, action="arm_trajectory", message="arm trajectory published")

    async def send_nav_goal(self, request: NavGoalRequest) -> RosActionResponse:
        # Bridge may block up to 3.5s in wait_for_server; allow some slack.
        result = await asyncio.to_thread(
            self._bridge_post, "/api/nav/goal",
            {"x": request.x, "y": request.y, "yaw": request.yaw},
            6.0,
        )
        return RosActionResponse(
            ok=bool(result.get("ok", False)),
            action="nav_goal",
            message=str(result.get("message", "")),
        )

    async def cancel_nav_goal(self) -> RosActionResponse:
        result = await asyncio.to_thread(self._bridge_post, "/api/nav/cancel", {}, 4.0)
        return RosActionResponse(
            ok=bool(result.get("ok", False)),
            action="nav_cancel",
            message=str(result.get("message", "")),
        )

    async def save_map(self, filename: str = "arena_map") -> SaveMapResponse:
        result = await asyncio.to_thread(
            self._bridge_post, "/api/map/save", {"filename": filename}, 10.0,
        )
        return SaveMapResponse(
            ok=bool(result.get("ok", False)),
            message=str(result.get("message", "")),
            path=str(result.get("message", "")) if result.get("ok") else "",
        )

    async def get_nav_status(self) -> NavStatusResponse:
        return await asyncio.to_thread(self._fetch_nav_status)

    def _fetch_nav_status(self) -> NavStatusResponse:
        url = self.settings.bridge_url.rstrip("/") + "/api/nav/status"
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                data = json.loads(r.read())
            return NavStatusResponse(
                state=str(data.get("state", "idle")),
                message=str(data.get("message", "")),
                server_ready=bool(data.get("server_ready", False)),
                goal=data.get("goal"),
                feedback=dict(data.get("feedback") or {}),
                visible_actions=list(data.get("visible_actions") or []),
            )
        except Exception as exc:
            raise RuntimeError(f"Bridge unavailable: {exc}") from exc

    async def get_imu_calibration_status(self) -> ImuCalibrationStatusResponse:
        try:
            raw = await self._receive_string_topic("/imu/calibration_state", timeout=2.0)
        except asyncio.TimeoutError:
            return ImuCalibrationStatusResponse(
                ok=False,
                state="unavailable",
                message="No /imu/calibration_state message received",
            )
        except Exception as exc:
            raise RuntimeError(f"Bridge unavailable: {exc}") from exc
        return _parse_imu_calibration_status(raw)

    async def start_imu_calibration(self) -> RosActionResponse:
        await self._publish_empty_topic("/imu/calibration/start")
        return RosActionResponse(
            ok=True,
            action="imu_calibration_start",
            message="IMU calibration window started",
        )


def _parse_imu_calibration_status(raw: str) -> ImuCalibrationStatusResponse:
    fields: dict[str, str] = {}
    for token in raw.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value

    state = fields.get("state", "unknown")
    gyro_bias: list[float] = []
    bias_text = fields.get("gyro_bias", "").strip("[]")
    if bias_text:
        try:
            gyro_bias = [float(item) for item in bias_text.split(",")]
        except ValueError:
            gyro_bias = []

    return ImuCalibrationStatusResponse(
        ok=state != "unknown",
        state=state,
        message=raw,
        stationary=_as_bool(fields.get("stationary")),
        converged=_as_bool(fields.get("converged")),
        online=_as_bool(fields.get("online")),
        manual_required=_as_bool(fields.get("manual_required")),
        manual_active=_as_bool(fields.get("manual_active")),
        calibration_active=_as_bool(fields.get("calibration_active")),
        manual_remaining_s=_as_float(fields.get("manual_remaining_s")),
        stationary_age_s=_as_float(fields.get("stationary_age_s")),
        convergence_age_s=_as_float(fields.get("convergence_age_s")),
        gyro_error_rad_s=_as_float(fields.get("gyro_error_rad_s")),
        gyro_bias=gyro_bias,
        raw=raw,
    )


def _as_bool(value: str | None) -> bool:
    return str(value).lower() == "true"


def _as_float(value: str | None) -> float:
    try:
        return float(value) if value is not None else 0.0
    except ValueError:
        return 0.0
