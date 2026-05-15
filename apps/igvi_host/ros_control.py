from __future__ import annotations

import asyncio
import json
import urllib.request
from typing import Any

from .config import HostSettings
from .models import CmdVelRequest, Pose2DRequest, RobotMapResponse, RobotPoseResponse, RosActionResponse, RosConnectionResponse, RosServiceCallRequest


class RosbridgeClient:
    def __init__(self, settings: HostSettings):
        self.settings = settings

    def _bridge_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        url = self.settings.bridge_url.rstrip("/") + path
        data = json.dumps(payload).encode()
        req = urllib.request.Request(
            url, data=data, headers={"Content-Type": "application/json"}, method="POST"
        )
        with urllib.request.urlopen(req, timeout=2) as r:
            return json.loads(r.read())

    async def _send(self, payload: dict[str, Any]) -> None:
        try:
            import websockets  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Python package 'websockets' is not installed") from exc

        async with websockets.connect(self.settings.rosbridge_url, open_timeout=2) as websocket:
            await websocket.send(json.dumps(payload))

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


