from __future__ import annotations

import asyncio
import json
import math
import urllib.request
from typing import Any

from .config import HostSettings
from .models import CmdVelRequest, Pose2DRequest, RobotMapResponse, RobotPoseResponse, RosActionResponse, RosConnectionResponse, RosServiceCallRequest


class RosbridgeClient:
    def __init__(self, settings: HostSettings):
        self.settings = settings

    async def _send(self, payload: dict[str, Any]) -> None:
        try:
            import websockets  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Python package 'websockets' is not installed") from exc

        async with websockets.connect(self.settings.rosbridge_url, open_timeout=2) as websocket:
            await websocket.send(json.dumps(payload))

    async def ping(self) -> RosConnectionResponse:
        try:
            await asyncio.wait_for(self._send({"op": "get_time", "id": "igvi_ping"}), timeout=3)
        except Exception as exc:
            return RosConnectionResponse(ok=False, url=self.settings.rosbridge_url, message=str(exc))
        return RosConnectionResponse(ok=True, url=self.settings.rosbridge_url, message="connected")

    async def publish_cmd_vel(self, request: CmdVelRequest) -> RosActionResponse:
        payload = {
            "op": "publish",
            "topic": "/cmd_vel",
            "type": "geometry_msgs/Twist",
            "msg": {
                "linear": {"x": request.linear_x, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": request.angular_z},
            },
        }
        await self._send(payload)
        return RosActionResponse(ok=True, action="cmd_vel", message="velocity command published")

    async def stop(self) -> RosActionResponse:
        return await self.publish_cmd_vel(CmdVelRequest(linear_x=0.0, angular_z=0.0))

    async def publish_goal_pose(self, request: Pose2DRequest) -> RosActionResponse:
        payload = {
            "op": "publish",
            "topic": "/goal_pose",
            "type": "geometry_msgs/PoseStamped",
            "msg": _pose_stamped(request),
        }
        await self._send(payload)
        return RosActionResponse(ok=True, action="goal_pose", message="goal pose published")

    async def publish_initial_pose(self, request: Pose2DRequest) -> RosActionResponse:
        pose = _pose_stamped(request)
        payload = {
            "op": "publish",
            "topic": "/initialpose",
            "type": "geometry_msgs/PoseWithCovarianceStamped",
            "msg": {
                "header": pose["header"],
                "pose": {
                    "pose": pose["pose"],
                    "covariance": [0.0] * 36,
                },
            },
        }
        await self._send(payload)
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


def _pose_stamped(request: Pose2DRequest) -> dict[str, Any]:
    half = request.yaw / 2.0
    return {
        "header": {"frame_id": request.frame_id},
        "pose": {
            "position": {"x": request.x, "y": request.y, "z": 0.0},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(half),
                "w": math.cos(half),
            },
        },
    }
