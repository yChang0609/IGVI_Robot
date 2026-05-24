from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any


class HostClientError(RuntimeError):
    pass


@dataclass
class HostClient:
    base_url: str = "http://127.0.0.1:8770"
    timeout: float = 2.5
    compose_timeout: float = 3600.0

    def _url(self, path: str, params: dict[str, Any] | None = None) -> str:
        url = self.base_url.rstrip("/") + path
        if params:
            url += "?" + urllib.parse.urlencode(params)
        return url

    def request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
        params: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> Any:
        data = None
        headers = {"accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["content-type"] = "application/json"

        request = urllib.request.Request(
            self._url(path, params=params),
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                body = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise HostClientError(f"{exc.code}: {detail}") from exc
        except OSError as exc:
            raise HostClientError(str(exc)) from exc

        if not body:
            return None
        return json.loads(body)

    def health(self) -> dict[str, Any]:
        return self.request("GET", "/api/health")

    def settings(self) -> dict[str, Any]:
        return self.request("GET", "/api/settings")

    def set_dev_mode(self, enabled: bool) -> dict[str, Any]:
        return self.request("POST", "/api/compose/dev-mode", {"enabled": enabled})

    def services(self) -> list[dict[str, Any]]:
        return self.request("GET", "/api/compose/services")

    def registry(self) -> list[dict[str, Any]]:
        return self.request("GET", "/api/compose/registry")

    def profiles(self) -> list[str]:
        return self.request("GET", "/api/compose/profiles")

    def stop_all(self) -> dict[str, Any]:
        return self.request("POST", "/api/compose/actions/stop_all", {}, timeout=self.compose_timeout)

    def remove_all(self) -> dict[str, Any]:
        return self.request("POST", "/api/compose/actions/remove_all", {}, timeout=self.compose_timeout)

    def logs(self, service: str, tail: int = 200) -> str:
        response = self.request("GET", f"/api/compose/services/{service}/logs", params={"tail": tail})
        return str(response.get("logs", ""))

    def compose_progress(self, tail: int = 50) -> dict[str, Any]:
        return self.request("GET", "/api/compose/progress", params={"tail": tail})

    def compose_action(
        self,
        action: str,
        service: str | None = None,
        services: list[str] | None = None,
        profile: str | None = None,
        no_cache: bool = False,
    ) -> dict[str, Any]:
        return self.request(
            "POST",
            f"/api/compose/actions/{action}",
            {"service": service, "services": services, "profile": profile, "no_cache": no_cache},
            timeout=self.compose_timeout,
        )

    def ros_stop(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/stop", {})

    def estop_set(self, engaged: bool) -> dict[str, Any]:
        return self.request("POST", "/api/ros/estop", {"engaged": engaged})

    def estop_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/estop")

    def cmd_vel(self, linear_x: float, angular_z: float) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/cmd_vel",
            {"linear_x": linear_x, "angular_z": angular_z},
        )

    def goal_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return self.request("POST", "/api/ros/goal_pose", {"x": x, "y": y, "yaw": yaw})

    def initial_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return self.request("POST", "/api/ros/initial_pose", {"x": x, "y": y, "yaw": yaw})

    def ui_bridge_health(self) -> dict[str, Any]:
        return self.request("GET", "/api/ui-bridge/health")

    def ros_map(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/map", timeout=5.0)

    def ros_costmap(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/costmap", timeout=3.0)

    def ros_pose(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/pose")

    def ros_plan(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/plan")

    def ros_approach_pose(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/approach_pose")

    def image_topics(self) -> list[str]:
        data = self.request("GET", "/api/ros/image/topics", timeout=3.0)
        return list((data or {}).get("topics", []))

    def image_frame(self, topic: str | None) -> tuple[bytes | None, str | None]:
        params = {"topic": topic} if topic else None
        request = urllib.request.Request(
            self._url("/api/ros/image/frame", params=params),
            headers={"accept": "image/jpeg"},
            method="GET",
        )
        try:
            with urllib.request.urlopen(request, timeout=3.0) as response:
                active = response.headers.get("X-Active-Topic") or None
                if response.status == 204:
                    return None, active
                return response.read(), active
        except urllib.error.HTTPError as exc:
            raise HostClientError(f"{exc.code}: image frame fetch failed") from exc
        except OSError as exc:
            raise HostClientError(str(exc)) from exc


    def set_params(self, node: str, params: dict[str, Any]) -> dict[str, Any]:
        return self.request(
            "POST", "/api/ros/params/set", {"node": node, "params": params}
        )

    def open_door_start(self, ready_distance_m: float = 0.0) -> dict[str, Any]:
        return self.request(
            "POST", "/api/ros/open_door/start", {"ready_distance_m": ready_distance_m}
        )

    def open_door_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/open_door/cancel", {})

    def open_door_save_poses(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/open_door/save_poses", {}, timeout=6.0)

    def open_door_step(self, step: str) -> dict[str, Any]:
        # run_press / run_push / go_home block server-side; allow the push its
        # full duration plus headroom before the HTTP call gives up.
        return self.request(
            "POST", f"/api/ros/open_door/step/{step}", {}, timeout=40.0
        )

    def open_door_status(self, timeout: float = 2.0) -> dict[str, Any]:
        return self.request("GET", "/api/ros/open_door/status", timeout=timeout)

    # ── Door mission: integrated nav → open_door (single task) ──────────────
    # The surface a UI uses to fire the whole door task at once. `start`
    # dispatches nav to the named waypoint (default "door_approach"); when nav
    # succeeds the bridge automatically kicks off open_door. Poll `status` for
    # phase = idle | navigating | opening | succeeded | failed | canceled.

    def door_mission_start(
        self, waypoint: str = "door_approach", ready_distance_m: float = 0.0
    ) -> dict[str, Any]:
        return self.request(
            "POST", "/api/ros/door_mission/start",
            {"waypoint": waypoint, "ready_distance_m": ready_distance_m},
            timeout=6.0,
        )

    def door_mission_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/door_mission/cancel", {}, timeout=4.0)

    def door_mission_status(self, timeout: float = 2.0) -> dict[str, Any]:
        return self.request("GET", "/api/ros/door_mission/status", timeout=timeout)

    def arm_temperatures(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/arm/temperatures")

    def battery_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/host/battery")

    def arm_trajectory(self, positions: list[float], time_from_start: float = 0.3) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/arm/trajectory",
            {"positions": positions, "time_from_start": time_from_start},
        )

    def nav_goal(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return self.request("POST", "/api/ros/nav/goal", {"x": x, "y": y, "yaw": yaw}, timeout=8.0)

    def nav_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/nav/cancel", {}, timeout=5.0)

    def clear_costmap(self, target: str = "local") -> dict[str, Any]:
        return self.request("POST", "/api/ros/costmap/clear", {"target": target}, timeout=5.0)

    def nav_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/nav/status")

    def search_retrieve_start(self, target_id: str, home_pose_x: float, home_pose_y: float, home_pose_yaw: float = 0.0) -> dict[str, Any]:
        return self.request(
            "POST", 
            "/api/ros/search_retrieve/start", 
            {"target_id": target_id, "home_pose_x": home_pose_x, "home_pose_y": home_pose_y, "home_pose_yaw": home_pose_yaw}, 
            timeout=8.0
        )

    def search_retrieve_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/search_retrieve/cancel", {}, timeout=5.0)

    def search_retrieve_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/search_retrieve/status")

    def bridge_retrieve_start(self, bridge_waypoint_name: str, target_class: str = "xiong_qiao") -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/bridge_retrieve/start",
            {"bridge_waypoint_name": bridge_waypoint_name, "target_class": target_class},
            timeout=8.0,
        )

    def bridge_retrieve_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/bridge_retrieve/cancel", {}, timeout=5.0)

    def bridge_retrieve_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/bridge_retrieve/status")

    def arena_mission_start(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/arena_mission/start", {}, timeout=8.0)

    def arena_mission_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/arena_mission/cancel", {}, timeout=5.0)

    def arena_mission_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/arena_mission/status")

    def semantic_memory(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/semantic_memory", timeout=3.0)

    def semantic_memory_clear(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/semantic_memory/clear", {}, timeout=5.0)

    def imu_calibration_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/imu/calibration", timeout=3.0)

    def start_imu_calibration(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/imu/calibration/start", {}, timeout=3.0)

    def save_map(self, filename: str = "arena_map") -> dict[str, Any]:
        return self.request("POST", "/api/ros/map/save", {"filename": filename}, timeout=10.0)

    def list_waypoints(self) -> dict[str, Any]:
        data = self.request("GET", "/api/ros/waypoints", timeout=3.0)
        return dict((data or {}).get("waypoints", {}))

    def save_waypoint(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float = 0.0,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {"name": name, "yaw": yaw}
        if x is not None and y is not None:
            payload["x"] = x
            payload["y"] = y
        return self.request("POST", "/api/ros/waypoints/save", payload, timeout=5.0)

    def delete_waypoint(self, name: str) -> dict[str, Any]:
        return self.request("POST", "/api/ros/waypoints/delete", {"name": name}, timeout=5.0)

    def goto_waypoint(self, name: str) -> dict[str, Any]:
        return self.request("POST", "/api/ros/waypoints/goto", {"name": name}, timeout=8.0)

    def calibration(self) -> dict[str, Any]:
        return self.request("GET", "/api/calibration")

    def set_calibration(self, data: dict[str, Any]) -> dict[str, Any]:
        return self.request("POST", "/api/calibration", data)
