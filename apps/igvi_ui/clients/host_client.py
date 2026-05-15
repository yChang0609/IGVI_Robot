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

    def down(self) -> dict[str, Any]:
        return self.request("POST", "/api/compose/actions/down", {}, timeout=self.compose_timeout)

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
    ) -> dict[str, Any]:
        return self.request(
            "POST",
            f"/api/compose/actions/{action}",
            {"service": service, "services": services, "profile": profile},
            timeout=self.compose_timeout,
        )

    def ros_stop(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/stop", {})

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
