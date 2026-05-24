from __future__ import annotations

import json
import urllib.request

from .config import HostSettings
from .models import UiBridgeHealth


def read_ui_bridge_health(settings: HostSettings) -> UiBridgeHealth:
    """Report UI-bridge health by pinging the bridge's HTTP /api/health.

    The bridge node serves its liveness over HTTP (igvi_bridge bridge_node's
    snapshot_health). Earlier this read a /mnt/ui_bridge_shm/health.json file
    that nothing in the system ever writes, so the badge was permanently
    "stale". We now treat reachability of /api/health as the source of truth.
    """
    url = settings.bridge_url.rstrip("/") + "/api/health"
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001 — any failure means bridge is unreachable
        return UiBridgeHealth(
            ok=False,
            path=url,
            stale=True,
            message=f"bridge unreachable: {exc}",
        )

    ok = bool(payload.get("ok", False))
    return UiBridgeHealth(
        ok=ok,
        path=url,
        stale=not ok,
        message="connected" if ok else "bridge not ready",
        payload=payload if isinstance(payload, dict) else {},
    )
