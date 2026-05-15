from __future__ import annotations

import json
from pathlib import Path

from .config import HostSettings
from .models import UiBridgeHealth


def read_ui_bridge_health(settings: HostSettings) -> UiBridgeHealth:
    health_file = Path(settings.shm_path) / "health.json"
    if not health_file.exists():
        return UiBridgeHealth(
            ok=False,
            path=str(health_file),
            stale=True,
            message="health.json not found",
        )

    try:
        payload = json.loads(health_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return UiBridgeHealth(
            ok=False,
            path=str(health_file),
            stale=True,
            message=f"failed to read health.json: {exc}",
        )

    stale = bool(payload.get("stale", False))
    return UiBridgeHealth(
        ok=not stale,
        path=str(health_file),
        stale=stale,
        message="stale" if stale else "fresh",
        payload=payload,
    )
