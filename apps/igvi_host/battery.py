from __future__ import annotations

import math
import platform
import re
import subprocess
from pathlib import Path

from .models import BatteryStatusResponse


def read_host_battery() -> BatteryStatusResponse:
    system = platform.system().lower()
    if system == "darwin":
        return _read_macos_battery()
    if system == "linux":
        return _read_linux_battery()
    return BatteryStatusResponse(
        ok=False,
        source="host",
        message=f"Host battery status is not supported on {platform.system() or 'this OS'}",
    )


def _read_macos_battery() -> BatteryStatusResponse:
    try:
        result = subprocess.run(
            ["pmset", "-g", "batt"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return BatteryStatusResponse(ok=False, source="host", message=str(exc))

    text = result.stdout.strip()
    match = re.search(r"(\d+)%;\s*([^;]+);", text)
    if not match:
        return BatteryStatusResponse(
            ok=False,
            source="host",
            message="No host battery was reported by pmset",
        )

    percentage = float(match.group(1))
    raw_status = match.group(2).strip().lower()
    status = _normalize_status(raw_status)
    return BatteryStatusResponse(
        ok=True,
        percentage=percentage,
        status=status,
        charging=status == "charging",
        present=True,
        source="host",
        message=text,
    )


def _read_linux_battery() -> BatteryStatusResponse:
    power_root = Path("/sys/class/power_supply")
    batteries = sorted(power_root.glob("BAT*"))
    if not batteries:
        return BatteryStatusResponse(
            ok=False,
            source="host",
            message="No host battery found in /sys/class/power_supply",
        )

    battery = batteries[0]
    percentage = _read_float_file(battery / "capacity")
    raw_status = _read_text_file(battery / "status").lower()
    status = _normalize_status(raw_status)
    present = _read_text_file(battery / "present") != "0"
    return BatteryStatusResponse(
        ok=percentage is not None,
        percentage=percentage,
        voltage=_read_scaled_float_file(battery / "voltage_now", 1_000_000.0),
        current=_read_scaled_float_file(battery / "current_now", 1_000_000.0),
        charge=_read_scaled_float_file(battery / "charge_now", 1_000_000.0)
        or _read_scaled_float_file(battery / "energy_now", 1_000_000.0),
        capacity=_read_scaled_float_file(battery / "charge_full", 1_000_000.0)
        or _read_scaled_float_file(battery / "energy_full", 1_000_000.0),
        status=status,
        charging=status == "charging",
        present=present,
        source="host",
        message=f"{battery.name}: {raw_status or 'unknown'}",
    )


def _read_text_file(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _read_float_file(path: Path) -> float | None:
    text = _read_text_file(path)
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _read_scaled_float_file(path: Path, scale: float) -> float | None:
    value = _read_float_file(path)
    if value is None:
        return None
    return value / scale


def _normalize_status(raw_status: str) -> str:
    status = raw_status.strip().replace(" ", "_").lower()
    if status in {"charged", "full"}:
        return "full"
    if status in {"charging", "discharging", "not_charging"}:
        return status
    return status or "unknown"
