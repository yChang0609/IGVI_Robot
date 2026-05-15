from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


def _has_services(path: Path) -> bool:
    try:
        import yaml
    except ImportError:
        return True
    try:
        doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return False
    services = doc.get("services") or {}
    return bool(services)


def find_repo_root(start: Path | None = None) -> Path:
    current = (start or Path(__file__)).resolve()
    for parent in (current, *current.parents):
        if (parent / "docker" / "compose" / "compose.yaml").exists():
            return parent
    return Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class ServiceSpec:
    name: str
    profile: str
    description: str


@dataclass
class HostSettings:
    repo_root: Path = field(default_factory=find_repo_root)
    project_name: str = "igvi_robot"
    host: str = "127.0.0.1"
    port: int = 8770
    rosbridge_url: str = "ws://127.0.0.1:9090"
    shm_path: Path = Path("/mnt/ui_bridge_shm")
    dev_mode: bool = False

    @property
    def apps_root(self) -> Path:
        return self.repo_root / "apps"

    @property
    def runtime_dir(self) -> Path:
        return self.apps_root / ".runtime"

    @property
    def settings_file(self) -> Path:
        return self.runtime_dir / "settings.json"

    @property
    def compose_file(self) -> Path:
        return self.repo_root / "docker" / "compose" / "compose.yaml"

    @property
    def compose_dev_file(self) -> Path:
        return self.repo_root / "docker" / "compose" / "compose.dev.yaml"

    @property
    def compose_files(self) -> list[Path]:
        files = [self.compose_file]
        if self.dev_mode and self.compose_dev_file.exists() and _has_services(self.compose_dev_file):
            files.append(self.compose_dev_file)
        return files

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "repo_root": str(self.repo_root),
            "project_name": self.project_name,
            "host": self.host,
            "port": self.port,
            "rosbridge_url": self.rosbridge_url,
            "shm_path": str(self.shm_path),
            "dev_mode": self.dev_mode,
        }

    @classmethod
    def from_json_dict(cls, data: dict[str, Any]) -> "HostSettings":
        return cls(
            repo_root=Path(data.get("repo_root") or find_repo_root()),
            project_name=str(data.get("project_name") or "igvi_robot"),
            host=str(data.get("host") or "127.0.0.1"),
            port=int(data.get("port") or 8770),
            rosbridge_url=str(data.get("rosbridge_url") or "ws://127.0.0.1:9090"),
            shm_path=Path(data.get("shm_path") or "/mnt/ui_bridge_shm"),
            dev_mode=bool(data.get("dev_mode", False)),
        )


def load_settings() -> HostSettings:
    settings = HostSettings()
    if not settings.settings_file.exists():
        return settings
    try:
        data = json.loads(settings.settings_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return settings
    return HostSettings.from_json_dict(data)


def save_settings(settings: HostSettings) -> None:
    settings.runtime_dir.mkdir(parents=True, exist_ok=True)
    settings.settings_file.write_text(
        json.dumps(settings.to_json_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
