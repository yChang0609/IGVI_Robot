from __future__ import annotations

from pathlib import Path

from igvi_host.config import HostSettings


def test_compose_files_respect_dev_mode(tmp_path: Path) -> None:
    compose_dir = tmp_path / "docker" / "compose"
    compose_dir.mkdir(parents=True)
    (compose_dir / "compose.yaml").write_text("services: {}\n", encoding="utf-8")
    (compose_dir / "compose.dev.yaml").write_text(
        "services:\n  ros:\n    image: ros:latest\n", encoding="utf-8"
    )

    settings = HostSettings(repo_root=tmp_path, dev_mode=False)
    assert settings.compose_files == [compose_dir / "compose.yaml"]

    settings.dev_mode = True
    assert settings.compose_files == [compose_dir / "compose.yaml", compose_dir / "compose.dev.yaml"]


def test_compose_files_skip_empty_dev_override(tmp_path: Path) -> None:
    compose_dir = tmp_path / "docker" / "compose"
    compose_dir.mkdir(parents=True)
    (compose_dir / "compose.yaml").write_text("services: {}\n", encoding="utf-8")
    (compose_dir / "compose.dev.yaml").write_text("services: {}\n", encoding="utf-8")

    settings = HostSettings(repo_root=tmp_path, dev_mode=True)
    assert settings.compose_files == [compose_dir / "compose.yaml"]


def test_settings_round_trip(tmp_path: Path) -> None:
    settings = HostSettings(repo_root=tmp_path, dev_mode=True, rosbridge_url="ws://example:9090")
    restored = HostSettings.from_json_dict(settings.to_json_dict())
    assert restored.repo_root == tmp_path
    assert restored.dev_mode is True
    assert restored.rosbridge_url == "ws://example:9090"
