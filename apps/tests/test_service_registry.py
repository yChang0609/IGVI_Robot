from __future__ import annotations

from pathlib import Path
from textwrap import dedent

import pytest

from igvi_host.service_registry import ServiceRegistry, discover_services


def _write_compose_tree(root: Path) -> Path:
    compose_dir = root / "docker" / "compose"
    services_dir = compose_dir / "services"
    services_dir.mkdir(parents=True)

    (compose_dir / "compose.yaml").write_text(
        dedent(
            """
            name: igvi_robot
            include:
              - services/rosbridge.yaml
              - services/camera_kinect.yaml
              - services/slam_fusion.yaml
            """
        ).strip(),
        encoding="utf-8",
    )
    (services_dir / "rosbridge.yaml").write_text(
        dedent(
            """
            services:
              rosbridge:
                profiles: ["bridge"]
                image: ros:rosbridge
            """
        ).strip(),
        encoding="utf-8",
    )
    (services_dir / "camera_kinect.yaml").write_text(
        dedent(
            """
            services:
              camera_kinect:
                profiles: ["sensors", "kinect"]
                image: wildbot/kinect
            """
        ).strip(),
        encoding="utf-8",
    )
    (services_dir / "slam_fusion.yaml").write_text(
        dedent(
            """
            services:
              slam_fusion:
                profiles: ["slam"]
                image: wildbot/slam
            """
        ).strip(),
        encoding="utf-8",
    )
    return compose_dir / "compose.yaml"


def test_discover_services_reads_included_files(tmp_path: Path) -> None:
    compose_file = _write_compose_tree(tmp_path)
    services = discover_services(compose_file)
    names = {s.name for s in services}
    assert names == {"rosbridge", "camera_kinect", "slam_fusion"}
    by_name = {s.name: s for s in services}
    assert by_name["camera_kinect"].profiles == ("sensors", "kinect")
    assert by_name["camera_kinect"].profile == "sensors"


def test_registry_validates_against_discovered_services(tmp_path: Path) -> None:
    compose_file = _write_compose_tree(tmp_path)
    registry = ServiceRegistry(compose_file)

    assert registry.validate_service("rosbridge") == "rosbridge"
    with pytest.raises(ValueError):
        registry.validate_service("not_real")

    assert registry.validate_services(["rosbridge", "rosbridge", "camera_kinect"]) == [
        "rosbridge",
        "camera_kinect",
    ]
    with pytest.raises(ValueError):
        registry.validate_services(["rosbridge", "ghost"])


def test_registry_validates_profiles(tmp_path: Path) -> None:
    compose_file = _write_compose_tree(tmp_path)
    registry = ServiceRegistry(compose_file)

    assert registry.validate_profile("slam") == "slam"
    assert registry.validate_profile(None) is None
    with pytest.raises(ValueError):
        registry.validate_profile("prod")

    assert registry.service_profile("camera_kinect") == "sensors"


def test_registry_handles_missing_compose_file(tmp_path: Path) -> None:
    registry = ServiceRegistry(tmp_path / "missing.yaml")
    assert registry.services() == ()
    assert registry.allowed_services() == frozenset()
