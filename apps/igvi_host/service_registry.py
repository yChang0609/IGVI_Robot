from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml


@dataclass(frozen=True)
class RegisteredService:
    name: str
    profile: str
    profiles: tuple[str, ...]
    description: str
    source: str


def _resolve_includes(compose_file: Path) -> list[Path]:
    if not compose_file.exists():
        return []
    try:
        root = yaml.safe_load(compose_file.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError:
        return []
    includes = root.get("include") or []
    paths: list[Path] = []
    base = compose_file.parent
    for entry in includes:
        if isinstance(entry, str):
            paths.append((base / entry).resolve())
        elif isinstance(entry, dict):
            path = entry.get("path")
            if isinstance(path, str):
                paths.append((base / path).resolve())
    return paths


def _load_services_from(yaml_path: Path) -> list[RegisteredService]:
    if not yaml_path.exists():
        return []
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError:
        return []
    services = document.get("services") or {}
    out: list[RegisteredService] = []
    for name, spec in services.items():
        if not isinstance(spec, dict):
            continue
        profiles = spec.get("profiles") or []
        if isinstance(profiles, str):
            profiles_tuple = (profiles,)
        else:
            profiles_tuple = tuple(str(p) for p in profiles)
        primary = profiles_tuple[0] if profiles_tuple else "default"
        out.append(
            RegisteredService(
                name=str(name),
                profile=primary,
                profiles=profiles_tuple,
                description=str(spec.get("x-description") or ""),
                source=str(yaml_path),
            )
        )
    return out


def discover_services(compose_file: Path) -> tuple[RegisteredService, ...]:
    seen: dict[str, RegisteredService] = {}
    for included in _resolve_includes(compose_file):
        for service in _load_services_from(included):
            if service.name not in seen:
                seen[service.name] = service
    # services defined directly in compose.yaml
    for service in _load_services_from(compose_file):
        if service.name not in seen:
            seen[service.name] = service
    return tuple(sorted(seen.values(), key=lambda s: (s.profile, s.name)))


def discover_profiles(services: Iterable[RegisteredService]) -> tuple[str, ...]:
    profiles: set[str] = set()
    for service in services:
        profiles.update(service.profiles or (service.profile,))
    return tuple(sorted(profiles))


class ServiceRegistry:
    """Live view over discovered compose services. Re-reads on every access."""

    def __init__(self, compose_file: Path):
        self.compose_file = compose_file

    def services(self) -> tuple[RegisteredService, ...]:
        return discover_services(self.compose_file)

    def service_map(self) -> dict[str, RegisteredService]:
        return {s.name: s for s in self.services()}

    def allowed_services(self) -> frozenset[str]:
        return frozenset(s.name for s in self.services())

    def allowed_profiles(self) -> frozenset[str]:
        return frozenset(discover_profiles(self.services()))

    def validate_service(self, service: str | None) -> str | None:
        if service is None or service == "":
            return None
        if service not in self.allowed_services():
            raise ValueError(f"Service is not allowed: {service}")
        return service

    def validate_services(self, services: list[str] | None) -> list[str] | None:
        if not services:
            return None
        allowed = self.allowed_services()
        validated: list[str] = []
        for service in services:
            if service is None or service == "":
                continue
            if service not in allowed:
                raise ValueError(f"Service is not allowed: {service}")
            if service not in validated:
                validated.append(service)
        return validated or None

    def validate_profile(self, profile: str | None) -> str | None:
        if profile is None or profile == "":
            return None
        if profile not in self.allowed_profiles():
            raise ValueError(f"Profile is not allowed: {profile}")
        return profile

    def service_profile(self, service: str) -> str:
        mapping = self.service_map()
        if service not in mapping:
            raise ValueError(f"Service is not allowed: {service}")
        return mapping[service].profile
