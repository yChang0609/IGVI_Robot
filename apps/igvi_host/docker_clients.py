from __future__ import annotations

import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

from .config import HostSettings
from .models import ComposeActionResponse, ContainerStatus
from .progress import ProgressBuffer, get_progress_buffer
from .service_registry import ServiceRegistry


def _utcnow() -> datetime:
    return datetime.now().astimezone()


def _format_ports(attrs: dict[str, Any]) -> list[str]:
    ports = attrs.get("NetworkSettings", {}).get("Ports") or {}
    formatted: list[str] = []
    for container_port, bindings in ports.items():
        if not bindings:
            formatted.append(str(container_port))
            continue
        for binding in bindings:
            host_ip = binding.get("HostIp", "")
            host_port = binding.get("HostPort", "")
            formatted.append(f"{host_ip}:{host_port}->{container_port}")
    return formatted


class DockerUnavailableError(RuntimeError):
    pass


class DockerContainerNotFoundError(RuntimeError):
    pass


class DockerComposeError(RuntimeError):
    pass


def _format_compose_exception(action: str, exc: Exception) -> str:
    command = getattr(exc, "docker_command", None)
    return_code = getattr(exc, "return_code", None)
    stdout = getattr(exc, "stdout", None)
    stderr = getattr(exc, "stderr", None)

    parts = [f"Docker compose failed during {action}"]
    if return_code is not None:
        parts.append(f"exit code: {return_code}")
    if command:
        parts.append("command: " + " ".join(command))
    if stderr:
        parts.append("stderr: " + str(stderr).strip())
    if stdout:
        parts.append("stdout: " + str(stdout).strip())
    if len(parts) == 1:
        parts.append(str(exc))
    return "\n".join(parts)


class DockerEngineClient:
    """Thin wrapper around docker.from_env()."""

    def __init__(self, settings: HostSettings, registry: ServiceRegistry | None = None):
        self.settings = settings
        self.registry = registry or ServiceRegistry(settings.compose_file)
        try:
            import docker  # type: ignore
        except ImportError as exc:
            raise DockerUnavailableError("Python package 'docker' is not installed") from exc

        self._docker = docker
        try:
            self.client = docker.from_env()
        except Exception as exc:
            raise DockerUnavailableError(str(exc)) from exc

    def ping(self) -> bool:
        try:
            return bool(self.client.ping())
        except Exception:
            return False

    def list_project_containers(self) -> list[ContainerStatus]:
        label = f"com.docker.compose.project={self.settings.project_name}"
        try:
            containers = self.client.containers.list(all=True, filters={"label": label})
        except Exception as exc:
            raise DockerUnavailableError(str(exc)) from exc

        service_map = self.registry.service_map()
        statuses: list[ContainerStatus] = []
        seen: set[str] = set()
        for container in containers:
            attrs = container.attrs or {}
            labels = attrs.get("Config", {}).get("Labels") or {}
            service_name = labels.get("com.docker.compose.service")
            if not service_name:
                continue
            registered = service_map.get(service_name)
            seen.add(service_name)
            state = attrs.get("State") or {}
            health = state.get("Health", {}).get("Status")
            image = attrs.get("Config", {}).get("Image")
            statuses.append(
                ContainerStatus(
                    service=service_name,
                    profile=registered.profile if registered else None,
                    container_id=container.short_id,
                    container_name=container.name,
                    image=image,
                    status=container.status or "unknown",
                    health=health,
                    state=state.get("Status"),
                    created=attrs.get("Created"),
                    ports=_format_ports(attrs),
                )
            )

        for service in service_map.values():
            if service.name not in seen:
                statuses.append(
                    ContainerStatus(
                        service=service.name,
                        profile=service.profile,
                        status="not_created",
                    )
                )
        return sorted(statuses, key=lambda item: (item.profile or "", item.service))

    def has_project_container(self, service: str) -> bool:
        labels = [
            f"com.docker.compose.project={self.settings.project_name}",
            f"com.docker.compose.service={service}",
        ]
        try:
            containers = self.client.containers.list(all=True, filters={"label": labels})
        except Exception as exc:
            raise DockerUnavailableError(str(exc)) from exc
        return bool(containers)

    def _container_for_service(self, service: str):
        labels = [
            f"com.docker.compose.project={self.settings.project_name}",
            f"com.docker.compose.service={service}",
        ]
        containers = self.client.containers.list(all=True, filters={"label": labels})
        if not containers:
            raise DockerContainerNotFoundError(f"No container found for service: {service}")
        return containers[0]

    def get_logs(self, service: str, tail: int = 200) -> str:
        container = self._container_for_service(service)
        data = container.logs(tail=tail, timestamps=True)
        if isinstance(data, bytes):
            return data.decode("utf-8", errors="replace")
        return str(data)

    def get_stats(self, service: str) -> dict[str, Any]:
        container = self._container_for_service(service)
        return container.stats(stream=False)

    def container_action(self, action: str, services: list[str]) -> ComposeActionResponse:
        if action not in {"start", "stop", "restart"}:
            raise ValueError(f"Container action is not supported: {action}")
        if not services:
            raise ValueError(f"{action} requires at least one service")

        started_at = _utcnow()
        for service in services:
            container = self._container_for_service(service)
            if action == "start":
                container.start()
            elif action == "stop":
                container.stop()
            else:
                container.restart()

        past = {"start": "Started", "stop": "Stopped", "restart": "Restarted"}[action]
        return ComposeActionResponse(
            ok=True,
            action=action,
            service=services[0] if len(services) == 1 else None,
            services=services,
            backend="docker-engine",
            started_at=started_at,
            finished_at=_utcnow(),
            message=f"{past} {', '.join(services)}",
            events_tail=[],
        )


class ComposeProjectClient:
    """Thin wrapper around python_on_whales.DockerClient(...).compose."""

    def __init__(
        self,
        settings: HostSettings,
        registry: ServiceRegistry | None = None,
        progress: ProgressBuffer | None = None,
    ):
        self.settings = settings
        self.registry = registry or ServiceRegistry(settings.compose_file)
        self.progress = progress or get_progress_buffer()
        try:
            from python_on_whales import DockerClient  # type: ignore
            from python_on_whales.exceptions import DockerException  # type: ignore
        except ImportError as exc:
            raise DockerUnavailableError("Python package 'python-on-whales' is not installed") from exc

        self._docker_client_cls = DockerClient
        self._docker_exception_cls = DockerException
        compose_files = [str(path) for path in settings.compose_files if Path(path).exists()]
        self.client = self._docker_client_cls(
            compose_files=compose_files,
            compose_project_name=settings.project_name,
            compose_project_directory=settings.repo_root / "docker" / "compose",
        )

    def _client_for_profile(self, profile: str | None):
        if not profile:
            return self.client
        compose_files = [str(path) for path in self.settings.compose_files if Path(path).exists()]
        return self._docker_client_cls(
            compose_files=compose_files,
            compose_profiles=[profile],
            compose_project_name=self.settings.project_name,
            compose_project_directory=self.settings.repo_root / "docker" / "compose",
        )

    def ping(self) -> bool:
        try:
            self.client.version()
            return True
        except Exception:
            return False

    def _run_compose(self, action: str, callback):
        try:
            return callback()
        except self._docker_exception_cls as exc:
            raise DockerComposeError(_format_compose_exception(action, exc)) from exc
        except Exception as exc:
            raise DockerComposeError(f"Docker compose failed during {action}: {exc}") from exc

    def _normalize_services(self, service: str | None = None, services: list[str] | None = None) -> list[str] | None:
        if services:
            return self.registry.validate_services(services)
        validated = self.registry.validate_service(service)
        return [validated] if validated else None

    def _result(self, action: str, services: list[str] | None, started_at: datetime, message: str) -> ComposeActionResponse:
        return ComposeActionResponse(
            ok=True,
            action=action,
            service=services[0] if services and len(services) == 1 else None,
            services=services or [],
            backend="compose-library",
            started_at=started_at,
            finished_at=_utcnow(),
            message=message,
            events_tail=[],
        )

    def _compose_base_cmd(self, profile: str | None = None) -> list[str]:
        docker_bin = shutil.which("docker") or "docker"
        cmd: list[str] = [docker_bin, "compose", "--progress=plain"]
        for path in self.settings.compose_files:
            if Path(path).exists():
                cmd.extend(["--file", str(path)])
        cmd.extend(["--project-name", self.settings.project_name])
        cmd.extend([
            "--project-directory",
            str(self.settings.repo_root / "docker" / "compose"),
        ])
        if profile:
            cmd.extend(["--profile", profile])
        return cmd

    def _stream_subprocess(self, action: str, cmd: list[str]) -> None:
        self.progress.start(action)
        try:
            self._stream_into_active_progress(action, cmd)
        finally:
            self.progress.finish()

    def build(self, service: str | None = None, services: list[str] | None = None, no_cache: bool = False) -> ComposeActionResponse:
        target_services = self._normalize_services(service, services)
        started_at = _utcnow()
        cmd = self._compose_base_cmd()
        cmd.append("build")
        if no_cache:
            cmd.append("--no-cache")
        if target_services:
            cmd.extend(target_services)
        action_name = "rebuild" if no_cache else "build"
        self._stream_subprocess(action_name, cmd)
        return self._result(action_name, target_services, started_at, "Build finished")

    def up(
        self,
        service: str | None = None,
        services: list[str] | None = None,
        profile: str | None = None,
        detach: bool = True,
        force_recreate: bool = False,
    ) -> ComposeActionResponse:
        target_services = self._normalize_services(service, services)
        profile = self.registry.validate_profile(profile)
        started_at = _utcnow()
        cmd = self._compose_base_cmd(profile=profile)
        cmd.append("up")
        if detach:
            cmd.append("--detach")
        if force_recreate:
            cmd.append("--force-recreate")
        if target_services:
            cmd.extend(target_services)
        self._stream_subprocess("start", cmd)
        target = ", ".join(target_services) if target_services else profile or "project"
        return self._result("start", target_services, started_at, f"Started {target}")

    def rebuild(
        self,
        service: str | None = None,
        services: list[str] | None = None,
        profile: str | None = None,
    ) -> ComposeActionResponse:
        target_services = self._normalize_services(service, services)
        profile_validated = self.registry.validate_profile(profile)
        started_at = _utcnow()

        self.progress.start("rebuild")
        try:
            build_cmd = self._compose_base_cmd()
            build_cmd.extend(["build", "--no-cache"])
            if target_services:
                build_cmd.extend(target_services)
            self._stream_into_active_progress("rebuild:build", build_cmd)

            up_cmd = self._compose_base_cmd(profile=profile_validated)
            up_cmd.extend(["up", "--detach", "--force-recreate"])
            if target_services:
                up_cmd.extend(target_services)
            self._stream_into_active_progress("rebuild:up", up_cmd)
        finally:
            self.progress.finish()

        target = ", ".join(target_services) if target_services else profile_validated or "project"
        return self._result("rebuild", target_services, started_at, f"Rebuilt and started {target}")

    def _stream_into_active_progress(self, phase: str, cmd: list[str]) -> None:
        self.progress.append(f"--- {phase} ---")
        self.progress.append(f"$ {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                text=True,
            )
        except OSError as exc:
            self.progress.append(f"error: {exc}")
            raise DockerComposeError(
                f"Docker compose failed during {phase}: {exc}"
            ) from exc

        assert proc.stdout is not None
        for raw in proc.stdout:
            line = raw.rstrip()
            if line:
                self.progress.append(line)
        return_code = proc.wait()
        if return_code != 0:
            snapshot = self.progress.snapshot()
            tail = "\n".join(snapshot.lines[-30:])
            raise DockerComposeError(
                f"Docker compose failed during {phase}\n"
                f"exit code: {return_code}\n"
                f"command: {' '.join(cmd)}\n"
                f"output:\n{tail}"
            )

    def stop(self, service: str | None = None, services: list[str] | None = None) -> ComposeActionResponse:
        target_services = self._normalize_services(service, services)
        if not target_services:
            raise ValueError("stop requires at least one service")
        started_at = _utcnow()
        self._run_compose("stop", lambda: self.client.compose.stop(services=target_services))
        return self._result("stop", target_services, started_at, f"Stopped {', '.join(target_services)}")

    def restart(self, service: str | None = None, services: list[str] | None = None) -> ComposeActionResponse:
        target_services = self._normalize_services(service, services)
        if not target_services:
            raise ValueError("restart requires at least one service")
        started_at = _utcnow()
        self._run_compose("restart", lambda: self.client.compose.restart(services=target_services))
        return self._result("restart", target_services, started_at, f"Restarted {', '.join(target_services)}")

    def stop_all(self) -> ComposeActionResponse:
        started_at = _utcnow()
        cmd = self._compose_base_cmd()
        cmd.append("stop")
        self._stream_subprocess("stop_all", cmd)
        return self._result("stop_all", None, started_at, "All services stopped")

    def remove_all(self) -> ComposeActionResponse:
        started_at = _utcnow()
        cmd = self._compose_base_cmd()
        cmd.extend(["down", "--remove-orphans"])
        self._stream_subprocess("remove_all", cmd)
        return self._result("remove_all", None, started_at, "All containers removed")
