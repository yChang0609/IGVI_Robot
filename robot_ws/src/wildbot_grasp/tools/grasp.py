#!/usr/bin/env python3
"""Convenience runner for the IGVI bear grasp stack.

The user-facing command is Python, but it intentionally controls only the
Docker Compose/ROS containers needed by the bear grasp task.
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
ENV_FILE = ROOT / "docker" / "compose" / ".env"
COMPOSE_FILE = ROOT / "docker" / "compose" / "compose.yaml"

GPU_SERVICES = ["kros_car", "motion_arbiter", "camera_kinect", "eto_eye_gpu", "wildbot_grasp"]
CPU_SERVICES = ["kros_car", "motion_arbiter", "camera_kinect", "eto_eye_cpu", "wildbot_grasp"]


def command_text(cmd: list[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in cmd)


def run(cmd: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    print(f"+ {command_text(cmd)}")
    return subprocess.run(cmd, cwd=ROOT, text=True, check=check)


def compose(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    if not ENV_FILE.exists():
        raise SystemExit("docker/compose/.env 不存在，請先執行 make install")
    return run([
        "docker",
        "compose",
        "--env-file",
        str(ENV_FILE),
        "-f",
        str(COMPOSE_FILE),
        *args,
    ], check=check)


def service_set(cpu: bool) -> list[str]:
    return list(CPU_SERVICES if cpu else GPU_SERVICES)


def ros_exec(service: str, ros_command: str, *, retries: int = 8) -> None:
    setup = (
        "source /opt/ros/jazzy/setup.bash && "
        "{ source /workspace/install/setup.bash 2>/dev/null || true; } && "
    )
    command = setup + ros_command
    for attempt in range(1, retries + 1):
        result = compose("exec", service, "bash", "-lc", command, check=False)
        if result.returncode == 0:
            return
        if attempt == retries:
            raise subprocess.CalledProcessError(result.returncode, result.args)
        print(f"waiting for {service} to be ready before ROS exec ({attempt}/{retries})...")
        time.sleep(2.0)


def up(args: argparse.Namespace, *, force_recreate: bool = False) -> None:
    cmd = ["up", "-d", "--remove-orphans"]
    if args.build:
        cmd.append("--build")
    if force_recreate:
        cmd.append("--force-recreate")
    cmd.extend(service_set(args.cpu))
    compose(*cmd)


def start(args: argparse.Namespace) -> None:
    if not args.no_up:
        up(args)
    ros_exec("wildbot_grasp", "ros2 run wildbot_grasp gripper_command home")
    ros_exec(
        "wildbot_grasp",
        "ros2 service call /bear_grasp_task_node/start std_srvs/srv/Trigger '{}'",
    )


def stop(_args: argparse.Namespace) -> None:
    ros_exec(
        "wildbot_grasp",
        "ros2 service call /bear_grasp_task_node/stop std_srvs/srv/Trigger '{}'",
    )


def state(args: argparse.Namespace) -> None:
    suffix = "" if args.watch else " --once"
    ros_exec("wildbot_grasp", f"ros2 topic echo /bear_grasp/state{suffix}")


def check(_args: argparse.Namespace) -> None:
    compose("ps")
    ros_exec("wildbot_grasp", "ros2 param get /bear_grasp_task_node target_max_distance_m")
    ros_exec("wildbot_grasp", "ros2 param get /bear_grasp_task_node target_center_x_px")
    ros_exec("wildbot_grasp", "ros2 topic echo /arm_joint_temperatures --once")
    ros_exec("kros_car", "ros2 param get /controller_manager update_rate")
    ros_exec("wildbot_grasp", "ros2 topic info -v /arm_controller/joint_trajectory")


def logs(args: argparse.Namespace) -> None:
    tail = str(args.tail)
    run(["docker", "logs", f"--tail={tail}", "igvi_robot-wildbot_grasp-1"], check=False)
    run(["docker", "logs", f"--tail={tail}", "igvi_robot-kros_car-1"], check=False)


def gripper(args: argparse.Namespace) -> None:
    ros_exec("wildbot_grasp", f"ros2 run wildbot_grasp gripper_command {args.command}")


def recreate(args: argparse.Namespace) -> None:
    up(args, force_recreate=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run and inspect the IGVI bear grasp stack.")
    parser.add_argument("--cpu", action="store_true", help="use eto_eye_cpu instead of eto_eye_gpu")
    parser.add_argument("--build", action="store_true", help="build images before starting services")

    sub = parser.add_subparsers(dest="command_name", required=True)

    sub.add_parser("up", help="start the required containers").set_defaults(func=lambda a: up(a))
    sub.add_parser("recreate", help="force-recreate containers after .env/config changes").set_defaults(func=recreate)

    start_parser = sub.add_parser("start", help="start the bear grasp task")
    start_parser.add_argument("--no-up", action="store_true", help="do not run compose up before calling the ROS service")
    start_parser.set_defaults(func=start)

    sub.add_parser("stop", help="stop the bear grasp task").set_defaults(func=stop)

    state_parser = sub.add_parser("state", help="print bear grasp state")
    state_parser.add_argument("--watch", action="store_true", help="keep echoing state")
    state_parser.set_defaults(func=state)

    logs_parser = sub.add_parser("logs", help="show wildbot_grasp and kros_car logs")
    logs_parser.add_argument("--tail", type=int, default=120)
    logs_parser.set_defaults(func=logs)

    sub.add_parser("check", help="run the usual grasp diagnostics").set_defaults(func=check)

    gripper_parser = sub.add_parser("gripper", help="test only the gripper command")
    gripper_parser.add_argument("command", choices=["open", "close", "home", "place", "release", "carry", "hold", "grasp"])
    gripper_parser.set_defaults(func=gripper)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
    except subprocess.CalledProcessError as exc:
        return exc.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
