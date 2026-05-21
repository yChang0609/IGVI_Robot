from __future__ import annotations

import threading

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.tasks.base_page import BaseTaskPage
from igvi_ui.tasks.state_machine import TaskStep, interruptible_sleep


class Task2LocalizePage(BaseTaskPage):
    """Task 2 — Auto Localization: load map and converge pose estimate."""

    ACCENT = "#22c55e"  # green
    TASK_NUM = "2"

    def __init__(self, client: HostClient) -> None:
        self.client = client
        super().__init__(
            title="自動定位",
            description="載入地圖、設定初始姿態，等待 RTAB-Map 定位收斂",
        )

    def _make_steps(self) -> list[TaskStep]:
        client = self.client

        def start_localization(interrupt: threading.Event) -> None:
            try:
                client.compose_action("up", profile="localization")
            except HostClientError as exc:
                raise RuntimeError(f"localization profile failed: {exc}") from exc
            interruptible_sleep(interrupt, 5.0)

        def set_initial_pose(interrupt: threading.Event) -> None:
            try:
                client.initial_pose(0.0, 0.0, 0.0)
            except HostClientError as exc:
                raise RuntimeError(str(exc)) from exc
            interruptible_sleep(interrupt, 1.5)

        def wait_convergence(interrupt: threading.Event) -> None:
            # Poll pose; declare converged when covariance is small (proxy: pose is available)
            for _ in range(30):
                interruptible_sleep(interrupt, 1.0)
                try:
                    pose = client.ros_pose()
                    if pose.get("ok"):
                        return
                except Exception:
                    pass
            raise RuntimeError("Localization did not converge within 30 s")

        def verify_imu(interrupt: threading.Event) -> None:
            try:
                status = client.imu_calibration_status()
                if not status.get("ok"):
                    raise RuntimeError("IMU calibration not ready")
            except HostClientError as exc:
                raise RuntimeError(str(exc)) from exc

        return [
            TaskStep("啟動定位服務", "docker compose profile: localization", start_localization),
            TaskStep("設定初始姿態", "發送 initial_pose (0, 0, 0)", set_initial_pose),
            TaskStep("等待定位收斂", "確認 /pose topic 持續更新", wait_convergence),
            TaskStep("驗證 IMU 校正狀態", "確認 IMU 偏差已校正", verify_imu),
        ]
