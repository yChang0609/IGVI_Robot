from __future__ import annotations

import threading

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.tasks.base_page import BaseTaskPage
from igvi_ui.tasks.state_machine import TaskStep, interruptible_sleep


class Task1MappingPage(BaseTaskPage):
    """Task 1 — SLAM Mapping: bring up sensors + SLAM, explore, save map."""

    ACCENT = "#3b82f6"  # blue
    TASK_NUM = "1"

    def __init__(self, client: HostClient) -> None:
        self.client = client
        super().__init__(
            title="SLAM 建圖",
            description="啟動感測器與 RTAB-Map，手動探索場地後儲存地圖",
        )

    def _make_steps(self) -> list[TaskStep]:
        client = self.client

        def start_sensors(interrupt: threading.Event) -> None:
            try:
                client.compose_action("up", profile="sensors")
            except HostClientError as exc:
                raise RuntimeError(f"sensors profile failed: {exc}") from exc
            interruptible_sleep(interrupt, 3.0)

        def start_slam(interrupt: threading.Event) -> None:
            try:
                client.compose_action("up", profile="slam")
            except HostClientError as exc:
                raise RuntimeError(f"slam profile failed: {exc}") from exc
            interruptible_sleep(interrupt, 5.0)

        def wait_map_init(interrupt: threading.Event) -> None:
            # Poll until map data is available or interrupted
            for _ in range(60):
                interruptible_sleep(interrupt, 1.0)
                try:
                    data = client.ros_map()
                    if data.get("ok") and data.get("width", 0) > 0:
                        return
                except Exception:
                    pass
            raise RuntimeError("Map did not initialise within 60 s")

        def explore_phase(interrupt: threading.Event) -> None:
            # Placeholder: user drives the robot; task waits until interrupted or timeout
            interruptible_sleep(interrupt, 120.0)

        def save_map(interrupt: threading.Event) -> None:
            try:
                result = client.save_map("arena_map")
                if not result.get("ok"):
                    raise RuntimeError(result.get("message", "save failed"))
            except HostClientError as exc:
                raise RuntimeError(str(exc)) from exc

        return [
            TaskStep("啟動感測器服務", "docker compose profile: sensors", start_sensors),
            TaskStep("啟動 SLAM 服務", "docker compose profile: slam", start_slam),
            TaskStep("等待地圖初始化", "確認 /map topic 有資料", wait_map_init),
            TaskStep("探索場地 (手動駕駛)", "可隨時按 Interrupt 跳過並儲存", explore_phase),
            TaskStep("儲存地圖", "呼叫 /api/ros/map/save → arena_map", save_map),
        ]
