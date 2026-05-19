from __future__ import annotations

import os
import sys

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.pages.docker_page import DockerPage
from igvi_ui.pages.robot_page import RobotPage
from igvi_ui.pages.settings_page import SettingsPage
from igvi_ui.theme import STYLE_SHEET
from igvi_ui.widgets.status_badge import StatusBadge


class MainWindow(QMainWindow):
    def __init__(self, client: HostClient):
        super().__init__()
        self.client = client
        self.setWindowTitle(f"IGVI Robot Control Center  —  {client.base_url}")
        self.resize(1480, 900)
        self._build_ui()

        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(4000)
        self.refresh_health()

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        rail = QFrame()
        rail.setObjectName("Rail")
        rail.setFixedWidth(96)
        rail_layout = QVBoxLayout(rail)
        rail_layout.setContentsMargins(10, 14, 10, 14)
        rail_layout.setSpacing(10)
        brand = QLabel("IGVI")
        brand.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.setMinimumHeight(44)
        brand.setStyleSheet(
            "font-weight: 800; border: 1px solid #3a4656; border-radius: 8px; background: #18202b;"
        )
        rail_layout.addWidget(brand)

        self.stack = QStackedWidget()
        self.buttons: list[QPushButton] = []
        docker_page = DockerPage(self.client)
        docker_page.log_message.connect(self.set_status_message)
        pages = [
            ("Docker", docker_page),
            ("Robot", RobotPage(self.client)),
            ("Settings", SettingsPage(self.client)),
        ]
        for index, (label, page) in enumerate(pages):
            button = QPushButton(label)
            button.setObjectName("RailButton")
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, idx=index: self.set_page(idx))
            self.buttons.append(button)
            rail_layout.addWidget(button)
            self.stack.addWidget(page)
        rail_layout.addStretch(1)
        self.buttons[0].setChecked(True)

        workspace = QWidget()
        workspace_layout = QVBoxLayout(workspace)
        workspace_layout.setContentsMargins(0, 0, 0, 0)
        workspace_layout.setSpacing(0)

        topbar = QFrame()
        topbar.setObjectName("TopBar")
        topbar_layout = QHBoxLayout(topbar)
        topbar_layout.setContentsMargins(22, 12, 22, 12)
        title_box = QVBoxLayout()
        title = QLabel("IGVI Robot Control Center")
        title.setObjectName("Title")
        subtitle = QLabel("Docker Compose · IGVI bridge · UI bridge")
        subtitle.setObjectName("Muted")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        topbar_layout.addLayout(title_box, 1)
        self.host_badge = StatusBadge("Host Agent", "muted")
        self.docker_badge = StatusBadge("Docker", "muted")
        self.compose_badge = StatusBadge("Compose", "muted")
        self.bridge_badge = StatusBadge("UI Bridge", "muted")
        topbar_layout.addWidget(self.host_badge)
        topbar_layout.addWidget(self.docker_badge)
        topbar_layout.addWidget(self.compose_badge)
        topbar_layout.addWidget(self.bridge_badge)

        self.status_line = QLabel("Ready")
        self.status_line.setObjectName("Muted")
        self.status_line.setMinimumHeight(26)
        self.status_line.setContentsMargins(14, 0, 14, 6)

        workspace_layout.addWidget(topbar)
        workspace_layout.addWidget(self.stack, 1)
        workspace_layout.addWidget(self.status_line)

        root_layout.addWidget(rail)
        root_layout.addWidget(workspace, 1)
        self.setCentralWidget(root)

    def set_page(self, index: int) -> None:
        self.stack.setCurrentIndex(index)
        for button_index, button in enumerate(self.buttons):
            button.setChecked(button_index == index)

    def set_status_message(self, message: str) -> None:
        self.status_line.setText(message)

    def refresh_health(self) -> None:
        try:
            health = self.client.health()
            bridge = self.client.ui_bridge_health()
        except HostClientError as exc:
            self.host_badge.set_state("Host Agent: offline", "danger")
            self.docker_badge.set_state("Docker: unknown", "muted")
            self.compose_badge.set_state("Compose: unknown", "muted")
            self.bridge_badge.set_state("UI Bridge: unknown", "muted")
            self.status_line.setText(f"Host Agent unavailable: {exc}")
            return

        self.host_badge.set_state("Host Agent: ready", "ok")
        self.docker_badge.set_state(
            "Docker: ok" if health.get("docker_available") else "Docker: unavailable",
            "ok" if health.get("docker_available") else "warn",
        )
        self.compose_badge.set_state(
            "Compose: ok" if health.get("compose_available") else "Compose: unavailable",
            "ok" if health.get("compose_available") else "warn",
        )
        self.bridge_badge.set_state(
            "UI Bridge: ok" if bridge.get("ok") else "UI Bridge: stale",
            "ok" if bridge.get("ok") else "warn",
        )

    def shutdown(self) -> None:
        """Stop every background thread before the QApplication tears down.

        Without this, the currently-visible page's poller (and Docker page
        workers) are still running when Qt destroys them, which raises
        "QThread: Destroyed while thread is still running" and aborts with
        SIGABRT (exit 134). Idempotent so closeEvent and aboutToQuit can both
        call it.
        """
        self.health_timer.stop()
        for index in range(self.stack.count()):
            page = self.stack.widget(index)
            page_shutdown = getattr(page, "shutdown", None)
            if callable(page_shutdown):
                page_shutdown()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE_SHEET)
    base_url = os.environ.get("IGVI_HOST_URL", "http://127.0.0.1:8770")
    window = MainWindow(HostClient(base_url=base_url))
    # Safety net for paths that bypass closeEvent (app.quit(), signals).
    app.aboutToQuit.connect(window.shutdown)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
