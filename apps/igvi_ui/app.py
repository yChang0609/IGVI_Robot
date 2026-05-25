from __future__ import annotations

import os
import sys
from ctypes import CDLL


def _linux_qt_platform_hint() -> str:
    platform = os.environ.get("QT_QPA_PLATFORM", "").strip()
    if platform:
        return platform.split(":", 1)[0].lower()
    return "xcb"


def _ensure_linux_qt_runtime() -> None:
    if not sys.platform.startswith("linux"):
        return

    platform = _linux_qt_platform_hint()
    if platform in {"xcb", "wayland"}:
        display_var = "WAYLAND_DISPLAY" if platform == "wayland" else "DISPLAY"
        if not os.environ.get(display_var):
            print(
                f"igvi-ui: Qt platform {platform!r} needs ${display_var}, but it is not set.\n"
                "Run the UI from a graphical desktop session, enable X11/Wayland forwarding, "
                "or set QT_QPA_PLATFORM=offscreen only for non-interactive smoke tests.",
                file=sys.stderr,
            )
            sys.exit(1)

    if platform == "xcb":
        try:
            CDLL("libxcb-cursor.so.0")
        except OSError:
            print(
                "igvi-ui: Qt xcb support needs libxcb-cursor.so.0.\n"
                "Install it with:\n"
                "  sudo apt update && sudo apt install -y libxcb-cursor0",
                file=sys.stderr,
            )
            sys.exit(1)


_ensure_linux_qt_runtime()

# Ensure Qt can find its platform plugins (needed on macOS with uv/venv)
if sys.platform == "darwin" and "QT_QPA_PLATFORM_PLUGIN_PATH" not in os.environ:
    import PySide6 as _p6
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(os.path.dirname(_p6.__file__), "Qt", "plugins", "platforms")
    del _p6

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
from igvi_ui.pages.door_page import DoorPage
from igvi_ui.pages.robot_page import RobotPage
from igvi_ui.pages.settings_page import SettingsPage
from igvi_ui.theme import build_style_sheet
from igvi_ui.widgets.approach_tuning_control import ApproachTuningControl
from igvi_ui.widgets.sensor_group import SensorGroup
from igvi_ui.widgets.status_badge import StatusBadge


class MainWindow(QMainWindow):
    def __init__(self, client: HostClient):
        super().__init__()
        self.client = client
        self.setWindowTitle(f"IGVI Robot Control Center  —  {client.base_url}")
        self.setMinimumWidth(900)
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
        door_page = DoorPage(self.client)
        door_page.log_message.connect(self.set_status_message)
        pages = [
            ("Docker", docker_page),
            ("Robot", RobotPage(self.client)),
            ("Door", door_page),
            ("Approach", ApproachTuningControl(self.client)),
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
        topbar.setFixedHeight(74)
        topbar_layout = QHBoxLayout(topbar)
        topbar_layout.setContentsMargins(22, 10, 22, 10)
        title_box = QVBoxLayout()
        title = QLabel("IGVI Robot Control Center")
        title.setObjectName("Title")
        subtitle = QLabel("Docker Compose · IGVI bridge · UI bridge")
        subtitle.setObjectName("Muted")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        topbar_layout.addLayout(title_box)
        topbar_layout.addStretch(1)

        # Status bar order: Sensors → Host Agent → Dock → Compose → UI Bridge
        # → Battery → Emergency Stop.
        self.sensor_group = SensorGroup()
        self.host_badge = StatusBadge("Host Agent", "muted")
        self.docker_badge = StatusBadge("Docker", "muted")
        self.compose_badge = StatusBadge("Compose", "muted")
        self.bridge_badge = StatusBadge("UI Bridge", "muted")
        self.battery_badge = StatusBadge("Battery: unknown", "muted")
        self.estop_btn = QPushButton("■ EMERGENCY STOP")
        self.estop_btn.setCheckable(True)
        self.estop_btn.setFixedHeight(30)
        self.estop_btn.setFixedWidth(180)
        self.estop_btn.setToolTip("Stop the base and arm immediately. Click again to release.")
        self.estop_btn.clicked.connect(self._toggle_estop)
        self.estop_btn.setStyleSheet(self._ESTOP_NORMAL)
        topbar_layout.addWidget(self.sensor_group)
        topbar_layout.addWidget(self.host_badge)
        topbar_layout.addWidget(self.docker_badge)
        topbar_layout.addWidget(self.compose_badge)
        topbar_layout.addWidget(self.bridge_badge)
        topbar_layout.addWidget(self.battery_badge)
        topbar_layout.addWidget(self.estop_btn)

        self.status_line = QLabel("Ready")
        self.status_line.setObjectName("Muted")
        self.status_line.setFixedHeight(26)
        self.status_line.setContentsMargins(14, 0, 14, 4)

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
            self.battery_badge.set_state("Battery: unknown", "muted")
            self.sensor_group.update_sources({}, available=False)
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
        bridge_ok = bool(bridge.get("ok"))
        self.bridge_badge.set_state(
            "UI Bridge: ok" if bridge_ok else "UI Bridge: stale",
            "ok" if bridge_ok else "warn",
        )
        fusion_sources = (bridge.get("payload") or {}).get("fusion_sources") or {}
        self.sensor_group.update_sources(fusion_sources, available=bridge_ok)
        self._refresh_battery_badge()
        self._refresh_estop_button()

    def _refresh_estop_button(self) -> None:
        try:
            status = self.client.estop_status()
        except HostClientError:
            return
        self._apply_estop_button(bool(status.get("engaged", False)))

    _ESTOP_NORMAL = (
        "QPushButton { background: #14432a; color: #86efac; border: 1px solid #166534;"
        " border-radius: 7px; font-weight: 600; }"
        "QPushButton:hover { background: #166534; }"
    )
    _ESTOP_ENGAGED = (
        "QPushButton { background: #7f1d1d; color: #fecaca; border: 1px solid #991b1b;"
        " border-radius: 7px; font-weight: 700; }"
        "QPushButton:hover { background: #991b1b; }"
    )

    def _apply_estop_button(self, engaged: bool) -> None:
        self.estop_btn.blockSignals(True)
        self.estop_btn.setChecked(engaged)
        self.estop_btn.blockSignals(False)
        self.estop_btn.setText("⚠ E-STOP ENGAGED" if engaged else "■ EMERGENCY STOP")
        self.estop_btn.setStyleSheet(self._ESTOP_ENGAGED if engaged else self._ESTOP_NORMAL)

    def _toggle_estop(self) -> None:
        desired = self.estop_btn.isChecked()
        try:
            result = self.client.estop_set(desired)
        except HostClientError as exc:
            self._apply_estop_button(not desired)  # revert; request didn't land
            self.status_line.setText(f"E-stop failed: {exc}")
            return
        engaged = bool(result.get("engaged", desired))
        self._apply_estop_button(engaged)
        self.status_line.setText(
            "EMERGENCY STOP ENGAGED — base and arm halted" if engaged
            else "Emergency stop released"
        )

    def _refresh_battery_badge(self) -> None:
        try:
            battery = self.client.battery_status()
        except HostClientError:
            self.battery_badge.set_state("Battery: unknown", "muted")
            return

        if not battery.get("ok"):
            self.battery_badge.set_state("Battery: unknown", "muted")
            return

        percentage = battery.get("percentage")
        try:
            percent_text = f"{float(percentage):.0f}%"
            percent_value = float(percentage)
        except (TypeError, ValueError):
            percent_text = "--%"
            percent_value = 100.0

        charging = bool(battery.get("charging"))
        charge_text = "charging" if charging else "not charging"
        if charging:
            state = "accent"
        elif percent_value <= 20.0:
            state = "danger"
        elif percent_value <= 35.0:
            state = "warn"
        else:
            state = "ok"
        self.battery_badge.set_state(f"Battery: {percent_text} · {charge_text}", state)

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

    screen = app.primaryScreen()
    avail = screen.availableGeometry()
    # Scale font from 11px (narrow) to 16px (wide) relative to 1600px reference width.
    base_px = max(11, min(16, round(avail.width() / 123)))
    app.setStyleSheet(build_style_sheet(base_px))

    base_url = os.environ.get("IGVI_HOST_URL", "http://127.0.0.1:8770")
    window = MainWindow(HostClient(base_url=base_url))
    # Safety net for paths that bypass closeEvent (app.quit(), signals).
    app.aboutToQuit.connect(window.shutdown)
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
