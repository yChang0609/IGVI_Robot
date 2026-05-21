from __future__ import annotations

from PySide6.QtCore import QCoreApplication, QObject, QThread, QTimer, Qt, Signal
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.arm_control import ArmControl
from igvi_ui.widgets.bridge_retrieve_control import BridgeRetrieveControl
from igvi_ui.widgets.image_view import ImageView
from igvi_ui.widgets.map_views import Map2DView
from igvi_ui.widgets.navigation_control import NavigationControl
from igvi_ui.widgets.waypoint_control import WaypointControl
from igvi_ui.widgets.search_retrieve_control import SearchRetrieveControl

_WASD: dict[Qt.Key, tuple[float, float]] = {
    Qt.Key.Key_W: (1.0, 0.0),
    Qt.Key.Key_S: (-1.0, 0.0),
    Qt.Key.Key_A: (0.0, 1.0),
    Qt.Key.Key_D: (0.0, -1.0),
}


class _RosPoller(QThread):
    """Background thread: polls /api/ros/pose every 200 ms, /api/ros/map every 3 s."""

    map_received = Signal(dict)
    pose_received = Signal(dict)
    semantic_memory_received = Signal(dict)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True

    def run(self) -> None:
        tick = 0
        while self._running:
            try:
                pose = self.client.ros_pose()
                if pose.get("ok"):
                    self.pose_received.emit(pose)
            except Exception:
                pass

            if tick % 15 == 0:
                try:
                    data = self.client.ros_map()
                    if data.get("ok"):
                        self.map_received.emit(data)
                except Exception:
                    pass

            if tick % 5 == 0:
                try:
                    data = self.client.semantic_memory()
                    self.semantic_memory_received.emit(data)
                except Exception:
                    pass

            tick += 1
            self.msleep(200)

    def stop(self) -> None:
        self._running = False


class _DriveControl(QWidget):
    """Drive sub-page: WASD/buttons → /cmd_vel via the bridge."""

    log_message = Signal(str)

    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        self._held_keys: set[Qt.Key] = set()
        self._keyboard_mode = False
        self._drive_timer = QTimer(self)
        self._drive_timer.setInterval(100)
        self._drive_timer.timeout.connect(self._send_held_velocity)
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        self._kb_btn = QPushButton("⌨  Keyboard Mode")
        self._kb_btn.setCheckable(True)
        self._kb_btn.clicked.connect(self._toggle_keyboard_mode)
        layout.addWidget(self._kb_btn)

        drive_grid = QGridLayout()
        drive_grid.setSpacing(8)
        forward = self._drive_button("▲ Forward", 1.0, 0.0)
        back = self._drive_button("▼ Back", -1.0, 0.0)
        left = self._drive_button("◄ Left", 0.0, 1.0)
        right = self._drive_button("Right ►", 0.0, -1.0)
        stop_btn = QPushButton("■ STOP")
        stop_btn.setObjectName("Danger")
        stop_btn.clicked.connect(self._stop)
        drive_grid.addWidget(forward, 0, 1)
        drive_grid.addWidget(left, 1, 0)
        drive_grid.addWidget(stop_btn, 1, 1)
        drive_grid.addWidget(right, 1, 2)
        drive_grid.addWidget(back, 2, 1)
        layout.addLayout(drive_grid)

        self.linear_slider = self._slider("Linear", layout, 36)
        self.angular_slider = self._slider("Angular", layout, 42)

        actions = QGridLayout()
        goal_btn = QPushButton("Set Goal")
        goal_btn.setObjectName("Primary")
        goal_btn.setToolTip("Click on the map to set a navigation goal")
        goal_btn.clicked.connect(lambda: QMessageBox.information(
            self, "Set Goal", "Click anywhere on the 2D map to send a Nav2 goal."
        ))
        initial_btn = QPushButton("Initial Pose")
        initial_btn.clicked.connect(lambda: self._pose_action("initial_pose"))
        clear_btn = QPushButton("Clear Costmap")
        clear_btn.clicked.connect(self._clear_costmap)
        estop_btn = QPushButton("E-Stop")
        estop_btn.setObjectName("Danger")
        estop_btn.clicked.connect(self._stop)
        actions.addWidget(goal_btn, 0, 0)
        actions.addWidget(initial_btn, 0, 1)
        actions.addWidget(clear_btn, 1, 0)
        actions.addWidget(estop_btn, 1, 1)
        layout.addLayout(actions)
        layout.addStretch(1)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        QCoreApplication.instance().installEventFilter(self)

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        QCoreApplication.instance().removeEventFilter(self)
        self._held_keys.clear()
        self._drive_timer.stop()
        self._stop()
        if self._keyboard_mode:
            self._toggle_keyboard_mode()

    # ── Keyboard mode ────────────────────────────────────────────────────────

    def _toggle_keyboard_mode(self) -> None:
        self._keyboard_mode = not self._keyboard_mode
        if self._keyboard_mode:
            self._kb_btn.setText("✕  Exit Keyboard Mode")
            self._kb_btn.setObjectName("Primary")
        else:
            self._kb_btn.setText("⌨  Keyboard Mode")
            self._kb_btn.setObjectName("")
            self._held_keys.clear()
            self._drive_timer.stop()
            self._stop()
        self._kb_btn.setChecked(self._keyboard_mode)
        self._kb_btn.style().unpolish(self._kb_btn)
        self._kb_btn.style().polish(self._kb_btn)
        self.map_2d.set_locked(self._keyboard_mode)

    def eventFilter(self, obj: QObject, event: QKeyEvent) -> bool:  # noqa: N802
        if not self._keyboard_mode:
            return False
        if event.type() == QKeyEvent.Type.KeyPress and not event.isAutoRepeat():
            key = Qt.Key(event.key())
            if key in _WASD and key not in self._held_keys:
                self._held_keys.add(key)
                self._send_held_velocity()
                if not self._drive_timer.isActive():
                    self._drive_timer.start()
                return True
        elif event.type() == QKeyEvent.Type.KeyRelease and not event.isAutoRepeat():
            key = Qt.Key(event.key())
            if key in _WASD:
                self._held_keys.discard(key)
                if not self._held_keys:
                    self._drive_timer.stop()
                    self._stop()
                return True
        return False

    def _send_held_velocity(self) -> None:
        if not self._held_keys:
            return
        linear = angular = 0.0
        for key in self._held_keys:
            l, a = _WASD[key]
            linear += l
            angular += a
        self._cmd_vel(linear, angular)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _drive_button(self, text: str, linear: float, angular: float) -> QPushButton:
        btn = QPushButton(text)
        btn.pressed.connect(lambda: self._cmd_vel(linear, angular))
        btn.released.connect(self._stop)
        return btn

    def _cmd_vel(self, linear: float, angular: float) -> None:
        lin_scale = max(self.linear_slider.value(), 1) / 100.0
        ang_scale = max(self.angular_slider.value(), 1) / 100.0
        try:
            self.client.cmd_vel(linear * lin_scale, angular * ang_scale)
        except HostClientError:
            pass

    def _stop(self) -> None:
        try:
            self.client.ros_stop()
        except HostClientError:
            pass

    def _pose_action(self, action: str) -> None:
        try:
            if action == "initial_pose":
                self.client.initial_pose(0.0, 0.0, 0.0)
        except HostClientError as exc:
            QMessageBox.warning(self, "Pose action failed", str(exc))

    def _clear_costmap(self) -> None:
        try:
            self.client.clear_costmap("local")
        except HostClientError as exc:
            QMessageBox.warning(self, "Clear costmap failed", str(exc))

    def _slider(self, label: str, parent_layout: QVBoxLayout, value: int) -> QSlider:
        row = QHBoxLayout()
        row.addWidget(QLabel(label))
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(value)
        row.addWidget(slider, 1)
        parent_layout.addLayout(row)
        return slider


class RobotPage(QWidget):
    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._poller: _RosPoller | None = None
        self._build_ui()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        map_panel = self._panel()
        map_layout = QVBoxLayout(map_panel)
        self._add_heading(map_layout, "Robot System")
        self.map_2d = Map2DView()
        self.map_2d.goal_requested.connect(self._on_goal_clicked)
        self.pose_label = QLabel("Pose —")
        self.pose_label.setObjectName("Muted")
        map_layout.addWidget(self.map_2d, 1)
        map_layout.addWidget(self.pose_label)
        layout.addWidget(map_panel, 0, 0, 2, 2)

        image_panel = self._panel()
        image_layout = QVBoxLayout(image_panel)
        self._add_heading(image_layout, "Camera")
        self.image_view = ImageView(self.client)
        image_layout.addWidget(self.image_view, 1)
        layout.addWidget(image_panel, 0, 2)

        control_panel = self._panel()
        control_layout = QVBoxLayout(control_panel)
        self._add_heading(control_layout, "Control")

        self.control_tabs = QTabWidget()
        self.drive_control = _DriveControl(self.client, self.map_2d)
        self.nav_control = NavigationControl(self.client, self.map_2d)
        self.waypoint_control = WaypointControl(self.client, self.map_2d)
        self.arm_control = ArmControl(self.client)
        self.search_retrieve_control = SearchRetrieveControl(self.client, self.map_2d)
        self.bridge_retrieve_control = BridgeRetrieveControl(self.client)
        
        self.control_tabs.addTab(self.drive_control, "Drive")
        self.control_tabs.addTab(self.nav_control, "Navigation")
        self.control_tabs.addTab(self.waypoint_control, "Waypoints")
        self.control_tabs.addTab(self.arm_control, "Arm")
        self.control_tabs.addTab(self.search_retrieve_control, "Search & Retrieve")
        self.control_tabs.addTab(self.bridge_retrieve_control, "Bridge Mission")
        control_layout.addWidget(self.control_tabs, 1)
        layout.addWidget(control_panel, 1, 2)

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 2)
        layout.setColumnStretch(2, 1)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._poller is None:
            self._poller = _RosPoller(self.client)
            self._poller.map_received.connect(self.map_2d.update_map)
            self._poller.pose_received.connect(self._on_pose)
            self._poller.semantic_memory_received.connect(self.search_retrieve_control.update_semantic_memory)
            self._poller.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self.shutdown()

    def shutdown(self) -> None:
        """Stop this page's poller and cascade to thread-owning children.

        Idempotent; called from hideEvent and from MainWindow.closeEvent so no
        QThread outlives the QApplication.
        """
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None
        self.image_view.shutdown()
        self.nav_control.shutdown()
        self.arm_control.shutdown()
        self.search_retrieve_control.shutdown()
        self.bridge_retrieve_control.shutdown()

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_pose(self, pose: dict) -> None:
        self.map_2d.update_pose(pose)
        self.pose_label.setText(
            f"Pose  x {pose['x']:.2f}  y {pose['y']:.2f}  yaw {pose['yaw']:.2f} rad"
        )

    def _on_goal_clicked(self, world_x: float, world_y: float, yaw: float) -> None:
        try:
            result = self.client.nav_goal(world_x, world_y, yaw)
        except HostClientError as exc:
            QMessageBox.warning(self, "Goal failed", str(exc))
            return
        if not result.get("ok"):
            QMessageBox.warning(
                self, "Goal rejected",
                str(result.get("message") or "Navigation server unavailable. Is the navigation profile up?"),
            )
            return
        # Auto-switch to Navigation tab so the user sees status feedback
        if hasattr(self, "control_tabs") and hasattr(self, "nav_control"):
            self.control_tabs.setCurrentWidget(self.nav_control)

    # ── Widget helpers ────────────────────────────────────────────────────────

    def _panel(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("Panel")
        return panel

    def _add_heading(self, layout: QVBoxLayout, title: str) -> None:
        layout.setContentsMargins(12, 12, 12, 12)
        heading = QLabel(title)
        heading.setObjectName("PanelTitle")
        layout.addWidget(heading)
