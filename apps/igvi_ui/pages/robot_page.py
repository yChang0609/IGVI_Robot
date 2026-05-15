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
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.map_views import Map2DView, Map3DView

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

            tick += 1
            self.msleep(200)

    def stop(self) -> None:
        self._running = False


class RobotPage(QWidget):
    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._poller: _RosPoller | None = None
        self._held_keys: set[Qt.Key] = set()
        self._drive_timer = QTimer(self)
        self._drive_timer.setInterval(100)
        self._drive_timer.timeout.connect(self._send_held_velocity)
        self._build_ui()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        map_panel = self._panel("Robot System")
        map_layout = QVBoxLayout(map_panel)
        self._add_heading(map_layout, "Robot System")
        self.map_2d = Map2DView()
        self.map_2d.goal_requested.connect(self._on_goal_clicked)
        self.pose_label = QLabel("Pose —")
        self.pose_label.setObjectName("Muted")
        map_layout.addWidget(self.map_2d, 1)
        map_layout.addWidget(self.pose_label)
        layout.addWidget(map_panel, 0, 0, 2, 2)

        view_panel = self._panel("3D Map")
        view_layout = QVBoxLayout(view_panel)
        self._add_heading(view_layout, "3D Map")
        view_layout.addWidget(Map3DView())
        layout.addWidget(view_panel, 0, 2)

        control_panel = self._panel("Control")
        control_layout = QVBoxLayout(control_panel)
        self._add_heading(control_layout, "Control  [WASD]")
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
        control_layout.addLayout(drive_grid)

        self.linear_slider = self._slider("Linear", control_layout, 36)
        self.angular_slider = self._slider("Angular", control_layout, 42)

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
        control_layout.addLayout(actions)
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
            self._poller.start()
        QCoreApplication.instance().installEventFilter(self)

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        QCoreApplication.instance().removeEventFilter(self)
        self._held_keys.clear()
        self._drive_timer.stop()
        self._stop()
        if self._poller:
            self._poller.stop()
            self._poller.wait(1000)
            self._poller = None

    # ── App-level event filter (catches keys regardless of focused widget) ─────

    def eventFilter(self, obj: QObject, event: QKeyEvent) -> bool:
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

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_pose(self, pose: dict) -> None:
        self.map_2d.update_pose(pose)
        self.pose_label.setText(
            f"Pose  x {pose['x']:.2f}  y {pose['y']:.2f}  yaw {pose['yaw']:.2f} rad"
        )

    def _on_goal_clicked(self, world_x: float, world_y: float) -> None:
        try:
            self.client.goal_pose(world_x, world_y, 0.0)
        except HostClientError as exc:
            QMessageBox.warning(self, "Goal failed", str(exc))

    def _clear_costmap(self) -> None:
        try:
            self.client.request(
                "POST",
                "/api/ros/service_call",
                {
                    "service": "/local_costmap/clear_entirely_local_costmap",
                    "service_type": "nav2_msgs/ClearEntireCostmap",
                    "args": {},
                },
            )
        except HostClientError as exc:
            QMessageBox.warning(self, "Clear costmap failed", str(exc))

    # ── Drive helpers ─────────────────────────────────────────────────────────

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

    # ── Widget helpers ────────────────────────────────────────────────────────

    def _panel(self, title: str) -> QFrame:
        panel = QFrame()
        panel.setObjectName("Panel")
        panel.setToolTip(title)
        return panel

    def _add_heading(self, layout: QVBoxLayout, title: str) -> None:
        layout.setContentsMargins(12, 12, 12, 12)
        heading = QLabel(title)
        heading.setObjectName("PanelTitle")
        layout.addWidget(heading)

    def _slider(self, label: str, parent_layout: QVBoxLayout, value: int) -> QSlider:
        row = QHBoxLayout()
        row.addWidget(QLabel(label))
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(value)
        row.addWidget(slider, 1)
        parent_layout.addLayout(row)
        return slider
