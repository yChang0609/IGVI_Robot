from __future__ import annotations

from PySide6.QtCore import Qt
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


class RobotPage(QWidget):
    def __init__(self, client: HostClient):
        super().__init__()
        self.client = client
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        map_panel = self._panel("Robot System")
        map_layout = QVBoxLayout(map_panel)
        self._add_heading(map_layout, "Robot System")
        self.map_2d = Map2DView()
        metrics = QHBoxLayout()
        for label in ("Pose x 2.42 y -0.83", "Velocity 0.18 m/s", "Map seq 1842", "Goal warehouse B"):
            value = QLabel(label)
            value.setObjectName("Muted")
            metrics.addWidget(value)
        map_layout.addWidget(self.map_2d, 1)
        map_layout.addLayout(metrics)
        layout.addWidget(map_panel, 0, 0, 2, 2)

        view_panel = self._panel("3D Map")
        view_layout = QVBoxLayout(view_panel)
        self._add_heading(view_layout, "3D Map")
        view_layout.addWidget(Map3DView())
        layout.addWidget(view_panel, 0, 2)

        control_panel = self._panel("Control")
        control_layout = QVBoxLayout(control_panel)
        self._add_heading(control_layout, "Control")
        drive_grid = QGridLayout()
        drive_grid.setSpacing(8)
        forward = self._drive_button("Forward", 0.25, 0.0)
        back = self._drive_button("Back", -0.18, 0.0)
        left = self._drive_button("Turn Left", 0.0, 0.45)
        right = self._drive_button("Turn Right", 0.0, -0.45)
        stop = QPushButton("STOP")
        stop.setObjectName("Danger")
        stop.clicked.connect(self._stop)
        drive_grid.addWidget(forward, 0, 1)
        drive_grid.addWidget(left, 1, 0)
        drive_grid.addWidget(stop, 1, 1)
        drive_grid.addWidget(right, 1, 2)
        drive_grid.addWidget(back, 2, 1)
        control_layout.addLayout(drive_grid)

        self.linear_slider = self._slider("Linear", control_layout, 36)
        self.angular_slider = self._slider("Angular", control_layout, 42)

        actions = QGridLayout()
        goal = QPushButton("Set Goal")
        goal.setObjectName("Primary")
        goal.clicked.connect(lambda: self._pose_action("goal_pose"))
        initial = QPushButton("Initial Pose")
        initial.clicked.connect(lambda: self._pose_action("initial_pose"))
        clear = QPushButton("Clear Costmap")
        clear.clicked.connect(lambda: QMessageBox.information(self, "Pending", "Clear costmap service call will be wired after Nav2 is stable."))
        estop = QPushButton("E-Stop")
        estop.setObjectName("Danger")
        estop.clicked.connect(self._stop)
        actions.addWidget(goal, 0, 0)
        actions.addWidget(initial, 0, 1)
        actions.addWidget(clear, 1, 0)
        actions.addWidget(estop, 1, 1)
        control_layout.addLayout(actions)
        layout.addWidget(control_panel, 1, 2)

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 2)
        layout.setColumnStretch(2, 1)

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

    def _drive_button(self, text: str, linear: float, angular: float) -> QPushButton:
        button = QPushButton(text)
        button.pressed.connect(lambda: self._cmd_vel(linear, angular))
        button.released.connect(self._stop)
        return button

    def _slider(self, label: str, parent_layout: QVBoxLayout, value: int) -> QSlider:
        row = QHBoxLayout()
        row.addWidget(QLabel(label))
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(value)
        row.addWidget(slider, 1)
        parent_layout.addLayout(row)
        return slider

    def _cmd_vel(self, linear: float, angular: float) -> None:
        linear_scale = max(self.linear_slider.value(), 1) / 100.0
        angular_scale = max(self.angular_slider.value(), 1) / 100.0
        try:
            self.client.cmd_vel(linear * linear_scale, angular * angular_scale)
        except HostClientError as exc:
            QMessageBox.warning(self, "Command failed", str(exc))

    def _stop(self) -> None:
        try:
            self.client.ros_stop()
        except HostClientError as exc:
            QMessageBox.warning(self, "Stop failed", str(exc))

    def _pose_action(self, action: str) -> None:
        try:
            if action == "goal_pose":
                self.client.goal_pose(2.0, -0.8, 0.0)
            else:
                self.client.initial_pose(0.0, 0.0, 0.0)
        except HostClientError as exc:
            QMessageBox.warning(self, "Pose action failed", str(exc))
