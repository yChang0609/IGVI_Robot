from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.map_views import Map2DView


DOOR_WAYPOINT_NAME = "door_approach"


class DoorMissionControl(QWidget):
    """Run the door mission: Nav2 waypoint approach, then open_door action."""

    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        self._door_pick_armed = False
        self._phase = "idle"
        self.map_2d.waypoint_point_picked.connect(self._on_door_point_picked)

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._poll_status)

        self._build_ui()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self._disarm_door_pick()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        waypoint_row = QHBoxLayout()
        waypoint_row.addWidget(QLabel("Door waypoint:"))
        self.waypoint_combo = QComboBox()
        waypoint_row.addWidget(self.waypoint_combo, 1)
        layout.addLayout(waypoint_row)

        grid = QGridLayout()
        grid.addWidget(QLabel("Door point:"), 0, 0)
        self.door_point_label = QLabel("not set")
        self.door_point_label.setObjectName("Muted")
        grid.addWidget(self.door_point_label, 0, 1)
        grid.addWidget(QLabel("Ready distance:"), 1, 0)
        self.ready_spin = QDoubleSpinBox()
        self.ready_spin.setRange(0.0, 2.0)
        self.ready_spin.setSingleStep(0.05)
        self.ready_spin.setDecimals(2)
        self.ready_spin.setValue(0.45)
        self.ready_spin.setSuffix(" m")
        grid.addWidget(self.ready_spin, 1, 1)
        layout.addLayout(grid)

        buttons = QGridLayout()
        buttons.setSpacing(8)
        self.pick_btn = QPushButton("Set Door Waypoint")
        self.pick_btn.setCheckable(True)
        self.pick_btn.clicked.connect(self._toggle_door_pick)
        self.start_btn = QPushButton("Start Door Mission")
        self.start_btn.setObjectName("Primary")
        self.start_btn.clicked.connect(self._start_task)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setObjectName("Danger")
        self.cancel_btn.clicked.connect(self._cancel_task)
        self.cancel_btn.setEnabled(False)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("Danger")
        self.stop_btn.clicked.connect(self._emergency_stop)
        buttons.addWidget(self.pick_btn, 0, 0, 1, 2)
        buttons.addWidget(self.start_btn, 1, 0)
        buttons.addWidget(self.cancel_btn, 1, 1)
        buttons.addWidget(self.stop_btn, 2, 0, 1, 2)
        layout.addLayout(buttons)

        self.status_label = QLabel("Ready")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        layout.addStretch(1)

        self.refresh_waypoints()

    def refresh_waypoints(self) -> None:
        try:
            waypoints = self.client.list_waypoints()
        except HostClientError as exc:
            self.status_label.setText(f"Bridge unavailable: {exc}")
            return

        self.map_2d.update_waypoints(waypoints)
        door_wp = waypoints.get(DOOR_WAYPOINT_NAME)
        if door_wp:
            self.door_point_label.setText(
                f"x {door_wp.get('x', 0.0):.2f}  y {door_wp.get('y', 0.0):.2f}  yaw {door_wp.get('yaw', 0.0):.2f}"
            )
        else:
            self.door_point_label.setText("not set")

        current = self.waypoint_combo.currentData()
        self.waypoint_combo.blockSignals(True)
        self.waypoint_combo.clear()
        restore_index = -1
        default_index = -1
        for i, name in enumerate(sorted(waypoints)):
            wp = waypoints[name]
            self.waypoint_combo.addItem(
                f"{name}  -  x {wp.get('x', 0):.2f}  y {wp.get('y', 0):.2f}",
                name,
            )
            if name == current:
                restore_index = i
            if name == DOOR_WAYPOINT_NAME:
                default_index = i
        if restore_index >= 0:
            self.waypoint_combo.setCurrentIndex(restore_index)
        elif default_index >= 0:
            self.waypoint_combo.setCurrentIndex(default_index)
        self.waypoint_combo.blockSignals(False)

    def _selected_waypoint(self) -> str:
        return str(self.waypoint_combo.currentData() or "")

    def _toggle_door_pick(self) -> None:
        if self.pick_btn.isChecked():
            self._door_pick_armed = True
            self.map_2d.set_pick_mode(True)
            self.pick_btn.setText("Click Map for Door...")
            self.status_label.setText("Click the map to save door_approach.")
        else:
            self._disarm_door_pick()

    def _disarm_door_pick(self) -> None:
        if self._door_pick_armed:
            self._door_pick_armed = False
            self.map_2d.set_pick_mode(False)
        self.pick_btn.setChecked(False)
        self.pick_btn.setText("Set Door Waypoint")

    def _on_door_point_picked(self, x: float, y: float, yaw: float) -> None:
        if not self._door_pick_armed:
            return
        self._disarm_door_pick()
        try:
            result = self.client.save_waypoint(DOOR_WAYPOINT_NAME, x, y, yaw)
        except HostClientError as exc:
            QMessageBox.warning(self, "Door waypoint failed", str(exc))
            return

        msg = str(result.get("message", ""))
        if result.get("ok"):
            self.status_label.setText(f"Door waypoint saved: {msg}")
            self.refresh_waypoints()
            self._select_waypoint(DOOR_WAYPOINT_NAME)
        else:
            QMessageBox.warning(self, "Door waypoint rejected", msg or "unknown error")

    def _select_waypoint(self, name: str) -> None:
        for index in range(self.waypoint_combo.count()):
            if self.waypoint_combo.itemData(index) == name:
                self.waypoint_combo.setCurrentIndex(index)
                return

    def _start_task(self) -> None:
        waypoint_name = self._selected_waypoint()
        if not waypoint_name:
            QMessageBox.warning(self, "Missing waypoint", "Please save a door_approach waypoint first.")
            return

        try:
            waypoints = self.client.list_waypoints()
            wp = waypoints.get(waypoint_name)
            if not wp:
                QMessageBox.warning(self, "Missing waypoint", f"No waypoint named '{waypoint_name}'.")
                return
            result = self.client.nav_goal(
                float(wp.get("x", 0.0)),
                float(wp.get("y", 0.0)),
                float(wp.get("yaw", 0.0)),
            )
        except HostClientError as exc:
            QMessageBox.warning(self, "Start failed", str(exc))
            return

        if not result.get("ok"):
            QMessageBox.warning(self, "Navigation rejected", str(result.get("message", "unknown error")))
            return

        self._phase = "navigation"
        self.start_btn.setEnabled(False)
        self.cancel_btn.setEnabled(True)
        self.status_label.setText(f"Navigating to {waypoint_name}...")
        self._status_timer.start()

    def _start_open_door(self) -> None:
        try:
            result = self.client.open_door_start(float(self.ready_spin.value()))
        except HostClientError as exc:
            self._finish_with_error(f"Open door failed: {exc}")
            return
        if not result.get("ok"):
            self._finish_with_error(str(result.get("message", "open_door rejected")))
            return
        self._phase = "open_door"
        self.status_label.setText("At door waypoint. Running open_door...")

    def _cancel_task(self) -> None:
        errors: list[str] = []
        for label, action in (
            ("open_door", self.client.open_door_cancel),
            ("navigation", self.client.nav_cancel),
        ):
            try:
                action()
            except HostClientError as exc:
                errors.append(f"{label}: {exc}")
        self.status_label.setText("Cancel requested." if not errors else "\n".join(errors))

    def _emergency_stop(self) -> None:
        errors: list[str] = []
        for label, action in (
            ("open_door", self.client.open_door_cancel),
            ("navigation", self.client.nav_cancel),
            ("robot stop", self.client.ros_stop),
        ):
            try:
                action()
            except HostClientError as exc:
                errors.append(f"{label}: {exc}")
        self._phase = "idle"
        self.start_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)
        self._status_timer.stop()
        if errors:
            QMessageBox.warning(self, "STOP partially failed", "\n".join(errors))
        else:
            self.status_label.setText("Emergency stop sent.")

    def _poll_status(self) -> None:
        if self._phase == "navigation":
            self._poll_navigation()
        elif self._phase == "open_door":
            self._poll_open_door()

    def _poll_navigation(self) -> None:
        try:
            status = self.client.nav_status()
        except HostClientError:
            return
        state = str(status.get("state", "idle"))
        msg = str(status.get("message", ""))
        fb = status.get("feedback", {}) or {}
        if fb:
            remaining = float(fb.get("distance_remaining", 0.0))
            self.status_label.setText(f"Navigation: {state}\nremaining {remaining:.2f} m\n{msg}")
        else:
            self.status_label.setText(f"Navigation: {state}\n{msg}")

        if state == "succeeded":
            self._start_open_door()
        elif state in {"aborted", "canceled", "failed", "rejected", "unavailable"}:
            self._finish_with_error(f"Navigation {state}: {msg}")

    def _poll_open_door(self) -> None:
        try:
            status = self.client.open_door_status()
        except HostClientError:
            return
        state = str(status.get("state", "idle"))
        stage = str(status.get("stage", ""))
        msg = str(status.get("message", ""))
        progress = float(status.get("progress", 0.0)) * 100.0
        self.status_label.setText(f"Open door: {state}\nProgress: {progress:.0f}%\n{stage}: {msg}")
        if state in {"succeeded", "canceled", "aborted", "error", "rejected", "unavailable"}:
            self._phase = "idle"
            self.start_btn.setEnabled(True)
            self.cancel_btn.setEnabled(False)
            self._status_timer.stop()

    def _finish_with_error(self, message: str) -> None:
        self._phase = "idle"
        self.start_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)
        self._status_timer.stop()
        self.status_label.setText(message)

    def shutdown(self) -> None:
        self._disarm_door_pick()
        self._status_timer.stop()
