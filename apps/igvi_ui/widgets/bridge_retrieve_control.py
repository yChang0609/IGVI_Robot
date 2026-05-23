from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QComboBox,
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


HOME_WAYPOINT_NAME = "home"
BRIDGE_WAYPOINT_NAME = "bridge_center"
DOOR_REF_WAYPOINT_NAME = "door_ref"


class BridgeRetrieveControl(QWidget):
    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        self._bridge_pick_armed = False
        self._door_pick_armed = False
        self.map_2d.waypoint_point_picked.connect(self._on_point_picked)
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._poll_status)
        self._build_ui()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self._disarm_bridge_pick()
        self._disarm_door_pick()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        waypoint_row = QHBoxLayout()
        waypoint_row.addWidget(QLabel("Bridge waypoint:"))
        self.waypoint_combo = QComboBox()
        waypoint_row.addWidget(self.waypoint_combo, 1)
        layout.addLayout(waypoint_row)

        target_grid = QGridLayout()
        target_grid.addWidget(QLabel("YOLO target:"), 0, 0)
        self.target_label = QLabel("xiong_qiao")
        self.target_label.setObjectName("Muted")
        target_grid.addWidget(self.target_label, 0, 1)
        target_grid.addWidget(QLabel("Bridge point:"), 1, 0)
        self.bridge_point_label = QLabel("not set")
        self.bridge_point_label.setObjectName("Muted")
        target_grid.addWidget(self.bridge_point_label, 1, 1)
        target_grid.addWidget(QLabel("Door ref point:"), 2, 0)
        self.door_ref_label = QLabel("not set")
        self.door_ref_label.setObjectName("Muted")
        target_grid.addWidget(self.door_ref_label, 2, 1)
        layout.addLayout(target_grid)

        buttons = QGridLayout()
        buttons.setSpacing(8)
        self.home_btn = QPushButton("Home")
        self.home_btn.clicked.connect(self._save_home)
        self.bridge_pick_btn = QPushButton("Set Bridge Waypoint")
        self.bridge_pick_btn.setCheckable(True)
        self.bridge_pick_btn.clicked.connect(self._toggle_bridge_pick)
        self.door_pick_btn = QPushButton("Set Door Ref Point")
        self.door_pick_btn.setCheckable(True)
        self.door_pick_btn.clicked.connect(self._toggle_door_pick)
        self.start_btn = QPushButton("Start Bridge Mission")
        self.start_btn.setObjectName("Primary")
        self.start_btn.clicked.connect(self._start_task)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setObjectName("Danger")
        self.cancel_btn.clicked.connect(self._cancel_task)
        self.cancel_btn.setEnabled(False)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("Danger")
        self.stop_btn.clicked.connect(self._emergency_stop)
        buttons.addWidget(self.home_btn, 0, 0, 1, 2)
        buttons.addWidget(self.bridge_pick_btn, 1, 0, 1, 2)
        buttons.addWidget(self.door_pick_btn, 2, 0, 1, 2)
        buttons.addWidget(self.start_btn, 3, 0)
        buttons.addWidget(self.cancel_btn, 3, 1)
        buttons.addWidget(self.stop_btn, 4, 0, 1, 2)
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
        bridge_wp = waypoints.get(BRIDGE_WAYPOINT_NAME)
        if bridge_wp:
            self.bridge_point_label.setText(
                f"x {bridge_wp.get('x', 0.0):.2f}  y {bridge_wp.get('y', 0.0):.2f}  yaw {bridge_wp.get('yaw', 0.0):.2f}"
            )
        else:
            self.bridge_point_label.setText("not set")

        door_wp = waypoints.get(DOOR_REF_WAYPOINT_NAME)
        if door_wp:
            self.door_ref_label.setText(
                f"x {door_wp.get('x', 0.0):.2f}  y {door_wp.get('y', 0.0):.2f}"
            )
        else:
            self.door_ref_label.setText("not set")

        current = self.waypoint_combo.currentData()
        self.waypoint_combo.blockSignals(True)
        self.waypoint_combo.clear()
        restore_index = -1
        default_index = -1
        for i, name in enumerate(sorted(waypoints)):
            wp = waypoints[name]
            self.waypoint_combo.addItem(
                f"{name}  ·  x {wp.get('x', 0):.2f}  y {wp.get('y', 0):.2f}",
                name,
            )
            if name == current:
                restore_index = i
            if name == "bridge_center":
                default_index = i
        if restore_index >= 0:
            self.waypoint_combo.setCurrentIndex(restore_index)
        elif default_index >= 0:
            self.waypoint_combo.setCurrentIndex(default_index)
        self.waypoint_combo.blockSignals(False)

    def _selected_waypoint(self) -> str:
        value = self.waypoint_combo.currentData()
        return str(value or "")

    def _save_home(self) -> None:
        try:
            result = self.client.save_waypoint(HOME_WAYPOINT_NAME)
        except HostClientError as exc:
            QMessageBox.warning(self, "Home failed", str(exc))
            return

        msg = str(result.get("message", ""))
        if result.get("ok"):
            self.status_label.setText(f"Home saved: {msg}")
            self.refresh_waypoints()
        else:
            QMessageBox.warning(self, "Home rejected", msg or "unknown error")

    def _toggle_bridge_pick(self) -> None:
        if self.bridge_pick_btn.isChecked():
            self._disarm_door_pick()
            self._bridge_pick_armed = True
            self.map_2d.set_pick_mode(True)
            self.bridge_pick_btn.setText("Click Map for Bridge...")
            self.status_label.setText("Click the map to save bridge_center.")
        else:
            self._disarm_bridge_pick()

    def _disarm_bridge_pick(self) -> None:
        if self._bridge_pick_armed:
            self._bridge_pick_armed = False
            self.map_2d.set_pick_mode(False)
        self.bridge_pick_btn.setChecked(False)
        self.bridge_pick_btn.setText("Set Bridge Waypoint")

    def _toggle_door_pick(self) -> None:
        if self.door_pick_btn.isChecked():
            self._disarm_bridge_pick()
            self._door_pick_armed = True
            self.map_2d.set_pick_mode(True)
            self.door_pick_btn.setText("Click Map for Door Ref...")
            self.status_label.setText("Click the map to save door_ref (direction only).")
        else:
            self._disarm_door_pick()

    def _disarm_door_pick(self) -> None:
        if self._door_pick_armed:
            self._door_pick_armed = False
            self.map_2d.set_pick_mode(False)
        self.door_pick_btn.setChecked(False)
        self.door_pick_btn.setText("Set Door Ref Point")

    def _on_point_picked(self, x: float, y: float, yaw: float) -> None:
        if self._door_pick_armed:
            self._disarm_door_pick()
            self._save_picked_waypoint(
                DOOR_REF_WAYPOINT_NAME, x, y, yaw, "Door ref", select=False
            )
        elif self._bridge_pick_armed:
            self._disarm_bridge_pick()
            self._save_picked_waypoint(
                BRIDGE_WAYPOINT_NAME, x, y, yaw, "Bridge waypoint", select=True
            )

    def _save_picked_waypoint(
        self, name: str, x: float, y: float, yaw: float, label: str, select: bool
    ) -> None:
        try:
            result = self.client.save_waypoint(name, x, y, yaw)
        except HostClientError as exc:
            QMessageBox.warning(self, f"{label} failed", str(exc))
            return

        msg = str(result.get("message", ""))
        if result.get("ok"):
            self.status_label.setText(f"{label} saved: {msg}")
            self.refresh_waypoints()
            if select:
                self._select_waypoint(name)
        else:
            QMessageBox.warning(self, f"{label} rejected", msg or "unknown error")

    def _select_waypoint(self, name: str) -> None:
        for index in range(self.waypoint_combo.count()):
            if self.waypoint_combo.itemData(index) == name:
                self.waypoint_combo.setCurrentIndex(index)
                return

    def _start_task(self) -> None:
        waypoint = self._selected_waypoint()
        if not waypoint:
            QMessageBox.warning(
                self,
                "Missing waypoint",
                "Please save a bridge-center waypoint first, preferably named bridge_center.",
            )
            return
        try:
            result = self.client.bridge_retrieve_start(waypoint, "xiong_qiao")
        except HostClientError as exc:
            QMessageBox.warning(self, "Start failed", str(exc))
            return
        if result.get("ok"):
            self.start_btn.setEnabled(False)
            self.cancel_btn.setEnabled(True)
            self.status_label.setText("Bridge mission started...")
            self._status_timer.start()
        else:
            QMessageBox.warning(self, "Start rejected", str(result.get("message", "unknown error")))

    def _cancel_task(self) -> None:
        try:
            self.client.bridge_retrieve_cancel()
            self.status_label.setText("Canceling...")
        except HostClientError as exc:
            QMessageBox.warning(self, "Cancel failed", str(exc))

    def _emergency_stop(self) -> None:
        errors: list[str] = []
        for label, action in (
            ("bridge mission", self.client.bridge_retrieve_cancel),
            ("navigation", self.client.nav_cancel),
            ("robot stop", self.client.ros_stop),
        ):
            try:
                action()
            except HostClientError as exc:
                errors.append(f"{label}: {exc}")

        self.start_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)
        self._status_timer.stop()
        if errors:
            QMessageBox.warning(self, "STOP partially failed", "\n".join(errors))
        else:
            self.status_label.setText("Emergency stop sent.")

    def _poll_status(self) -> None:
        try:
            status = self.client.bridge_retrieve_status()
        except HostClientError:
            return
        state = status.get("state", "idle")
        msg = status.get("message", "")
        fb = status.get("feedback", {})
        if fb and state == "active":
            progress = float(fb.get("progress", 0.0)) * 100.0
            self.status_label.setText(
                f"State: {state}\nProgress: {progress:.0f}%\n"
                f"{fb.get('stage', '')}: {fb.get('detail', '')}"
            )
        else:
            self.status_label.setText(f"State: {state}\n{msg}")

        if state == "idle":
            self.start_btn.setEnabled(True)
            self.cancel_btn.setEnabled(False)
            self._status_timer.stop()

    def shutdown(self) -> None:
        self._disarm_bridge_pick()
        self._disarm_door_pick()
        self._status_timer.stop()
