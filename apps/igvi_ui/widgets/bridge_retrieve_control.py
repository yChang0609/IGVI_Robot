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


class BridgeRetrieveControl(QWidget):
    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._poll_status)
        self._build_ui()

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
        layout.addLayout(target_grid)

        buttons = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh Waypoints")
        self.refresh_btn.clicked.connect(self.refresh_waypoints)
        self.start_btn = QPushButton("Start Bridge Mission")
        self.start_btn.setObjectName("Primary")
        self.start_btn.clicked.connect(self._start_task)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setObjectName("Danger")
        self.cancel_btn.clicked.connect(self._cancel_task)
        self.cancel_btn.setEnabled(False)
        buttons.addWidget(self.refresh_btn)
        buttons.addWidget(self.start_btn)
        buttons.addWidget(self.cancel_btn)
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
        self._status_timer.stop()
