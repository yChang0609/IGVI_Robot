from __future__ import annotations

from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.map_views import Map2DView

_STATE_TONE = {
    "idle":       ("Idle", "#94a3b8"),
    "sending":    ("Sending…", "#fbbf24"),
    "accepted":   ("Accepted", "#22c55e"),
    "navigating": ("Navigating", "#22c55e"),
    "succeeded":  ("Goal reached", "#22c55e"),
    "canceling":  ("Canceling…", "#fbbf24"),
    "canceled":   ("Canceled", "#94a3b8"),
    "aborted":    ("Aborted", "#ef4444"),
    "rejected":   ("Rejected", "#ef4444"),
    "failed":     ("Failed", "#ef4444"),
    "unavailable": ("Nav server offline", "#ef4444"),
}


class _StatusPoller(QThread):
    status_received = Signal(dict)
    error = Signal(str)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                self.status_received.emit(self.client.nav_status())
            except Exception as exc:  # noqa: BLE001
                self.error.emit(str(exc))
            self.msleep(400)


class NavigationControl(QWidget):
    log_message = Signal(str)
    map_unlock_requested = Signal(bool)  # True → lock map for selection mode

    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        self._poller: _StatusPoller | None = None
        self._last_state: str = "idle"
        self._initial_pose_pick_armed = False
        self.map_2d.initial_pose_picked.connect(self._on_initial_pose_picked)
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        hint = QLabel(
            "Click on the 2D map to set a goal; drag in the desired heading direction to set yaw."
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.state_label = QLabel("Idle")
        self.state_label.setStyleSheet("font-weight: 700; font-size: 14px; color: #94a3b8;")
        layout.addWidget(self.state_label)

        self.detail_label = QLabel("—")
        self.detail_label.setObjectName("Muted")
        self.detail_label.setWordWrap(True)
        layout.addWidget(self.detail_label)

        self.feedback_label = QLabel("")
        self.feedback_label.setObjectName("Muted")
        self.feedback_label.setWordWrap(True)
        layout.addWidget(self.feedback_label)

        buttons = QGridLayout()
        buttons.setSpacing(8)
        self.cancel_btn = QPushButton("Cancel Goal")
        self.cancel_btn.setObjectName("Danger")
        self.cancel_btn.clicked.connect(self._cancel)
        self.cancel_btn.setEnabled(False)
        self.clear_btn = QPushButton("Clear Costmap")
        self.clear_btn.clicked.connect(self._clear_costmap)
        self.set_initial_btn = QPushButton("Set Initial Pose (click map)")
        self.set_initial_btn.setCheckable(True)
        self.set_initial_btn.setToolTip(
            "Toggle on, then click+drag on the 2D map to tell SLAM where the "
            "robot really is. Drag direction sets heading."
        )
        self.set_initial_btn.clicked.connect(self._toggle_initial_pose_pick)
        self.initial_btn = QPushButton("Reset to Origin")
        self.initial_btn.setToolTip("Send initial pose (0, 0, 0) — only useful at the map origin.")
        self.initial_btn.clicked.connect(self._reset_initial)
        self.probe_btn = QPushButton("Probe Actions")
        self.probe_btn.setToolTip("List action servers visible to the bridge")
        self.probe_btn.clicked.connect(self._probe_actions)
        self.save_map_btn = QPushButton("Save Arena Map")
        self.save_map_btn.setToolTip(
            "Snapshot the live RTAB-Map DB into data/slam/arena_map.db "
            "(used by slam_localization on next boot)"
        )
        self.save_map_btn.clicked.connect(self._save_map)
        buttons.addWidget(self.cancel_btn, 0, 0)
        buttons.addWidget(self.clear_btn, 0, 1)
        buttons.addWidget(self.set_initial_btn, 1, 0)
        buttons.addWidget(self.initial_btn, 1, 1)
        buttons.addWidget(self.probe_btn, 2, 0, 1, 2)
        buttons.addWidget(self.save_map_btn, 3, 0, 1, 2)
        layout.addLayout(buttons)
        layout.addStretch(1)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._poller is None:
            self._poller = _StatusPoller(self.client)
            self._poller.status_received.connect(self._on_status)
            self._poller.error.connect(self._on_error)
            self._poller.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self._disarm_initial_pose_pick()
        self.shutdown()

    def shutdown(self) -> None:
        """Stop the status poller. Idempotent; safe to call on app exit."""
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_status(self, status: dict) -> None:
        state = str(status.get("state") or "idle")
        message = str(status.get("message") or "")
        label, color = _STATE_TONE.get(state, (state, "#94a3b8"))
        if not status.get("server_ready") and state in ("idle", "unavailable"):
            label, color = "Nav server offline", "#ef4444"
        self.state_label.setText(label)
        self.state_label.setStyleSheet(f"font-weight: 700; font-size: 14px; color: {color};")
        goal = status.get("goal") or {}
        if goal:
            self.detail_label.setText(
                f"Goal x {goal.get('x', 0):.2f}  y {goal.get('y', 0):.2f}  "
                f"yaw {goal.get('yaw', 0):.2f}  ·  {message}"
            )
        else:
            self.detail_label.setText(message or "—")
        fb = status.get("feedback") or {}
        if fb:
            self.feedback_label.setText(
                f"distance remaining {fb.get('distance_remaining', 0):.2f} m  ·  "
                f"eta {fb.get('estimated_time_remaining', 0):.1f}s  ·  "
                f"recoveries {fb.get('recoveries', 0)}"
            )
        else:
            self.feedback_label.setText("")
        self.cancel_btn.setEnabled(state in ("sending", "accepted", "navigating"))
        if state != self._last_state:
            self._last_state = state
            self.log_message.emit(f"Nav: {label}")

    def _on_error(self, message: str) -> None:
        self.state_label.setText("Bridge unavailable")
        self.state_label.setStyleSheet("font-weight: 700; color: #ef4444;")
        self.detail_label.setText(message)

    def _cancel(self) -> None:
        try:
            self.client.nav_cancel()
        except HostClientError as exc:
            QMessageBox.warning(self, "Cancel failed", str(exc))

    def _clear_costmap(self) -> None:
        try:
            self.client.clear_costmap("global")
            self.map_2d.clear_costmap()
        except HostClientError as exc:
            QMessageBox.warning(self, "Clear costmap failed", str(exc))

    def _reset_initial(self) -> None:
        try:
            self.client.initial_pose(0.0, 0.0, 0.0)
        except HostClientError as exc:
            QMessageBox.warning(self, "Initial pose failed", str(exc))

    def _toggle_initial_pose_pick(self) -> None:
        if self.set_initial_btn.isChecked():
            self._initial_pose_pick_armed = True
            self.map_2d.set_initial_pose_mode(True)
            self.set_initial_btn.setText("Click map to place pose…")
            self.detail_label.setText(
                "Click the saved map where the robot actually is; drag in its heading direction."
            )
        else:
            self._disarm_initial_pose_pick()

    def _disarm_initial_pose_pick(self) -> None:
        if self._initial_pose_pick_armed:
            self._initial_pose_pick_armed = False
            self.map_2d.set_initial_pose_mode(False)
        self.set_initial_btn.setChecked(False)
        self.set_initial_btn.setText("Set Initial Pose (click map)")

    def _on_initial_pose_picked(self, x: float, y: float, yaw: float) -> None:
        if not self._initial_pose_pick_armed:
            return
        self._disarm_initial_pose_pick()
        try:
            self.client.initial_pose(x, y, yaw)
        except HostClientError as exc:
            QMessageBox.warning(self, "Initial pose failed", str(exc))
            return
        self.detail_label.setText(
            f"Initial pose published: x {x:.2f}  y {y:.2f}  yaw {yaw:.2f}"
        )
        self.log_message.emit(f"Initial pose set to ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def _probe_actions(self) -> None:
        try:
            status = self.client.nav_status()
        except HostClientError as exc:
            QMessageBox.warning(self, "Probe failed", str(exc))
            return
        actions = status.get("visible_actions") or []
        ready = status.get("server_ready")
        lines = [
            f"navigate_to_pose ready: {ready}",
            f"Bridge sees {len(actions)} action server(s):",
        ]
        lines.extend(f"  • {a}" for a in actions) if actions else lines.append("  (none discovered)")
        QMessageBox.information(self, "Action server probe", "\n".join(lines))

    def _save_map(self) -> None:
        try:
            result = self.client.save_map()
        except HostClientError as exc:
            QMessageBox.warning(self, "Save map failed", str(exc))
            return
        if result.get("ok"):
            QMessageBox.information(self, "Arena map saved", f"Saved to:\n{result.get('message', '')}")
            self.log_message.emit("Arena map saved successfully")
        else:
            QMessageBox.warning(self, "Save arena map failed", result.get("message", "unknown error"))
