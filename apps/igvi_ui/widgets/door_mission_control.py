"""Reusable "Door Mission" control widget.

A self-contained card that drives the integrated nav → open_door task:
navigate to a named waypoint (default ``door_approach``), and as soon as nav
succeeds the bridge automatically dispatches the open_door action. The widget
shows live phase + nested nav/open_door progress, plus Cancel.

Designed to be drop-in: any page that has a HostClient can do::

    from igvi_ui.widgets.door_mission_control import DoorMissionControl

    mission = DoorMissionControl(self.client)
    mission.log_message.connect(self.set_status_message)
    some_layout.addWidget(mission)
    # ... and in the page's hideEvent/closeEvent:
    mission.shutdown()

That's it — no other wiring is needed. Everything (waypoint name, ready
distance, status polling, lifecycle) is internal.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError


DEFAULT_WAYPOINT = "door_approach"
OPEN_DOOR_NODE = "open_door_server"

# Phase → (display label, chip color). Mirrors the server-side phase strings
# in bridge_node.start_door_mission / _advance_door_mission_*.
_PHASE_STYLE: dict[str, tuple[str, str]] = {
    "idle":        ("Idle",        "#6b7280"),
    "navigating":  ("Navigating",  "#3b82f6"),
    "opening":     ("Opening Door", "#3b82f6"),
    "succeeded":   ("Succeeded",   "#22c55e"),
    "failed":      ("Failed",      "#ef4444"),
    "canceled":    ("Canceled",    "#facc15"),
    "unavailable": ("Unavailable", "#6b7280"),
}


class _MissionPoller(QThread):
    """Polls door_mission_status off the UI thread."""

    status_received = Signal(dict)

    _POLL_TIMEOUT = 2.0
    _IDLE_MS = 600

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                self.status_received.emit(
                    self.client.door_mission_status(timeout=self._POLL_TIMEOUT)
                )
            except Exception:  # noqa: BLE001 — network blips are normal
                pass
            waited = 0
            while self._running and waited < self._IDLE_MS:
                self.msleep(100)
                waited += 100


class DoorMissionControl(QWidget):
    """Self-contained 'Door Mission' card.

    Emits ``log_message`` for human-readable events (dispatch / cancel /
    failures) so the host page can route them to its own status bar.
    """

    log_message = Signal(str)

    def __init__(self, client: HostClient, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.client = client
        self._poller: _MissionPoller | None = None
        self._build_ui()

    # ── UI construction ─────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)

        heading = QLabel("Door Mission — navigate to waypoint, then open door")
        heading.setStyleSheet("font-weight: 600;")
        layout.addWidget(heading)

        hint = QLabel(
            "Single task: drive to the named waypoint, then automatically run "
            "the open_door action. Default waypoint is "
            f"'<b>{DEFAULT_WAYPOINT}</b>'."
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        wp_row = QHBoxLayout()
        wp_row.addWidget(QLabel("Waypoint"))
        self.waypoint_edit = QLineEdit(DEFAULT_WAYPOINT)
        self.waypoint_edit.setPlaceholderText(DEFAULT_WAYPOINT)
        wp_row.addWidget(self.waypoint_edit, 1)
        layout.addLayout(wp_row)

        rd_row = QHBoxLayout()
        rd_row.addWidget(QLabel("Ready distance (m)"))
        self.ready_spin = QDoubleSpinBox()
        self.ready_spin.setRange(0.0, 2.0)
        self.ready_spin.setSingleStep(0.05)
        self.ready_spin.setDecimals(2)
        self.ready_spin.setValue(0.0)  # 0.0 → server uses its persisted param
        self.ready_spin.setToolTip(
            "Override open_door's ready_distance_m for this run. 0.0 keeps the "
            "value already set on open_door_server."
        )
        rd_row.addWidget(self.ready_spin)
        layout.addLayout(rd_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Approach speed (m/s)"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.05, 0.30)
        self.speed_spin.setSingleStep(0.02)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setValue(0.10)
        self.speed_spin.setToolTip(
            "Forward speed during open_door's APPROACH phase. Applied as a live "
            "param on open_door_server, so it affects both this mission and "
            "any direct open_door run."
        )
        self.speed_spin.valueChanged.connect(self._apply_approach_speed)
        speed_row.addWidget(self.speed_spin)
        layout.addLayout(speed_row)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("Start Door Mission")
        self.start_btn.clicked.connect(self._on_start)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self._on_cancel)
        self.cancel_btn.setEnabled(False)
        btn_row.addWidget(self.start_btn, 1)
        btn_row.addWidget(self.cancel_btn)
        layout.addLayout(btn_row)

        # Phase chip + the message line + sub-action breadcrumb. Kept compact
        # so the widget fits into any side panel.
        self.phase_chip = QLabel("Idle")
        self.phase_chip.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.phase_chip.setStyleSheet(self._chip_style("#6b7280"))
        layout.addWidget(self.phase_chip)

        self.message_label = QLabel("")
        self.message_label.setObjectName("Muted")
        self.message_label.setWordWrap(True)
        layout.addWidget(self.message_label)

        self.sub_label = QLabel("")
        self.sub_label.setObjectName("Muted")
        self.sub_label.setWordWrap(True)
        layout.addWidget(self.sub_label)

        outer.addWidget(card)

    @staticmethod
    def _chip_style(color: str) -> str:
        return (
            f"background:{color}; color:white; padding:3px 10px; "
            "border-radius:8px; font-weight:600;"
        )

    # ── Lifecycle ───────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._poller is None:
            self._poller = _MissionPoller(self.client)
            self._poller.status_received.connect(self._on_status)
            self._poller.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        # Caller should also invoke shutdown() from its own teardown — same
        # pattern as ImageView — but stop the poller here so a hidden card
        # isn't spinning HTTP requests.
        self._stop_poller()

    def shutdown(self) -> None:
        """Stop the status poller. Idempotent; safe from any teardown path."""
        self._stop_poller()

    def _stop_poller(self) -> None:
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None

    # ── Actions ─────────────────────────────────────────────────────────────

    def _on_start(self) -> None:
        waypoint = self.waypoint_edit.text().strip() or DEFAULT_WAYPOINT
        ready = float(self.ready_spin.value())
        # Push the current approach_speed before dispatching, so a value the
        # user typed without firing valueChanged still takes effect.
        self._apply_approach_speed(float(self.speed_spin.value()), quiet=True)
        try:
            result = self.client.door_mission_start(waypoint=waypoint, ready_distance_m=ready)
        except HostClientError as exc:
            self.log_message.emit(f"Door mission start failed: {exc}")
            return
        msg = str(result.get("message", "door mission dispatched"))
        self.log_message.emit(msg)

    def _apply_approach_speed(self, value: float, quiet: bool = False) -> None:
        try:
            result = self.client.set_params(OPEN_DOOR_NODE, {"approach_speed": float(value)})
        except HostClientError as exc:
            if not quiet:
                self.log_message.emit(f"Approach speed set failed: {exc}")
            return
        if not result.get("ok", False) and not quiet:
            self.log_message.emit(
                f"approach_speed: {result.get('message', 'rejected')}"
            )

    def _on_cancel(self) -> None:
        try:
            result = self.client.door_mission_cancel()
        except HostClientError as exc:
            self.log_message.emit(f"Door mission cancel failed: {exc}")
            return
        self.log_message.emit(str(result.get("message", "door mission cancel requested")))

    # ── Status rendering ────────────────────────────────────────────────────

    def _on_status(self, data: dict) -> None:
        active = bool(data.get("active", False))
        phase = str(data.get("phase", "idle"))
        message = str(data.get("message", ""))
        waypoint = str(data.get("waypoint", ""))

        label, color = _PHASE_STYLE.get(phase, (phase.title(), "#6b7280"))
        self.phase_chip.setText(label)
        self.phase_chip.setStyleSheet(self._chip_style(color))
        self.message_label.setText(message)

        # Disable Start during an active mission; enable Cancel only while
        # there's something to cancel (navigating / opening).
        self.start_btn.setEnabled(not active)
        self.cancel_btn.setEnabled(phase in ("navigating", "opening"))

        nav = data.get("nav") or {}
        door = data.get("open_door") or {}
        breadcrumb_parts: list[str] = []
        if waypoint:
            breadcrumb_parts.append(f"waypoint: {waypoint}")
        nav_state = str(nav.get("state", ""))
        if nav_state and nav_state != "idle":
            breadcrumb_parts.append(f"nav: {nav_state}")
        door_state = str(door.get("state", ""))
        door_stage = str(door.get("stage", ""))
        if door_state and door_state != "idle":
            breadcrumb_parts.append(
                f"open_door: {door_state}" + (f" · {door_stage}" if door_stage else "")
            )
        self.sub_label.setText(" | ".join(breadcrumb_parts))
