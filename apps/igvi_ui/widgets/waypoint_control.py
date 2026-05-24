from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.map_views import Map2DView


class WaypointControl(QWidget):
    """Save / recall named map-frame poses that persist across runs.

    Two ways to capture a waypoint:
      • "Mark current position" — drive the robot there, then snapshot its
        live pose (bridge reads it; no coords sent).
      • "Pick on map" — toggle on, then click the 2D map; the clicked point
        (drag for heading) is saved instead of sent as a nav goal.
    """

    log_message = Signal(str)

    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        self._pick_armed = False
        self.map_2d.waypoint_point_picked.connect(self._on_point_picked)
        self._build_ui()

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)

        layout = QVBoxLayout(inner)
        layout.setContentsMargins(0, 0, 4, 0)
        layout.setSpacing(10)

        hint = QLabel(
            "Saved points persist across runs (in the loaded map's frame). "
            "Select one, then Go to navigate there."
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.list_widget = QListWidget()
        self.list_widget.setMinimumHeight(120)
        self.list_widget.itemSelectionChanged.connect(self._sync_buttons)
        layout.addWidget(self.list_widget)

        self.detail_label = QLabel("—")
        self.detail_label.setObjectName("Muted")
        self.detail_label.setWordWrap(True)
        layout.addWidget(self.detail_label)

        buttons = QGridLayout()
        buttons.setSpacing(8)
        self.go_btn = QPushButton("Go")
        self.go_btn.setObjectName("Primary")
        self.go_btn.clicked.connect(self._go)
        self.go_btn.setEnabled(False)
        self.delete_btn = QPushButton("Delete")
        self.delete_btn.setObjectName("Danger")
        self.delete_btn.clicked.connect(self._delete)
        self.delete_btn.setEnabled(False)
        self.mark_btn = QPushButton("Mark current position")
        self.mark_btn.clicked.connect(self._mark_current)
        self.pick_btn = QPushButton("Pick on map")
        self.pick_btn.setCheckable(True)
        self.pick_btn.clicked.connect(self._toggle_pick)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh)
        buttons.addWidget(self.go_btn, 0, 0)
        buttons.addWidget(self.delete_btn, 0, 1)
        buttons.addWidget(self.mark_btn, 1, 0)
        buttons.addWidget(self.pick_btn, 1, 1)
        buttons.addWidget(self.refresh_btn, 2, 0, 1, 2)
        layout.addLayout(buttons)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        self.refresh()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self._disarm_pick()

    # ── Data ──────────────────────────────────────────────────────────────────

    def refresh(self) -> None:
        try:
            waypoints = self.client.list_waypoints()
        except HostClientError as exc:
            self.detail_label.setText(f"Bridge unavailable: {exc}")
            return
        selected = self._selected_name()
        self.list_widget.clear()
        for name in sorted(waypoints):
            wp = waypoints[name]
            item = QListWidgetItem(
                f"{name}   ·   x {wp.get('x', 0):.2f}  y {wp.get('y', 0):.2f}  "
                f"yaw {wp.get('yaw', 0):.2f}"
            )
            item.setData(256, name)  # Qt.UserRole
            self.list_widget.addItem(item)
            if name == selected:
                item.setSelected(True)
        self.map_2d.update_waypoints(waypoints)
        self._sync_buttons()

    def _selected_name(self) -> str | None:
        items = self.list_widget.selectedItems()
        return items[0].data(256) if items else None

    def _sync_buttons(self) -> None:
        has = self._selected_name() is not None
        self.go_btn.setEnabled(has)
        self.delete_btn.setEnabled(has)

    # ── Capture ───────────────────────────────────────────────────────────────

    def _mark_current(self) -> None:
        name, ok = QInputDialog.getText(self, "Mark current position", "Waypoint name:")
        if not ok or not name.strip():
            return
        self._save(name.strip(), None, None, 0.0)

    def _toggle_pick(self) -> None:
        if self.pick_btn.isChecked():
            self._pick_armed = True
            self.map_2d.set_pick_mode(True)
            self.pick_btn.setText("Pick on map (click map…)")
            self.detail_label.setText("Click the map to place a waypoint.")
        else:
            self._disarm_pick()

    def _disarm_pick(self) -> None:
        if self._pick_armed:
            self._pick_armed = False
            self.map_2d.set_pick_mode(False)
        self.pick_btn.setChecked(False)
        self.pick_btn.setText("Pick on map")

    def _on_point_picked(self, x: float, y: float, yaw: float) -> None:
        if not self._pick_armed:
            return
        self._disarm_pick()
        name, ok = QInputDialog.getText(
            self, "Save picked waypoint", f"Name for point ({x:.2f}, {y:.2f}):"
        )
        if not ok or not name.strip():
            return
        self._save(name.strip(), x, y, yaw)

    def _save(self, name: str, x: float | None, y: float | None, yaw: float) -> None:
        try:
            result = self.client.save_waypoint(name, x, y, yaw)
        except HostClientError as exc:
            QMessageBox.warning(self, "Save failed", str(exc))
            return
        msg = str(result.get("message", ""))
        if result.get("ok"):
            self.log_message.emit(f"Waypoint: {msg}")
            self.refresh()
        else:
            QMessageBox.warning(self, "Save failed", msg or "unknown error")

    # ── Recall ────────────────────────────────────────────────────────────────

    def _go(self) -> None:
        name = self._selected_name()
        if not name:
            return
        try:
            result = self.client.goto_waypoint(name)
        except HostClientError as exc:
            QMessageBox.warning(self, "Go failed", str(exc))
            return
        msg = str(result.get("message", ""))
        if result.get("ok"):
            self.detail_label.setText(f"Navigating to '{name}' · {msg}")
            self.log_message.emit(f"Waypoint: go '{name}' — {msg}")
        else:
            QMessageBox.warning(
                self, "Go rejected",
                msg or "Navigation server unavailable. Is the navigation profile up?",
            )

    def _delete(self) -> None:
        name = self._selected_name()
        if not name:
            return
        if QMessageBox.question(
            self, "Delete waypoint", f"Delete '{name}'?"
        ) != QMessageBox.StandardButton.Yes:
            return
        try:
            result = self.client.delete_waypoint(name)
        except HostClientError as exc:
            QMessageBox.warning(self, "Delete failed", str(exc))
            return
        if result.get("ok"):
            self.log_message.emit(f"Waypoint: deleted '{name}'")
            self.refresh()
        else:
            QMessageBox.warning(
                self, "Delete failed", str(result.get("message", "unknown error"))
            )
