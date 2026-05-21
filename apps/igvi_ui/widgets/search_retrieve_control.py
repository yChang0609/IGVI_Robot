from __future__ import annotations

import math
from typing import Any

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


class SearchRetrieveControl(QWidget):
    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        
        self.map_2d.home_pose_picked.connect(self._on_home_pose_picked)
        
        self._home_pose: tuple[float, float, float] | None = None
        self._target_id: str = ""
        self._current_objects: list = []
        
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._poll_status)
        
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        # Target Selection
        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target Object:"))
        self.target_combo = QComboBox()
        self.target_combo.currentIndexChanged.connect(self._on_target_selected)
        target_layout.addWidget(self.target_combo, 1)
        self.clear_memory_btn = QPushButton("Clear")
        self.clear_memory_btn.setObjectName("Danger")
        self.clear_memory_btn.setFixedWidth(52)
        self.clear_memory_btn.setToolTip("Clear all objects from semantic memory")
        self.clear_memory_btn.clicked.connect(self._clear_memory)
        target_layout.addWidget(self.clear_memory_btn)
        layout.addLayout(target_layout)

        # Home Pose Selection
        home_layout = QGridLayout()
        home_layout.addWidget(QLabel("Home Pose:"), 0, 0)
        self.home_label = QLabel("Not set")
        self.home_label.setObjectName("Muted")
        home_layout.addWidget(self.home_label, 0, 1)
        
        self.set_home_btn = QPushButton("Set Home on Map")
        self.set_home_btn.clicked.connect(self._toggle_set_home)
        self.set_home_btn.setCheckable(True)
        home_layout.addWidget(self.set_home_btn, 1, 0, 1, 2)
        layout.addLayout(home_layout)

        # Actions
        actions_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Retrieve")
        self.start_btn.setObjectName("Primary")
        self.start_btn.clicked.connect(self._start_task)
        
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setObjectName("Danger")
        self.cancel_btn.clicked.connect(self._cancel_task)
        self.cancel_btn.setEnabled(False)
        
        actions_layout.addWidget(self.start_btn)
        actions_layout.addWidget(self.cancel_btn)
        layout.addLayout(actions_layout)

        # Status
        self.status_label = QLabel("Ready")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        
        layout.addStretch(1)

    def update_semantic_memory(self, memory: dict[str, Any]) -> None:
        current_id = self.target_combo.currentData()
        self.target_combo.blockSignals(True)
        self.target_combo.clear()

        objects = memory.get('objects', [])
        self._current_objects = objects
        idx_to_restore = -1

        for i, obj in enumerate(objects):
            obj_id = obj.get('id', '')
            short_id = obj_id[:6] if len(obj_id) > 6 else obj_id
            name = obj.get('class_name', 'Unknown')
            display_text = f"{name} ({short_id})"
            self.target_combo.addItem(display_text, obj_id)
            if obj_id == current_id:
                idx_to_restore = i

        if idx_to_restore >= 0:
            self.target_combo.setCurrentIndex(idx_to_restore)
            self._target_id = current_id
        elif objects:
            self.target_combo.setCurrentIndex(0)
            self._target_id = self.target_combo.itemData(0)
        else:
            self._target_id = ""

        self.target_combo.blockSignals(False)
        self._sync_map()

    def _on_target_selected(self, index: int) -> None:
        if index >= 0:
            self._target_id = self.target_combo.itemData(index)
        self._sync_map()

    def _sync_map(self) -> None:
        if self._target_id:
            self.map_2d.update_semantic_objects(self._current_objects, self._target_id)
        else:
            self.map_2d.update_semantic_objects([], "")

    def _toggle_set_home(self) -> None:
        is_setting = self.set_home_btn.isChecked()
        self.map_2d.set_home_pose_mode(is_setting)
        if is_setting:
            self.set_home_btn.setText("Click Map to Set Home...")
        else:
            self.set_home_btn.setText("Set Home on Map")

    def _on_home_pose_picked(self, x: float, y: float, yaw: float) -> None:
        self._home_pose = (x, y, yaw)
        self.home_label.setText(f"x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}")
        self.set_home_btn.setChecked(False)
        self.set_home_btn.setText("Set Home on Map")
        self.map_2d.set_home_pose_mode(False)

    def _start_task(self) -> None:
        if not self._target_id:
            QMessageBox.warning(self, "Error", "Please select a target object first.")
            return
        if not self._home_pose:
            QMessageBox.warning(self, "Error", "Please set a home pose on the map first.")
            return
            
        hx, hy, hyaw = self._home_pose
        try:
            res = self.client.search_retrieve_start(self._target_id, hx, hy, hyaw)
            if res.get("ok"):
                self.start_btn.setEnabled(False)
                self.cancel_btn.setEnabled(True)
                self.status_label.setText("Task started...")
                self._status_timer.start()
            else:
                QMessageBox.warning(self, "Error", res.get("message", "Failed to start"))
        except HostClientError as e:
            QMessageBox.warning(self, "Error", str(e))

    def _cancel_task(self) -> None:
        try:
            self.client.search_retrieve_cancel()
            self.status_label.setText("Canceling...")
        except HostClientError as e:
            QMessageBox.warning(self, "Error", str(e))

    def _poll_status(self) -> None:
        try:
            status = self.client.search_retrieve_status()
            state = status.get("state", "idle")
            msg = status.get("message", "")
            
            fb = status.get("feedback", {})
            if fb and state == "active":
                stage = fb.get("stage", "")
                detail = fb.get("detail", "")
                prog = fb.get("progress", 0.0) * 100
                self.status_label.setText(f"State: {state}\nProgress: {prog:.0f}%\n{stage}: {detail}")
            else:
                self.status_label.setText(f"State: {state}\n{msg}")
                
            if state == "idle":
                self.start_btn.setEnabled(True)
                self.cancel_btn.setEnabled(False)
                self._status_timer.stop()
                
        except HostClientError:
            pass

    def _clear_memory(self) -> None:
        try:
            res = self.client.semantic_memory_clear()
            if res.get("ok"):
                self.status_label.setText("Memory cleared.")
            else:
                QMessageBox.warning(self, "Clear failed", res.get("message", "Unknown error"))
        except HostClientError as e:
            QMessageBox.warning(self, "Clear failed", str(e))

    def shutdown(self) -> None:
        self._status_timer.stop()
