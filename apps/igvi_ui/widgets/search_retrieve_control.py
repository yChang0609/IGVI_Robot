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
    QScrollArea,
    QFrame,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.map_views import Map2DView


class SearchRetrieveControl(QWidget):
    def __init__(self, client: HostClient, map_2d: Map2DView) -> None:
        super().__init__()
        self.client = client
        self.map_2d = map_2d
        
        self.map_2d.home_pose_picked.connect(self._on_home_pose_picked)
        self.map_2d.arena_pose_picked.connect(self._on_arena_pose_picked)
        
        self._home_pose: tuple[float, float, float] | None = None
        self._arena_setting_target: str = ""
        self._arena_patrol_count: int = 0
        self._target_id: str = ""
        self._current_objects: list = []
        self._patrol_names: list[str] = []
        self._patrol_start_idx: int = 0
        self._last_feedback_patrol: str = ""
        
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._poll_status)
        self._status_timer.start()
        
        self._build_ui()
        
        try:
            waypoints = self.client.list_waypoints()
            if "home" in waypoints:
                hw = waypoints["home"]
                self._home_pose = (hw.get("x", 0.0), hw.get("y", 0.0), hw.get("yaw", 0.0))
                self.home_label.setText(f"x: {self._home_pose[0]:.2f}, y: {self._home_pose[1]:.2f}, yaw: {self._home_pose[2]:.2f}")
            self._patrol_names = sorted(k for k in waypoints if k.startswith("patrol_"))
            for name in self._patrol_names:
                self.patrol_start_combo.addItem(name, name)
        except HostClientError:
            pass

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll_widget = QWidget()
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)
        
        layout = QVBoxLayout(scroll_widget)
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
        
        # Arena Mission
        arena_layout = QVBoxLayout()
        arena_layout.addWidget(QLabel("─── Arena Mission ───"))
        
        btn_layout1 = QHBoxLayout()
        self.arena_our_base_btn = QPushButton("Set Our Base")
        self.arena_our_base_btn.setCheckable(True)
        self.arena_our_base_btn.clicked.connect(lambda: self._toggle_arena_set("our_base"))
        self.arena_enemy_base_btn = QPushButton("Set Enemy Base")
        self.arena_enemy_base_btn.setCheckable(True)
        self.arena_enemy_base_btn.clicked.connect(lambda: self._toggle_arena_set("enemy_base"))
        btn_layout1.addWidget(self.arena_our_base_btn)
        btn_layout1.addWidget(self.arena_enemy_base_btn)
        arena_layout.addLayout(btn_layout1)

        btn_layout2 = QHBoxLayout()
        self.arena_patrol_btn = QPushButton("Add Patrol Point")
        self.arena_patrol_btn.setCheckable(True)
        self.arena_patrol_btn.clicked.connect(lambda: self._toggle_arena_set("patrol"))
        self.arena_clear_btn = QPushButton("Clear Patrols")
        self.arena_clear_btn.setObjectName("Danger")
        self.arena_clear_btn.clicked.connect(self._clear_patrols)
        btn_layout2.addWidget(self.arena_patrol_btn)
        btn_layout2.addWidget(self.arena_clear_btn)
        arena_layout.addLayout(btn_layout2)
        
        patrol_start_layout = QHBoxLayout()
        patrol_start_layout.addWidget(QLabel("Start from:"))
        self.patrol_start_combo = QComboBox()
        self.patrol_start_combo.setToolTip("Select which patrol waypoint to start from")
        self.patrol_start_combo.currentIndexChanged.connect(self._on_patrol_start_changed)
        patrol_start_layout.addWidget(self.patrol_start_combo, 1)
        arena_layout.addLayout(patrol_start_layout)

        arena_actions = QHBoxLayout()
        self.start_arena_btn = QPushButton("Start Arena")
        self.start_arena_btn.setObjectName("Primary")
        self.start_arena_btn.clicked.connect(self._start_arena)
        self.cancel_arena_btn = QPushButton("Cancel Arena")
        self.cancel_arena_btn.setObjectName("Danger")
        self.cancel_arena_btn.clicked.connect(self._cancel_arena)
        self.cancel_arena_btn.setEnabled(False)
        arena_actions.addWidget(self.start_arena_btn)
        arena_actions.addWidget(self.cancel_arena_btn)
        arena_layout.addLayout(arena_actions)
        
        self.arena_status_label = QLabel("Arena Ready")
        self.arena_status_label.setWordWrap(True)
        arena_layout.addWidget(self.arena_status_label)
        
        layout.addLayout(arena_layout)
        
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
        
        try:
            self.client.save_waypoint("home", x, y, yaw)
            self.map_2d.update_waypoints(self.client.list_waypoints())
        except HostClientError as e:
            QMessageBox.warning(self, "Error", f"Failed to save home waypoint: {e}")

    def _start_task(self) -> None:
        if not self._target_id:
            QMessageBox.warning(self, "Error", "Please select a target object first.")
            return
        if not self._home_pose:
            QMessageBox.warning(self, "Error", "Please set a home pose on the map first.")
            return
            
        hx, hy, hyaw = self._home_pose
        
        # Disable button immediately to prevent double-clicks while the request is processing
        self.start_btn.setEnabled(False)
        self.status_label.setText("Starting...")
        
        try:
            res = self.client.search_retrieve_start(self._target_id, hx, hy, hyaw)
            if res.get("ok"):
                self.cancel_btn.setEnabled(True)
                self.status_label.setText("Task started...")
                self._status_timer.start()
            else:
                self.start_btn.setEnabled(True)
                QMessageBox.warning(self, "Error", res.get("message", "Failed to start"))
        except HostClientError as e:
            self.start_btn.setEnabled(True)
            QMessageBox.warning(self, "Error", str(e))

    def _cancel_task(self) -> None:
        self.cancel_btn.setEnabled(False)
        try:
            self.client.search_retrieve_cancel()
            self.status_label.setText("Canceling...")
        except HostClientError as e:
            self.cancel_btn.setEnabled(True)
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
                
        except HostClientError:
            pass
            
        # Poll Arena Status
        try:
            status = self.client.arena_mission_status()
            state = status.get("state", "idle")
            msg = status.get("message", "")
            
            fb = status.get("feedback", {})
            if fb and state == "active":
                stage = fb.get("stage", "")
                detail = fb.get("detail", "")
                self.arena_status_label.setText(f"Arena State: {state}\n{stage}: {detail}")
                if stage == "patrolling" and detail:
                    self._update_patrol_start_from_feedback(detail)
            else:
                self.arena_status_label.setText(f"Arena State: {state}\n{msg}")
            if state in ("idle", "error"):
                self.start_arena_btn.setEnabled(True)
                self.cancel_arena_btn.setEnabled(False)
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

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if not self._status_timer.isActive():
            self._status_timer.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self._status_timer.stop()



    def _toggle_arena_set(self, target: str) -> None:
        if target == "":
            self.map_2d.set_arena_pose_mode(False)
            self._arena_setting_target = ""
            self.arena_our_base_btn.setText("Set Our Base")
            self.arena_enemy_base_btn.setText("Set Enemy Base")
            self.arena_patrol_btn.setText("Add Patrol Point")
            return
            
        btn = None
        if target == "our_base": btn = self.arena_our_base_btn
        if target == "enemy_base": btn = self.arena_enemy_base_btn
        if target == "patrol": btn = self.arena_patrol_btn
        
        is_setting = btn.isChecked()
        self.map_2d.set_arena_pose_mode(is_setting)
        
        if is_setting:
            self._arena_setting_target = target
            for b in [self.arena_our_base_btn, self.arena_enemy_base_btn, self.arena_patrol_btn]:
                if b != btn: b.setChecked(False)
            btn.setText(f"Click Map to Set {target}...")
        else:
            self._arena_setting_target = ""
            self.arena_our_base_btn.setText("Set Our Base")
            self.arena_enemy_base_btn.setText("Set Enemy Base")
            self.arena_patrol_btn.setText("Add Patrol Point")

    def _on_arena_pose_picked(self, x: float, y: float, yaw: float) -> None:
        target = self._arena_setting_target
        if not target: return
        
        name = target
        if target == "patrol":
            self._arena_patrol_count += 1
            name = f"patrol_{self._arena_patrol_count}"
            
        try:
            self.client.save_waypoint(name, x, y, yaw)
            self.map_2d.update_waypoints(self.client.list_waypoints())
            QMessageBox.information(self, "Saved", f"Saved {name} waypoint.")
        except HostClientError as e:
            QMessageBox.warning(self, "Error", f"Failed to save {name}: {e}")
            
        self.arena_our_base_btn.setChecked(False)
        self.arena_enemy_base_btn.setChecked(False)
        self.arena_patrol_btn.setChecked(False)
        self._toggle_arena_set("")  # reset texts

    def _clear_patrols(self) -> None:
        try:
            waypoints = self.client.list_waypoints()
            deleted = 0
            for w in waypoints.keys():
                if w.startswith("patrol_"):
                    self.client.delete_waypoint(w)
                    deleted += 1
            self._arena_patrol_count = 0
            self.map_2d.update_waypoints(self.client.list_waypoints())
            QMessageBox.information(self, "Cleared", f"Cleared {deleted} patrol waypoints.")
        except HostClientError as e:
            QMessageBox.warning(self, "Error", str(e))

    def _start_arena(self) -> None:
        self._refresh_patrol_list()
        self.start_arena_btn.setEnabled(False)
        self._last_feedback_patrol = ""
        self.arena_status_label.setText("Starting Arena...")
        try:
            res = self.client.arena_mission_start(self._patrol_start_idx)
            if res.get("ok"):
                self.cancel_arena_btn.setEnabled(True)
                self.arena_status_label.setText("Arena started...")
            else:
                self.start_arena_btn.setEnabled(True)
                QMessageBox.warning(self, "Error", res.get("message", "Failed to start"))
        except HostClientError as e:
            self.start_arena_btn.setEnabled(True)
            QMessageBox.warning(self, "Error", str(e))

    def _refresh_patrol_list(self) -> None:
        try:
            waypoints = self.client.list_waypoints()
            names = sorted(k for k in waypoints if k.startswith("patrol_"))
            if names == self._patrol_names:
                return
            self._patrol_names = names
            current = self.patrol_start_combo.currentData()
            self.patrol_start_combo.blockSignals(True)
            self.patrol_start_combo.clear()
            for name in names:
                self.patrol_start_combo.addItem(name, name)
            if current in names:
                self.patrol_start_combo.setCurrentIndex(names.index(current))
            else:
                self.patrol_start_combo.setCurrentIndex(0)
            self._patrol_start_idx = max(0, self.patrol_start_combo.currentIndex())
            self.patrol_start_combo.blockSignals(False)
        except HostClientError:
            pass

    def _on_patrol_start_changed(self, index: int = -1) -> None:
        if index < 0:
            index = self.patrol_start_combo.currentIndex()
        self._patrol_start_idx = max(0, index)

    def _update_patrol_start_from_feedback(self, detail: str) -> None:
        try:
            # detail format: "Patrolling to patrol_N (x.xx, y.yy)"
            patrol_name = detail.split("Patrolling to ")[1].split(" (")[0]
        except (IndexError, ValueError):
            return
        if patrol_name not in self._patrol_names or patrol_name == self._last_feedback_patrol:
            return
        self._last_feedback_patrol = patrol_name
        idx = self._patrol_names.index(patrol_name)
        self.patrol_start_combo.blockSignals(True)
        self.patrol_start_combo.setCurrentIndex(idx)
        self._patrol_start_idx = idx
        self.patrol_start_combo.blockSignals(False)

    def _cancel_arena(self) -> None:
        self.cancel_arena_btn.setEnabled(False)
        try:
            self.client.arena_mission_cancel()
            self.arena_status_label.setText("Canceling Arena...")
        except HostClientError as e:
            self.cancel_arena_btn.setEnabled(True)
            QMessageBox.warning(self, "Error", str(e))
