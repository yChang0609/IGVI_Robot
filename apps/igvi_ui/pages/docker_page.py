from __future__ import annotations

from PySide6.QtCore import QSize, QThread, QTimer, Signal, Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMenu,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTextEdit,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError


_RUNNING = {"running", "restarting"}

_UI_GROUPS: list[tuple[str, str]] = [
    ("communication", "Communication"),
    ("robot", "Robot"),
    ("slam_system", "SLAM System"),
    ("application", "Application"),
    ("task_server", "Task Server"),
]

_SLAM_MUTEX = {
    "slam_fusion": "slam_localization",
    "slam_localization": "slam_fusion",
}

_TREE_STYLE = """
QTreeWidget {
    border: 1px solid #1e2228;
    outline: none;
}
QTreeWidget::item {
    padding: 3px 0px;
}
QTreeWidget::item:hover {
    background: #181c22;
}
QTreeWidget::item:selected {
    background: #1a2636;
}
"""

_GRP_BTN = (
    "QPushButton { padding: 3px 16px; border-radius: 4px;"
    " font-size: 12px; font-weight: 500; min-width: 70px; }"
)
_GRP_START_STYLE = (
    _GRP_BTN
    + " QPushButton { background: #24472e; color: #b8dcc4; border: 1px solid #2f5c3c; }"
    " QPushButton:hover { background: #2f5c3c; }"
)
_GRP_STOP_STYLE = (
    _GRP_BTN
    + " QPushButton { background: #47242a; color: #dcb8bc; border: 1px solid #5c2f35; }"
    " QPushButton:hover { background: #5c2f35; }"
)
_GRP_SLAM_STYLE = (
    _GRP_BTN
    + " QPushButton { background: #243047; color: #b8c4dc; border: 1px solid #2f405c; }"
    " QPushButton:hover { background: #2f405c; }"
)
_GRP_RESTART_STYLE = (
    _GRP_BTN
    + " QPushButton { background: #2d2e1a; color: #d8d4a0; border: 1px solid #4a4820; }"
    " QPushButton:hover { background: #4a4820; }"
)


class ComposeActionWorker(QThread):
    completed = Signal(str, dict)
    failed = Signal(str, str)

    def __init__(
        self,
        client: HostClient,
        action: str,
        services: list[str] | None = None,
        profile: str | None = None,
        no_cache: bool = False,
    ):
        super().__init__()
        self.client = client
        self.action = action
        self.services = services
        self.profile = profile
        self.no_cache = no_cache

    def run(self) -> None:
        try:
            if self.action == "stop_all":
                result = self.client.stop_all()
            elif self.action == "remove_all":
                result = self.client.remove_all()
            else:
                result = self.client.compose_action(
                    self.action,
                    services=self.services,
                    profile=self.profile,
                    no_cache=self.no_cache,
                )
        except HostClientError as exc:
            self.failed.emit(self.action, str(exc))
            return
        self.completed.emit(self.action, result)


class LogTailWorker(QThread):
    fetched = Signal(str, str)
    failed = Signal(str, str)

    def __init__(self, client: HostClient, service: str, tail: int = 400):
        super().__init__()
        self.client = client
        self.service = service
        self.tail = tail

    def run(self) -> None:
        try:
            text = self.client.logs(self.service, tail=self.tail)
        except HostClientError as exc:
            self.failed.emit(self.service, str(exc))
            return
        self.fetched.emit(self.service, text)


class DockerPage(QWidget):
    log_message = Signal(str)

    COLUMNS = ("Service", "Status", "Health", "Image", "Ports")

    def __init__(self, client: HostClient):
        super().__init__()
        self.client = client
        self.services: list[dict] = []
        self.action_workers: list[ComposeActionWorker] = []
        self.log_worker: LogTailWorker | None = None
        self.busy_actions: set[str] = set()
        self.current_log_service: str | None = None
        self._progress_busy: bool = False
        self._progress_target: str = ""
        self._pending_actions: dict[str, str] = {}
        self._group_items: dict[str, QTreeWidgetItem] = {}
        self._build_ui()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(5000)

        self.log_timer = QTimer(self)
        self.log_timer.timeout.connect(self._reload_log_if_following)
        self.log_timer.start(3000)

        self.progress_timer = QTimer(self)
        self.progress_timer.timeout.connect(self._poll_progress)

        self.refresh()

    def shutdown(self) -> None:
        for timer in (self.refresh_timer, self.log_timer, self.progress_timer):
            timer.stop()
        stop_thread(self.log_worker)
        self.log_worker = None
        for worker in list(self.action_workers):
            stop_thread(worker)
        self.action_workers.clear()

    # ------------------------------------------------------------------ build
    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        layout.addWidget(self._build_toolbar())

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(self._build_tree_panel())
        splitter.addWidget(self._build_log_panel())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, 1)

    @staticmethod
    def _toolbar_sep() -> QFrame:
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setFixedWidth(2)
        sep.setStyleSheet("color: #333a44;")
        return sep

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("PanelHeader")
        row = QHBoxLayout(bar)
        row.setContentsMargins(12, 8, 12, 8)
        row.setSpacing(4)

        title = QLabel("Docker Compose")
        title.setObjectName("PanelTitle")
        row.addWidget(title)
        row.addStretch(1)

        # Group: Start | Start+Build
        self.start_button = QPushButton("Start")
        self.start_button.setObjectName("Primary")
        self.start_button.clicked.connect(lambda: self._run_action("start"))
        row.addWidget(self.start_button)

        self.build_start_button = QPushButton("Start+Build")
        self.build_start_button.setObjectName("Primary")
        self.build_start_button.clicked.connect(lambda: self._run_action("build_start"))
        row.addWidget(self.build_start_button)

        row.addWidget(self._toolbar_sep())

        # Group: Stop | Stop+Rm
        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("Danger")
        self.stop_button.clicked.connect(lambda: self._run_action("stop"))
        row.addWidget(self.stop_button)

        self.stop_rm_button = QPushButton("Stop+Rm")
        self.stop_rm_button.setObjectName("Danger")
        self.stop_rm_button.clicked.connect(lambda: self._run_action("remove"))
        row.addWidget(self.stop_rm_button)

        row.addWidget(self._toolbar_sep())

        # Restart
        self.restart_button = QPushButton("Restart")
        self.restart_button.clicked.connect(lambda: self._run_action("restart"))
        row.addWidget(self.restart_button)

        row.addWidget(self._toolbar_sep())

        # Group: Build | Rebuild | Cache
        self.build_button = QPushButton("Build")
        self.build_button.clicked.connect(lambda: self._run_action("build"))
        row.addWidget(self.build_button)

        self.rebuild_button = QPushButton("Rebuild")
        self.rebuild_button.setObjectName("Warning")
        self.rebuild_button.clicked.connect(lambda: self._run_action("rebuild"))
        row.addWidget(self.rebuild_button)

        self.cache_check = QCheckBox("Cache")
        self.cache_check.setChecked(True)
        row.addWidget(self.cache_check)

        row.addWidget(self._toolbar_sep())

        # Group: Stop All | Rm All
        self.stop_all_button = QPushButton("Stop All")
        self.stop_all_button.setObjectName("Danger")
        self.stop_all_button.clicked.connect(self._run_stop_all)
        row.addWidget(self.stop_all_button)

        self.remove_all_button = QPushButton("Remove All")
        self.remove_all_button.setObjectName("Danger")
        self.remove_all_button.clicked.connect(self._run_remove_all)
        row.addWidget(self.remove_all_button)

        row.addWidget(self._toolbar_sep())

        # Refresh
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh)
        row.addWidget(refresh)

        return bar

    def _build_tree_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("Panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self.summary = QLabel("Loading services...")
        self.summary.setObjectName("Muted")
        self.summary.setFixedHeight(20)
        layout.addWidget(self.summary)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(list(self.COLUMNS))
        self.tree.setAlternatingRowColors(False)
        self.tree.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.tree.setRootIsDecorated(True)
        self.tree.setExpandsOnDoubleClick(False)
        self.tree.setIndentation(24)
        self.tree.setUniformRowHeights(False)
        self.tree.setStyleSheet(_TREE_STYLE)
        header = self.tree.header()
        header.setMinimumSectionSize(60)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.tree.setColumnWidth(0, 200)
        self.tree.setColumnWidth(1, 110)
        self.tree.setColumnWidth(2, 80)
        self.tree.itemSelectionChanged.connect(self._on_selection_changed)
        self.tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._open_context_menu)
        layout.addWidget(self.tree, 1)
        return panel

    def _build_log_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("Panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        header = QHBoxLayout()
        self.log_title = QLabel("Logs -- (select a service)")
        self.log_title.setObjectName("PanelTitle")
        header.addWidget(self.log_title)
        header.addStretch(1)

        self.follow_check = QCheckBox("Follow")
        self.follow_check.setChecked(True)
        header.addWidget(self.follow_check)

        reload_btn = QPushButton("Reload")
        reload_btn.clicked.connect(self._reload_log)
        header.addWidget(reload_btn)

        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(lambda: self.log_view.clear())
        header.addWidget(clear_btn)

        layout.addLayout(header)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        layout.addWidget(self.log_view, 1)
        return panel

    # --------------------------------------------------------------- refresh
    def refresh(self) -> None:
        try:
            self.client.health()
            self.services = self.client.services()
        except HostClientError as exc:
            self.summary.setText(f"Host Agent unavailable: {exc}")
            self.services = []
        self._render_services()

    def _services_by_profile(self) -> dict[str, list[dict]]:
        grouped: dict[str, list[dict]] = {}
        for svc in self.services:
            profile = str(svc.get("profile") or "default")
            grouped.setdefault(profile, []).append(svc)
        return grouped

    def _render_services(self) -> None:
        scrollbar = self.tree.verticalScrollBar()
        saved_scroll = scrollbar.value()

        collapsed = {
            key for key, item in self._group_items.items()
            if not item.isExpanded()
        }
        selected_names = set(self._selected_services())
        col_widths = [self.tree.columnWidth(i) for i in range(len(self.COLUMNS))]

        self.tree.clear()
        self._group_items.clear()
        grouped = self._services_by_profile()
        total = 0
        running = 0

        known_profiles = [p for p, _ in _UI_GROUPS]

        for profile_key, display_name in _UI_GROUPS:
            services = grouped.pop(profile_key, [])
            if not services:
                continue
            group_item = self._create_group(profile_key, display_name, services)
            self.tree.addTopLevelItem(group_item)
            row = self.tree.indexOfTopLevelItem(group_item)
            self.tree.setFirstColumnSpanned(row, self.tree.rootIndex(), True)
            self._group_items[profile_key] = group_item
            self._attach_group_buttons(group_item, profile_key, services)
            for svc in services:
                self._add_service_child(group_item, svc)
                total += 1
                if str(svc.get("status", "")) in _RUNNING:
                    running += 1

        for profile_key, services in sorted(grouped.items()):
            if not services:
                continue
            group_item = self._create_group(profile_key, profile_key.title(), services)
            self.tree.addTopLevelItem(group_item)
            row = self.tree.indexOfTopLevelItem(group_item)
            self.tree.setFirstColumnSpanned(row, self.tree.rootIndex(), True)
            self._group_items[profile_key] = group_item
            self._attach_group_buttons(group_item, profile_key, services)
            for svc in services:
                self._add_service_child(group_item, svc)
                total += 1
                if str(svc.get("status", "")) in _RUNNING:
                    running += 1

        self.tree.expandAll()
        for key, item in self._group_items.items():
            if key in collapsed:
                item.setExpanded(False)

        # Restore column widths so header doesn't jump on each refresh.
        for i, w in enumerate(col_widths):
            if w > 0 and i < len(self.COLUMNS) - 2:  # skip Stretch cols (3, 4)
                self.tree.setColumnWidth(i, w)

        # Restore selection without triggering log reload.
        if selected_names:
            self.tree.blockSignals(True)
            for top_i in range(self.tree.topLevelItemCount()):
                group = self.tree.topLevelItem(top_i)
                for child_i in range(group.childCount()):
                    child = group.child(child_i)
                    name = str(child.data(0, Qt.ItemDataRole.UserRole) or "")
                    if name in selected_names:
                        child.setSelected(True)
            self.tree.blockSignals(False)

        self.summary.setText(f"{running} running · {total} total")

        if saved_scroll > 0:
            QTimer.singleShot(0, lambda v=saved_scroll: self.tree.verticalScrollBar().setValue(v))

        _NOT_CREATED = {"not_created", "", "docker_unavailable"}
        created = sum(1 for s in self.services if str(s.get("status", "")) not in _NOT_CREATED)
        self.stop_all_button.setEnabled(running > 0)
        self.remove_all_button.setEnabled(created > 0)

    def _create_group(self, profile_key: str, display_name: str, services: list[dict]) -> QTreeWidgetItem:
        running_count = sum(1 for s in services if str(s.get("status", "")) in _RUNNING)
        item = QTreeWidgetItem()
        item.setText(0, f"{display_name}  ({running_count}/{len(services)} running)")
        item.setData(0, Qt.ItemDataRole.UserRole, profile_key)
        font = item.font(0)
        font.setBold(True)
        font.setPointSize(font.pointSize() + 1)
        item.setFont(0, font)
        item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsSelectable)
        item.setSizeHint(0, QSize(-1, 38))
        return item

    def _running_in_group(self, svc_names: list[str]) -> list[str]:
        running = {str(s.get("service")) for s in self.services if str(s.get("status", "")) in _RUNNING}
        return [n for n in svc_names if n in running]

    def _group_restart(self, svc_names: list[str]) -> None:
        targets = self._running_in_group(svc_names)
        if not targets:
            QMessageBox.information(self, "Nothing running", "No services in this group are currently running.")
            return
        self._start_worker("restart", services=targets)

    def _group_stop_running(self, svc_names: list[str]) -> None:
        targets = self._running_in_group(svc_names)
        if targets:
            self._start_worker("remove", services=targets)

    def _attach_group_buttons(self, group_item: QTreeWidgetItem, profile_key: str, services: list[dict]) -> None:
        widget = QWidget()
        widget.setStyleSheet("background: transparent;")
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(4, 4, 8, 4)
        layout.setSpacing(6)

        label = QLabel(group_item.text(0))
        font = label.font()
        font.setBold(True)
        font.setPointSize(font.pointSize() + 1)
        label.setFont(font)
        layout.addWidget(label)
        layout.addStretch(1)

        svc_names = [str(s.get("service", "")) for s in services]

        if profile_key == "slam_system":
            construction_btn = QPushButton("Construction")
            construction_btn.setStyleSheet(_GRP_SLAM_STYLE)
            construction_btn.setFixedHeight(26)
            construction_btn.clicked.connect(lambda: self._slam_switch("slam_fusion"))
            layout.addWidget(construction_btn)

            localization_btn = QPushButton("Localization")
            localization_btn.setStyleSheet(_GRP_SLAM_STYLE)
            localization_btn.setFixedHeight(26)
            localization_btn.clicked.connect(lambda: self._slam_switch("slam_localization"))
            layout.addWidget(localization_btn)

            restart_btn = QPushButton("Restart")
            restart_btn.setStyleSheet(_GRP_RESTART_STYLE)
            restart_btn.setFixedHeight(26)
            restart_btn.clicked.connect(lambda _=False, names=svc_names: self._group_restart(names))
            layout.addWidget(restart_btn)

            stop_btn = QPushButton("Stop+Rm")
            stop_btn.setStyleSheet(_GRP_STOP_STYLE)
            stop_btn.setFixedHeight(26)
            stop_btn.clicked.connect(lambda _=False, names=svc_names: self._group_stop_running(names))
            layout.addWidget(stop_btn)
        else:
            start_btn = QPushButton("Start")
            start_btn.setStyleSheet(_GRP_START_STYLE)
            start_btn.setFixedHeight(26)
            start_btn.clicked.connect(lambda _=False, names=svc_names: self._start_worker("start", services=names))
            layout.addWidget(start_btn)

            restart_btn = QPushButton("Restart")
            restart_btn.setStyleSheet(_GRP_RESTART_STYLE)
            restart_btn.setFixedHeight(26)
            restart_btn.clicked.connect(lambda _=False, names=svc_names: self._group_restart(names))
            layout.addWidget(restart_btn)

            stop_btn = QPushButton("Stop+Rm")
            stop_btn.setStyleSheet(_GRP_STOP_STYLE)
            stop_btn.setFixedHeight(26)
            stop_btn.clicked.connect(lambda _=False, names=svc_names: self._start_worker("remove", services=names))
            layout.addWidget(stop_btn)

        self.tree.setItemWidget(group_item, 0, widget)
        group_item.setText(0, "")

    def _add_service_child(self, parent: QTreeWidgetItem, svc: dict) -> None:
        name = str(svc.get("service", ""))
        status = str(svc.get("status") or "")
        child = QTreeWidgetItem(parent)
        child.setText(0, name)
        child.setData(0, Qt.ItemDataRole.UserRole, name)
        pending = self._pending_actions.get(name)
        if pending:
            child.setText(1, self._PENDING_LABEL.get(pending, f"{pending}..."))
            child.setForeground(1, Qt.GlobalColor.darkYellow)
            font = child.font(1)
            font.setItalic(True)
            child.setFont(1, font)
        else:
            child.setText(1, status)
            if status in _RUNNING:
                child.setForeground(1, Qt.GlobalColor.green)
            elif status in {"exited", "dead"}:
                child.setForeground(1, Qt.GlobalColor.red)
            elif status in {"not_created", ""}:
                child.setForeground(1, Qt.GlobalColor.gray)
            elif status == "docker_unavailable":
                child.setForeground(1, Qt.GlobalColor.darkYellow)
        child.setText(2, str(svc.get("health") or ""))
        child.setText(3, str(svc.get("image") or ""))
        child.setText(4, ", ".join(svc.get("ports") or []))
        child.setSizeHint(0, QSize(-1, 28))

    _PENDING_LABEL = {
        "start": "starting...",
        "restart": "restarting...",
        "stop": "stopping...",
        "build": "building...",
        "rebuild": "rebuilding...",
        "build_start": "building & starting...",
        "stop_all": "stopping...",
        "remove_all": "removing...",
        "remove": "removing...",
    }

    # ------------------------------------------------------------- selection
    def _selected_services(self) -> list[str]:
        services: list[str] = []
        for item in self.tree.selectedItems():
            if item.parent() is not None:
                name = str(item.data(0, Qt.ItemDataRole.UserRole) or "")
                if name and name not in services:
                    services.append(name)
        return services

    def _on_selection_changed(self) -> None:
        services = self._selected_services()
        if len(services) == 1:
            self._show_log(services[0])
        elif len(services) > 1:
            self.log_title.setText(f"Logs -- {len(services)} services selected")
            self.log_view.setPlainText(
                "Multiple services selected:\n"
                + "\n".join(f"- {service}" for service in services)
                + "\n\nUse the toolbar Start / Stop / Restart buttons."
            )
            self.current_log_service = None

    # ----------------------------------------------------------- context menu
    def _open_context_menu(self, position) -> None:
        services = self._selected_services()
        if not services:
            return
        menu = QMenu(self)
        single = services[0] if len(services) == 1 else None
        if single:
            menu.addAction(f"Show logs -- {single}", lambda: self._show_log(single))
            menu.addSeparator()
        menu.addAction(f"Start ({len(services)})", lambda: self._run_action("start"))
        menu.addAction(f"Restart ({len(services)})", lambda: self._run_action("restart"))
        menu.addAction(f"Stop ({len(services)})", lambda: self._run_action("stop"))
        menu.addSeparator()
        menu.addAction(f"Build ({len(services)})", lambda: self._run_action("build"))
        menu.addAction(f"Rebuild ({len(services)})", lambda: self._run_action("rebuild"))
        menu.addAction(f"Start + Build ({len(services)})", lambda: self._run_action("build_start"))
        menu.addAction(f"Stop + Rm ({len(services)})", lambda: self._run_action("remove"))
        menu.exec(self.tree.viewport().mapToGlobal(position))

    # ----------------------------------------------------------------- logs
    def _show_log(self, service: str) -> None:
        self.current_log_service = service
        self.log_title.setText(f"Logs -- {service}")
        self._reload_log()

    def _reload_log(self) -> None:
        service = self.current_log_service
        if not service:
            return
        if self.log_worker is not None and self.log_worker.isRunning():
            return
        worker = LogTailWorker(self.client, service, tail=400)
        worker.fetched.connect(self._log_fetched)
        worker.failed.connect(self._log_failed)
        worker.finished.connect(self._log_worker_finished)
        self.log_worker = worker
        worker.start()

    def _reload_log_if_following(self) -> None:
        if self.follow_check.isChecked() and self.current_log_service:
            self._reload_log()

    def _log_fetched(self, service: str, text: str) -> None:
        if service != self.current_log_service:
            return
        scrollbar = self.log_view.verticalScrollBar()
        at_bottom = scrollbar.value() >= scrollbar.maximum() - 4
        self.log_view.setPlainText(text)
        if at_bottom or self.follow_check.isChecked():
            scrollbar.setValue(scrollbar.maximum())

    def _log_failed(self, service: str, message: str) -> None:
        if service != self.current_log_service:
            return
        self.log_view.setPlainText(f"Failed to load logs for {service}:\n{message}")

    def _log_worker_finished(self) -> None:
        self.log_worker = None

    # -------------------------------------------------------- SLAM mutex
    def _slam_switch(self, target: str) -> None:
        rival = _SLAM_MUTEX.get(target)
        if not rival:
            self._start_worker("start", services=[target])
            return

        rival_running = any(
            str(s.get("service")) == rival and str(s.get("status", "")) in _RUNNING
            for s in self.services
        )
        if rival_running:
            target_label = "Construction (slam_fusion)" if target == "slam_fusion" else "Localization (slam_localization)"
            rival_label = "Construction (slam_fusion)" if rival == "slam_fusion" else "Localization (slam_localization)"
            answer = QMessageBox.question(
                self,
                "SLAM Conflict",
                f"{rival_label} is currently running.\n"
                f"It shares the same database files as {target_label} -- "
                f"running both simultaneously will corrupt the map.\n\n"
                f"Stop {rival_label} and start {target_label}?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
            self._slam_stop_then_start(rival, target)
        else:
            self._start_worker("start", services=[target])

    def _slam_stop_then_start(self, stop_service: str, start_service: str) -> None:
        worker = ComposeActionWorker(self.client, "stop", services=[stop_service])
        self._pending_actions[stop_service] = "stop"
        self._render_services()

        def on_stop_done(_action: str, _result: dict) -> None:
            self._pending_actions.pop(stop_service, None)
            self.refresh()
            self._start_worker("start", services=[start_service])

        def on_stop_fail(_action: str, message: str) -> None:
            self._pending_actions.pop(stop_service, None)
            self._render_services()
            QMessageBox.critical(self, "SLAM switch failed", f"Could not stop {stop_service}:\n{message}")

        worker.completed.connect(on_stop_done)
        worker.failed.connect(on_stop_fail)
        worker.finished.connect(lambda w=worker: self._worker_finished(w))
        self.action_workers.append(worker)
        self.busy_actions.add("stop")
        self._set_busy(True)
        worker.start()

    # --------------------------------------------------------------- actions
    def _run_action(self, action: str) -> None:
        services = self._selected_services()
        if action in {"stop", "restart", "start", "remove"} and not services:
            QMessageBox.warning(self, "No service", "Select one or more services first.")
            return
        if action in {"build", "rebuild", "build_start"} and not services:
            confirm = QMessageBox.question(
                self,
                f"{action.title()} all?",
                f"No services selected. {action.title()} every service in the project?",
            )
            if confirm != QMessageBox.StandardButton.Yes:
                return

        if action in {"start", "build_start", "restart"}:
            conflict = self._check_slam_conflict(services)
            if conflict is not None:
                return

        self._start_worker(action, services=services or None)

    def _check_slam_conflict(self, services: list[str]) -> str | None:
        for svc in services:
            rival = _SLAM_MUTEX.get(svc)
            if rival and rival not in services:
                rival_running = any(
                    str(s.get("service")) == rival and str(s.get("status", "")) in _RUNNING
                    for s in self.services
                )
                if rival_running:
                    QMessageBox.warning(
                        self,
                        "SLAM Conflict",
                        f"Cannot start {svc} while {rival} is running.\n"
                        f"They share the same database files.\n\n"
                        f"Stop {rival} first, or use the Construction/Localization buttons in the SLAM System group.",
                    )
                    return svc
        return None

    def _run_stop_all(self) -> None:
        confirm = QMessageBox.question(
            self,
            "Stop all?",
            "Stop all running containers? They will remain and can be restarted.",
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return
        self._start_worker("stop_all")

    def _run_remove_all(self) -> None:
        confirm = QMessageBox.question(
            self,
            "Remove all?",
            "Remove all project containers including orphans?\n\nThe list will reset to a clean state.",
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return
        self._start_worker("remove_all")

    def _start_worker(self, action: str, services: list[str] | None = None, profile: str | None = None) -> None:
        no_cache = not self.cache_check.isChecked() if action in {"build", "build_start"} else False
        target = ", ".join(services) if services else profile or "all services"
        self.log_message.emit(f"{action} started for {target}")
        self.summary.setText(f"{action} running for {target}...")
        pending_targets = services or ([s["service"] for s in self.services] if action in {"stop_all", "remove_all"} else [])
        for name in pending_targets:
            self._pending_actions[name] = action
        if pending_targets:
            self._render_services()
        worker = ComposeActionWorker(self.client, action, services=services, profile=profile, no_cache=no_cache)
        worker.completed.connect(lambda a, r, names=list(pending_targets): self._action_completed(a, r, names))
        worker.failed.connect(lambda a, m, names=list(pending_targets): self._action_failed(a, m, names))
        worker.finished.connect(lambda worker=worker: self._worker_finished(worker))
        self.action_workers.append(worker)
        self.busy_actions.add(action)
        self._progress_target = target
        if action in {"start", "build", "rebuild", "build_start", "stop_all", "remove_all", "remove"}:
            self._progress_busy = True
            self.progress_timer.start(800)
        self._set_busy(True)
        worker.start()

    def _poll_progress(self) -> None:
        try:
            data = self.client.compose_progress(tail=1)
        except HostClientError:
            return
        last = str(data.get("last_line") or "").strip()
        action = str(data.get("action") or "")
        busy = bool(data.get("busy"))
        if last:
            short = last if len(last) <= 160 else last[:157] + "..."
            self.summary.setText(f"{action or 'compose'} · {self._progress_target} · {short}")
        if not busy:
            self.progress_timer.stop()
            self._progress_busy = False

    def _action_completed(self, action: str, result: dict, services: list[str] | None = None) -> None:
        message = str(result.get("message", result))
        self.log_message.emit(f"{action} ok · {message}")
        for name in services or []:
            self._pending_actions.pop(name, None)
        self.refresh()

    def _action_failed(self, action: str, message: str, services: list[str] | None = None) -> None:
        self.log_message.emit(f"{action} failed")
        for name in services or []:
            self._pending_actions.pop(name, None)
        self._render_services()
        QMessageBox.critical(self, f"{action} failed", message)

    def _worker_finished(self, worker: ComposeActionWorker) -> None:
        if worker in self.action_workers:
            self.action_workers.remove(worker)
        if not self.action_workers:
            self.busy_actions.clear()
            self._set_busy(False)
            self.progress_timer.stop()
            self._progress_busy = False

    def _set_busy(self, busy: bool) -> None:
        for button in (
            self.start_button,
            self.build_start_button,
            self.restart_button,
            self.stop_button,
            self.stop_rm_button,
            self.build_button,
            self.rebuild_button,
            self.stop_all_button,
            self.remove_all_button,
        ):
            button.setEnabled(not busy)
