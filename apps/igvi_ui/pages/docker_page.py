from __future__ import annotations

from PySide6.QtCore import QThread, QTimer, Signal, Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMenu,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError


_RUNNING = {"running", "restarting"}


class ComposeActionWorker(QThread):
    completed = Signal(str, dict)
    failed = Signal(str, str)

    def __init__(
        self,
        client: HostClient,
        action: str,
        services: list[str] | None = None,
        profile: str | None = None,
    ):
        super().__init__()
        self.client = client
        self.action = action
        self.services = services
        self.profile = profile

    def run(self) -> None:
        try:
            if self.action == "down":
                result = self.client.down()
            else:
                result = self.client.compose_action(
                    self.action,
                    services=self.services,
                    profile=self.profile,
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

    COLUMNS = ("Service", "Profile", "Status", "Health", "Image", "Ports")

    def __init__(self, client: HostClient):
        super().__init__()
        self.client = client
        self.services: list[dict] = []
        self.action_workers: list[ComposeActionWorker] = []
        self.log_worker: LogTailWorker | None = None
        self.busy_actions: set[str] = set()
        self.current_log_service: str | None = None
        self.profile_filter = "All"
        self._progress_busy: bool = False
        self._progress_target: str = ""
        self._pending_actions: dict[str, str] = {}
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
        self._reload_profiles()

    def shutdown(self) -> None:
        """Stop timers and join all worker threads. Idempotent.

        Docker is the default page, so its workers are the most likely to be
        alive at exit; a running QThread destroyed here aborts the process.
        """
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
        splitter.addWidget(self._build_table_panel())
        splitter.addWidget(self._build_log_panel())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, 1)

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("PanelHeader")
        row = QHBoxLayout(bar)
        row.setContentsMargins(12, 8, 12, 8)
        row.setSpacing(8)

        title = QLabel("Docker Compose")
        title.setObjectName("PanelTitle")
        row.addWidget(title)
        row.addSpacing(10)

        row.addWidget(QLabel("Profile:"))
        self.profile_combo = QComboBox()
        self.profile_combo.setMinimumWidth(140)
        self.profile_combo.addItem("All")
        self.profile_combo.currentTextChanged.connect(self._on_profile_filter_changed)
        row.addWidget(self.profile_combo)

        self.up_profile_button = QPushButton("Up Profile")
        self.up_profile_button.setObjectName("Primary")
        self.up_profile_button.clicked.connect(self._up_profile)
        row.addWidget(self.up_profile_button)

        row.addSpacing(12)
        self.dev_button = QPushButton("Dev Mode")
        self.dev_button.setCheckable(True)
        self.dev_button.clicked.connect(self._toggle_dev_mode)
        row.addWidget(self.dev_button)

        row.addStretch(1)

        self.start_button = QPushButton("Start")
        self.start_button.setObjectName("Primary")
        self.start_button.clicked.connect(lambda: self._run_action("start"))
        row.addWidget(self.start_button)

        self.restart_button = QPushButton("Restart")
        self.restart_button.clicked.connect(lambda: self._run_action("restart"))
        row.addWidget(self.restart_button)

        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("Danger")
        self.stop_button.clicked.connect(lambda: self._run_action("stop"))
        row.addWidget(self.stop_button)

        row.addSpacing(12)

        self.build_button = QPushButton("Build")
        self.build_button.clicked.connect(lambda: self._run_action("build"))
        row.addWidget(self.build_button)

        self.rebuild_button = QPushButton("Rebuild")
        self.rebuild_button.setObjectName("Warning")
        self.rebuild_button.clicked.connect(lambda: self._run_action("rebuild"))
        row.addWidget(self.rebuild_button)

        self.down_button = QPushButton("Down All")
        self.down_button.setObjectName("Danger")
        self.down_button.clicked.connect(self._run_down)
        row.addWidget(self.down_button)

        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh)
        row.addWidget(refresh)

        return bar

    def _build_table_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("Panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self.summary = QLabel("Loading services…")
        self.summary.setObjectName("Muted")
        layout.addWidget(self.summary)

        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(list(self.COLUMNS))
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        header.setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self._on_selection_changed)
        self.table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self._open_context_menu)
        layout.addWidget(self.table, 1)
        return panel

    def _build_log_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("Panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        header = QHBoxLayout()
        self.log_title = QLabel("Logs — (select a service)")
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
            health = self.client.health()
            self.dev_button.setChecked(bool(health.get("dev_mode", False)))
            self.services = self.client.services()
        except HostClientError as exc:
            self.summary.setText(f"Host Agent unavailable: {exc}")
            self.services = []
        self._render_services()

    def _reload_profiles(self) -> None:
        try:
            profiles = self.client.profiles()
        except HostClientError:
            return
        current = self.profile_combo.currentText() or "All"
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        self.profile_combo.addItem("All")
        for profile in profiles:
            self.profile_combo.addItem(profile)
        idx = self.profile_combo.findText(current)
        self.profile_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self.profile_combo.blockSignals(False)

    def _filtered_services(self) -> list[dict]:
        if self.profile_filter == "All":
            return self.services
        return [s for s in self.services if str(s.get("profile") or "") == self.profile_filter]

    def _render_services(self) -> None:
        filtered = self._filtered_services()
        self.table.setRowCount(len(filtered))
        running = 0
        for row, service in enumerate(filtered):
            name = str(service.get("service", ""))
            status = str(service.get("status") or "")
            if status in _RUNNING:
                running += 1
            self.table.setItem(row, 0, QTableWidgetItem(name))
            self.table.setItem(row, 1, QTableWidgetItem(str(service.get("profile") or "")))
            pending = self._pending_actions.get(name)
            if pending:
                self.table.setItem(row, 2, self._make_pending_item(pending))
            else:
                self.table.setItem(row, 2, self._make_status_item(status))
            self.table.setItem(row, 3, QTableWidgetItem(str(service.get("health") or "")))
            self.table.setItem(row, 4, QTableWidgetItem(str(service.get("image") or "")))
            self.table.setItem(row, 5, QTableWidgetItem(", ".join(service.get("ports") or [])))
        self.table.resizeColumnToContents(0)
        self.table.resizeColumnToContents(1)
        self.table.resizeColumnToContents(2)
        self.table.resizeColumnToContents(3)
        total = len(self.services)
        shown = len(filtered)
        self.summary.setText(
            f"{running} running · {shown} shown · {total} total"
            + (f" · filter: {self.profile_filter}" if self.profile_filter != "All" else "")
        )

    def _make_status_item(self, status: str) -> QTableWidgetItem:
        item = QTableWidgetItem(status)
        if status in _RUNNING:
            item.setForeground(Qt.GlobalColor.green)
        elif status in {"exited", "dead"}:
            item.setForeground(Qt.GlobalColor.red)
        elif status in {"not_created", ""}:
            item.setForeground(Qt.GlobalColor.gray)
        elif status == "docker_unavailable":
            item.setForeground(Qt.GlobalColor.darkYellow)
        return item

    _PENDING_LABEL = {
        "start": "starting…",
        "restart": "restarting…",
        "stop": "stopping…",
        "build": "building…",
        "rebuild": "rebuilding…",
        "down": "stopping…",
    }

    def _make_pending_item(self, action: str) -> QTableWidgetItem:
        item = QTableWidgetItem(self._PENDING_LABEL.get(action, f"{action}…"))
        item.setForeground(Qt.GlobalColor.darkYellow)
        font = item.font()
        font.setItalic(True)
        item.setFont(font)
        return item

    # ------------------------------------------------------------- selection
    def _selected_services(self) -> list[str]:
        filtered = self._filtered_services()
        rows = sorted({index.row() for index in self.table.selectionModel().selectedRows()})
        services: list[str] = []
        for row in rows:
            if 0 <= row < len(filtered):
                name = str(filtered[row].get("service") or "")
                if name and name not in services:
                    services.append(name)
        return services

    def _on_selection_changed(self) -> None:
        services = self._selected_services()
        if len(services) == 1:
            self._show_log(services[0])
        elif len(services) > 1:
            self.log_title.setText(f"Logs — {len(services)} services selected")
            self.log_view.setPlainText(
                "Multiple services selected:\n"
                + "\n".join(f"- {service}" for service in services)
                + "\n\nUse the toolbar Start / Stop / Restart buttons."
            )
            self.current_log_service = None

    def _on_profile_filter_changed(self, text: str) -> None:
        self.profile_filter = text or "All"
        self._render_services()

    # ----------------------------------------------------------- context menu
    def _open_context_menu(self, position) -> None:
        services = self._selected_services()
        if not services:
            return
        menu = QMenu(self)
        single = services[0] if len(services) == 1 else None
        if single:
            menu.addAction(f"Show logs — {single}", lambda: self._show_log(single))
            menu.addSeparator()
        menu.addAction(f"Start ({len(services)})", lambda: self._run_action("start"))
        menu.addAction(f"Restart ({len(services)})", lambda: self._run_action("restart"))
        menu.addAction(f"Stop ({len(services)})", lambda: self._run_action("stop"))
        menu.addSeparator()
        menu.addAction(f"Build ({len(services)})", lambda: self._run_action("build"))
        menu.addAction(f"Rebuild ({len(services)})", lambda: self._run_action("rebuild"))
        menu.exec(self.table.viewport().mapToGlobal(position))

    # ----------------------------------------------------------------- logs
    def _show_log(self, service: str) -> None:
        self.current_log_service = service
        self.log_title.setText(f"Logs — {service}")
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

    # --------------------------------------------------------------- actions
    def _run_action(self, action: str) -> None:
        services = self._selected_services()
        if action in {"stop", "restart", "start"} and not services:
            QMessageBox.warning(self, "No service", "Select one or more services first.")
            return
        if action in {"build", "rebuild"} and not services:
            confirm = QMessageBox.question(
                self,
                f"{action.title()} all?",
                f"No services selected. {action.title()} every service in the project?",
            )
            if confirm != QMessageBox.StandardButton.Yes:
                return
        self._start_worker(action, services=services or None)

    def _up_profile(self) -> None:
        profile = self.profile_combo.currentText()
        if not profile or profile == "All":
            QMessageBox.warning(self, "Select profile", "Pick a profile other than All first.")
            return
        self._start_worker("start", profile=profile)

    def _run_down(self) -> None:
        confirm = QMessageBox.question(
            self,
            "Down all?",
            "This will stop and remove every container in the project. Continue?",
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return
        self._start_worker("down")

    def _start_worker(self, action: str, services: list[str] | None = None, profile: str | None = None) -> None:
        target = ", ".join(services) if services else profile or "all services"
        self.log_message.emit(f"{action} started for {target}")
        self.summary.setText(f"{action} running for {target}…")
        pending_targets = services or ([s["service"] for s in self.services] if action == "down" else [])
        for name in pending_targets:
            self._pending_actions[name] = action
        if pending_targets:
            self._render_services()
        worker = ComposeActionWorker(self.client, action, services=services, profile=profile)
        worker.completed.connect(lambda a, r, names=list(pending_targets): self._action_completed(a, r, names))
        worker.failed.connect(lambda a, m, names=list(pending_targets): self._action_failed(a, m, names))
        worker.finished.connect(lambda worker=worker: self._worker_finished(worker))
        self.action_workers.append(worker)
        self.busy_actions.add(action)
        self._progress_target = target
        if action in {"start", "build", "rebuild"}:
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
            short = last if len(last) <= 160 else last[:157] + "…"
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
            self.dev_button,
            self.up_profile_button,
            self.start_button,
            self.restart_button,
            self.stop_button,
            self.build_button,
            self.rebuild_button,
            self.down_button,
        ):
            button.setEnabled(not busy)

    def _toggle_dev_mode(self) -> None:
        try:
            self.client.set_dev_mode(self.dev_button.isChecked())
        except HostClientError as exc:
            QMessageBox.critical(self, "Dev mode failed", str(exc))
            self.dev_button.setChecked(not self.dev_button.isChecked())
            return
        self._reload_profiles()
        self.refresh()
