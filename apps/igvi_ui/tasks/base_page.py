from __future__ import annotations

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.tasks.state_machine import TaskState


class _StatusPoller(QThread):
    """Polls /api/task/{task_id}/status every 500 ms while the page is visible."""

    status_received = Signal(dict)

    def __init__(self, client: HostClient, task_id: str) -> None:
        super().__init__()
        self.client = client
        self.task_id = task_id
        self._running = True

    def run(self) -> None:
        while self._running:
            try:
                data = self.client.task_status(self.task_id)
                self.status_received.emit(data)
            except Exception:
                pass
            self.msleep(500)

    def stop(self) -> None:
        self._running = False


class BaseTaskPage(QWidget):
    """UI shell for a competition task.

    Displays the step list and live state from the ROS task_core node.
    Sends start / pause / resume / interrupt commands via the host API.
    Contains NO execution logic.

    Subclasses must set ACCENT, TASK_NUM, TASK_ID and implement _step_defs()
    returning [(step_name, step_description), ...].
    """

    task_state_changed = Signal(object)  # TaskState — consumed by sidebar button

    ACCENT: str = "#3b82f6"
    TASK_NUM: str = "1"
    TASK_ID: str = "task1"

    def __init__(self, title: str, description: str, client: HostClient) -> None:
        super().__init__()
        self._title = title
        self._description = description
        self.client = client
        self._poller: _StatusPoller | None = None
        self._current_state = TaskState.IDLE
        self._log_cursor: int = 0  # tracks how many log lines we have shown
        self._build_ui()

    # ── Subclass interface ────────────────────────────────────────────────────

    def _step_defs(self) -> list[tuple[str, str]]:
        """Return [(step_name, tooltip_description), ...] — display only."""
        raise NotImplementedError

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_header())
        root.addWidget(self._build_body(), 1)

    def _build_header(self) -> QFrame:
        header = QFrame()
        header.setFixedHeight(72)
        header.setStyleSheet(
            f"background: {self.ACCENT}18;"
            f"border-bottom: 2px solid {self.ACCENT}60;"
        )
        layout = QHBoxLayout(header)
        layout.setContentsMargins(20, 10, 20, 10)

        num_lbl = QLabel(self.TASK_NUM)
        num_lbl.setFixedWidth(44)
        num_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        num_lbl.setStyleSheet(
            f"font-size: 24px; font-weight: 900; color: {self.ACCENT};"
            " border: none; background: transparent;"
        )

        text_col = QVBoxLayout()
        title_lbl = QLabel(self._title)
        title_lbl.setStyleSheet(
            f"font-size: 16px; font-weight: 700; color: {self.ACCENT};"
            " border: none; background: transparent;"
        )
        desc_lbl = QLabel(self._description)
        desc_lbl.setStyleSheet(
            "color: #9aa7b8; font-size: 12px; border: none; background: transparent;"
        )
        text_col.addWidget(title_lbl)
        text_col.addWidget(desc_lbl)

        self._state_badge = QLabel("Idle")
        self._state_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state_badge.setFixedWidth(110)
        self._apply_badge_style(TaskState.IDLE)

        layout.addWidget(num_lbl)
        layout.addLayout(text_col, 1)
        layout.addWidget(self._state_badge)
        return header

    def _build_body(self) -> QWidget:
        body = QWidget()
        layout = QHBoxLayout(body)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)

        # Left: step list
        step_col = QVBoxLayout()
        lbl = QLabel("Steps")
        lbl.setStyleSheet("font-weight: 700; font-size: 13px;")
        self._step_list = QListWidget()
        self._step_list.setStyleSheet(
            "QListWidget { background: #171b22; border: 1px solid #323b48;"
            " border-radius: 8px; }"
            "QListWidget::item { padding: 9px 12px;"
            " border-bottom: 1px solid #232b36; color: #9aa7b8; }"
            "QListWidget::item:selected { background: transparent; }"
        )
        self._step_list.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self._step_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._populate_steps()
        step_col.addWidget(lbl)
        step_col.addWidget(self._step_list, 1)

        # Right: controls + log
        right_col = QVBoxLayout()
        right_col.setSpacing(10)
        right_col.addLayout(self._build_controls())

        log_lbl = QLabel("Log")
        log_lbl.setStyleSheet("font-weight: 700; font-size: 13px;")
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setStyleSheet(
            "QTextEdit { background: #0d1117; border: 1px solid #323b48;"
            " border-radius: 8px;"
            " font-family: 'SFMono-Regular', 'Consolas', monospace;"
            " font-size: 12px; color: #c9d1d9; }"
        )
        right_col.addWidget(log_lbl)
        right_col.addWidget(self._log, 1)

        layout.addLayout(step_col, 1)
        layout.addLayout(right_col, 2)
        return body

    def _build_controls(self) -> QHBoxLayout:
        row = QHBoxLayout()

        self._start_btn = QPushButton("▶  Start")
        self._start_btn.setObjectName("Primary")
        self._start_btn.clicked.connect(self._cmd_start)

        self._pause_btn = QPushButton("⏸  Pause")
        self._pause_btn.setEnabled(False)
        self._pause_btn.clicked.connect(self._cmd_pause_resume)

        self._interrupt_btn = QPushButton("■  Interrupt")
        self._interrupt_btn.setObjectName("Danger")
        self._interrupt_btn.setEnabled(False)
        self._interrupt_btn.clicked.connect(self._cmd_interrupt)

        row.addWidget(self._start_btn)
        row.addWidget(self._pause_btn)
        row.addWidget(self._interrupt_btn)
        row.addStretch()
        return row

    # ── Step list helpers ─────────────────────────────────────────────────────

    def _populate_steps(self) -> None:
        self._step_list.clear()
        for name, desc in self._step_defs():
            item = QListWidgetItem(f"  ○  {name}")
            item.setToolTip(desc)
            self._step_list.addItem(item)

    def _highlight_step(self, index: int, icon: str, color: str) -> None:
        item = self._step_list.item(index)
        if item is None:
            return
        name = self._step_defs()[index][0] if index < len(self._step_defs()) else "?"
        item.setText(f"  {icon}  {name}")
        item.setForeground(QBrush(QColor(color)))

    def _sync_steps_to(self, step_index: int, state: TaskState) -> None:
        """Refresh step list icons to match current execution position."""
        steps = self._step_defs()
        for i in range(len(steps)):
            if i < step_index:
                self._highlight_step(i, "✓", "#22c55e")
            elif i == step_index:
                if state == TaskState.RUNNING:
                    self._highlight_step(i, "▶", self.ACCENT)
                elif state == TaskState.PAUSED:
                    self._highlight_step(i, "⏸", "#f59e0b")
                elif state in (TaskState.FAILED,):
                    self._highlight_step(i, "✗", "#ef4444")
                elif state == TaskState.INTERRUPTED:
                    self._highlight_step(i, "✗", "#a855f7")
                else:
                    self._highlight_step(i, "○", "#9aa7b8")
            else:
                self._highlight_step(i, "○", "#9aa7b8")

        item = self._step_list.item(step_index)
        if item:
            self._step_list.scrollToItem(item)

    # ── Commands → host API ───────────────────────────────────────────────────

    def _cmd_start(self) -> None:
        try:
            self.client.task_start(self.TASK_ID)
        except HostClientError as exc:
            self._log.append(f"[error] start: {exc}")

    def _cmd_pause_resume(self) -> None:
        try:
            if self._current_state == TaskState.PAUSED:
                self.client.task_resume(self.TASK_ID)
            else:
                self.client.task_pause(self.TASK_ID)
        except HostClientError as exc:
            self._log.append(f"[error] pause/resume: {exc}")

    def _cmd_interrupt(self) -> None:
        try:
            self.client.task_interrupt(self.TASK_ID)
        except HostClientError as exc:
            self._log.append(f"[error] interrupt: {exc}")

    # ── Status polling slot ───────────────────────────────────────────────────

    def _on_status(self, data: dict) -> None:
        if not data.get("ok"):
            return

        state = TaskState.from_str(data.get("state", "idle"))
        step_index = int(data.get("step_index", 0))
        log_lines: list[str] = data.get("log", [])

        # Update state
        if state != self._current_state:
            self._current_state = state
            self._apply_badge_style(state)
            self.task_state_changed.emit(state)
            self._update_buttons(state)

        # Update step highlights
        self._sync_steps_to(step_index, state)

        # Append new log lines only
        new_lines = log_lines[self._log_cursor:]
        for line in new_lines:
            self._log.append(line)
        self._log_cursor = len(log_lines)

    def _update_buttons(self, state: TaskState) -> None:
        running = state == TaskState.RUNNING
        paused = state == TaskState.PAUSED
        active = running or paused
        terminal = state in (TaskState.COMPLETED, TaskState.FAILED, TaskState.INTERRUPTED)

        self._start_btn.setEnabled(state == TaskState.IDLE or terminal)
        self._pause_btn.setEnabled(active)
        self._interrupt_btn.setEnabled(active)

        if paused:
            self._pause_btn.setText("▶  Resume")
        else:
            self._pause_btn.setText("⏸  Pause")

        if terminal:
            # Reset log cursor so next run starts fresh
            self._log_cursor = 0

    # ── Badge style ───────────────────────────────────────────────────────────

    def _apply_badge_style(self, state: TaskState) -> None:
        c = state.color()
        self._state_badge.setText(state.label())
        self._state_badge.setStyleSheet(
            f"border: 1px solid {c}55; border-radius: 8px; padding: 4px 10px;"
            f"background: {c}18; color: {c}; font-weight: 600;"
        )

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._poller is None:
            self._poller = _StatusPoller(self.client, self.TASK_ID)
            self._poller.status_received.connect(self._on_status)
            self._poller.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self.shutdown()

    def shutdown(self) -> None:
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None
