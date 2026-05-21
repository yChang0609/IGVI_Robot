from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient
from igvi_ui.tasks.state_machine import TaskState
from igvi_ui.tasks.task1_bridge import Task1BridgePage
from igvi_ui.tasks.task2_bridge_grab import Task2BridgeGrabPage
from igvi_ui.tasks.task3_door import Task3DoorPage
from igvi_ui.tasks.task4_find_grab import Task4FindGrabPage


_TASKS = [
    # (num_label, title, accent_color, page_class)
    ("1", "上下橋",     "#3b82f6", Task1BridgePage),
    ("2", "上橋抓熊下橋", "#22c55e", Task2BridgeGrabPage),
    ("3", "導航開門",   "#f59e0b", Task3DoorPage),
    ("4", "導航找熊抓熊", "#a855f7", Task4FindGrabPage),
]


class _TaskNavButton(QPushButton):
    """Sidebar button for a single task — shows number, name, and live state badge."""

    def __init__(self, num: str, title: str, accent: str) -> None:
        super().__init__()
        self._accent = accent
        self._num = num
        self._title = title
        self._task_state = TaskState.IDLE
        self.setCheckable(True)
        self.setFixedHeight(80)
        self._num_lbl = QLabel(num)
        self._name_lbl = QLabel(title)
        self._badge_lbl = QLabel("Idle")

        self._num_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._name_lbl.setWordWrap(True)
        self._badge_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        inner = QVBoxLayout(self)
        inner.setContentsMargins(8, 6, 8, 6)
        inner.setSpacing(2)

        top_row = QHBoxLayout()
        top_row.addWidget(self._num_lbl)
        top_row.addWidget(self._name_lbl, 1)
        inner.addLayout(top_row)
        inner.addWidget(self._badge_lbl)

        self._apply_style(checked=False)

    def update_task_state(self, state: TaskState) -> None:
        self._task_state = state
        self._apply_style(self.isChecked())

    def setChecked(self, checked: bool) -> None:
        super().setChecked(checked)
        self._apply_style(checked)

    def _apply_style(self, checked: bool) -> None:
        s = self._task_state
        c = self._accent
        badge_color = s.color()

        bg = f"{c}22" if checked else "#12171e"
        border_left = f"4px solid {c}" if checked else f"4px solid {c}44"
        btn_style = (
            f"QPushButton {{"
            f"  background: {bg};"
            f"  border: 1px solid {'#3a4656' if not checked else c + '66'};"
            f"  border-left: {border_left};"
            f"  border-radius: 8px;"
            f"  text-align: left;"
            f"  padding: 0;"
            f"}}"
            f"QPushButton:hover {{ background: {c}18; }}"
        )
        self.setStyleSheet(btn_style)

        self._num_lbl.setStyleSheet(
            f"font-size: 22px; font-weight: 900; color: {c};"
            " background: transparent; border: none; min-width: 32px;"
        )
        self._name_lbl.setStyleSheet(
            f"font-size: 11px; font-weight: 600; color: {'#eef2f7' if checked else '#9aa7b8'};"
            " background: transparent; border: none;"
        )
        self._badge_lbl.setStyleSheet(
            f"font-size: 10px; color: {badge_color}; background: {badge_color}18;"
            f" border: 1px solid {badge_color}44; border-radius: 4px;"
            " padding: 1px 6px;"
        )
        self._badge_lbl.setText(s.label())


class TasksPage(QWidget):
    """Main Tasks page: left sidebar navigation + right stacked task subpages."""

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._task_pages: list[Task1BridgePage | Task2BridgeGrabPage | Task3DoorPage | Task4FindGrabPage] = []
        self._nav_buttons: list[_TaskNavButton] = []
        self._build_ui()

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Left sidebar ───────────────────────────────────────────────────────
        sidebar = QFrame()
        sidebar.setObjectName("TaskSidebar")
        sidebar.setFixedWidth(150)
        sidebar.setStyleSheet(
            "#TaskSidebar {"
            "  background: #0c0f14;"
            "  border-right: 1px solid #1e2a38;"
            "}"
        )
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(8, 14, 8, 14)
        sidebar_layout.setSpacing(8)

        header_lbl = QLabel("競賽任務")
        header_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_lbl.setStyleSheet(
            "font-size: 11px; font-weight: 700; color: #6f7e91;"
            " letter-spacing: 1px; padding-bottom: 6px;"
            " border-bottom: 1px solid #1e2a38; margin-bottom: 4px;"
        )
        sidebar_layout.addWidget(header_lbl)

        self._stack = QStackedWidget()

        for i, (num, title, accent, PageClass) in enumerate(_TASKS):
            page = PageClass(self.client)
            self._task_pages.append(page)
            self._stack.addWidget(page)

            btn = _TaskNavButton(num, title, accent)
            btn.clicked.connect(lambda _, idx=i: self._set_task(idx))
            self._nav_buttons.append(btn)
            sidebar_layout.addWidget(btn)

            # Wire live state updates back to the sidebar button
            page.task_state_changed.connect(
                lambda state, b=btn: b.update_task_state(state)
            )

        sidebar_layout.addStretch(1)

        # Active task indicator at bottom
        self._active_lbl = QLabel("Task 1 / 4")
        self._active_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._active_lbl.setStyleSheet("font-size: 10px; color: #6f7e91;")
        sidebar_layout.addWidget(self._active_lbl)

        root.addWidget(sidebar)
        root.addWidget(self._stack, 1)

        self._set_task(0)

    def _set_task(self, index: int) -> None:
        self._stack.setCurrentIndex(index)
        for i, btn in enumerate(self._nav_buttons):
            btn.setChecked(i == index)
        num, title, _, _ = _TASKS[index]
        self._active_lbl.setText(f"Task {num} / {len(_TASKS)}")

    def shutdown(self) -> None:
        for page in self._task_pages:
            shutdown = getattr(page, "shutdown", None)
            if callable(shutdown):
                shutdown()
