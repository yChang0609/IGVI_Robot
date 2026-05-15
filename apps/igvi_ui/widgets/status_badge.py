from __future__ import annotations

from PySide6.QtWidgets import QLabel

from igvi_ui.theme import badge_style


class StatusBadge(QLabel):
    def __init__(self, text: str, state: str = "muted"):
        super().__init__(text)
        self.setMinimumHeight(30)
        self.set_state(text, state)

    def set_state(self, text: str, state: str = "muted") -> None:
        self.setText(text)
        self.setStyleSheet(badge_style(state))
