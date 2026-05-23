from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QSizePolicy

from igvi_ui.theme import badge_style


class StatusBadge(QLabel):
    def __init__(self, text: str, state: str = "muted"):
        super().__init__(text)
        self.setFixedHeight(30)
        # Fixed width prevents layout reflow when text changes every few seconds.
        self.setMaximumWidth(200)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self.set_state(text, state)

    def set_state(self, text: str, state: str = "muted") -> None:
        self.setText(text)
        self.setStyleSheet(badge_style(state))
