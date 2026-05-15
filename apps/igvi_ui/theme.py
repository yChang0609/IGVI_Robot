from __future__ import annotations


STYLE_SHEET = """
QMainWindow, QWidget {
    background: #101318;
    color: #eef2f7;
    font-family: "Inter", "Segoe UI", "Noto Sans TC";
    font-size: 13px;
}
QFrame#Rail {
    background: #0c0f14;
    border-right: 1px solid #323b48;
}
QFrame#TopBar, QFrame#PanelHeader {
    background: #12171e;
    border-bottom: 1px solid #323b48;
}
QFrame#Panel, QTableWidget, QTextEdit {
    background: #171b22;
    border: 1px solid #323b48;
    border-radius: 8px;
}
QLabel#Title {
    font-size: 21px;
    font-weight: 700;
}
QLabel#PanelTitle {
    font-size: 14px;
    font-weight: 700;
}
QLabel#Muted {
    color: #9aa7b8;
}
QPushButton {
    min-height: 30px;
    padding: 4px 10px;
    border: 1px solid #3a4656;
    border-radius: 7px;
    background: #202936;
    color: #e5edf8;
}
QPushButton:hover {
    border-color: #547198;
    background: #243145;
}
QPushButton:pressed {
    background: #1d2735;
}
QPushButton#Primary {
    border-color: #2563eb;
    background: #1d3a68;
}
QPushButton#Warning {
    border-color: #92400e;
    background: #3b2a14;
    color: #fde68a;
}
QPushButton#Danger {
    border-color: #7f1d1d;
    background: #3b1d25;
    color: #fecaca;
}
QPushButton#RailButton {
    min-width: 44px;
    min-height: 44px;
    border-radius: 8px;
    padding: 0;
}
QPushButton#RailButton:checked {
    border-color: #315f9e;
    background: #172842;
}
QTableWidget {
    gridline-color: #303a47;
    selection-background-color: #263448;
    alternate-background-color: #1b222c;
}
QHeaderView::section {
    background: #161c24;
    color: #d6deeb;
    border: 0;
    border-right: 1px solid #323b48;
    padding: 7px;
}
QLineEdit, QComboBox {
    min-height: 30px;
    border: 1px solid #3a4656;
    border-radius: 7px;
    background: #11161d;
    color: #eef2f7;
    padding: 2px 8px;
}
QSlider::groove:horizontal {
    height: 5px;
    background: #303a47;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 8px;
    background: #3b82f6;
}
QTextEdit {
    font-family: "SFMono-Regular", "Consolas", monospace;
    font-size: 12px;
    background: #0d1117;
}
"""


def badge_style(state: str) -> str:
    colors = {
        "ok": "#22c55e",
        "warn": "#f59e0b",
        "danger": "#ef4444",
        "muted": "#6f7e91",
        "accent": "#3b82f6",
    }
    color = colors.get(state, colors["muted"])
    return f"color: #d6deeb; background: #171d25; border: 1px solid #323b48; border-radius: 8px; padding: 4px 9px; border-left: 4px solid {color};"
