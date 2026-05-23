from __future__ import annotations

from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel

# (fusion_sources key, display name) — order shown left-to-right in the bar.
_SENSORS = [
    ("wheel", "Wheel"),
    ("lidar", "Lidar"),
    ("imu", "IMU"),
    ("camera", "Camera"),
]


class SensorGroup(QFrame):
    """Grouped sensor-freshness badge for the top status bar.

    Shows Wheel / Lidar / IMU / Camera, each ● (fresh) or ○ (stale/missing),
    driven by the bridge's fusion_sources freshness flags.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("SensorGroup")
        self.setFixedHeight(36)
        self.setStyleSheet(
            "QFrame#SensorGroup { background: #171d25; border: 1px solid #323b48; "
            "border-radius: 8px; }"
        )
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(11)

        title = QLabel("Sensors")
        title.setStyleSheet("color: #8aa0b8; font-weight: 700;")
        layout.addWidget(title)

        self._labels: dict[str, QLabel] = {}
        for key, name in _SENSORS:
            label = QLabel(f"○ {name}")
            self._labels[key] = label
            layout.addWidget(label)
        self.update_sources({}, available=False)

    def update_sources(self, sources: dict, available: bool = True) -> None:
        for key, name in _SENSORS:
            label = self._labels[key]
            if not available or not sources:
                label.setText(f"○ {name}")
                label.setStyleSheet("color: #6f7e91;")
                continue
            ok = bool(sources.get(key, False))
            glyph = "●" if ok else "○"
            color = "#22c55e" if ok else "#ef4444"
            label.setText(f"{glyph} {name}")
            # Avoid font-weight changes — they cause 1-2px height variation on macOS.
            label.setStyleSheet(f"color: {color};")
