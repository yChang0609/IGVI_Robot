from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError


APPROACH_NODE = "search_retrieve_server"

# label, parameter, min, max, default, step, decimals, tooltip
_TUNE_PARAMS: list[tuple[str, str, float, float, float, float, int, str]] = [
    (
        "Pose cost limit",
        "approach_max_endpoint_cost",
        20.0,
        100.0,
        80.0,
        1.0,
        0,
        "Lower rejects approach poses closer to walls. Raise it if every candidate is rejected.",
    ),
    (
        "Corridor limit",
        "approach_max_corridor_cost",
        20.0,
        100.0,
        85.0,
        1.0,
        0,
        "Lower rejects candidates whose short straight drive toward the bear passes near obstacles.",
    ),
    (
        "Pose cost weight",
        "approach_endpoint_cost_weight",
        0.0,
        0.10,
        0.02,
        0.005,
        3,
        "Higher prefers safer approach poses even if the Nav2 path is longer.",
    ),
    (
        "Corridor weight",
        "approach_corridor_cost_weight",
        0.0,
        0.10,
        0.02,
        0.005,
        3,
        "Higher prefers safer final drive corridors even if the Nav2 path is longer.",
    ),
    (
        "Pose clearance radius",
        "approach_clearance_radius_m",
        0.05,
        0.30,
        0.12,
        0.01,
        2,
        "How far around the approach pose to check. Larger means more clearance.",
    ),
    (
        "Corridor width",
        "approach_corridor_width_m",
        0.05,
        0.40,
        0.16,
        0.01,
        2,
        "Width checked along the short drive toward the bear. Larger catches side collisions earlier.",
    ),
    (
        "Ignore near target",
        "approach_target_cost_exclusion_m",
        0.10,
        0.50,
        0.30,
        0.01,
        2,
        "Cost near the bear is ignored by this distance so the bear itself does not reject the approach.",
    ),
]


class ApproachTuningControl(QWidget):
    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._sliders: dict[str, QSlider] = {}
        self._readouts: dict[str, QLabel] = {}
        self._specs = {
            param: (lo, hi, default, step, decimals)
            for _, param, lo, hi, default, step, decimals, _ in _TUNE_PARAMS
        }
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        scroll.setWidget(content)
        root.addWidget(scroll)

        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 4, 0)
        layout.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel("Approach safety tuning")
        title.setStyleSheet("font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)

        self.enabled_box = QCheckBox("Use costmap scoring")
        self.enabled_box.setChecked(True)
        self.enabled_box.toggled.connect(
            lambda checked: self._apply_params({"approach_costmap_enabled": bool(checked)})
        )
        header.addWidget(self.enabled_box)
        layout.addLayout(header)

        hint = QLabel(
            "Lower cost limits are stricter. Larger radius/width asks for more clearance. "
            "Changes apply the next time an approach pose is selected."
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        for label, param, lo, hi, default, step, decimals, tooltip in _TUNE_PARAMS:
            layout.addLayout(
                self._slider_row(label, param, lo, hi, default, step, decimals, tooltip)
            )

        actions = QHBoxLayout()
        apply_btn = QPushButton("Apply All")
        apply_btn.setObjectName("Primary")
        apply_btn.clicked.connect(self.apply_all)
        defaults_btn = QPushButton("Defaults")
        defaults_btn.clicked.connect(self.reset_defaults)
        actions.addWidget(apply_btn)
        actions.addWidget(defaults_btn)
        layout.addLayout(actions)

        self.status_label = QLabel("Ready")
        self.status_label.setObjectName("Muted")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        layout.addStretch(1)

    def _slider_row(
        self,
        label: str,
        param: str,
        lo: float,
        hi: float,
        default: float,
        step: float,
        decimals: int,
        tooltip: str,
    ) -> QHBoxLayout:
        row = QHBoxLayout()
        name = QLabel(label)
        name.setMinimumWidth(118)
        name.setToolTip(tooltip)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, int(round((hi - lo) / step)))
        slider.setValue(self._to_slider(default, lo, step))
        slider.setToolTip(tooltip)

        readout = QLabel(self._format(default, decimals))
        readout.setMinimumWidth(58)
        readout.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        slider.valueChanged.connect(
            lambda raw, p=param, r=readout: r.setText(
                self._format(self._from_slider(p, raw), self._specs[p][4])
            )
        )
        slider.sliderReleased.connect(
            lambda p=param, s=slider: self._apply_params({p: self._from_slider(p, s.value())})
        )

        self._sliders[param] = slider
        self._readouts[param] = readout
        row.addWidget(name)
        row.addWidget(slider, 1)
        row.addWidget(readout)
        return row

    def _to_slider(self, value: float, lo: float, step: float) -> int:
        return int(round((float(value) - lo) / step))

    def _from_slider(self, param: str, raw: int) -> float:
        lo, _hi, _default, step, decimals = self._specs[param]
        value = lo + raw * step
        return round(value, decimals)

    def _format(self, value: float, decimals: int) -> str:
        if decimals <= 0:
            return str(int(round(value)))
        return f"{value:.{decimals}f}"

    def apply_all(self) -> None:
        params = {"approach_costmap_enabled": bool(self.enabled_box.isChecked())}
        for param, slider in self._sliders.items():
            params[param] = self._from_slider(param, slider.value())
        self._apply_params(params)

    def reset_defaults(self) -> None:
        self.enabled_box.blockSignals(True)
        self.enabled_box.setChecked(True)
        self.enabled_box.blockSignals(False)

        for param, slider in self._sliders.items():
            lo, _hi, default, step, decimals = self._specs[param]
            slider.blockSignals(True)
            slider.setValue(self._to_slider(default, lo, step))
            slider.blockSignals(False)
            self._readouts[param].setText(self._format(default, decimals))
        self.apply_all()

    def _apply_params(self, params: dict[str, bool | float]) -> None:
        try:
            result = self.client.set_params(APPROACH_NODE, params)
        except HostClientError as exc:
            self._set_status(f"Set failed: {exc}", ok=False)
            return

        message = str(result.get("message", ""))
        if result.get("ok", False):
            changed = ", ".join(params.keys())
            self._set_status(f"Applied {changed}", ok=True)
        else:
            self._set_status(message or "Set rejected", ok=False)

    def _set_status(self, message: str, *, ok: bool) -> None:
        self.status_label.setText(message)
        color = "#86efac" if ok else "#fca5a5"
        self.status_label.setStyleSheet(f"color: {color};")
