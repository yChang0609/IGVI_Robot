"""Door-opening visualization + control page.

Three things in one tab:
  * Live /open_door/debug_image stream (what the red-bar detector sees).
  * HSV tuning sliders that live-set parameters on open_door_server via the
    host → bridge → /<node>/set_parameters service — no redeploy needed.
  * A trigger button that dispatches the /open_door action goal and shows the
    FSM's live stage/progress.

The image stream reuses the existing host → igvi_bridge → JPEG path that powers
the Robot page camera view.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.image_view import ImageView


DEBUG_TOPIC = "/open_door/debug_image"
SERVER_NODE = "open_door_server"

# (label, param name on open_door_server, min, max, default)
_TUNE_SLIDERS: list[tuple[str, str, int, int, int]] = [
    ("Saturation min", "red_sat_min", 0, 255, 120),
    ("Value min", "red_val_min", 0, 255, 70),
    ("Hue band-1 hi", "red_hue_hi1", 0, 40, 10),
    ("Hue band-2 lo", "red_hue_lo2", 140, 179, 170),
    ("Min area (px)", "min_red_area_px", 100, 8000, 800),
    ("Aim offset (px)", "aim_offset_px", -60, 60, 0),
    ("Depth inset (px)", "depth_inset_px", 0, 40, 8),
]

_LEGEND_HTML = """
<b>Overlay legend</b><br>
<span style="color:#ef4444;">■</span> Red wash &mdash; pixels passing the HSV
threshold. Bar partly missed? Lower <i>Saturation min</i> / <i>Value min</i>.<br>
<span style="color:#facc15;">▭</span> Yellow box &mdash; largest red blob.<br>
<span style="color:#22c55e;">✚</span> Green crosshair &mdash; aim point (bar right
edge &plusmn; <i>Aim offset</i>). ALIGN drives it onto the gray centerline.<br>
<span style="color:#06b6d4;">●</span> Cyan dot &mdash; depth-sample pixel
(turns red if depth invalid).
"""


class _StatusPoller(QThread):
    status_received = Signal(dict)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                self.status_received.emit(self.client.open_door_status())
            except Exception:  # noqa: BLE001
                pass
            self.msleep(400)


class DoorPage(QWidget):
    log_message = Signal(str)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._poller: _StatusPoller | None = None
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(10)

        header = QVBoxLayout()
        header.setSpacing(2)
        title = QLabel("Door Opener — Red-Bar Detector")
        title.setObjectName("Title")
        subtitle = QLabel(
            "Live /open_door/debug_image, HSV tuning, and one-click trigger. "
            "Tuning sliders set parameters on open_door_server instantly."
        )
        subtitle.setObjectName("Muted")
        subtitle.setWordWrap(True)
        header.addWidget(title)
        header.addWidget(subtitle)
        layout.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(12)

        self.image_view = ImageView(self.client, preferred_topics=[DEBUG_TOPIC])
        self.image_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        body.addWidget(self.image_view, 3)

        body.addWidget(self._build_side_panel(), 1)
        layout.addLayout(body, 1)

    def _build_side_panel(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(300)
        scroll.setMaximumWidth(400)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(12)

        layout.addWidget(self._build_action_card())
        layout.addWidget(self._build_tuning_card())
        layout.addWidget(self._build_legend_card())
        layout.addStretch(1)

        scroll.setWidget(inner)
        return scroll

    def _build_action_card(self) -> QWidget:
        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)

        heading = QLabel("Trigger")
        heading.setStyleSheet("font-weight: 600;")
        layout.addWidget(heading)

        row = QHBoxLayout()
        row.addWidget(QLabel("Ready distance (m)"))
        self.ready_spin = QDoubleSpinBox()
        self.ready_spin.setRange(0.0, 2.0)
        self.ready_spin.setSingleStep(0.05)
        self.ready_spin.setValue(0.45)
        self.ready_spin.setDecimals(2)
        row.addWidget(self.ready_spin)
        layout.addLayout(row)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("Start Open Door")
        self.start_btn.clicked.connect(self._on_start)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self._on_cancel)
        btn_row.addWidget(self.start_btn, 1)
        btn_row.addWidget(self.cancel_btn)
        layout.addLayout(btn_row)

        self.state_label = QLabel("state: —")
        self.state_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self.state_label)

        self.detail_label = QLabel("")
        self.detail_label.setObjectName("Muted")
        self.detail_label.setWordWrap(True)
        layout.addWidget(self.detail_label)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        return card

    def _build_tuning_card(self) -> QWidget:
        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        heading = QLabel("Red threshold tuning")
        heading.setStyleSheet("font-weight: 600;")
        layout.addWidget(heading)
        hint = QLabel("Drag, then release to apply on open_door_server.")
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self._sliders: dict[str, QSlider] = {}
        for label, param, lo, hi, default in _TUNE_SLIDERS:
            layout.addLayout(self._tune_row(label, param, lo, hi, default))

        return card

    def _tune_row(self, label: str, param: str, lo: int, hi: int, default: int) -> QHBoxLayout:
        row = QHBoxLayout()
        name = QLabel(label)
        name.setMinimumWidth(108)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(default)
        readout = QLabel(str(default))
        readout.setMinimumWidth(42)
        readout.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        slider.valueChanged.connect(lambda v, r=readout: r.setText(str(v)))
        slider.sliderReleased.connect(lambda p=param, s=slider: self._apply_param(p, s.value()))

        self._sliders[param] = slider
        row.addWidget(name)
        row.addWidget(slider, 1)
        row.addWidget(readout)
        return row

    def _build_legend_card(self) -> QWidget:
        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        legend = QLabel(_LEGEND_HTML)
        legend.setWordWrap(True)
        legend.setTextFormat(Qt.TextFormat.RichText)
        layout.addWidget(legend)
        return card

    # ── Actions ────────────────────────────────────────────────────────────

    def _apply_param(self, param: str, value: int) -> None:
        try:
            result = self.client.set_params(SERVER_NODE, {param: int(value)})
        except HostClientError as exc:
            self.log_message.emit(f"Param set failed: {exc}")
            return
        if not result.get("ok", False):
            self.log_message.emit(f"{param}: {result.get('message', 'rejected')}")
        else:
            self.log_message.emit(f"{param} = {value}")

    def _on_start(self) -> None:
        try:
            result = self.client.open_door_start(float(self.ready_spin.value()))
        except HostClientError as exc:
            self.log_message.emit(f"Open-door start failed: {exc}")
            return
        self.log_message.emit(str(result.get("message", "open_door dispatched")))

    def _on_cancel(self) -> None:
        try:
            result = self.client.open_door_cancel()
        except HostClientError as exc:
            self.log_message.emit(f"Cancel failed: {exc}")
            return
        self.log_message.emit(str(result.get("message", "cancel requested")))

    def _on_status(self, data: dict) -> None:
        available = bool(data.get("available", False))
        state = str(data.get("state", "unavailable"))
        stage = str(data.get("stage", ""))
        message = str(data.get("message", ""))
        progress = float(data.get("progress", 0.0))

        self.start_btn.setEnabled(available and state not in ("running", "sending"))
        self.cancel_btn.setEnabled(state in ("running", "sending"))
        label = state if not stage else f"{state} · {stage}"
        self.state_label.setText(f"state: {label}")
        self.detail_label.setText(message)
        self.progress.setValue(int(max(0.0, min(1.0, progress)) * 100))

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._poller is None:
            self._poller = _StatusPoller(self.client)
            self._poller.status_received.connect(self._on_status)
            self._poller.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self._stop_poller()

    def _stop_poller(self) -> None:
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None

    def shutdown(self) -> None:
        """Forwarded by MainWindow.shutdown so background threads exit cleanly."""
        self._stop_poller()
        self.image_view.shutdown()
