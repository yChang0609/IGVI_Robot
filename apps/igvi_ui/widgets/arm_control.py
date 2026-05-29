from __future__ import annotations

import math

from PySide6.QtCore import QCoreApplication, QObject, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient, HostClientError

# Joint limits in degrees (matched to physical mechanism)
ARM1_MIN, ARM1_MAX = 30.0, 210.0
ARM2_MIN, ARM2_MAX = 0.0, 240.0
GRIPPER_MIN_SAFE, GRIPPER_MAX = 168.0, 240.0  # < 168° will burn the motor
GRIPPER_RETRACT_DEG = 2.0
GRIPPER_RETRACT_DELAY_MS = 500

# Default starting target / Home pose (arm_1 up, arm_2 folded, gripper open)
DEFAULT_ARM1 = 210.0
DEFAULT_ARM2 = 0.0
DEFAULT_GRIPPER = 240.0
# Default starting target (mid-range, gripper open)
DEFAULT_ARM1 = 190.0
DEFAULT_ARM2 = 0.0
DEFAULT_GRIPPER = GRIPPER_MAX

# Continuous step rate (deg/sec) while a key is held
ARM_RATE_DEG_PER_SEC = 25.0
TICK_MS = 80
TRAJ_TIME_FROM_START = 0.25
TEMP_WARN_C = 60.0
TEMP_MAX_C = 68.0

# u/j → arm_1 ; i/k → arm_2 ; o open / l close gripper
_HELD_KEYS = {Qt.Key.Key_U, Qt.Key.Key_J, Qt.Key.Key_I, Qt.Key.Key_K}




class _TemperaturePoller(QThread):
    temperatures_received = Signal(dict)
    error = Signal(str)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                self.temperatures_received.emit(self.client.arm_temperatures())
            except Exception as exc:  # noqa: BLE001
                self.error.emit(str(exc))
            self.msleep(1000)


class ArmControl(QWidget):
    """Keyboard-driven arm control. Publishes JointTrajectory via the host bridge."""

    log_message = Signal(str)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._keyboard_mode = False
        self._held_keys: set[Qt.Key] = set()
        self._arm1 = DEFAULT_ARM1
        self._arm2 = DEFAULT_ARM2
        self._gripper = DEFAULT_GRIPPER
        self._tick_timer = QTimer(self)
        self._tick_timer.setInterval(TICK_MS)
        self._tick_timer.timeout.connect(self._on_tick)
        self._retract_timer = QTimer(self)
        self._retract_timer.setSingleShot(True)
        self._retract_timer.timeout.connect(self._do_retract)
        self._temperature_poller: _TemperaturePoller | None = None
        self._build_ui()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._refresh_labels()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)

        layout = QVBoxLayout(inner)
        layout.setContentsMargins(0, 0, 4, 0)
        layout.setSpacing(10)

        self._kb_btn = QPushButton("⌨  Keyboard Mode")
        self._kb_btn.setCheckable(True)
        self._kb_btn.clicked.connect(self._toggle_keyboard_mode)
        layout.addWidget(self._kb_btn)

        hint = QLabel(
            "Hold u/j → arm_1   |   Hold i/k → arm_2   |   o → open   l → close"
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.temperature_label = QLabel("Temperatures —")
        self.temperature_label.setObjectName("Muted")
        self.temperature_label.setWordWrap(True)
        layout.addWidget(self.temperature_label)

        self.arm1_slider, self.arm1_label = self._slider_row(
            "arm_1", ARM1_MIN, ARM1_MAX, DEFAULT_ARM1, lambda v: self._set_arm1(float(v), publish=True)
        )
        layout.addLayout(self.arm1_slider.parent_layout)
        self.arm2_slider, self.arm2_label = self._slider_row(
            "arm_2", ARM2_MIN, ARM2_MAX, DEFAULT_ARM2, lambda v: self._set_arm2(float(v), publish=True)
        )
        layout.addLayout(self.arm2_slider.parent_layout)
        self.gripper_slider, self.gripper_label = self._slider_row(
            "gripper", GRIPPER_MIN_SAFE, GRIPPER_MAX, DEFAULT_GRIPPER,
            lambda v: self._set_gripper(float(v), publish=True),
        )
        layout.addLayout(self.gripper_slider.parent_layout)

        actions = QGridLayout()
        open_btn = QPushButton("Open")
        open_btn.clicked.connect(self._open_gripper)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self._close_gripper)
        home_btn = QPushButton("Home")
        home_btn.setToolTip("Reset target to safe mid-range")
        home_btn.clicked.connect(self._go_home)
        send_btn = QPushButton("Send")
        send_btn.setObjectName("Primary")
        send_btn.setToolTip("Publish current targets once")
        send_btn.clicked.connect(self._publish_once)
        actions.addWidget(open_btn, 0, 0)
        actions.addWidget(close_btn, 0, 1)
        actions.addWidget(home_btn, 1, 0)
        actions.addWidget(send_btn, 1, 1)
        layout.addLayout(actions)

    def _slider_row(self, label: str, lo: float, hi: float, value: float, on_value):
        row = QHBoxLayout()
        title = QLabel(label)
        title.setMinimumWidth(54)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(int(lo), int(hi))
        slider.setValue(int(value))
        slider.valueChanged.connect(on_value)
        readout = QLabel(f"{value:.0f}°")
        readout.setMinimumWidth(48)
        readout.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        row.addWidget(title)
        row.addWidget(slider, 1)
        row.addWidget(readout)
        slider.parent_layout = row  # type: ignore[attr-defined]
        return slider, readout

    # ── State setters (degrees, clamped) ─────────────────────────────────────

    def _set_arm1(self, deg: float, *, publish: bool) -> None:
        new = max(ARM1_MIN, min(ARM1_MAX, deg))
        if math.isclose(new, self._arm1):
            return
        self._arm1 = new
        self.arm1_slider.blockSignals(True)
        self.arm1_slider.setValue(int(new))
        self.arm1_slider.blockSignals(False)
        self.arm1_label.setText(f"{new:.0f}°")
        if publish:
            self._publish_once()

    def _set_arm2(self, deg: float, *, publish: bool) -> None:
        new = max(ARM2_MIN, min(ARM2_MAX, deg))
        if math.isclose(new, self._arm2):
            return
        self._arm2 = new
        self.arm2_slider.blockSignals(True)
        self.arm2_slider.setValue(int(new))
        self.arm2_slider.blockSignals(False)
        self.arm2_label.setText(f"{new:.0f}°")
        if publish:
            self._publish_once()

    def _set_gripper(self, deg: float, *, publish: bool) -> None:
        new = max(GRIPPER_MIN_SAFE, min(GRIPPER_MAX, deg))
        if math.isclose(new, self._gripper):
            return
        self._gripper = new
        self.gripper_slider.blockSignals(True)
        self.gripper_slider.setValue(int(new))
        self.gripper_slider.blockSignals(False)
        self.gripper_label.setText(f"{new:.0f}°")
        if publish:
            self._publish_once()

    def _refresh_labels(self) -> None:
        self.arm1_label.setText(f"{self._arm1:.0f}°")
        self.arm2_label.setText(f"{self._arm2:.0f}°")
        self.gripper_label.setText(f"{self._gripper:.0f}°")

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        QCoreApplication.instance().installEventFilter(self)
        if self._temperature_poller is None:
            self._temperature_poller = _TemperaturePoller(self.client)
            self._temperature_poller.temperatures_received.connect(self._on_temperatures)
            self._temperature_poller.error.connect(self._on_temperature_error)
            self._temperature_poller.start()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        QCoreApplication.instance().removeEventFilter(self)
        self._exit_keyboard_mode()
        self.shutdown()

    def shutdown(self) -> None:
        if self._temperature_poller is not None:
            stop_thread(self._temperature_poller)
            self._temperature_poller = None


    def _on_temperatures(self, data: dict) -> None:
        temperatures = []
        for value in data.get("temperatures") or []:
            try:
                temperatures.append(float(value))
            except (TypeError, ValueError):
                temperatures.append(math.nan)
        if not data.get("ok") or not temperatures:
            self.temperature_label.setText("Temperatures unavailable")
            self.temperature_label.setStyleSheet("color: #94a3b8;")
            return

        names = ["arm_1", "arm_2", "gripper"]
        parts = []
        for index, value in enumerate(temperatures):
            name = names[index] if index < len(names) else f"joint_{index + 1}"
            reading = "unknown" if math.isnan(value) else f"{value:.1f}°C"
            parts.append(f"{name} {reading}")

        try:
            gripper_index = int(data.get("gripper_index", 2))
        except (TypeError, ValueError):
            gripper_index = 2
        gripper_temp = temperatures[gripper_index] if 0 <= gripper_index < len(temperatures) else None
        if gripper_temp is None or math.isnan(gripper_temp):
            tone = "#94a3b8"
            state = "unknown"
        elif gripper_temp >= TEMP_MAX_C:
            tone = "#ef4444"
            state = "overheat"
        elif gripper_temp >= TEMP_WARN_C:
            tone = "#fbbf24"
            state = "warm"
        else:
            tone = "#22c55e"
            state = "ok"

        self.temperature_label.setText("Temperatures: " + "  ·  ".join(parts) + f"  ·  gripper {state}")
        self.temperature_label.setStyleSheet(f"font-weight: 600; color: {tone};")

    def _on_temperature_error(self, message: str) -> None:
        self.temperature_label.setText(f"Temperatures unavailable: {message}")
        self.temperature_label.setStyleSheet("color: #ef4444;")

    # ── Keyboard mode toggle ──────────────────────────────────────────────────

    def _toggle_keyboard_mode(self) -> None:
        if self._keyboard_mode:
            self._exit_keyboard_mode()
        else:
            self._keyboard_mode = True
            self._kb_btn.setText("✕  Exit Keyboard Mode")
            self._kb_btn.setObjectName("Primary")
            self._kb_btn.setChecked(True)
            self._restyle(self._kb_btn)
            self.log_message.emit("Arm keyboard mode: u/j i/k o/l")

    def _exit_keyboard_mode(self) -> None:
        if not self._keyboard_mode:
            return
        self._keyboard_mode = False
        self._held_keys.clear()
        self._tick_timer.stop()
        self._kb_btn.setText("⌨  Keyboard Mode")
        self._kb_btn.setObjectName("")
        self._kb_btn.setChecked(False)
        self._restyle(self._kb_btn)

    @staticmethod
    def _restyle(widget: QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    # ── App-level event filter ────────────────────────────────────────────────

    def eventFilter(self, obj: QObject, event) -> bool:  # noqa: N802
        if not self._keyboard_mode:
            return False
        if event.type() == QKeyEvent.Type.KeyPress and not event.isAutoRepeat():
            key = Qt.Key(event.key())
            if key == Qt.Key.Key_O:
                self._open_gripper()
                return True
            if key == Qt.Key.Key_L:
                self._close_gripper()
                return True
            if key in _HELD_KEYS and key not in self._held_keys:
                self._held_keys.add(key)
                if not self._tick_timer.isActive():
                    self._tick_timer.start()
                return True
        elif event.type() == QKeyEvent.Type.KeyRelease and not event.isAutoRepeat():
            key = Qt.Key(event.key())
            if key in self._held_keys:
                self._held_keys.discard(key)
                if not self._held_keys:
                    self._tick_timer.stop()
                return True
        return False

    def _on_tick(self) -> None:
        if not self._held_keys:
            self._tick_timer.stop()
            return
        step = ARM_RATE_DEG_PER_SEC * (TICK_MS / 1000.0)
        changed = False
        if Qt.Key.Key_U in self._held_keys:
            self._set_arm1(self._arm1 + step, publish=False)
            changed = True
        if Qt.Key.Key_J in self._held_keys:
            self._set_arm1(self._arm1 - step, publish=False)
            changed = True
        if Qt.Key.Key_I in self._held_keys:
            self._set_arm2(self._arm2 + step, publish=False)
            changed = True
        if Qt.Key.Key_K in self._held_keys:
            self._set_arm2(self._arm2 - step, publish=False)
            changed = True
        if changed:
            self._publish_once()

    # ── Gripper actions ───────────────────────────────────────────────────────

    def _open_gripper(self) -> None:
        self._retract_timer.stop()
        self._set_gripper(GRIPPER_MAX, publish=False)
        self._publish_once()

    def _close_gripper(self) -> None:
        self._set_gripper(GRIPPER_MIN_SAFE, publish=False)
        self._publish_once()
        self._retract_timer.start(GRIPPER_RETRACT_DELAY_MS)

    def _do_retract(self) -> None:
        target = min(GRIPPER_MAX, self._gripper + GRIPPER_RETRACT_DEG)
        self._set_gripper(target, publish=False)
        self._publish_once()
        self.log_message.emit(f"Gripper auto-retract → {target:.0f}°")

    def _go_home(self) -> None:
        self._retract_timer.stop()
        self._set_arm1(DEFAULT_ARM1, publish=False)
        self._set_arm2(DEFAULT_ARM2, publish=False)
        self._set_gripper(DEFAULT_GRIPPER, publish=False)
        self._publish_once()

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_once(self) -> None:
        positions = [
            math.radians(self._arm1),
            math.radians(self._arm2),
            math.radians(self._gripper),
        ]
        try:
            self.client.arm_trajectory(positions, time_from_start=TRAJ_TIME_FROM_START)
        except HostClientError as exc:
            self.log_message.emit(f"Arm publish failed: {exc}")
