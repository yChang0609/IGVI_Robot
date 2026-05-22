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

import math

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
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

# Arm joint limits (degrees), mirrored from widgets/arm_control.py.
# gripper < 168° overheats the motor, so it is clamped here too.
ARM1_MIN, ARM1_MAX = 30.0, 210.0
ARM2_MIN, ARM2_MAX = 0.0, 240.0
GRIPPER_MIN_SAFE, GRIPPER_MAX = 168.0, 240.0
ARM_TRAJ_TIME = 0.4          # time_from_start for jog trajectories (s)
JOG_THROTTLE_MS = 70         # coalesce rapid slider drags into one publish

# (label, min, max, default) for the three jog joints.
_ARM_JOINTS: list[tuple[str, float, float, float]] = [
    ("arm_1", ARM1_MIN, ARM1_MAX, 167.0),
    ("arm_2", ARM2_MIN, ARM2_MAX, 80.0),
    ("gripper", GRIPPER_MIN_SAFE, GRIPPER_MAX, 170.6),
]

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

    # Network timeout (2s) is kept well under stop_thread's 6s join window so a
    # stop() request always lets run() return on its own — terminate() (which
    # crashes with "QThread: Destroyed while thread is still running") is never
    # reached. The 800ms idle gap is split into short hops so stop is prompt.
    _POLL_TIMEOUT = 2.0
    _IDLE_MS = 800

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            try:
                self.status_received.emit(self.client.open_door_status(timeout=self._POLL_TIMEOUT))
            except Exception:  # noqa: BLE001
                pass
            waited = 0
            while self._running and waited < self._IDLE_MS:
                self.msleep(100)
                waited += 100


class _StepWorker(QThread):
    """One-shot worker for a debug door step (run_press/run_push/go_home).

    These calls block server-side (the push drives the base for push_duration_sec),
    so running them off the UI thread keeps the window responsive.
    """

    finished_step = Signal(bool, str)  # ok, message

    def __init__(self, fn) -> None:
        super().__init__()
        self._fn = fn

    def run(self) -> None:
        try:
            result = self._fn()
            self.finished_step.emit(
                bool(result.get("ok", False)), str(result.get("message", ""))
            )
        except Exception as exc:  # noqa: BLE001
            self.finished_step.emit(False, str(exc))


class DoorPage(QWidget):
    log_message = Signal(str)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._poller: _StatusPoller | None = None
        self._step_worker: _StepWorker | None = None
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

        # Three independently tunable + savable + runnable door motions:
        #   1) arm slam-down (unlatch)   2) arm push (shove open)   3) base drive
        self._step_buttons: list[QPushButton] = []
        layout.addWidget(self._build_action_card())
        layout.addWidget(self._build_arm_card())
        layout.addWidget(self._build_arm_push_card())
        layout.addWidget(self._build_push_card())
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

    def _build_arm_card(self) -> QWidget:
        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        heading = QLabel("1 · Arm slam-down (unlatch) — poses 1 → 2 → 3")
        heading.setStyleSheet("font-weight: 600;")
        layout.addWidget(heading)

        hint = QLabel(
            "Dial the arm with the sliders, then capture it into a slam pose or "
            "the arm-push pose below. Sliders are shared by both arm motions."
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.jog_check = QCheckBox("Live jog — slider moves the real arm")
        self.jog_check.setChecked(False)
        layout.addWidget(self.jog_check)

        # Coalesce rapid slider motion into one trajectory publish.
        self._jog_timer = QTimer(self)
        self._jog_timer.setSingleShot(True)
        self._jog_timer.setInterval(JOG_THROTTLE_MS)
        self._jog_timer.timeout.connect(self._publish_jog)

        self._arm_sliders: dict[str, QSlider] = {}
        for name, lo, hi, default in _ARM_JOINTS:
            layout.addLayout(self._arm_row(name, lo, hi, default))

        save_row = QHBoxLayout()
        for n in (1, 2, 3):
            btn = QPushButton(f"Save → Pose {n}")
            btn.clicked.connect(lambda _c=False, idx=n: self._save_pose(idx))
            save_row.addWidget(btn)
        layout.addLayout(save_row)

        run_row = QHBoxLayout()
        slam_btn = QPushButton("Run SLAM (arm)")
        slam_btn.setToolTip("Play door_pose 1→2→3 on the arm. Base does not move.")
        slam_btn.clicked.connect(lambda: self._run_step("run_press", "SLAM"))
        self._step_buttons.append(slam_btn)
        run_row.addWidget(slam_btn)
        home_btn = QPushButton("Go Home pose")
        home_btn.clicked.connect(self._go_home)
        run_row.addWidget(home_btn)
        layout.addLayout(run_row)

        save_yaml_btn = QPushButton("Save slam to YAML")
        save_yaml_btn.setToolTip("Persist current poses + tuning so they survive a restart")
        save_yaml_btn.clicked.connect(self._save_poses_yaml)
        layout.addWidget(save_yaml_btn)

        self.pose_status = QLabel("")
        self.pose_status.setObjectName("Muted")
        self.pose_status.setWordWrap(True)
        layout.addWidget(self.pose_status)

        return card

    def _arm_row(self, name: str, lo: float, hi: float, default: float) -> QHBoxLayout:
        row = QHBoxLayout()
        label = QLabel(name)
        label.setMinimumWidth(60)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(int(lo), int(hi))
        slider.setValue(int(default))
        readout = QLabel(f"{int(default)}°")
        readout.setMinimumWidth(40)
        readout.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        slider.valueChanged.connect(lambda v, r=readout: r.setText(f"{v}°"))
        slider.valueChanged.connect(self._on_jog_changed)
        self._arm_sliders[name] = slider
        row.addWidget(label)
        row.addWidget(slider, 1)
        row.addWidget(readout)
        return row

    def _current_arm_deg(self) -> list[float]:
        return [float(self._arm_sliders[name].value()) for name, *_ in _ARM_JOINTS]

    def _on_jog_changed(self) -> None:
        # Only move the real arm when the user has armed live jog; either way
        # the slider values are kept so Save→Pose captures the dialed-in angles.
        if self.jog_check.isChecked():
            self._jog_timer.start()  # restart throttle window

    def _publish_jog(self) -> None:
        positions_rad = [math.radians(d) for d in self._current_arm_deg()]
        try:
            self.client.arm_trajectory(positions_rad, time_from_start=ARM_TRAJ_TIME)
        except HostClientError as exc:
            self.log_message.emit(f"Arm jog failed: {exc}")

    def _save_pose(self, n: int) -> None:
        pose = self._current_arm_deg()
        try:
            result = self.client.set_params(SERVER_NODE, {f"door_pose_{n}_deg": pose})
        except HostClientError as exc:
            self.log_message.emit(f"Save Pose {n} failed: {exc}")
            return
        if result.get("ok", False):
            pretty = ", ".join(f"{v:.1f}" for v in pose)
            self.pose_status.setText(f"Pose {n} = [{pretty}]")
            self.log_message.emit(f"Saved Pose {n} = [{pretty}]")
        else:
            self.log_message.emit(f"Pose {n}: {result.get('message', 'rejected')}")

    def _save_arm_push_pose(self) -> None:
        pose = self._current_arm_deg()
        try:
            result = self.client.set_params(SERVER_NODE, {"door_arm_push_pose_deg": pose})
        except HostClientError as exc:
            self.log_message.emit(f"Save Arm Push failed: {exc}")
            return
        if result.get("ok", False):
            pretty = ", ".join(f"{v:.1f}" for v in pose)
            self.pose_status.setText(f"Arm Push pose = [{pretty}]")
            self.log_message.emit(f"Saved Arm Push pose = [{pretty}]")
        else:
            self.log_message.emit(f"Arm Push: {result.get('message', 'rejected')}")

    def _save_poses_yaml(self) -> None:
        try:
            result = self.client.open_door_save_poses()
        except HostClientError as exc:
            self.log_message.emit(f"Save to YAML failed: {exc}")
            return
        msg = str(result.get("message", ""))
        self.pose_status.setText(msg)
        self.log_message.emit(msg if result.get("ok") else f"Save to YAML: {msg}")

    def _go_home(self) -> None:
        if not self.jog_check.isChecked():
            self.log_message.emit("Enable 'Live jog' first to move the arm home.")
            return
        # door_home default; the server clamps/uses its own param on the FSM path.
        positions_rad = [math.radians(d) for d in (167.0, 75.0, 170.6)]
        try:
            self.client.arm_trajectory(positions_rad, time_from_start=1.0)
        except HostClientError as exc:
            self.log_message.emit(f"Go home failed: {exc}")

    def _build_arm_push_card(self) -> QWidget:
        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        heading = QLabel("2 · Arm push (shove door open with arm)")
        heading.setStyleSheet("font-weight: 600;")
        layout.addWidget(heading)

        hint = QLabel(
            "Dial the arm-push target with the sliders above, then capture it. "
            "ARM_PUSH runs after the slam to shove the door open with the arm."
        )
        hint.setObjectName("Muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.arm_push_after_check = QCheckBox("Arm push after slam (in full Open Door)")
        self.arm_push_after_check.setToolTip(
            "Leave OFF until the slam reliably unlatches — prevents driving the "
            "arm into a still-latched door."
        )
        self.arm_push_after_check.toggled.connect(
            lambda v: self._apply_param("arm_push_after_slam", bool(v))
        )
        layout.addWidget(self.arm_push_after_check)

        hold_row = QHBoxLayout()
        hold_row.addWidget(QLabel("Arm push hold (s)"))
        self.arm_push_hold = QDoubleSpinBox()
        self.arm_push_hold.setRange(0.0, 5.0)
        self.arm_push_hold.setSingleStep(0.1)
        self.arm_push_hold.setDecimals(1)
        self.arm_push_hold.setValue(0.5)
        self.arm_push_hold.valueChanged.connect(
            lambda v: self._apply_param("arm_push_hold_sec", float(v))
        )
        hold_row.addWidget(self.arm_push_hold)
        layout.addLayout(hold_row)

        btn_row = QHBoxLayout()
        save_btn = QPushButton("Save → Arm Push pose")
        save_btn.setToolTip("Capture the current arm sliders as door_arm_push_pose")
        save_btn.clicked.connect(self._save_arm_push_pose)
        btn_row.addWidget(save_btn)
        run_btn = QPushButton("Run ARM-PUSH")
        run_btn.setToolTip("Swing the arm to the arm-push pose. Base does not move.")
        run_btn.clicked.connect(lambda: self._run_step("run_arm_push", "ARM-PUSH"))
        self._step_buttons.append(run_btn)
        btn_row.addWidget(run_btn)
        layout.addLayout(btn_row)

        save_yaml_btn = QPushButton("Save arm push to YAML")
        save_yaml_btn.setToolTip("Persist current poses + tuning so they survive a restart")
        save_yaml_btn.clicked.connect(self._save_poses_yaml)
        layout.addWidget(save_yaml_btn)

        return card

    def _build_push_card(self) -> QWidget:
        card = QFrame()
        card.setObjectName("Card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        heading = QLabel("3 · Base drive (push door open)")
        heading.setStyleSheet("font-weight: 600;")
        layout.addWidget(heading)

        self.drive_check = QCheckBox("Drive forward after poses")
        self.drive_check.setToolTip(
            "Leave OFF until pose 3 reliably opens the door — prevents ramming a latched door."
        )
        self.drive_check.toggled.connect(
            lambda v: self._apply_param("drive_forward_after_poses", bool(v))
        )
        layout.addWidget(self.drive_check)

        self.hold_check = QCheckBox("Hold handle down while pushing")
        self.hold_check.setChecked(True)
        self.hold_check.toggled.connect(
            lambda v: self._apply_param("hold_pose_during_push", bool(v))
        )
        layout.addWidget(self.hold_check)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Push speed (m/s)"))
        self.push_speed = QDoubleSpinBox()
        self.push_speed.setRange(0.0, 0.30)
        self.push_speed.setSingleStep(0.01)
        self.push_speed.setDecimals(2)
        self.push_speed.setValue(0.08)
        self.push_speed.valueChanged.connect(
            lambda v: self._apply_param("push_speed", float(v))
        )
        speed_row.addWidget(self.push_speed)
        layout.addLayout(speed_row)

        dur_row = QHBoxLayout()
        dur_row.addWidget(QLabel("Push duration (s)"))
        self.push_dur = QDoubleSpinBox()
        self.push_dur.setRange(0.0, 6.0)
        self.push_dur.setSingleStep(0.5)
        self.push_dur.setDecimals(1)
        self.push_dur.setValue(3.0)
        self.push_dur.valueChanged.connect(
            lambda v: self._apply_param("push_duration_sec", float(v))
        )
        dur_row.addWidget(self.push_dur)
        layout.addLayout(dur_row)

        # Base forward drive, triggerable on its own (no camera / no nav needed)
        # so it can be debugged apart from the arm motions. Run the arm slam (and
        # arm push, if used) first so the door is already unlatched.
        run_btn = QPushButton("Run DRIVE (base)")
        run_btn.setToolTip(
            "Drive the base forward for push_duration_sec, holding pose 3. "
            "Run SLAM first so the handle is already pressed."
        )
        run_btn.clicked.connect(lambda: self._run_step("run_push", "DRIVE"))
        self._step_buttons.append(run_btn)
        layout.addWidget(run_btn)

        save_yaml_btn = QPushButton("Save base drive to YAML")
        save_yaml_btn.setToolTip("Persist current poses + tuning so they survive a restart")
        save_yaml_btn.clicked.connect(self._save_poses_yaml)
        layout.addWidget(save_yaml_btn)

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

    def _apply_param(self, param: str, value) -> None:
        # value keeps its Python type (int/float/bool); the bridge infers the
        # ROS parameter type from it.
        try:
            result = self.client.set_params(SERVER_NODE, {param: value})
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

    def _run_step(self, step: str, label: str) -> None:
        if self._step_worker is not None and self._step_worker.isRunning():
            self.log_message.emit("A door step is already running; wait for it to finish.")
            return
        self._set_step_buttons_enabled(False)
        self.log_message.emit(f"{label}: running…")
        worker = _StepWorker(lambda: self.client.open_door_step(step))
        worker.finished_step.connect(
            lambda ok, msg, lbl=label: self._on_step_done(ok, msg, lbl)
        )
        worker.finished.connect(self._clear_step_worker)
        self._step_worker = worker
        worker.start()

    def _on_step_done(self, ok: bool, msg: str, label: str) -> None:
        self._set_step_buttons_enabled(True)
        self.pose_status.setText(msg)
        self.log_message.emit(msg if ok else f"{label} failed: {msg}")

    def _clear_step_worker(self) -> None:
        self._step_worker = None

    def _set_step_buttons_enabled(self, enabled: bool) -> None:
        for btn in self._step_buttons:
            btn.setEnabled(enabled)

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
        # Full shutdown (not just the status poller): nested ImageView is not
        # guaranteed its own hideEvent inside a QStackedWidget, so stop its
        # poller explicitly here — same pattern as RobotPage.
        self.shutdown()

    def _stop_poller(self) -> None:
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None

    def shutdown(self) -> None:
        """Stop every background thread. Idempotent; called from hideEvent and
        MainWindow.closeEvent so neither leaves a thread running at teardown."""
        self._stop_poller()
        if self._step_worker is not None:
            stop_thread(self._step_worker)
            self._step_worker = None
        self.image_view.shutdown()
