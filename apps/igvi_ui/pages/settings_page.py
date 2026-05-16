from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError


class SettingsPage(QWidget):
    def __init__(self, client: HostClient):
        super().__init__()
        self.client = client
        self._build_ui()
        self.refresh()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)

        self._build_host_tab()
        self._build_camera_tab()
        self._build_wheel_tab()
        self._build_imu_tab()
        self._build_ekf_tab()

        self.health_label = QLabel()
        self.health_label.setObjectName("Muted")
        self.health_label.setWordWrap(True)
        layout.addWidget(self.health_label)

    # ── Host Tab ──────────────────────────────────────────────────────────────

    def _build_host_tab(self) -> None:
        page = QWidget()
        outer = QVBoxLayout(page)
        toolbar = QHBoxLayout()
        toolbar.addStretch(1)
        refresh = QPushButton("Reload")
        refresh.clicked.connect(self.refresh)
        toolbar.addWidget(refresh)
        save = QPushButton("Save")
        save.setObjectName("Primary")
        save.clicked.connect(self._save_host)
        toolbar.addWidget(save)
        outer.addLayout(toolbar)

        frame = QFrame()
        frame.setObjectName("Panel")
        form = QFormLayout(frame)
        form.setContentsMargins(14, 14, 14, 14)
        self.repo_root = QLineEdit()
        self.project_name = QLineEdit()
        self.host = QLineEdit()
        self.port = QLineEdit()
        self.rosbridge_url = QLineEdit()
        self.shm_path = QLineEdit()
        form.addRow("Repo root", self.repo_root)
        form.addRow("Compose project", self.project_name)
        form.addRow("Host", self.host)
        form.addRow("Port", self.port)
        form.addRow("rosbridge", self.rosbridge_url)
        form.addRow("UI bridge shm", self.shm_path)
        outer.addWidget(frame)
        outer.addStretch(1)
        self.tabs.addTab(page, "Host")

    # ── Camera Extrinsics Tab ─────────────────────────────────────────────────

    def _build_camera_tab(self) -> None:
        page = QWidget()
        outer = QVBoxLayout(page)
        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("Camera mount (base_link -> camera_base)"))
        toolbar.addStretch(1)
        save = QPushButton("Save")
        save.setObjectName("Primary")
        save.clicked.connect(self._save_calibration)
        toolbar.addWidget(save)
        restart = QPushButton("Restart SLAM")
        restart.clicked.connect(lambda: self._restart_services(["slam_fusion", "slam_localization"]))
        toolbar.addWidget(restart)
        outer.addLayout(toolbar)

        frame = QFrame()
        frame.setObjectName("Panel")
        form = QFormLayout(frame)
        form.setContentsMargins(14, 14, 14, 14)
        self.cam_x = self._spin(-2.0, 2.0, 0.01, 4)
        self.cam_y = self._spin(-2.0, 2.0, 0.01, 4)
        self.cam_z = self._spin(-2.0, 2.0, 0.01, 4)
        self.cam_roll = self._spin(-3.15, 3.15, 0.01, 4)
        self.cam_pitch = self._spin(-3.15, 3.15, 0.01, 4)
        self.cam_yaw = self._spin(-3.15, 3.15, 0.01, 4)
        form.addRow("X (m, forward+)", self.cam_x)
        form.addRow("Y (m, left+)", self.cam_y)
        form.addRow("Z (m, up+)", self.cam_z)
        form.addRow("Roll (rad)", self.cam_roll)
        form.addRow("Pitch (rad)", self.cam_pitch)
        form.addRow("Yaw (rad)", self.cam_yaw)
        outer.addWidget(frame)
        outer.addStretch(1)
        self.tabs.addTab(page, "Camera")

    # ── Wheel Odometry Tab ────────────────────────────────────────────────────

    def _build_wheel_tab(self) -> None:
        page = QWidget()
        outer = QVBoxLayout(page)
        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("Wheel odometry parameters"))
        toolbar.addStretch(1)
        save = QPushButton("Save")
        save.setObjectName("Primary")
        save.clicked.connect(self._save_calibration)
        toolbar.addWidget(save)
        restart = QPushButton("Restart kros_car")
        restart.clicked.connect(lambda: self._restart_services(["kros_car"]))
        toolbar.addWidget(restart)
        outer.addLayout(toolbar)

        frame = QFrame()
        frame.setObjectName("Panel")
        form = QFormLayout(frame)
        form.setContentsMargins(14, 14, 14, 14)
        self.wheel_sep = self._spin(0.05, 1.0, 0.001, 4)
        self.wheel_sep_mult = self._spin(0.5, 5.0, 0.01, 3)
        self.wheel_radius = self._spin(0.01, 0.2, 0.0001, 5)
        form.addRow("Wheel separation (m)", self.wheel_sep)
        form.addRow("Separation multiplier", self.wheel_sep_mult)
        form.addRow("Wheel radius (m)", self.wheel_radius)
        outer.addWidget(frame)
        outer.addStretch(1)
        self.tabs.addTab(page, "Wheel")

    # ── IMU Tab ───────────────────────────────────────────────────────────────

    def _build_imu_tab(self) -> None:
        page = QWidget()
        outer = QVBoxLayout(page)
        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("IMU gyro bias seed + online calibration"))
        toolbar.addStretch(1)
        save = QPushButton("Save")
        save.setObjectName("Primary")
        save.clicked.connect(self._save_calibration)
        toolbar.addWidget(save)
        restart = QPushButton("Restart SLAM")
        restart.clicked.connect(lambda: self._restart_services(["slam_fusion", "slam_localization"]))
        toolbar.addWidget(restart)
        outer.addLayout(toolbar)

        frame = QFrame()
        frame.setObjectName("Panel")
        form = QFormLayout(frame)
        form.setContentsMargins(14, 14, 14, 14)
        self.gyro_x = self._spin(-1.0, 1.0, 0.0001, 6)
        self.gyro_y = self._spin(-1.0, 1.0, 0.0001, 6)
        self.gyro_z = self._spin(-1.0, 1.0, 0.0001, 6)
        form.addRow("Gyro bias X (rad/s)", self.gyro_x)
        form.addRow("Gyro bias Y (rad/s)", self.gyro_y)
        form.addRow("Gyro bias Z (rad/s)", self.gyro_z)
        outer.addWidget(frame)
        outer.addStretch(1)
        self.tabs.addTab(page, "IMU")

    # ── EKF Tab ───────────────────────────────────────────────────────────────

    def _build_ekf_tab(self) -> None:
        page = QWidget()
        outer = QVBoxLayout(page)
        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("EKF state estimator tuning"))
        toolbar.addStretch(1)
        save = QPushButton("Save")
        save.setObjectName("Primary")
        save.clicked.connect(self._save_calibration)
        toolbar.addWidget(save)
        restart = QPushButton("Restart SLAM")
        restart.clicked.connect(lambda: self._restart_services(["slam_fusion", "slam_localization"]))
        toolbar.addWidget(restart)
        outer.addLayout(toolbar)

        frame = QFrame()
        frame.setObjectName("Panel")
        form = QFormLayout(frame)
        form.setContentsMargins(14, 14, 14, 14)
        self.ekf_freq = QSpinBox()
        self.ekf_freq.setRange(10, 200)
        self.ekf_freq.setSuffix(" Hz")
        self.ekf_timeout = self._spin(0.05, 5.0, 0.05, 2)
        form.addRow("Frequency", self.ekf_freq)
        form.addRow("Sensor timeout (s)", self.ekf_timeout)
        outer.addWidget(frame)
        outer.addStretch(1)
        self.tabs.addTab(page, "EKF")

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _spin(min_val: float, max_val: float, step: float, decimals: int) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(min_val, max_val)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        return s

    # ── Data ──────────────────────────────────────────────────────────────────

    def refresh(self) -> None:
        try:
            settings = self.client.settings()
            health = self.client.health()
            bridge = self.client.ui_bridge_health()
        except HostClientError as exc:
            self.health_label.setText(f"Host Agent unavailable: {exc}")
            return
        self.repo_root.setText(str(settings.get("repo_root") or ""))
        self.project_name.setText(str(settings.get("project_name") or ""))
        self.host.setText(str(settings.get("host") or ""))
        self.port.setText(str(settings.get("port") or ""))
        self.rosbridge_url.setText(str(settings.get("rosbridge_url") or ""))
        self.shm_path.setText(str(settings.get("shm_path") or ""))
        self.health_label.setText(
            f"Docker: {'ok' if health.get('docker_available') else 'unavailable'}    "
            f"Compose: {'ok' if health.get('compose_available') else 'unavailable'}    "
            f"Dev mode: {health.get('dev_mode')}    "
            f"UI bridge: {bridge.get('message')}"
        )
        try:
            calib = self.client.calibration()
        except HostClientError:
            return
        self.cam_x.setValue(calib.get("camera_x", 0.17))
        self.cam_y.setValue(calib.get("camera_y", 0.0))
        self.cam_z.setValue(calib.get("camera_z", 0.25))
        self.cam_roll.setValue(calib.get("camera_roll", 0.0))
        self.cam_pitch.setValue(calib.get("camera_pitch", 0.48))
        self.cam_yaw.setValue(calib.get("camera_yaw", 0.0))
        self.wheel_sep.setValue(calib.get("wheel_separation", 0.274))
        self.wheel_sep_mult.setValue(calib.get("wheel_separation_multiplier", 2.21))
        self.wheel_radius.setValue(calib.get("wheel_radius", 0.05035))
        self.gyro_x.setValue(calib.get("gyro_bias_x", 0.0))
        self.gyro_y.setValue(calib.get("gyro_bias_y", 0.0))
        self.gyro_z.setValue(calib.get("gyro_bias_z", 0.0))
        self.ekf_freq.setValue(int(calib.get("ekf_frequency", 50)))
        self.ekf_timeout.setValue(calib.get("ekf_sensor_timeout", 0.2))

    def _save_host(self) -> None:
        payload = {
            "repo_root": self.repo_root.text().strip(),
            "project_name": self.project_name.text().strip(),
            "host": self.host.text().strip(),
            "port": int(self.port.text().strip() or 0),
            "rosbridge_url": self.rosbridge_url.text().strip(),
            "shm_path": self.shm_path.text().strip(),
            "dev_mode": False,
        }
        try:
            self.client.request("POST", "/api/settings", payload)
        except HostClientError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        QMessageBox.information(self, "Saved", "Settings saved. Restart host agent if port/host changed.")
        self.refresh()

    def _save_calibration(self) -> None:
        payload = {
            "camera_x": self.cam_x.value(),
            "camera_y": self.cam_y.value(),
            "camera_z": self.cam_z.value(),
            "camera_roll": self.cam_roll.value(),
            "camera_pitch": self.cam_pitch.value(),
            "camera_yaw": self.cam_yaw.value(),
            "gyro_bias_x": self.gyro_x.value(),
            "gyro_bias_y": self.gyro_y.value(),
            "gyro_bias_z": self.gyro_z.value(),
            "wheel_separation": self.wheel_sep.value(),
            "wheel_separation_multiplier": self.wheel_sep_mult.value(),
            "wheel_radius": self.wheel_radius.value(),
            "ekf_frequency": self.ekf_freq.value(),
            "ekf_sensor_timeout": self.ekf_timeout.value(),
        }
        try:
            self.client.set_calibration(payload)
        except HostClientError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        QMessageBox.information(self, "Saved", "Calibration saved to YAML. Restart relevant service to apply.")

    def _restart_services(self, services: list[str]) -> None:
        reply = QMessageBox.question(
            self, "Restart",
            f"Restart {', '.join(services)}? Changes take effect after restart.",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            self.client.compose_action("restart", services=services)
        except HostClientError as exc:
            QMessageBox.warning(self, "Restart failed", str(exc))
            return
        QMessageBox.information(self, "Restarted", f"{', '.join(services)} restarted.")

    def shutdown(self) -> None:
        pass
