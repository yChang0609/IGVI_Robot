from __future__ import annotations

from PySide6.QtWidgets import (
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
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

        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("Settings"))
        toolbar.addStretch(1)
        refresh = QPushButton("Reload")
        refresh.clicked.connect(self.refresh)
        toolbar.addWidget(refresh)
        save = QPushButton("Save")
        save.setObjectName("Primary")
        save.clicked.connect(self.save)
        toolbar.addWidget(save)
        layout.addLayout(toolbar)

        form_frame = QFrame()
        form_frame.setObjectName("Panel")
        form = QFormLayout(form_frame)
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
        layout.addWidget(form_frame)

        self.health_label = QLabel()
        self.health_label.setObjectName("Muted")
        self.health_label.setWordWrap(True)
        layout.addWidget(self.health_label)
        layout.addStretch(1)

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

    def save(self) -> None:
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
