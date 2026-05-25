from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path

from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.image_view import ImageView


class CapturePage(QWidget):
    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._capture_dir = self._resolve_capture_dir()
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(12)

        header = QFrame()
        header.setObjectName("Panel")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(14, 10, 14, 10)

        title_box = QVBoxLayout()
        title = QLabel("Camera Capture")
        title.setObjectName("SectionTitle")
        self.path_label = QLabel(f"Save path: {self._capture_dir}")
        self.path_label.setObjectName("Muted")
        title_box.addWidget(title)
        title_box.addWidget(self.path_label)
        header_layout.addLayout(title_box, 1)

        self.capture_btn = QPushButton("Capture")
        self.capture_btn.setObjectName("Primary")
        self.capture_btn.setFixedWidth(120)
        self.capture_btn.clicked.connect(self._capture)
        header_layout.addWidget(self.capture_btn)
        layout.addWidget(header)

        self.image_view = ImageView(self.client)
        self.image_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout.addWidget(self.image_view, 1)

        self.status = QLabel("Select a topic and wait for the live frame, then capture.")
        self.status.setObjectName("Muted")
        layout.addWidget(self.status)

    def shutdown(self) -> None:
        self.image_view.shutdown()

    def _resolve_capture_dir(self) -> Path:
        try:
            settings = self.client.settings()
            repo_root = Path(str(settings.get("repo_root") or Path.cwd()))
        except (HostClientError, OSError):
            repo_root = Path.cwd()
        return repo_root / "captures"

    def _capture(self) -> None:
        payload, topic = self.image_view.latest_frame()
        if not payload:
            QMessageBox.warning(self, "Capture unavailable", "No live camera frame is available yet.")
            return

        self._capture_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        topic_name = _safe_topic_name(topic or "camera")
        path = self._capture_dir / f"capture_{timestamp}_{topic_name}.jpg"
        try:
            path.write_bytes(payload)
        except OSError as exc:
            QMessageBox.warning(self, "Capture failed", str(exc))
            return

        self.status.setText(f"Saved: {path}")


def _safe_topic_name(topic: str) -> str:
    value = topic.strip().strip("/") or "camera"
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value)
    return value[:80] or "camera"
