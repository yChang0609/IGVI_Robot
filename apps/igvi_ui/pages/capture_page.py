from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtWidgets import (
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from igvi_ui.clients.host_client import HostClient, HostClientError
from igvi_ui.widgets.image_view import ImageView


class CapturePage(QWidget):
    _SETTINGS_KEY = "capture/save_dir"

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._settings = QSettings("IGVI", "IGVI UI")
        self._capture_dir = self._resolve_capture_dir()
        self._auto_timer = QTimer(self)
        self._auto_timer.timeout.connect(self._auto_capture)
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
        self.path_label.setWordWrap(True)
        self.path_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        title_box.addWidget(title)
        title_box.addWidget(self.path_label)
        header_layout.addLayout(title_box, 1)

        choose_btn = QPushButton("Choose...")
        choose_btn.clicked.connect(self._choose_capture_dir)
        header_layout.addWidget(choose_btn)

        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(1, 60)
        self.interval_spin.setValue(2)
        self.interval_spin.setSuffix(" s")
        self.interval_spin.setToolTip("Auto capture interval")
        self.interval_spin.valueChanged.connect(self._update_auto_interval)
        header_layout.addWidget(self.interval_spin)

        self.auto_btn = QPushButton("Auto Capture")
        self.auto_btn.setCheckable(True)
        self.auto_btn.clicked.connect(self._toggle_auto_capture)
        header_layout.addWidget(self.auto_btn)

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
        self._auto_timer.stop()
        self.image_view.shutdown()

    def _resolve_capture_dir(self) -> Path:
        saved_dir = self._settings.value(self._SETTINGS_KEY, "", str)
        if saved_dir:
            return Path(saved_dir).expanduser()

        try:
            settings = self.client.settings()
            repo_root = Path(str(settings.get("repo_root") or Path.cwd()))
        except (HostClientError, OSError):
            repo_root = Path.cwd()
        return repo_root / "captures"

    def _choose_capture_dir(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "Choose Capture Folder",
            str(self._capture_dir),
            QFileDialog.Option.ShowDirsOnly,
        )
        if not selected:
            return

        self._capture_dir = Path(selected).expanduser()
        self._settings.setValue(self._SETTINGS_KEY, str(self._capture_dir))
        self._update_path_label()
        self.status.setText(f"Save path changed: {self._capture_dir}")

    def _update_path_label(self) -> None:
        self.path_label.setText(f"Save path: {self._capture_dir}")

    def _capture(self) -> None:
        path = self._save_current_frame()
        if path is not None:
            self.status.setText(f"Saved: {path}")

    def _toggle_auto_capture(self, checked: bool) -> None:
        if checked:
            self._update_auto_interval()
            self._auto_timer.start()
            self.status.setText(f"Auto capture started: every {self.interval_spin.value()}s")
            self._auto_capture()
        else:
            self._auto_timer.stop()
            self.status.setText("Auto capture stopped")

    def _update_auto_interval(self) -> None:
        self._auto_timer.setInterval(int(self.interval_spin.value()) * 1000)

    def _auto_capture(self) -> None:
        path = self._save_current_frame(show_warning=False)
        if path is not None:
            self.status.setText(f"Auto saved: {path}")

    def _save_current_frame(self, show_warning: bool = True) -> Path | None:
        payload, topic = self.image_view.latest_frame()
        if not payload:
            if show_warning:
                QMessageBox.warning(
                    self,
                    "Capture unavailable",
                    "No live camera frame is available yet.",
                )
            else:
                self.status.setText("Auto capture waiting for a live frame...")
            return None

        self._capture_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        topic_name = _safe_topic_name(topic or "camera")
        path = self._unique_capture_path(timestamp, topic_name)
        try:
            path.write_bytes(payload)
        except OSError as exc:
            if show_warning:
                QMessageBox.warning(self, "Capture failed", str(exc))
            else:
                self.status.setText(f"Auto capture failed: {exc}")
            return None

        return path

    def _unique_capture_path(self, timestamp: str, topic_name: str) -> Path:
        base = self._capture_dir / f"capture_{timestamp}_{topic_name}.jpg"
        if not base.exists():
            return base
        for index in range(1, 1000):
            path = self._capture_dir / f"capture_{timestamp}_{topic_name}_{index:03d}.jpg"
            if not path.exists():
                return path
        return self._capture_dir / f"capture_{timestamp}_{topic_name}_999.jpg"


def _safe_topic_name(topic: str) -> str:
    value = topic.strip().strip("/") or "camera"
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value)
    return value[:80] or "camera"
