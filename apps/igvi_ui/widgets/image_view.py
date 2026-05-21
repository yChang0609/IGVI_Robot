from __future__ import annotations

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPainter, QPen, QPixmap, QColor
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from igvi_ui._qt import stop_thread
from igvi_ui.clients.host_client import HostClient


class _ImagePoller(QThread):
    frame_received = Signal(bytes, str)
    error = Signal(str)

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._running = True
        self._topic: str | None = None

    def set_topic(self, topic: str | None) -> None:
        self._topic = topic

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            topic = self._topic
            if topic:
                try:
                    payload, active = self.client.image_frame(topic)
                    if payload:
                        self.frame_received.emit(payload, active or "")
                except Exception as exc:  # noqa: BLE001
                    self.error.emit(str(exc))
            self.msleep(200)


class _Canvas(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._pixmap: QPixmap | None = None
        self._placeholder = "Select an image topic"

    def set_placeholder(self, text: str) -> None:
        self._placeholder = text
        self._pixmap = None
        self.update()

    def set_image(self, pixmap: QPixmap) -> None:
        self._pixmap = pixmap
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#151d28"))
        if self._pixmap is None:
            painter.setPen(QPen(QColor(148, 163, 184, 60), 1))
            step = 32
            for x in range(0, self.width(), step):
                painter.drawLine(x, 0, x, self.height())
            for y in range(0, self.height(), step):
                painter.drawLine(0, y, self.width(), y)
            painter.setPen(QColor(100, 116, 139))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._placeholder)
            return
        scaled = self._pixmap.scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        ox = (self.width() - scaled.width()) // 2
        oy = (self.height() - scaled.height()) // 2
        painter.drawPixmap(ox, oy, scaled)


class ImageView(QWidget):
    """Subscribes to a ROS sensor_msgs/Image topic via the host bridge and renders frames."""

    def __init__(self, client: HostClient) -> None:
        super().__init__()
        self.client = client
        self._poller: _ImagePoller | None = None
        self._active_topic: str | None = None
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        bar = QHBoxLayout()
        bar.setSpacing(6)
        self.topic_combo = QComboBox()
        self.topic_combo.setMinimumWidth(180)
        self.topic_combo.currentTextChanged.connect(self._on_topic_changed)
        self.refresh_btn = QPushButton("↻")
        self.refresh_btn.setFixedWidth(28)
        self.refresh_btn.setToolTip("Refresh topic list")
        self.refresh_btn.clicked.connect(self.refresh_topics)
        bar.addWidget(QLabel("Topic"))
        bar.addWidget(self.topic_combo, 1)
        bar.addWidget(self.refresh_btn)
        layout.addLayout(bar)

        self.canvas = _Canvas()
        layout.addWidget(self.canvas, 1)

        self.status = QLabel("Idle")
        self.status.setObjectName("Muted")
        layout.addWidget(self.status)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._poller is None:
            self._poller = _ImagePoller(self.client)
            self._poller.frame_received.connect(self._on_frame)
            self._poller.error.connect(self._on_error)
            self._poller.set_topic(self._active_topic)
            self._poller.start()
        self.refresh_topics()

    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)
        self.shutdown()

    def shutdown(self) -> None:
        """Stop the poller thread. Idempotent; safe to call on app exit."""
        if self._poller is not None:
            stop_thread(self._poller)
            self._poller = None

    # ── Topic management ──────────────────────────────────────────────────────

    def refresh_topics(self) -> None:
        try:
            topics = self.client.image_topics()
        except Exception as exc:  # noqa: BLE001
            self.status.setText(f"Topic list error: {exc}")
            return
        current = self.topic_combo.currentText() or self._active_topic or ""
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        if not topics:
            self.topic_combo.addItem("(no image topics)")
            self.topic_combo.setEnabled(False)
        else:
            self.topic_combo.setEnabled(True)
            self.topic_combo.addItems(topics)
            preferred = current if current in topics else self._pick_default(topics)
            if preferred:
                self.topic_combo.setCurrentText(preferred)
        self.topic_combo.blockSignals(False)
        chosen = self.topic_combo.currentText() if self.topic_combo.isEnabled() else None
        if chosen and chosen != self._active_topic:
            self._on_topic_changed(chosen)

    @staticmethod
    def _pick_default(topics: list[str]) -> str | None:
        for preference in (
            "/eto_eye/annotated_image/compressed",
            "/rgb/image_bgr8",
            "/rgb/image_raw",
        ):
            if preference in topics:
                return preference
        return topics[0] if topics else None

    def _on_topic_changed(self, topic: str) -> None:
        if not topic or topic.startswith("("):
            return
        self._active_topic = topic
        if self._poller:
            self._poller.set_topic(topic)
        self.canvas.set_placeholder(f"Waiting for {topic}…")
        self.status.setText(f"Subscribed: {topic}")

    def _on_frame(self, payload: bytes, active: str) -> None:
        image = QImage.fromData(payload, "JPEG")
        if image.isNull():
            return
        self.canvas.set_image(QPixmap.fromImage(image))
        if active and active != self._active_topic:
            self._active_topic = active

    def _on_error(self, message: str) -> None:
        self.status.setText(f"Frame error: {message}")
