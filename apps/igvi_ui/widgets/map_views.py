from __future__ import annotations

import math
from typing import Any

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QWidget


class Map2DView(QWidget):
    goal_requested = Signal(float, float, float)  # world x, y, yaw

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(360)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self._map_data: dict[str, Any] | None = None
        self._map_pixmap: QPixmap | None = None
        self._pose: dict[str, float] | None = None
        self._goal: tuple[float, float, float] | None = None  # x, y, yaw
        self._locked = False
        self._drag_origin: tuple[float, float] | None = None  # widget px while dragging
        self._drag_current: tuple[float, float] | None = None
        self._drag_world: tuple[float, float] | None = None

    def set_locked(self, locked: bool) -> None:
        self._locked = locked
        self.setCursor(Qt.CursorShape.ForbiddenCursor if locked else Qt.CursorShape.CrossCursor)
        self.update()

    # ── Public update API ─────────────────────────────────────────────────────

    def update_map(self, data: dict[str, Any]) -> None:
        if not data or not data.get("width") or not data.get("data"):
            return
        self._map_data = data
        self._map_pixmap = QPixmap.fromImage(_build_image(data))
        self.update()

    def update_pose(self, pose: dict[str, float]) -> None:
        self._pose = pose
        self.update()

    # ── Qt events ────────────────────────────────────────────────────────────

    def paintEvent(self, _event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#1a2535"))

        if not self._map_pixmap:
            _draw_placeholder(painter, self.rect())
            return

        scaled = self._map_pixmap.scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation
        )
        ox = (self.width() - scaled.width()) // 2
        oy = (self.height() - scaled.height()) // 2
        painter.drawPixmap(ox, oy, scaled)
        map_rect = QRectF(ox, oy, scaled.width(), scaled.height())

        if self._goal:
            gx, gy, gyaw = self._goal
            pt = self._world_to_widget(gx, gy, map_rect)
            if pt:
                painter.setPen(QPen(QColor("#f59e0b"), 2))
                painter.setBrush(QColor("#f59e0b"))
                painter.drawEllipse(pt, 8, 8)
                painter.setPen(QPen(QColor("#ffffff"), 1))
                r = 14.0
                painter.drawLine(QPointF(pt.x() - r, pt.y()), QPointF(pt.x() + r, pt.y()))
                painter.drawLine(QPointF(pt.x(), pt.y() - r), QPointF(pt.x(), pt.y() + r))
                dx = math.cos(gyaw) * 22
                dy = -math.sin(gyaw) * 22
                painter.setPen(QPen(QColor("#f59e0b"), 2))
                painter.drawLine(pt, QPointF(pt.x() + dx, pt.y() + dy))

        if self._drag_origin and self._drag_current:
            ox, oy = self._drag_origin
            cx, cy = self._drag_current
            painter.setPen(QPen(QColor("#fbbf24"), 2, Qt.PenStyle.DashLine))
            painter.drawLine(QPointF(ox, oy), QPointF(cx, cy))
            painter.setBrush(QColor("#fbbf24"))
            painter.setPen(QPen(QColor("#fbbf24"), 1))
            painter.drawEllipse(QPointF(ox, oy), 5, 5)

        if self._pose:
            pt = self._world_to_widget(self._pose["x"], self._pose["y"], map_rect)
            if pt:
                painter.setPen(QPen(QColor("#dbeafe"), 2))
                painter.setBrush(QColor("#3b82f6"))
                painter.drawEllipse(pt, 10, 10)
                yaw = self._pose.get("yaw", 0.0)
                dx = math.cos(yaw) * 18
                dy = -math.sin(yaw) * 18
                painter.setPen(QPen(QColor("#f59e0b"), 3))
                painter.drawLine(pt, QPointF(pt.x() + dx, pt.y() + dy))

        if self._locked:
            painter.fillRect(self.rect(), QColor(0, 0, 0, 55))
            font = painter.font()
            font.setBold(True)
            # The painter's font may be pixel-sized (pointSize() == -1 with the
            # app stylesheet); -1 + 1 == 0 spams "QFont::setPointSize <= 0".
            point_size = font.pointSize()
            if point_size > 0:
                font.setPointSize(point_size + 1)
            else:
                font.setPixelSize(max(13, font.pixelSize() + 2))
            painter.setFont(font)
            painter.setPen(QColor("#f59e0b"))
            banner = self.rect().adjusted(0, 6, 0, 0)
            painter.drawText(banner, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter,
                             "⌨  Keyboard Control Mode")

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if self._locked or event.button() != Qt.MouseButton.LeftButton or not self._map_data:
            return
        px, py = event.position().x(), event.position().y()
        world = self._widget_to_world(px, py)
        if not world:
            return
        self._drag_origin = (px, py)
        self._drag_current = (px, py)
        self._drag_world = world
        self.update()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._drag_origin is None:
            return
        self._drag_current = (event.position().x(), event.position().y())
        self.update()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if self._drag_origin is None or self._drag_world is None:
            return
        if event.button() != Qt.MouseButton.LeftButton:
            return
        ox, oy = self._drag_origin
        cx, cy = self._drag_current or self._drag_origin
        # yaw: positive CCW in world frame; widget Y is flipped
        dx = cx - ox
        dy = cy - oy
        if math.hypot(dx, dy) < 4.0:
            yaw = 0.0  # treat as click without drag
        else:
            yaw = math.atan2(-dy, dx)
        gx, gy = self._drag_world
        self._goal = (gx, gy, yaw)
        self._drag_origin = None
        self._drag_current = None
        self._drag_world = None
        self.goal_requested.emit(gx, gy, yaw)
        self.update()

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _map_rect(self) -> QRectF:
        if not self._map_pixmap:
            return QRectF(self.rect())
        scaled = self._map_pixmap.scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation
        )
        ox = (self.width() - scaled.width()) / 2
        oy = (self.height() - scaled.height()) / 2
        return QRectF(ox, oy, scaled.width(), scaled.height())

    def _world_to_widget(self, wx: float, wy: float, map_rect: QRectF) -> QPointF | None:
        if not self._map_data:
            return None
        md = self._map_data
        col = (wx - md["origin_x"]) / md["resolution"]
        row = md["height"] - 1 - (wy - md["origin_y"]) / md["resolution"]
        sx = map_rect.x() + col / md["width"] * map_rect.width()
        sy = map_rect.y() + row / md["height"] * map_rect.height()
        return QPointF(sx, sy)

    def _widget_to_world(self, px: float, py: float) -> tuple[float, float] | None:
        if not self._map_data:
            return None
        map_rect = self._map_rect()
        if not map_rect.contains(QPointF(px, py)):
            return None
        md = self._map_data
        col = (px - map_rect.x()) / map_rect.width() * md["width"]
        row_flipped = (py - map_rect.y()) / map_rect.height() * md["height"]
        row = md["height"] - 1 - row_flipped
        world_x = md["origin_x"] + col * md["resolution"]
        world_y = md["origin_y"] + row * md["resolution"]
        return world_x, world_y


class Map3DView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(220)

    def paintEvent(self, _event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        painter.fillRect(rect, QColor("#151d28"))

        center = QPointF(rect.width() * 0.50, rect.height() * 0.50)
        painter.setPen(QPen(QColor(139, 215, 255, 190), 3))
        for i in range(26):
            x = (i * 47) % max(rect.width(), 1)
            y = (i * 83 + 31) % max(rect.height(), 1)
            painter.drawPoint(x, y)

        painter.setPen(QPen(QColor("#dbeafe"), 2))
        painter.setBrush(QColor(59, 130, 246, 45))
        body = QRectF(center.x() - 42, center.y() - 22, 84, 44)
        painter.save()
        painter.translate(center)
        painter.rotate(-18)
        painter.translate(-center)
        painter.drawRoundedRect(body, 8, 8)
        painter.restore()

        origin = QPointF(28, rect.height() - 28)
        painter.setPen(QPen(QColor("#ef4444"), 4))
        painter.drawLine(origin, QPointF(origin.x() + 56, origin.y() - 14))
        painter.setPen(QPen(QColor("#22c55e"), 4))
        painter.drawLine(origin, QPointF(origin.x() + 18, origin.y() - 58))
        painter.setPen(QPen(QColor("#60a5fa"), 4))
        painter.drawLine(origin, QPointF(origin.x() + 42, origin.y() + 24))


# ── Helpers ───────────────────────────────────────────────────────────────────

def _build_image(data: dict[str, Any]) -> QImage:
    width: int = data["width"]
    height: int = data["height"]
    raw: list[int] = data["data"]

    buf = bytearray(width * height * 3)
    for i, val in enumerate(raw):
        if val < 0:
            r = g = b = 128
        elif val < 50:
            r = g = b = 210
        else:
            r = g = b = 30
        off = i * 3
        buf[off] = r
        buf[off + 1] = g
        buf[off + 2] = b

    img = QImage(bytes(buf), width, height, width * 3, QImage.Format.Format_RGB888)
    return img.mirrored(False, True)


def _draw_placeholder(painter: QPainter, rect) -> None:
    painter.setPen(QPen(QColor(148, 163, 184, 60), 1))
    step = 32
    for x in range(0, rect.width(), step):
        painter.drawLine(x, 0, x, rect.height())
    for y in range(0, rect.height(), step):
        painter.drawLine(0, y, rect.width(), y)
    painter.setPen(QColor(100, 116, 139))
    painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "Waiting for map…")
