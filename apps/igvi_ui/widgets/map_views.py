from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget


class Map2DView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(360)

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        painter.fillRect(rect, QColor("#243041"))

        grid_pen = QPen(QColor(148, 163, 184, 42), 1)
        painter.setPen(grid_pen)
        step = 32
        for x in range(0, rect.width(), step):
            painter.drawLine(x, 0, x, rect.height())
        for y in range(0, rect.height(), step):
            painter.drawLine(0, y, rect.width(), y)

        margin = 46
        room = QRectF(margin, margin, rect.width() - margin * 2, rect.height() - margin * 2)
        painter.setPen(QPen(QColor("#9aa7b8"), 3))
        painter.setBrush(QColor(15, 23, 42, 80))
        painter.drawRect(room)

        wall_pen = QPen(QColor("#a7b3c4"), 9)
        wall_pen.setCapStyle(Qt.PenCapStyle.SquareCap)
        painter.setPen(wall_pen)
        painter.drawLine(room.left() + room.width() * 0.18, room.top(), room.left() + room.width() * 0.18, room.top() + room.height() * 0.38)
        painter.drawLine(room.left() + room.width() * 0.18, room.top() + room.height() * 0.38, room.left() + room.width() * 0.38, room.top() + room.height() * 0.38)
        painter.drawLine(room.left() + room.width() * 0.64, room.top(), room.left() + room.width() * 0.64, room.top() + room.height() * 0.58)
        painter.drawLine(room.left() + room.width() * 0.64, room.top() + room.height() * 0.58, room.right(), room.top() + room.height() * 0.58)

        path = QPainterPath()
        start = QPointF(room.left() + room.width() * 0.35, room.bottom() - room.height() * 0.10)
        goal = QPointF(room.left() + room.width() * 0.85, room.top() + room.height() * 0.32)
        path.moveTo(start)
        path.cubicTo(
            QPointF(room.left() + room.width() * 0.48, room.bottom() - room.height() * 0.26),
            QPointF(room.left() + room.width() * 0.66, room.top() + room.height() * 0.72),
            goal,
        )
        painter.setPen(QPen(QColor("#60a5fa"), 4))
        painter.drawPath(path)

        dash_pen = QPen(QColor("#14b8a6"), 5)
        dash_pen.setDashPattern([6, 5])
        painter.setPen(dash_pen)
        painter.drawPath(path)

        painter.setPen(QPen(QColor("#dbeafe"), 4))
        painter.setBrush(QColor("#3b82f6"))
        painter.drawEllipse(start, 15, 15)
        painter.setPen(QPen(QColor("#ffedd5"), 4))
        painter.setBrush(QColor("#f59e0b"))
        painter.drawEllipse(goal, 12, 12)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#ef4444"))
        for point in (
            QPointF(room.left() + 55, room.top() + 60),
            QPointF(room.right() - 70, room.bottom() - 90),
            QPointF(room.left() + room.width() * 0.70, room.top() + 45),
        ):
            painter.drawEllipse(point, 5, 5)


class Map3DView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(220)

    def paintEvent(self, event) -> None:  # noqa: N802
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
