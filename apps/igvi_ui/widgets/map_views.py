from __future__ import annotations

import math
from typing import Any

import numpy as np

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QWidget


class Map2DView(QWidget):
    goal_requested = Signal(float, float, float)  # world x, y, yaw
    waypoint_point_picked = Signal(float, float, float)  # world x, y, yaw
    initial_pose_picked = Signal(float, float, float)  # world x, y, yaw
    home_pose_picked = Signal(float, float, float)  # world x, y, yaw
    arena_pose_picked = Signal(float, float, float)  # world x, y, yaw

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(160)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self._map_data: dict[str, Any] | None = None
        self._map_pixmap: QPixmap | None = None
        self._costmap_data: dict[str, Any] | None = None
        self._costmap_image: QImage | None = None
        self._pose: dict[str, float] | None = None
        self._plan: dict[str, Any] | None = None
        self._approach_pose: dict[str, float] | None = None
        self._goal: tuple[float, float, float] | None = None  # x, y, yaw
        self._home_pose: tuple[float, float, float] | None = None
        self._waypoints: dict[str, dict[str, float]] = {}
        self._semantic_objects: list[dict[str, Any]] = []
        self._selected_target_id: str = ""
        # Pick target controls what mouseRelease emits:
        #   ""             → nav goal (default)
        #   "waypoint"     → waypoint_point_picked
        #   "initial_pose" → initial_pose_picked
        self._pick_mode: str = ""
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

    def update_costmap(self, data: dict[str, Any]) -> None:
        if not data or not data.get("width") or not data.get("data"):
            return
        self._costmap_data = data
        self._costmap_image = _build_costmap_image(data)
        self.update()

    def clear_costmap(self) -> None:
        self._costmap_data = None
        self._costmap_image = None
        self.update()

    def update_pose(self, pose: dict[str, float]) -> None:
        self._pose = pose
        self.update()

    def update_waypoints(self, waypoints: dict[str, dict[str, float]]) -> None:
        self._waypoints = dict(waypoints or {})
        self.update()

    def set_home_pose(self, home_pose: tuple[float, float, float] | None) -> None:
        self._home_pose = home_pose
        self.update()

    def update_semantic_objects(self, objects: list[dict[str, Any]], selected_id: str = "") -> None:
        self._semantic_objects = list(objects or [])
        self._selected_target_id = selected_id
        self.update()

    def update_plan(self, data: dict[str, Any]) -> None:
        self._plan = data
        self.update()

    def update_approach_pose(self, pose: dict[str, float]) -> None:
        self._approach_pose = pose
        self.update()

    def set_pick_mode(self, enabled: bool) -> None:
        """Back-compat: when on, the next map click emits waypoint_point_picked."""
        self._set_pick_target("waypoint" if enabled else "")

    def set_initial_pose_mode(self, enabled: bool) -> None:
        """When on, the next map click emits initial_pose_picked (for SLAM relocalize)."""
        self._set_pick_target("initial_pose" if enabled else "")

    def set_home_pose_mode(self, enabled: bool) -> None:
        self._set_pick_target("home_pose" if enabled else "")

    def set_arena_pose_mode(self, enabled: bool) -> None:
        self._set_pick_target("arena_pose" if enabled else "")

    def set_nav_goal_mode(self, enabled: bool) -> None:
        self._set_pick_target("nav_goal" if enabled else "")

    def _set_pick_target(self, target: str) -> None:
        self._pick_mode = target
        self.setCursor(
            Qt.CursorShape.PointingHandCursor if target else Qt.CursorShape.CrossCursor
        )
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

        if self._costmap_image and self._costmap_data and self._map_data:
            cd = self._costmap_data
            pt_bl = self._world_to_widget(cd["origin_x"], cd["origin_y"], map_rect)
            pt_tr = self._world_to_widget(
                cd["origin_x"] + cd["width"] * cd["resolution"],
                cd["origin_y"] + cd["height"] * cd["resolution"],
                map_rect,
            )
            if pt_bl and pt_tr and pt_tr.x() > pt_bl.x() and pt_bl.y() > pt_tr.y():
                cm_rect = QRectF(pt_bl.x(), pt_tr.y(), pt_tr.x() - pt_bl.x(), pt_bl.y() - pt_tr.y())
                painter.setOpacity(0.6)
                painter.drawImage(cm_rect, self._costmap_image)
                painter.setOpacity(1.0)

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

        if self._home_pose:
            hx, hy, hyaw = self._home_pose
            pt = self._world_to_widget(hx, hy, map_rect)
            if pt:
                painter.setPen(QPen(QColor("#9333ea"), 2)) # Purple for home pose
                painter.setBrush(QColor("#c084fc"))
                painter.drawEllipse(pt, 8, 8)
                dx = math.cos(hyaw) * 22
                dy = -math.sin(hyaw) * 22
                painter.setPen(QPen(QColor("#9333ea"), 2))
                painter.drawLine(pt, QPointF(pt.x() + dx, pt.y() + dy))
                painter.setPen(QColor("#d8b4fe"))
                painter.drawText(QPointF(pt.x() + 9, pt.y() - 7), "Home")

        if self._plan and self._plan.get("poses"):
            painter.setPen(QPen(QColor("#14b8a6"), 3)) # Teal line
            poses = self._plan["poses"]
            for i in range(len(poses) - 1):
                pt1 = self._world_to_widget(poses[i]["x"], poses[i]["y"], map_rect)
                pt2 = self._world_to_widget(poses[i+1]["x"], poses[i+1]["y"], map_rect)
                if pt1 and pt2:
                    painter.drawLine(pt1, pt2)

        if self._approach_pose:
            ax, ay, ayaw = self._approach_pose.get("x", 0.0), self._approach_pose.get("y", 0.0), self._approach_pose.get("yaw", 0.0)
            pt = self._world_to_widget(ax, ay, map_rect)
            if pt:
                painter.setPen(QPen(QColor("#ec4899"), 2)) # Pink for approach pose
                painter.setBrush(QColor("#f472b6"))
                painter.drawEllipse(pt, 8, 8)
                dx = math.cos(ayaw) * 22
                dy = -math.sin(ayaw) * 22
                painter.setPen(QPen(QColor("#ec4899"), 2))
                painter.drawLine(pt, QPointF(pt.x() + dx, pt.y() + dy))
                painter.setPen(QColor("#fbcfe8"))
                painter.drawText(QPointF(pt.x() + 9, pt.y() - 7), "Approach")

        for name, wp in self._waypoints.items():
            pt = self._world_to_widget(wp.get("x", 0.0), wp.get("y", 0.0), map_rect)
            if pt is None:
                continue
            is_bridge = str(name) == "bridge_center"
            is_our_base = str(name) == "our_base"
            is_enemy_base = str(name) == "enemy_base"
            is_patrol = str(name).startswith("patrol_")
            is_home = str(name) == "home"

            if is_bridge:
                pen_color = QColor("#38bdf8")
                fill_color = QColor(56, 189, 248, 120)
                label_color = QColor("#bae6fd")
                radius = 8
            elif is_home:
                pen_color = QColor("#eab308") # Yellow
                fill_color = QColor(234, 179, 8, 120)
                label_color = QColor("#fef08a")
                radius = 8
            elif is_our_base:
                pen_color = QColor("#3b82f6") # Blue
                fill_color = QColor(59, 130, 246, 120)
                label_color = QColor("#93c5fd")
                radius = 8
            elif is_enemy_base:
                pen_color = QColor("#ef4444") # Red
                fill_color = QColor(239, 68, 68, 120)
                label_color = QColor("#fca5a5")
                radius = 8
            elif is_patrol:
                pen_color = QColor("#a855f7") # Purple
                fill_color = QColor(168, 85, 247, 90)
                label_color = QColor("#d8b4fe")
                radius = 6
            else:
                pen_color = QColor("#22c55e")
                fill_color = QColor(34, 197, 94, 90)
                label_color = QColor("#bbf7d0")
                radius = 6
            painter.setPen(QPen(pen_color, 2))
            painter.setBrush(fill_color)
            painter.drawEllipse(pt, radius, radius)
            if is_bridge:
                painter.setPen(QPen(QColor("#ffffff"), 1))
                cross = 12.0
                painter.drawLine(QPointF(pt.x() - cross, pt.y()), QPointF(pt.x() + cross, pt.y()))
                painter.drawLine(QPointF(pt.x(), pt.y() - cross), QPointF(pt.x(), pt.y() + cross))
            elif is_our_base or is_enemy_base:
                res = self._map_data.get("resolution", 0.05)
                exclusion_radius_px = 0.3 / res if res > 0 else 6
                painter.setPen(QPen(pen_color, 1, Qt.PenStyle.DashLine))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(pt, exclusion_radius_px, exclusion_radius_px)
            wyaw = wp.get("yaw", 0.0)
            painter.setPen(QPen(pen_color, 2))
            painter.drawLine(
                pt,
                QPointF(pt.x() + math.cos(wyaw) * 16, pt.y() - math.sin(wyaw) * 16),
            )
            painter.setPen(label_color)
            display_name = "Bridge" if is_bridge else "Our Base" if is_our_base else "Enemy Base" if is_enemy_base else "Home" if is_home else str(name)
            painter.drawText(QPointF(pt.x() + 9, pt.y() - 7), display_name)

        for obj in self._semantic_objects:
            pos = obj.get("position", {})
            wx, wy = pos.get("x"), pos.get("y")
            if wx is None or wy is None:
                continue
            pt = self._world_to_widget(wx, wy, map_rect)
            if pt is None:
                continue
            obj_id = obj.get("id", "")
            is_selected = obj_id == self._selected_target_id
            name = obj.get("class_name", "?")
            short_id = obj_id[:6] if len(obj_id) > 6 else obj_id
            if is_selected:
                painter.setPen(QPen(QColor("#f97316"), 2))
                painter.setBrush(QColor(249, 115, 22, 200))
                painter.drawEllipse(pt, 9, 9)
                # Cross-hair on selected target
                painter.setPen(QPen(QColor("#ffffff"), 1))
                r = 15.0
                painter.drawLine(QPointF(pt.x() - r, pt.y()), QPointF(pt.x() + r, pt.y()))
                painter.drawLine(QPointF(pt.x(), pt.y() - r), QPointF(pt.x(), pt.y() + r))
                painter.setPen(QColor("#fed7aa"))
                painter.drawText(QPointF(pt.x() + 11, pt.y() - 8), f"{name} [{short_id}]")
            else:
                painter.setPen(QPen(QColor("#4ade80"), 1))
                painter.setBrush(QColor(74, 222, 128, 140))
                painter.drawEllipse(pt, 6, 6)
                painter.setPen(QColor("#86efac"))
                painter.drawText(QPointF(pt.x() + 8, pt.y() - 5), short_id)

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
        self._drag_origin = None
        self._drag_current = None
        self._drag_world = None
        if self._pick_mode == "waypoint":
            self.waypoint_point_picked.emit(gx, gy, yaw)
        elif self._pick_mode == "initial_pose":
            self.initial_pose_picked.emit(gx, gy, yaw)
        elif self._pick_mode == "home_pose":
            self.home_pose_picked.emit(gx, gy, yaw)
            self._home_pose = (gx, gy, yaw)
        elif self._pick_mode == "arena_pose":
            self.arena_pose_picked.emit(gx, gy, yaw)
        elif self._pick_mode == "nav_goal":
            self._pick_mode = ""
            self.setCursor(Qt.CursorShape.CrossCursor)
            self._goal = (gx, gy, yaw)
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
    raw = np.asarray(data["data"], dtype=np.int8)

    # unknown (-1) → gray 128, free (0-49) → light 210, occupied (50+) → dark 30
    grey = np.full(len(raw), 128, dtype=np.uint8)
    grey[raw >= 0] = 210
    grey[raw >= 50] = 30

    buf = np.stack([grey, grey, grey], axis=1).flatten().tobytes()
    img = QImage(buf, width, height, width * 3, QImage.Format.Format_RGB888)
    return img.mirrored(False, True)


def _build_costmap_image(data: dict[str, Any]) -> QImage:
    width: int = data["width"]
    height: int = data["height"]
    raw = np.asarray(data["data"], dtype=np.int8)

    rgba = np.zeros((raw.size, 4), dtype=np.uint8)

    # inflation (1–99) → orange, alpha scales with cost value
    inf_mask = (raw >= 1) & (raw < 100)
    rgba[inf_mask, 0] = 255
    rgba[inf_mask, 1] = 140
    rgba[inf_mask, 2] = 0
    rgba[inf_mask, 3] = np.clip(raw[inf_mask].astype(np.int16) * 2, 30, 180).astype(np.uint8)

    # lethal (100+) → red
    let_mask = raw >= 100
    rgba[let_mask, 0] = 220
    rgba[let_mask, 1] = 40
    rgba[let_mask, 2] = 40
    rgba[let_mask, 3] = 200

    buf = rgba.tobytes()
    img = QImage(buf, width, height, width * 4, QImage.Format.Format_RGBA8888)
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
