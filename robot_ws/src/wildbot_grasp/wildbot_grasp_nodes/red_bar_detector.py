"""HSV color detector for the door's red push-bar.

Finds the largest red blob in an RGB frame, treats the bar's right-edge as the
knob position (the physical knob sits at the rightmost end of the bar), and
samples depth from the aligned depth image at a pixel just inside the bar so
the surface returns a valid depth read.

The detector is intentionally stateless and ROS-free: pass it the latest BGR
image and depth image, get a RedBarDetection back. open_door_server and
tune_open_door both consume it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class RedBarParams:
    hue_lo1: int = 0
    hue_hi1: int = 10
    hue_lo2: int = 170
    hue_hi2: int = 179
    sat_min: int = 120
    val_min: int = 70
    min_area_px: int = 800
    aim_offset_px: int = 0       # +ve nudges aim point right of the bar's right edge
    depth_inset_px: int = 8      # pixels inside the right edge where depth is sampled
    depth_window_px: int = 5     # half-window for median depth sampling


@dataclass
class RedBarDetection:
    x_norm: float                # [-1, 1] horizontal offset of aim point from image center
    y_norm: float                # [-1, 1] vertical offset (informational)
    depth_m: float               # 0.0 when invalid
    depth_valid: bool
    aim_pixel: tuple[int, int]
    depth_pixel: tuple[int, int]
    bar_bbox: tuple[int, int, int, int]  # (x, y, w, h)
    bar_area_px: int


def detect(
    rgb_bgr: Optional[np.ndarray],
    depth: Optional[np.ndarray],
    params: RedBarParams,
) -> tuple[Optional[RedBarDetection], np.ndarray]:
    """Return (largest red blob's right-edge aim & depth or None, binary mask).

    The mask is always returned (even when no blob meets the area threshold) so
    callers can visualize whether the HSV bounds are even firing.
    """
    if rgb_bgr is None or rgb_bgr.size == 0:
        return None, np.zeros((0, 0), dtype=np.uint8)
    h, w = rgb_bgr.shape[:2]
    hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(
        hsv,
        (params.hue_lo1, params.sat_min, params.val_min),
        (params.hue_hi1, 255, 255),
    )
    m2 = cv2.inRange(
        hsv,
        (params.hue_lo2, params.sat_min, params.val_min),
        (params.hue_hi2, 255, 255),
    )
    mask = cv2.bitwise_or(m1, m2)
    # Speckle removal + small-hole fill so the bar reads as one solid blob.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    bar = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(bar))
    if area < params.min_area_px:
        return None, mask

    bx, by, bw, bh = cv2.boundingRect(bar)
    right_edge = bx + bw - 1
    aim_x = min(w - 1, max(0, right_edge + params.aim_offset_px))
    aim_y = by + bh // 2
    depth_x = min(w - 1, max(0, right_edge - params.depth_inset_px))
    depth_y = aim_y

    depth_m, depth_valid = (0.0, False)
    if depth is not None and depth.size > 0:
        depth_m, depth_valid = _sample_depth(depth, depth_x, depth_y, params.depth_window_px)

    det = RedBarDetection(
        x_norm=(aim_x - 0.5 * w) / (0.5 * w),
        y_norm=(aim_y - 0.5 * h) / (0.5 * h),
        depth_m=depth_m,
        depth_valid=depth_valid,
        aim_pixel=(int(aim_x), int(aim_y)),
        depth_pixel=(int(depth_x), int(depth_y)),
        bar_bbox=(int(bx), int(by), int(bw), int(bh)),
        bar_area_px=area,
    )
    return det, mask


def annotate(
    rgb_bgr: np.ndarray,
    det: Optional[RedBarDetection],
    mask: np.ndarray,
    *,
    fsm_state: Optional[str] = None,
) -> np.ndarray:
    """Return rgb_bgr with mask overlay, bbox, aim/depth markers, and text."""
    out = rgb_bgr.copy()
    h, w = out.shape[:2]

    # Semi-transparent red wash on the mask, so you see what the threshold caught.
    if mask is not None and mask.size > 0 and mask.shape[:2] == (h, w):
        wash = out.copy()
        wash[mask > 0] = (0, 0, 255)  # BGR red
        out = cv2.addWeighted(wash, 0.35, out, 0.65, 0)

    # Vertical centerline so you can eyeball x_norm.
    cv2.line(out, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)

    if det is not None:
        bx, by, bw, bh = det.bar_bbox
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)  # yellow bbox

        ax, ay = det.aim_pixel
        cv2.drawMarker(out, (ax, ay), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)  # green aim

        dx, dy = det.depth_pixel
        depth_color = (255, 255, 0) if det.depth_valid else (0, 0, 255)     # cyan / red
        cv2.circle(out, (dx, dy), 6, depth_color, -1)

        lines = [
            f"state: {fsm_state or '-'}",
            f"x_norm: {det.x_norm:+.3f}",
            f"depth: {det.depth_m:.3f} m  valid={det.depth_valid}",
            f"area:  {det.bar_area_px} px",
        ]
    else:
        lines = [
            f"state: {fsm_state or '-'}",
            "NO RED BAR — check HSV / lighting / framing",
        ]

    _draw_text_block(out, lines, x=12, y=28)
    return out


def _draw_text_block(img: np.ndarray, lines: list[str], x: int, y: int) -> None:
    for i, line in enumerate(lines):
        py = y + i * 26
        cv2.putText(img, line, (x, py), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 4, cv2.LINE_AA)            # black shadow
        cv2.putText(img, line, (x, py), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 1, cv2.LINE_AA)      # white text


def _sample_depth(
    depth: np.ndarray, x: int, y: int, half_win: int
) -> tuple[float, bool]:
    """Median of valid depths in a window. Handles 16UC1 (mm) and 32FC1 (m)."""
    h, w = depth.shape[:2]
    x0 = max(0, x - half_win)
    x1 = min(w, x + half_win + 1)
    y0 = max(0, y - half_win)
    y1 = min(h, y + half_win + 1)
    patch = depth[y0:y1, x0:x1]
    if np.issubdtype(patch.dtype, np.integer):
        vals = patch.astype(np.float32) / 1000.0  # k4a depth_to_rgb is uint16 mm
    else:
        vals = patch.astype(np.float32)            # 32FC1 already meters
    vals = vals[np.isfinite(vals) & (vals > 0.05) & (vals < 10.0)]
    if vals.size < 5:
        return 0.0, False
    return float(np.median(vals)), True
