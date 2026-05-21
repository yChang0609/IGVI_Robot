"""Task 4 — 導航找熊抓熊 (loop)

Flow per target waypoint i:
  ┌─ 導航 target_find_grab_i ──────────────────────────────────┐
  │   找熊 (rotate-search until found or waypoints exhausted)  │
  └─────────────────────────────────────────────────────────────┘
       ↓ found
  抓熊  →  導航 home_bridge  →  放熊  →  i+1  → loop back
"""
from __future__ import annotations

import math

from geometry_msgs.msg import Twist

from igvi_task_core.fsm import TaskFSM
from igvi_task_core.tasks.base import BaseTask

_BEAR_LABEL = "bear"
_DETECT_TIMEOUT = 8.0     # seconds to look for bear after each rotation step
_ROTATE_STEP_RAD = math.pi / 4   # 45° per rotation attempt
_MAX_ROTATIONS = 8         # full 360° search before giving up at this waypoint


class Task4FindGrab(BaseTask):
    def run(self, fsm: TaskFSM) -> None:
        n = self.node
        targets = n.wps("target_find_grab")   # list of (x, y, yaw)
        hx, hy, hyaw = n.wp("home_bridge")

        if not targets:
            raise RuntimeError("target_find_grab waypoints not configured")

        for i, (tx, ty, tyaw) in enumerate(targets):
            self.log(f"[task4] 搜尋目標點 {i + 1}/{len(targets)}")

            with self.step(fsm, 0, f"導航 → 找熊目標點 {i + 1}"):
                self.nav.go_to(tx, ty, tyaw, fsm)

            det = self._search_for_bear(fsm, i + 1)
            if det is None:
                self.log(f"  目標點 {i + 1} 未找到熊，繼續下一個")
                continue

            dist = det.distance_m or 0.4
            self.log(f"  偵測到熊 conf={det.confidence:.2f} dist={dist:.2f}m")

            with self.step(fsm, 2, "抓熊"):
                ok, grasped = self.grab.execute(_BEAR_LABEL, dist, fsm)
                if not grasped:
                    self.log(f"  抓取失敗 (success={ok})，繼續搜尋")
                    continue

            with self.step(fsm, 3, "導航 → Home"):
                self.nav.go_to(hx, hy, hyaw, fsm)

            with self.step(fsm, 4, "放熊"):
                fsm.sleep(0.5)

            # Reset step index for next bear
            fsm.advance(0, "準備下一目標點")

        self.log("[task4] 所有目標點巡邏完畢")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _search_for_bear(self, fsm: TaskFSM, waypoint_num: int):
        """Rotate in place up to 360° looking for bear. Returns Detection or None."""
        for rotation in range(_MAX_ROTATIONS):
            fsm.check()

            # Quick non-blocking peek first
            det = self.detect.peek(_BEAR_LABEL)
            if det:
                return det

            with self.step(fsm, 1, f"找熊 (旋轉 {rotation + 1}/{_MAX_ROTATIONS})"):
                # Rotate one step
                self._rotate_step(fsm)
                # Then look with timeout
                det = self.detect.find(_BEAR_LABEL, fsm, timeout=_DETECT_TIMEOUT)
                if det:
                    return det

        return None

    def _rotate_step(self, fsm: TaskFSM) -> None:
        """Send one in-place rotation command and wait for it to settle."""
        try:
            cmd = Twist()
            cmd.angular.z = 0.4   # rad/s
            duration = _ROTATE_STEP_RAD / abs(cmd.angular.z)  # ≈ 1.96 s
            self.node.cmd_vel_pub.publish(cmd)
            fsm.sleep(duration)
            cmd.angular.z = 0.0
            self.node.cmd_vel_pub.publish(cmd)
            fsm.sleep(0.3)  # settle
        except InterruptedError:
            cmd = Twist()
            self.node.cmd_vel_pub.publish(cmd)  # stop on interrupt
            raise
