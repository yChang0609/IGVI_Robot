"""Task 2 — 上橋抓熊下橋

Flow:
  導航 target_bridge  →  找熊  →  抓熊  →  導航 home_bridge  →  放熊
"""
from __future__ import annotations

from igvi_task_core.fsm import TaskFSM
from igvi_task_core.tasks.base import BaseTask

_BEAR_LABEL = "bear"
_FIND_TIMEOUT = 15.0   # seconds to search for bear at bridge


class Task2BridgeGrab(BaseTask):
    def run(self, fsm: TaskFSM) -> None:
        n = self.node
        tx, ty, tyaw = n.wp("target_bridge")
        hx, hy, hyaw = n.wp("home_bridge")

        with self.step(fsm, 0, "導航 → 橋目標點"):
            self.nav.go_to(tx, ty, tyaw, fsm)

        with self.step(fsm, 1, "找熊"):
            det = self.detect.find(_BEAR_LABEL, fsm, timeout=_FIND_TIMEOUT)
            if det is None:
                raise RuntimeError("橋上未偵測到目標熊")
            dist = det.distance_m or 0.3
            self.log(f"  偵測到熊 conf={det.confidence:.2f} dist={dist:.2f}m")

        with self.step(fsm, 2, "抓熊"):
            ok, grasped = self.grab.execute(_BEAR_LABEL, dist, fsm)
            if not grasped:
                raise RuntimeError(f"抓取未成功 (success={ok} grasped={grasped})")

        with self.step(fsm, 3, "導航 → Home"):
            self.nav.go_to(hx, hy, hyaw, fsm)

        with self.step(fsm, 4, "放熊"):
            # GrabObject server's place_pose is executed as part of the action.
            # A dedicated release step can be added here if the arm needs to
            # re-open after arriving at home (e.g. a second arm trajectory call).
            fsm.sleep(0.5)  # settle
