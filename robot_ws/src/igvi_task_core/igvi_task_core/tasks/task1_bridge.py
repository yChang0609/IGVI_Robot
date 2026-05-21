"""Task 1 — 上下橋

Flow:
  導航 target_bridge  →  導航 home_bridge
"""
from __future__ import annotations

from igvi_task_core.fsm import TaskFSM
from igvi_task_core.tasks.base import BaseTask


class Task1Bridge(BaseTask):
    def run(self, fsm: TaskFSM) -> None:
        n = self.node
        tx, ty, tyaw = n.wp("target_bridge")
        hx, hy, hyaw = n.wp("home_bridge")

        with self.step(fsm, 0, "導航 → 橋目標點"):
            self.nav.go_to(tx, ty, tyaw, fsm)

        with self.step(fsm, 1, "導航 → Home"):
            self.nav.go_to(hx, hy, hyaw, fsm)
