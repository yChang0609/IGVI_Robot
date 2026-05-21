"""Task 3 — 導航開門

Flow:
  導航 target_door  →  找門  →  開門
"""
from __future__ import annotations

from igvi_task_core.fsm import TaskFSM
from igvi_task_core.tasks.base import BaseTask

_DOOR_LABEL = "door"
_FIND_TIMEOUT = 15.0


class Task3Door(BaseTask):
    def run(self, fsm: TaskFSM) -> None:
        n = self.node
        tx, ty, tyaw = n.wp("target_door")

        with self.step(fsm, 0, "導航 → 門目標點"):
            self.nav.go_to(tx, ty, tyaw, fsm)

        with self.step(fsm, 1, "找門"):
            det = self.detect.find(_DOOR_LABEL, fsm, timeout=_FIND_TIMEOUT)
            if det is None:
                raise RuntimeError("未偵測到目標門")
            dist = det.distance_m or 0.5
            self.log(f"  偵測到門 conf={det.confidence:.2f} dist={dist:.2f}m")

        with self.step(fsm, 2, "開門"):
            # GrabObject action reused: server will run its grasp_pose sequence.
            # Adjust object_label to "door_handle" if your model has that class.
            ok, _ = self.grab.execute(_DOOR_LABEL, dist, fsm)
            if not ok:
                raise RuntimeError("開門動作失敗")
