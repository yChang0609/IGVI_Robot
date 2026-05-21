"""BaseTask and _StepCtx — shared scaffolding for all competition tasks."""
from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Generator

from igvi_task_core.fsm import TaskFSM

if TYPE_CHECKING:
    from igvi_task_core.node import TaskManagerNode


class BaseTask:
    """Subclasses implement run(fsm) with the full task flow.

    Access ROS action wrappers via self.nav, self.grab, self.detect.
    Use the step() context manager to delimit named steps.
    """

    def __init__(self, node: "TaskManagerNode") -> None:
        self.node = node
        self.log = node.task_log          # append(str) → goes to /task/status log
        self.nav = node.navigator
        self.grab = node.grabber
        self.detect = node.detector

    def run(self, fsm: TaskFSM) -> None:
        raise NotImplementedError

    @contextmanager
    def step(self, fsm: TaskFSM, index: int, name: str) -> Generator[None, None, None]:
        """Name a step, check for interrupt, emit transition, log completion."""
        fsm.check()
        fsm.advance(index, name)
        self.log(f"▶  {name}")
        try:
            yield
        except InterruptedError:
            self.log(f"✗  {name} (interrupted)")
            raise
        except Exception as exc:
            self.log(f"✗  {name}: {exc}")
            raise
        self.log(f"✓  {name}")
