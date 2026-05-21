"""
TaskFSM — pure-Python, ROS-free finite state machine.

The task execution thread calls check() / sleep() at every safe interrupt point.
The control thread (HTTP handler) calls pause() / resume() / interrupt().
State transitions are reported via the on_transition callback.
"""
from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Callable


class TaskState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


OnTransition = Callable[[TaskState, int, str], None]


class TaskFSM:
    def __init__(self, on_transition: OnTransition) -> None:
        self._state = TaskState.IDLE
        self._step_index = 0
        self._step_name = ""
        self._interrupt = threading.Event()
        self._allow_run = threading.Event()
        self._allow_run.set()  # set = running allowed, clear = paused
        self._lock = threading.Lock()
        self._on_transition = on_transition

    # ── Read-only properties (safe from any thread) ───────────────────────────

    @property
    def state(self) -> TaskState:
        return self._state

    @property
    def step_index(self) -> int:
        return self._step_index

    @property
    def step_name(self) -> str:
        return self._step_name

    # ── Control interface (called from HTTP handler thread) ───────────────────

    def start(self) -> bool:
        terminal = (TaskState.COMPLETED, TaskState.FAILED, TaskState.INTERRUPTED)
        if self._state not in (TaskState.IDLE, *terminal):
            return False
        self._interrupt.clear()
        self._allow_run.set()
        self._transition(TaskState.RUNNING, 0, "")
        return True

    def pause(self) -> bool:
        if self._state != TaskState.RUNNING:
            return False
        self._allow_run.clear()
        self._transition(TaskState.PAUSED, self._step_index, self._step_name)
        return True

    def resume(self) -> bool:
        if self._state != TaskState.PAUSED:
            return False
        self._allow_run.set()
        self._transition(TaskState.RUNNING, self._step_index, self._step_name)
        return True

    def interrupt(self) -> bool:
        if self._state not in (TaskState.RUNNING, TaskState.PAUSED):
            return False
        self._interrupt.set()
        self._allow_run.set()  # unblock if currently paused
        return True

    # ── Task-thread interface (called from task execution thread) ─────────────

    def advance(self, index: int, name: str) -> None:
        """Record new step and emit transition. Call at the start of each step."""
        with self._lock:
            self._step_index = index
            self._step_name = name
        self._on_transition(self._state, index, name)

    def check(self) -> None:
        """
        Block while paused. Raise InterruptedError if interrupted.
        Call this between every atomic action inside a step.
        """
        while True:
            if self._interrupt.is_set():
                raise InterruptedError
            if self._allow_run.wait(timeout=0.05):
                break
        if self._interrupt.is_set():
            raise InterruptedError

    def sleep(self, seconds: float) -> None:
        """Interruptible sleep. Raises InterruptedError if interrupted."""
        deadline = time.monotonic() + seconds
        while True:
            self.check()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.05, remaining))

    # ── Terminal transitions (called from task thread via _run_task wrapper) ──

    def mark_completed(self) -> None:
        self._transition(TaskState.COMPLETED, self._step_index, self._step_name)

    def mark_failed(self) -> None:
        self._transition(TaskState.FAILED, self._step_index, self._step_name)

    def mark_interrupted(self) -> None:
        self._transition(TaskState.INTERRUPTED, self._step_index, self._step_name)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _transition(self, state: TaskState, step: int, name: str) -> None:
        with self._lock:
            self._state = state
            self._step_index = step
            self._step_name = name
        self._on_transition(state, step, name)
