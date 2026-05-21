from __future__ import annotations

from enum import Enum, auto


class TaskState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    FAILED = auto()
    INTERRUPTED = auto()

    @classmethod
    def from_str(cls, value: str) -> "TaskState":
        return _STR_MAP.get(value.lower(), cls.IDLE)

    def label(self) -> str:
        return {
            TaskState.IDLE: "Idle",
            TaskState.RUNNING: "Running",
            TaskState.PAUSED: "Paused",
            TaskState.COMPLETED: "Completed",
            TaskState.FAILED: "Failed",
            TaskState.INTERRUPTED: "Interrupted",
        }[self]

    def color(self) -> str:
        return {
            TaskState.IDLE: "#6f7e91",
            TaskState.RUNNING: "#3b82f6",
            TaskState.PAUSED: "#f59e0b",
            TaskState.COMPLETED: "#22c55e",
            TaskState.FAILED: "#ef4444",
            TaskState.INTERRUPTED: "#a855f7",
        }[self]


_STR_MAP: dict[str, TaskState] = {
    "idle": TaskState.IDLE,
    "running": TaskState.RUNNING,
    "paused": TaskState.PAUSED,
    "completed": TaskState.COMPLETED,
    "failed": TaskState.FAILED,
    "interrupted": TaskState.INTERRUPTED,
}
