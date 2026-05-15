from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Iterable


@dataclass(frozen=True)
class ProgressSnapshot:
    action: str | None
    busy: bool
    started_at: datetime | None
    finished_at: datetime | None
    lines: list[str]
    last_line: str
    seq: int


class ProgressBuffer:
    """Thread-safe ring buffer for streaming docker compose output to the UI."""

    def __init__(self, capacity: int = 500) -> None:
        self._lock = threading.Lock()
        self._lines: deque[str] = deque(maxlen=capacity)
        self._action: str | None = None
        self._busy: bool = False
        self._started_at: datetime | None = None
        self._finished_at: datetime | None = None
        self._seq: int = 0

    def start(self, action: str) -> None:
        with self._lock:
            self._lines.clear()
            self._action = action
            self._busy = True
            self._started_at = datetime.now().astimezone()
            self._finished_at = None
            self._seq += 1

    def append(self, line: str) -> None:
        if not line:
            return
        with self._lock:
            self._lines.append(line)
            self._seq += 1

    def extend(self, lines: Iterable[str]) -> None:
        with self._lock:
            for line in lines:
                if line:
                    self._lines.append(line)
                    self._seq += 1

    def finish(self) -> None:
        with self._lock:
            self._busy = False
            self._finished_at = datetime.now().astimezone()
            self._seq += 1

    def snapshot(self, since_seq: int = 0) -> ProgressSnapshot:
        with self._lock:
            lines = list(self._lines)
            last = lines[-1] if lines else ""
            return ProgressSnapshot(
                action=self._action,
                busy=self._busy,
                started_at=self._started_at,
                finished_at=self._finished_at,
                lines=lines,
                last_line=last,
                seq=self._seq,
            )


_progress = ProgressBuffer()


def get_progress_buffer() -> ProgressBuffer:
    return _progress
