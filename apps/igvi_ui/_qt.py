from __future__ import annotations

from PySide6.QtCore import QThread


def stop_thread(thread: QThread | None, wait_ms: int = 6000) -> None:
    """Stop a QThread and block until it has actually finished.

    Destroying a still-running QThread makes Qt call qFatal() ("QThread:
    Destroyed while thread is still running"), which aborts the process with
    SIGABRT (exit 134). Every owner must call this before dropping its thread
    or before the QApplication exits.

    Worker threads expose a ``stop()`` that flips their run-loop flag; one-shot
    workers don't and simply finish when their blocking call returns. As a last
    resort (e.g. a worker stuck in a long compose timeout) the thread is
    terminated so shutdown cannot hang.
    """
    if thread is None:
        return
    stop = getattr(thread, "stop", None)
    if callable(stop):
        stop()
    thread.requestInterruption()
    if not thread.wait(wait_ms):
        thread.terminate()
        thread.wait()
