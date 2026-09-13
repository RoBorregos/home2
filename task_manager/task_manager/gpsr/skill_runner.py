"""
Single-worker execution of blocking subtask-manager calls.

Subtask methods block (they spin ROS futures internally), so running them inside a
py_trees tick makes the whole plan execute in one tick: nothing can be preempted and
the Timeout decorators never fire. Handing them to one worker thread lets leaves
return RUNNING, which is what makes Timeout, Deadline and the tick loop's ROS spin work.

One worker on purpose: skills drive the same arm and base, so they must never overlap.
"""

import threading
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Callable, Optional


class SkillRunner:
    """Serialized background execution of blocking skill calls."""

    def __init__(self, thread_name_prefix: str = "frida-skill"):
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix=thread_name_prefix)
        self._lock = threading.Lock()
        self._abandoned = 0

    def submit(self, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Future:
        """Queue a skill call. Runs after any already-queued call completes."""
        return self._pool.submit(fn, *args, **kwargs)

    def abandon(self, future: Optional[Future]) -> None:
        """
        Give up waiting on a call that timed out.

        A blocking skill cannot be killed, so the work continues on the worker and the
        next submit queues behind it. This keeps the robot physically consistent while
        letting the tree move on. Cancellable skills should instead expose a cancel hook.
        """
        if future is None or future.done():
            return
        with self._lock:
            self._abandoned += 1

    @property
    def abandoned_count(self) -> int:
        """How many calls outlived their timeout this run."""
        with self._lock:
            return self._abandoned

    def busy(self) -> bool:
        """True while an abandoned call is still occupying the worker."""
        return self._pool._work_queue.qsize() > 0

    def shutdown(self, wait: bool = False) -> None:
        self._pool.shutdown(wait=wait)
