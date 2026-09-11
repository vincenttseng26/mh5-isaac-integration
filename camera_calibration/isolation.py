"""Run a function in a throwaway subprocess so a native segfault inside
cv2.aruco cannot take down the caller (pytest, a live capture loop, ...).

Why 'spawn' and not 'fork': forking after cv2/numpy/BLAS have already
initialized threads is itself a documented source of native crashes
(the child inherits a half-initialized thread pool). 'spawn' starts a
completely fresh interpreter, which both avoids that class of bug and
mirrors what actually happens when a user runs a plain `python foo.py`.
"""
from __future__ import annotations

import dataclasses
import importlib
import multiprocessing as mp
import signal
import time
import traceback
from typing import Any, Optional, Sequence


@dataclasses.dataclass
class IsolatedResult:
    status: str  # "ok" | "error" | "crashed" | "timeout" | "unknown"
    value: Any = None
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        return self.status == "ok"


def _worker(module_name: str, func_name: str, args, kwargs, conn) -> None:
    try:
        module = importlib.import_module(module_name)
        func = getattr(module, func_name)
        result = func(*args, **kwargs)
        conn.send(("ok", result))
    except Exception:  # noqa: BLE001 - report every exception to the parent
        conn.send(("error", traceback.format_exc()))
    finally:
        conn.close()


def run_isolated(
    module_name: str,
    func_name: str,
    args: Sequence[Any] = (),
    kwargs: Optional[dict] = None,
    timeout: float = 30.0,
) -> IsolatedResult:
    """Run module_name.func_name(*args, **kwargs) in a spawned subprocess.

    module_name/func_name (rather than a direct function reference) because
    'spawn' re-imports the target in the child, so the function must be
    importable by name — a bound closure or lambda would not survive.

    Note on the polling loop below: a naive `proc.join(timeout)` followed by
    `parent_conn.recv()` deadlocks for any result bigger than the OS pipe
    buffer (~64KB on Linux, e.g. a rendered board image). `Connection.send`
    blocks until the reader drains the pipe, but the parent never reads
    until join() returns — and join() never returns because the child is
    stuck in send(). So we must poll the connection *while* waiting for the
    child, not after.
    """
    kwargs = kwargs or {}
    ctx = mp.get_context("spawn")
    parent_conn, child_conn = ctx.Pipe()
    proc = ctx.Process(target=_worker, args=(module_name, func_name, args, kwargs, child_conn))
    proc.start()
    child_conn.close()  # only the child should hold the write end open

    deadline = time.monotonic() + timeout
    status: Optional[str] = None
    payload: Any = None
    got_result = False
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if parent_conn.poll(min(remaining, 0.1)):
            try:
                status, payload = parent_conn.recv()
                got_result = True
            except EOFError:
                pass  # child died before/while sending; fall through to exitcode check
            break
        if not proc.is_alive():
            break

    # Give the child a moment to actually exit after sending (it closes its
    # end of the pipe right after) before deciding on timeout/crash; if we
    # never got a result, spend whatever's left of the original budget.
    join_timeout = 5.0 if got_result else max(0.0, deadline - time.monotonic())
    proc.join(join_timeout)

    if proc.is_alive():
        proc.terminate()
        proc.join()
        return IsolatedResult(status="timeout", error=f"Worker timed out after {timeout}s")

    parent_conn.close()

    if not got_result and proc.exitcode is not None and proc.exitcode < 0:
        signum = -proc.exitcode
        try:
            sig_name = signal.Signals(signum).name
        except ValueError:
            sig_name = str(signum)
        return IsolatedResult(
            status="crashed",
            error=(
                f"Worker process died from signal {signum} ({sig_name}). "
                "This means the crash is native (inside cv2/aruco), not a "
                "Python exception. Run camera_calibration/diagnose.py and "
                "see README.md 'Known OpenCV issues'."
            ),
        )

    if got_result:
        if status == "ok":
            return IsolatedResult(status="ok", value=payload)
        return IsolatedResult(status="error", error=payload)

    return IsolatedResult(
        status="unknown",
        error="Worker exited cleanly but sent no result; treat as a failure.",
    )
