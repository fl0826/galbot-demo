import json
import os
import sys
import threading
import time
import atexit
from queue import Queue, Empty
from dataclasses import dataclass
from functools import wraps
from logging.handlers import TimedRotatingFileHandler
import logging
from contextlib import AbstractContextManager
from typing import Callable, Dict, Optional


_PROFILE_ENABLED_OVERRIDE = None


def set_profile_enabled(enabled: bool) -> None:
    global _PROFILE_ENABLED_OVERRIDE
    _PROFILE_ENABLED_OVERRIDE = bool(enabled)


def profile_enabled() -> bool:
    if _PROFILE_ENABLED_OVERRIDE is not None:
        return bool(_PROFILE_ENABLED_OVERRIDE)
    # Enabled by default. Use set_profile_enabled(False) to disable at runtime.
    return True


def _default_profile_log_dir() -> str:
    # Match vla_logger behavior when running under systemd (LOG_DIR is set),
    # otherwise default to the robot path and fall back to a local logs dir.
    return os.environ.get("VLA_PROFILE_LOG_DIR") or os.environ.get("LOG_DIR") or "/userdata/log/galbot_mobile_manipulation"


_profile_logger = None
_profile_logger_lock = threading.Lock()
_profile_queue = None
_profile_writer_thread = None
_profile_writer_started = False
_profile_writer_lock = threading.Lock()
_PROFILE_ASYNC_ENABLED_OVERRIDE = None
_PROFILE_BUFFER_ENABLED_OVERRIDE = None
_profile_buffer_queue = None
_profile_buffer_dropped = 0
_profile_buffer_lock = threading.Lock()


def set_profile_async_enabled(enabled: bool) -> None:
    global _PROFILE_ASYNC_ENABLED_OVERRIDE
    _PROFILE_ASYNC_ENABLED_OVERRIDE = bool(enabled)


def profile_async_enabled() -> bool:
    if _PROFILE_ASYNC_ENABLED_OVERRIDE is not None:
        return bool(_PROFILE_ASYNC_ENABLED_OVERRIDE)
    # Enabled by default to reduce per-span overhead.
    return True


def set_profile_buffering(enabled: bool) -> None:
    global _PROFILE_BUFFER_ENABLED_OVERRIDE
    _PROFILE_BUFFER_ENABLED_OVERRIDE = bool(enabled)


def profile_buffering_enabled() -> bool:
    if _PROFILE_BUFFER_ENABLED_OVERRIDE is not None:
        return bool(_PROFILE_BUFFER_ENABLED_OVERRIDE)
    # Buffering is enabled by default to avoid runtime logging overhead.
    # Use set_profile_buffering(False) to stream logs continuously instead.
    return True


def _buffer_max_items() -> int:
    try:
        return int(os.environ.get("VLA_PROFILE_BUFFER_MAX", "500000"))
    except Exception:
        return 500000


def _ensure_profile_buffer() -> None:
    global _profile_buffer_queue
    if _profile_buffer_queue is not None:
        return
    with _profile_buffer_lock:
        if _profile_buffer_queue is None:
            _profile_buffer_queue = Queue(maxsize=_buffer_max_items())
            atexit.register(flush_profile_buffer)


def flush_profile_buffer() -> int:
    """Flush buffered spans to vla_profile.log. Returns the number of flushed records."""
    global _profile_buffer_dropped
    if _profile_buffer_queue is None:
        return 0

    logger = get_profile_logger()
    flushed = 0

    # Fast path: write directly to handler streams (much faster than logger.info per line).
    handlers = list(getattr(logger, "handlers", []) or [])
    stream_handlers = [h for h in handlers if hasattr(h, "stream") and getattr(h, "stream", None) is not None]

    # Drain the queue into a local list first to minimize lock contention.
    payloads = []
    while True:
        try:
            payload = _profile_buffer_queue.get_nowait()
        except Empty:
            break
        except Exception:
            break
        else:
            payloads.append(payload)

    if not payloads:
        return 0

    if stream_handlers:
        for h in stream_handlers:
            try:
                h.acquire()
                for payload in payloads:
                    try:
                        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                        h.stream.write(line + "\n")
                    except Exception:
                        pass
                try:
                    h.flush()
                except Exception:
                    pass
            finally:
                try:
                    h.release()
                except Exception:
                    pass
        flushed = len(payloads)
    else:
        for payload in payloads:
            try:
                line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                logger.info(line)
                flushed += 1
            except Exception:
                pass

    # Emit a meta record if we dropped data (viewer ignores non-span kinds).
    try:
        dropped = int(_profile_buffer_dropped)
        if dropped > 0:
            meta = {
                "kind": "meta",
                "name": "vla_profile_buffer_dropped",
                "dropped": dropped,
                "wall_time_ns": int(time.time_ns()),
                "pid": int(os.getpid()),
            }
            meta_line = json.dumps(meta, ensure_ascii=False, separators=(",", ":"))
            if stream_handlers:
                h0 = stream_handlers[0]
                try:
                    h0.acquire()
                    h0.stream.write(meta_line + "\n")
                    h0.flush()
                finally:
                    h0.release()
            else:
                logger.info(meta_line)
            _profile_buffer_dropped = 0
    except Exception:
        pass

    return flushed


def _ensure_profile_writer_started(logger: logging.Logger) -> None:
    global _profile_queue, _profile_writer_thread, _profile_writer_started
    if _profile_writer_started or not profile_async_enabled():
        return
    with _profile_writer_lock:
        if _profile_writer_started or not profile_async_enabled():
            return
        _profile_queue = Queue(maxsize=200000)

        def _writer() -> None:
            # Write spans in a background thread so span creation doesn't block control loops.
            while True:
                try:
                    line = _profile_queue.get()
                    if line is None:
                        return
                    try:
                        logger.info(line)
                    except Exception:
                        # Drop if logging fails; profiling must never break robot control.
                        pass
                except Exception:
                    # Never die.
                    continue

        t = threading.Thread(target=_writer, name="vla_profile_writer", daemon=True)
        t.start()
        _profile_writer_thread = t
        _profile_writer_started = True

        def _shutdown_writer() -> None:
            # Best-effort flush for short-lived scripts. Robot processes typically run forever.
            try:
                if _profile_queue is not None:
                    try:
                        _profile_queue.put_nowait(None)
                    except Exception:
                        pass
                if _profile_writer_thread is not None:
                    _profile_writer_thread.join(timeout=0.5)
            except Exception:
                pass

        atexit.register(_shutdown_writer)


def get_profile_logger(
    log_dir: Optional[str] = None,
    log_name: str = "vla_profile",
    retain_days: int = 7,
) -> logging.Logger:
    global _profile_logger
    if _profile_logger is not None:
        return _profile_logger

    with _profile_logger_lock:
        if _profile_logger is not None:
            return _profile_logger

        logger = logging.getLogger(log_name)
        logger.setLevel(logging.INFO)
        logger.propagate = False

        # Avoid duplicate handlers if re-imported.
        if logger.hasHandlers():
            _profile_logger = logger
            return logger

        chosen_dir = log_dir or _default_profile_log_dir()
        try:
            os.makedirs(chosen_dir, exist_ok=True)
        except PermissionError:
            chosen_dir = os.path.join(os.getcwd(), "logs")
            os.makedirs(chosen_dir, exist_ok=True)

        log_path = os.path.join(chosen_dir, f"{log_name}.log")
        handler = TimedRotatingFileHandler(
            log_path, when="midnight", interval=1, backupCount=int(retain_days), encoding="utf-8"
        )
        handler.suffix = "_%Y_%m_%d.log"
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(handler)

        _profile_logger = logger
        if profile_buffering_enabled():
            _ensure_profile_buffer()
        else:
            _ensure_profile_writer_started(logger)
        return logger


@dataclass(frozen=True)
class ProfileSpan:
    name: str
    cat: str
    start_ns: int
    dur_ns: int
    pid: int
    tid: int
    thread_name: str
    ok: bool
    src_file: Optional[str] = None
    src_line: Optional[int] = None
    src_func: Optional[str] = None
    exc_type: Optional[str] = None
    exc_msg: Optional[str] = None
    extra: Optional[Dict] = None

    def to_dict(self) -> dict:
        payload = {
            "kind": "span",
            "name": self.name,
            "cat": self.cat,
            "start_ns": int(self.start_ns),
            "dur_ns": int(self.dur_ns),
            "pid": int(self.pid),
            "tid": int(self.tid),
            "thread_name": self.thread_name,
            "ok": bool(self.ok),
            "wall_time_ns": int(time.time_ns()),
        }
        if self.src_file is not None:
            payload["src_file"] = self.src_file
        if self.src_line is not None:
            payload["src_line"] = int(self.src_line)
        if self.src_func is not None:
            payload["src_func"] = self.src_func
        if self.exc_type is not None:
            payload["exc_type"] = self.exc_type
        if self.exc_msg is not None:
            payload["exc_msg"] = self.exc_msg
        if self.extra:
            payload["extra"] = self.extra
        return payload


def _emit_span(span: ProfileSpan) -> None:
    try:
        logger = get_profile_logger()
        payload = span.to_dict()
        if profile_buffering_enabled() and _profile_buffer_queue is not None:
            try:
                _profile_buffer_queue.put_nowait(payload)
                return
            except Exception:
                # Drop if full; keep running.
                global _profile_buffer_dropped
                _profile_buffer_dropped += 1
                return
        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if profile_async_enabled() and _profile_queue is not None:
            try:
                _profile_queue.put_nowait(line)
                return
            except Exception:
                # Fall back to synchronous logging.
                pass
        logger.info(line)
    except Exception:
        # Profiling must never break robot control.
        return


def profile_timeline(
    *,
    name: Optional[str] = None,
    cat: str = "vla",
    enabled: Optional[bool] = None,
    min_duration_ms: float = 0.0,
    extra: Optional[Callable[..., Dict]] = None,
):
    """Decorator to record a timeline span into vla_profile.log (JSONL).

    Enabled by default; use set_profile_enabled(False) to disable at runtime,
    or pass enabled=False at decoration time.
    """

    def decorator(func):
        event_name = name or f"{func.__module__}.{getattr(func, '__qualname__', func.__name__)}"

        @wraps(func)
        def wrapper(*args, **kwargs):
            if enabled is None:
                if not profile_enabled():
                    return func(*args, **kwargs)
            elif not enabled:
                return func(*args, **kwargs)

            start_ns = time.perf_counter_ns()
            ok = True
            exc_type = None
            exc_msg = None
            result = None
            try:
                result = func(*args, **kwargs)
            except Exception as e:
                ok = False
                exc_type = type(e).__name__
                try:
                    exc_msg = str(e)
                except Exception:
                    exc_msg = "<unprintable>"
                raise
            finally:
                dur_ns = time.perf_counter_ns() - start_ns
                if dur_ns >= int(float(min_duration_ms) * 1_000_000.0):
                    extra_payload = None
                    if extra is not None:
                        try:
                            extra_payload = extra(*args, **kwargs)
                        except Exception:
                            extra_payload = {"extra_error": "failed_to_build_extra"}

                    span = ProfileSpan(
                        name=event_name,
                        cat=str(cat),
                        start_ns=int(start_ns),
                        dur_ns=int(dur_ns),
                        pid=os.getpid(),
                        tid=threading.get_ident(),
                        thread_name=threading.current_thread().name,
                        ok=ok,
                        src_file=getattr(getattr(func, "__code__", None), "co_filename", None),
                        src_line=getattr(getattr(func, "__code__", None), "co_firstlineno", None),
                        src_func=getattr(func, "__qualname__", getattr(func, "__name__", None)),
                        exc_type=exc_type,
                        exc_msg=exc_msg,
                        extra=extra_payload,
                    )
                    _emit_span(span)

            return result

        return wrapper

    return decorator


class profile_span(AbstractContextManager):
    """Context manager to record a nested span into vla_profile.log (JSONL)."""

    def __init__(
        self,
        *,
        name: str,
        cat: str = "vla",
        enabled: Optional[bool] = None,
        min_duration_ms: float = 0.0,
        extra: Optional[Dict] = None,
    ) -> None:
        self._enabled = enabled
        self._name = name
        self._cat = cat
        self._min_duration_ns = int(float(min_duration_ms) * 1_000_000.0)
        self._extra = extra
        self._start_ns: Optional[int] = None
        self._src_file: Optional[str] = None
        self._src_line: Optional[int] = None
        self._src_func: Optional[str] = None

    def __enter__(self):
        if self._enabled is None:
            if not profile_enabled():
                return self
        elif not self._enabled:
            return self
        try:
            frame = sys._getframe(1)
            self._src_file = frame.f_code.co_filename
            self._src_line = int(frame.f_lineno)
            self._src_func = frame.f_code.co_name
        except Exception:
            self._src_file = None
            self._src_line = None
            self._src_func = None
        self._start_ns = time.perf_counter_ns()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._start_ns is None:
            return False

        dur_ns = time.perf_counter_ns() - self._start_ns
        if dur_ns < self._min_duration_ns:
            return False

        ok = exc_type is None
        exc_name = None
        exc_msg = None
        if exc_type is not None:
            exc_name = getattr(exc_type, "__name__", str(exc_type))
            try:
                exc_msg = str(exc) if exc is not None else ""
            except Exception:
                exc_msg = "<unprintable>"

        span = ProfileSpan(
            name=self._name,
            cat=str(self._cat),
            start_ns=int(self._start_ns),
            dur_ns=int(dur_ns),
            pid=os.getpid(),
            tid=threading.get_ident(),
            thread_name=threading.current_thread().name,
            ok=ok,
            src_file=self._src_file,
            src_line=self._src_line,
            src_func=self._src_func,
            exc_type=exc_name,
            exc_msg=exc_msg,
            extra=self._extra,
        )
        _emit_span(span)
        return False
