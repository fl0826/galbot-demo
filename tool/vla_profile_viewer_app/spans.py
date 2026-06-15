import json
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def iter_jsonl(path: str) -> Iterable[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue
            if isinstance(obj, dict):
                yield obj


def spans_from_records(records: List[Dict[str, Any]], *, order: str = "asc") -> List[Dict[str, Any]]:
    spans = [r for r in records if isinstance(r, dict) and r.get("kind") == "span"]
    cleaned: List[Dict[str, Any]] = []
    for s in spans:
        try:
            start_ns = int(s["start_ns"])
            dur_ns = int(s["dur_ns"])
        except Exception:
            continue
        if dur_ns < 0:
            continue

        cleaned.append(
            {
                "name": str(s.get("name", "span")),
                "cat": str(s.get("cat", "vla")),
                "start_ns": start_ns,
                "dur_ns": dur_ns,
                "wall_time_ns": int(s.get("wall_time_ns", 0) or 0),
                "src_file": s.get("src_file"),
                "src_line": int(s.get("src_line", 0) or 0),
                "src_func": s.get("src_func"),
                "pid": int(s.get("pid", 0) or 0),
                "tid": int(s.get("tid", 0) or 0),
                "thread_name": str(s.get("thread_name", "")),
                "ok": bool(s.get("ok", True)),
                "exc_type": s.get("exc_type"),
                "exc_msg": s.get("exc_msg"),
                "extra": s.get("extra"),
            }
        )
    reverse = str(order).strip().lower() == "desc"
    cleaned.sort(key=lambda x: (x["start_ns"], x["dur_ns"]), reverse=reverse)
    return cleaned


def filter_spans_by_named_runs(
    spans: Sequence[Dict[str, Any]],
    *,
    run_name: str = "GalbotVLA.run",
    keep_last_n: int = 0,
) -> List[Dict[str, Any]]:
    """Keep only spans overlapping the last N occurrences of a 'run' span.

    Intended for cutting a long vla_profile.log down to the most recent N runs of
    a top-level timeline span like 'GalbotVLA.run'.
    """
    if keep_last_n <= 0:
        return list(spans)

    def _span_end_ns(s: Dict[str, Any]) -> Optional[int]:
        try:
            return int(s["start_ns"]) + int(s["dur_ns"])
        except Exception:
            return None

    # Prefer exact match, but fall back to suffix match for older naming conventions.
    run_spans = [s for s in spans if str(s.get("name", "")) == run_name]
    if not run_spans:
        run_spans = [s for s in spans if str(s.get("name", "")).endswith(run_name)]
    if not run_spans:
        return list(spans)

    run_spans_sorted = sorted(run_spans, key=lambda s: int(s.get("start_ns", 0) or 0))
    selected = run_spans_sorted[-keep_last_n:]

    windows: List[Tuple[int, int]] = []
    for r in selected:
        try:
            rs = int(r["start_ns"])
            re = _span_end_ns(r)
        except Exception:
            continue
        if re is None:
            continue
        windows.append((rs, re))
    if not windows:
        return list(spans)

    def _overlaps_any_window(s: Dict[str, Any]) -> bool:
        try:
            ss = int(s["start_ns"])
            se = _span_end_ns(s)
        except Exception:
            return False
        if se is None:
            return False
        for ws, we in windows:
            if (ss <= we) and (se >= ws):
                return True
        return False

    return [s for s in spans if _overlaps_any_window(s)]
