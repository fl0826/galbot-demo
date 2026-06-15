import hashlib
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple


def hash_color(key: str) -> str:
    palette = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
        "#bcbd22",
        "#17becf",
        "#4c78a8",
        "#f58518",
        "#54a24b",
        "#e45756",
        "#b279a2",
        "#ff9da6",
        "#9d755d",
        "#bab0ac",
    ]
    if not palette:
        return "#888888"
    digest = hashlib.md5(key.encode("utf-8", errors="ignore")).digest()
    idx = int.from_bytes(digest[:4], byteorder="little", signed=False) % len(palette)
    return palette[idx]


def format_wall_time_ns(ns: int) -> str:
    try:
        dt = datetime.fromtimestamp(ns / 1_000_000_000.0)
        return dt.strftime("%H:%M:%S.") + f"{int(dt.microsecond / 1000):03d}"
    except Exception:
        return str(ns)


def span_wall_start_ns(span: Dict[str, Any]) -> Optional[int]:
    wall_end = int(span.get("wall_time_ns", 0) or 0)
    if wall_end <= 0:
        return None
    try:
        return wall_end - int(span["dur_ns"])
    except Exception:
        return None


def compute_time_bounds(spans: List[Dict[str, Any]]) -> Optional[Tuple[int, int]]:
    if not spans:
        return None
    min_start = min(int(s["start_ns"]) for s in spans)
    max_end = max(int(s["start_ns"]) + int(s["dur_ns"]) for s in spans)
    if max_end < min_start:
        max_end = min_start
    return min_start, max_end

