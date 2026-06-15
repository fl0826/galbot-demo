import argparse
import os
from typing import List

from .render_html import render_html
from .spans import filter_spans_by_named_runs, iter_jsonl, spans_from_records


def main(argv: List[str] = None) -> int:
    parser = argparse.ArgumentParser(description="Visualize vla_profile.log (JSONL) as an interactive HTML timeline.")
    parser.add_argument(
        "--input",
        "-i",
        default=os.environ.get("VLA_PROFILE_LOG", "/userdata/log/galbot_mobile_manipulation/vla_profile.log"),
        help="Path to vla_profile.log (JSONL).",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output base path for auto-named HTML (default: <input>).",
    )
    parser.add_argument(
        "--html",
        nargs="?",
        const="auto",
        default="auto",
        help="Write a self-contained HTML timeline (default: <output>.html).",
    )
    parser.add_argument(
        "--min-dur-ms",
        type=float,
        default=0.0,
        help="Drop spans shorter than this duration (ms) from HTML output.",
    )
    parser.add_argument(
        "--max-threads",
        type=int,
        default=200,
        help="Maximum number of thread lanes to draw in HTML.",
    )
    parser.add_argument(
        "--max-spans",
        type=int,
        default=20000,
        help="Maximum number of spans to draw in HTML (keeps earliest spans).",
    )
    parser.add_argument(
        "--order",
        choices=["asc", "desc"],
        default=None,
        help="Span ordering for rendering/truncation. Defaults to 'desc' when --runs>0 else 'asc'.",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=0,
        help="If >0, only show spans overlapping the last N 'GalbotVLA.run' spans.",
    )
    parser.add_argument(
        "--galbotvla-run-name",
        type=str,
        default="GalbotVLA.run",
        help="Span name used to detect runs (default: GalbotVLA.run).",
    )
    args = parser.parse_args(argv)

    in_path = args.input
    out_base = args.output or in_path

    records = list(iter_jsonl(in_path))
    order = args.order
    if order is None:
        order = "desc" if int(args.runs) > 0 else "asc"
    spans = spans_from_records(records, order=str(order))
    spans = filter_spans_by_named_runs(
        spans, run_name=str(args.galbotvla_run_name), keep_last_n=int(args.runs)
    )

    html_path = (out_base + ".html") if args.html == "auto" else str(args.html)
    n = render_html(
        spans,
        html_path,
        title=os.path.basename(in_path),
        min_dur_ms=float(args.min_dur_ms),
        max_threads=int(args.max_threads),
        max_spans=int(args.max_spans),
    )
    print(f"Wrote HTML ({n} spans) to: {html_path}")

    return 0
