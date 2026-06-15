import json
import os
from typing import Any, Dict, List, Tuple
from xml.sax.saxutils import escape

from .utils import compute_time_bounds, hash_color, span_wall_start_ns


def render_html(
    spans: List[Dict[str, Any]],
    out_path: str,
    *,
    title: str = "VLA Profile Timeline",
    min_dur_ms: float = 0.0,
    max_threads: int = 200,
    max_spans: int = 20000,
) -> int:
    if not spans:
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write("<!doctype html><meta charset='utf-8'><title>Empty</title><body>No spans</body>")
        return 0

    min_dur_ns = int(float(min_dur_ms) * 1_000_000.0)
    if min_dur_ns > 0:
        spans = [s for s in spans if int(s["dur_ns"]) >= min_dur_ns]
    if not spans:
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write("<!doctype html><meta charset='utf-8'><title>Empty</title><body>No spans</body>")
        return 0

    if len(spans) > max_spans:
        spans = spans[:max_spans]

    # Thread ordering by first appearance.
    thread_keys: List[Tuple[int, str]] = []
    thread_seen = set()
    for s in spans:
        key = (int(s.get("tid", 0)), str(s.get("thread_name", "")))
        if key in thread_seen:
            continue
        thread_seen.add(key)
        thread_keys.append(key)
        if len(thread_keys) >= max_threads:
            break

    key_set = set(thread_keys)
    spans = [s for s in spans if (int(s.get("tid", 0)), str(s.get("thread_name", ""))) in key_set]

    bounds = compute_time_bounds(spans)
    if bounds is None:
        return 0
    min_start_ns, max_end_ns = bounds
    total_ms = max((max_end_ns - min_start_ns) / 1_000_000.0, 1e-6)

    wall_starts = [ws for ws in (span_wall_start_ns(s) for s in spans) if ws is not None]
    wall_base_ns = min(wall_starts) if wall_starts else None

    threads_payload = [{"tid": tid, "thread_name": tname} for (tid, tname) in thread_keys]
    lane_index = {k: i for i, k in enumerate(thread_keys)}

    spans_payload = []
    for s in spans:
        key = (int(s.get("tid", 0)), str(s.get("thread_name", "")))
        if key not in lane_index:
            continue
        spans_payload.append(
            {
                "lane": lane_index[key],
                "tid": int(s.get("tid", 0) or 0),
                "thread_name": str(s.get("thread_name", "")),
                "name": s.get("name", "span"),
                "cat": s.get("cat", "vla"),
                "start_ns": int(s["start_ns"]),
                "dur_ns": int(s["dur_ns"]),
                "ok": bool(s.get("ok", True)),
                "src_file": s.get("src_file"),
                "src_line": int(s.get("src_line", 0) or 0),
                "src_func": s.get("src_func"),
            }
        )

    cats = []
    cat_seen = set()
    for s in spans_payload:
        c = str(s.get("cat", "vla"))
        if c in cat_seen:
            continue
        cat_seen.add(c)
        cats.append(c)
        if len(cats) >= 24:
            break

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

    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>{escape(title)}</title>
  <style>
    :root {{
      --bg: #ffffff;
      --fg: #111111;
      --muted: #666666;
      --grid: #eaeaea;
      --border: #dddddd;
      --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
      --sans: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, "Apple Color Emoji", "Segoe UI Emoji";
    }}
    body {{ margin: 0; background: var(--bg); color: var(--fg); font-family: var(--sans); height: 100vh; display: flex; flex-direction: column; }}
    #topbar {{
      position: sticky; top: 0; z-index: 10;
      background: var(--bg); border-bottom: 1px solid var(--border);
      padding: 10px 12px;
      display: grid; gap: 8px;
    }}
    #title {{ font-weight: 600; }}
    #controls {{
      display: flex; flex-wrap: wrap; gap: 10px; align-items: center;
      font-family: var(--mono); font-size: 12px; color: var(--muted);
    }}
    #controls .pill {{
      padding: 4px 8px; border: 1px solid var(--border); border-radius: 999px; background: #fafafa;
    }}
    #status {{
      flex: 0 0 100%;
      white-space: nowrap;
    }}
    #legend {{ display: flex; flex-wrap: wrap; gap: 10px; font-family: var(--mono); font-size: 12px; color: var(--muted); }}
    #legend .item {{ display: inline-flex; gap: 6px; align-items: center; }}
    #legend .swatch {{ width: 12px; height: 12px; border-radius: 2px; border: 1px solid rgba(0,0,0,0.15); }}
    #axisWrap {{ border-bottom: 1px solid var(--border); }}
    #hScrollWrap {{
      overflow-x: auto;
      overflow-y: hidden;
      height: 16px;
      border-bottom: 1px solid var(--border);
      background: #fff;
    }}
    #hScrollContent {{ height: 1px; }}
    #axis {{ display: block; width: 100%; height: 42px; }}
    #main {{ flex: 1; display: flex; flex-direction: column; min-height: 0; }}
    #scrollWrap {{ flex: 1; overflow: auto; min-height: 0; }}
    #timeline {{ display: block; }}
    #splitter {{
      height: 10px;
      background: linear-gradient(to bottom, rgba(0,0,0,0.00), rgba(0,0,0,0.04), rgba(0,0,0,0.00));
      border-top: 1px solid rgba(0,0,0,0.06);
      border-bottom: 1px solid rgba(0,0,0,0.06);
      cursor: row-resize;
      user-select: none;
      touch-action: none;
    }}
    #splitter:hover {{
      background: linear-gradient(to bottom, rgba(30,144,255,0.00), rgba(30,144,255,0.10), rgba(30,144,255,0.00));
    }}
    #statsWrap {{
      border-top: 1px solid var(--border);
      background: #fff;
      height: 280px;
      overflow: auto;
    }}
    #statsBar {{
      position: sticky; top: 0; z-index: 5;
      display: flex; align-items: center; justify-content: space-between;
      gap: 10px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--border);
      background: #fff;
      font-family: var(--mono); font-size: 12px; color: var(--muted);
    }}
    #statsBar .left {{ display: inline-flex; align-items: center; gap: 10px; flex-wrap: wrap; }}
    #statsBar input, #statsBar select {{
      font-family: var(--mono);
      font-size: 12px;
      padding: 6px 8px;
      border: 1px solid var(--border);
      border-radius: 6px;
      background: #fafafa;
      color: var(--fg);
    }}
    #statsTable {{ width: 100%; border-collapse: collapse; font-family: var(--mono); font-size: 12px; }}
    #statsTable th, #statsTable td {{ padding: 7px 10px; border-bottom: 1px solid #f0f0f0; }}
    #statsTable th {{
      position: sticky; top: 44px; z-index: 4;
      text-align: left;
      background: #fbfbfb;
      border-bottom: 1px solid var(--border);
      cursor: pointer;
      user-select: none;
      color: var(--fg);
      font-weight: 600;
    }}
    #statsTable th .arrow {{ color: var(--muted); margin-left: 6px; }}
    #statsTable tr:hover td {{ background: #fcfcff; }}
    .catdot {{ width: 10px; height: 10px; border-radius: 2px; display: inline-block; border: 1px solid rgba(0,0,0,0.15); margin-right: 8px; vertical-align: -1px; }}
    .right {{ text-align: right; }}
    .muted {{ color: var(--muted); }}
    #tooltip {{
      position: fixed; pointer-events: none; z-index: 20;
      background: rgba(20,20,20,0.92); color: white;
      border-radius: 6px; padding: 8px 10px;
      font-family: var(--mono); font-size: 12px; max-width: min(900px, 92vw);
      max-height: 70vh;
      overflow: auto;
      overflow-wrap: anywhere;
      word-break: break-word;
      display: none;
    }}
    #tooltip .k {{ color: #ddd; }}
    #contextMenu {{
      position: fixed; z-index: 30;
      background: rgba(20,20,20,0.96); color: white;
      border-radius: 8px; padding: 10px 10px;
      font-family: var(--mono); font-size: 12px; max-width: min(980px, 92vw);
      max-height: 70vh;
      overflow: auto;
      overflow-wrap: anywhere;
      word-break: break-word;
      display: none;
      box-shadow: 0 10px 30px rgba(0,0,0,0.30);
    }}
    #contextMenu .row {{ margin: 2px 0; }}
    #contextMenu .k {{ color: #ddd; }}
    #contextMenu .actions {{ display: flex; flex-wrap: wrap; gap: 8px; margin-top: 8px; }}
    #contextMenu button {{
      cursor: pointer;
      border: 1px solid rgba(255,255,255,0.18);
      background: rgba(255,255,255,0.08);
      color: white;
      border-radius: 6px;
      padding: 6px 8px;
      font-family: var(--mono);
      font-size: 12px;
    }}
    #contextMenu button:hover {{
      background: rgba(255,255,255,0.14);
    }}
  </style>
</head>
<body>
  <div id="topbar">
    <div id="title">{escape(title)}</div>
	    <div id="controls">
	      <span class="pill">Wheel: zoom</span>
	      <span class="pill">Shift+Wheel / horizontal scroll: pan</span>
	      <span class="pill">Scroll: vertical lanes</span>
	      <span class="pill">Drag: measure</span>
	      <span class="pill">Click lane label: expand</span>
	      <span class="pill">Right click span: open</span>
	      <span id="status" class="pill"></span>
	    </div>
    <div id="legend"></div>
  </div>
	  <div id="main">
	    <div id="axisWrap"><canvas id="axis"></canvas></div>
	    <div id="hScrollWrap"><div id="hScrollContent"></div></div>
	    <div id="scrollWrap"><canvas id="timeline"></canvas></div>
	    <div id="splitter" title="Drag to resize"></div>
	    <div id="statsWrap">
      <div id="statsBar">
        <div class="left">
          <span>Stats</span>
          <select id="statsScope">
            <option value="all">All</option>
            <option value="sel">Selection</option>
          </select>
          <select id="statsTid">
            <option value="all">All threads</option>
          </select>
          <input id="statsFilter" type="text" placeholder="filter (name/cat)" />
        </div>
        <div id="statsSummary" class="muted"></div>
      </div>
      <table id="statsTable">
        <thead>
          <tr>
            <th data-key="name">Name<span class="arrow"></span></th>
            <th data-key="cat">Cat<span class="arrow"></span></th>
            <th class="right" data-key="count">Count<span class="arrow"></span></th>
            <th class="right" data-key="mean_ms">Mean ms<span class="arrow"></span></th>
            <th class="right" data-key="std_ms">Std ms<span class="arrow"></span></th>
            <th class="right" data-key="min_ms">Min ms<span class="arrow"></span></th>
            <th class="right" data-key="max_ms">Max ms<span class="arrow"></span></th>
            <th class="right" data-key="total_ms">Total ms<span class="arrow"></span></th>
          </tr>
        </thead>
        <tbody id="statsBody"></tbody>
      </table>
    </div>
  </div>
  <div id="tooltip"></div>
  <div id="contextMenu"></div>

  <script>
    window.__VLA_THREADS__ = {json.dumps(threads_payload, ensure_ascii=False)};
    window.__VLA_SPANS__ = {json.dumps(spans_payload, ensure_ascii=False)};
    window.__VLA_META__ = {json.dumps({"min_start_ns": min_start_ns, "total_ms": total_ms, "wall_base_ns": wall_base_ns, "cats": cats}, ensure_ascii=False)};
  </script>

  <script>
  (function() {{
    const threads = window.__VLA_THREADS__;
    const spans = window.__VLA_SPANS__;
    const meta = window.__VLA_META__;
    const minStartNs = meta.min_start_ns;
    const wallBaseNs = meta.wall_base_ns;
    const totalMs = meta.total_ms;
    const cats = meta.cats || [];

    const axisCanvas = document.getElementById("axis");
	    const axisCtx = axisCanvas.getContext("2d");
	    const timelineCanvas = document.getElementById("timeline");
	    const ctx = timelineCanvas.getContext("2d");
	    const scrollWrap = document.getElementById("scrollWrap");
	    const hScrollWrap = document.getElementById("hScrollWrap");
	    const hScrollContent = document.getElementById("hScrollContent");
	    const splitter = document.getElementById("splitter");
    const statsWrap = document.getElementById("statsWrap");
    const statsScope = document.getElementById("statsScope");
    const statsTid = document.getElementById("statsTid");
    const statsFilter = document.getElementById("statsFilter");
    const statsBody = document.getElementById("statsBody");
    const statsSummary = document.getElementById("statsSummary");
    const statsTable = document.getElementById("statsTable");
    const tooltip = document.getElementById("tooltip");
    const contextMenu = document.getElementById("contextMenu");
    const status = document.getElementById("status");
    const legend = document.getElementById("legend");

    const css = getComputedStyle(document.documentElement);
    const gridColor = css.getPropertyValue("--grid").trim() || "#eaeaea";
    const fgColor = css.getPropertyValue("--fg").trim() || "#111";
    const mutedColor = css.getPropertyValue("--muted").trim() || "#666";

    const marginLeft = 260;
    const marginTop = 18;
    const laneH = 18;
    const laneGap = 6;
    const barH = 12;
    const rowPitch = barH + 3;
    const lanePadTop = 2;
    const lanePadBottom = 6;
    const axisH = 42;

    function catColor(cat) {{
      const palette = {json.dumps(palette)};
      let h = 0;
      const s = String(cat || "vla");
      for (let i = 0; i < s.length; i++) h = (h * 31 + s.charCodeAt(i)) >>> 0;
      return palette[h % palette.length];
    }}

    function fmtMs(x) {{
      if (!isFinite(x)) return "";
      if (x === 0) return "0";
      if (Math.abs(x) >= 1000) return x.toFixed(1);
      if (Math.abs(x) >= 100) return x.toFixed(2);
      if (Math.abs(x) >= 10) return x.toFixed(3);
      return x.toFixed(4);
    }}

    function fmtWall(ns) {{
      if (!ns) return "";
      const d = new Date(ns / 1e6);
      const pad = (n, w) => String(n).padStart(w, "0");
      return `${{pad(d.getHours(),2)}}:${{pad(d.getMinutes(),2)}}:${{pad(d.getSeconds(),2)}}.${{pad(d.getMilliseconds(),3)}}`;
    }}

    function escHtml(s) {{
      return String(s || "")
        .replace(/&/g, "&amp;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;")
        .replace(/\"/g, "&quot;")
        .replace(/'/g, "&#39;");
    }}

    function hideContextMenu() {{
      contextMenu.style.display = "none";
    }}

    function vscodeUri(file, line) {{
      if (!file) return null;
      const p = String(file);
      const abs = p.startsWith("/") ? p : ("/" + p);
      const ln = (line && Number(line) > 0) ? (":" + Number(line)) : "";
      return "vscode://file" + encodeURI(abs) + ln;
    }}

    function fallbackCopyText(s) {{
      const ta = document.createElement("textarea");
      ta.value = String(s);
      ta.setAttribute("readonly", "");
      ta.style.position = "fixed";
      ta.style.left = "-1000px";
      ta.style.top = "-1000px";
      document.body.appendChild(ta);
      ta.focus();
      ta.select();
      try {{ document.execCommand("copy"); }} catch (e) {{}}
      try {{ document.body.removeChild(ta); }} catch (e2) {{}}
    }}

    async function copyText(text) {{
      const s = String(text);
      try {{
        // `navigator.clipboard` often fails on file:// or without permissions.
        if (navigator.clipboard && (window.isSecureContext || location.protocol === "https:")) {{
          await navigator.clipboard.writeText(s);
          return;
        }}
      }} catch (e) {{}}
      try {{
        fallbackCopyText(s);
        return;
      }} catch (e2) {{}}
      try {{ window.prompt("Copy to clipboard:", s); }} catch (e3) {{}}
    }}

    // Legend
    legend.innerHTML = cats.map(c => {{
      const col = catColor(c);
      return `<span class="item"><span class="swatch" style="background:${{col}}"></span><span>${{c}}</span></span>`;
    }}).join("");

    // Group spans by lane.
    const byLane = Array.from({{length: threads.length}}, () => []);
    for (const s of spans) {{
      if (s.lane >= 0 && s.lane < byLane.length) byLane[s.lane].push(s);
    }}
    for (const arr of byLane) arr.sort((a,b) => a.start_ns - b.start_ns);

    // Stats thread selector.
    if (statsTid) {{
      const seenT = new Set();
      const opts = [];
      for (const t of threads) {{
        const tid = Number(t.tid || 0);
        if (seenT.has(tid)) continue;
        seenT.add(tid);
        const tname = String(t.thread_name || \"\").trim();
        const label = tname ? (tname + \" (tid=\" + tid + \")\") : (\"tid=\" + tid);
        opts.push({{tid, label}});
      }}
      opts.sort((a,b) => a.tid - b.tid);
      for (const o of opts) {{
        const el = document.createElement(\"option\");
        el.value = String(o.tid);
        el.textContent = o.label;
        statsTid.appendChild(el);
      }}
    }}

    // Assign nesting depth per lane (best-effort by containment).
    const laneMaxDepth = new Array(threads.length).fill(0);
    const laneMaxDurNs = new Array(threads.length).fill(0);
    for (let lane = 0; lane < byLane.length; lane++) {{
      const stackEnds = [];
      let maxD = 0;
      let maxDur = 0;
      for (const s of byLane[lane]) {{
        const end = s.start_ns + s.dur_ns;
        while (stackEnds.length && s.start_ns >= stackEnds[stackEnds.length - 1]) stackEnds.pop();
        s.depth = stackEnds.length;
        s.end_ns = end;
        stackEnds.push(end);
        if (s.depth > maxD) maxD = s.depth;
        if (s.dur_ns > maxDur) maxDur = s.dur_ns;
      }}
      laneMaxDepth[lane] = maxD;
      laneMaxDurNs[lane] = maxDur;
    }}

    const expandedLane = new Array(threads.length).fill(false);
    let laneTops = new Array(threads.length).fill(0);
    let laneHeights = new Array(threads.length).fill(laneH);
    let totalHeight = 0;

    function recomputeLaneLayout() {{
      let y = marginTop;
      for (let lane = 0; lane < threads.length; lane++) {{
        const maxD = laneMaxDepth[lane] || 0;
        const expanded = !!expandedLane[lane];
        const h = expanded ? Math.max(laneH, lanePadTop + (maxD + 1) * rowPitch + lanePadBottom) : laneH;
        laneTops[lane] = y;
        laneHeights[lane] = h;
        y += h + laneGap;
      }}
      totalHeight = y + 40;
    }}

		    let scale = 1.0; // px per ms
		    let offsetX = 0.0; // px
		    let suppressHScroll = false;
			    let isSelecting = false;
		    let selectStartNs = null;
		    let selectEndNs = null;
        let selectDragStartX = null;
		    let suppressNextClick = false;
		    let cursorX = null; // canvas-local x (CSS px)
	    let cursorLocked = false;
	    let rafPending = false;

      // Stats
      let statsAll = null;
      let statsSortKey = "total_ms";
      let statsSortDir = "desc";
      let statsDirty = true;
      let statsPending = false;

    function requestRender() {{
      if (rafPending) return;
      rafPending = true;
      requestAnimationFrame(() => {{
        rafPending = false;
        render();
      }});
    }}

    function computeStats(scopeNs0, scopeNs1, tidFilter) {{
      const byKey = new Map();
      const hasScope = (scopeNs0 !== null && scopeNs1 !== null);
      for (const s of spans) {{
        if (tidFilter !== null && Number(s.tid || 0) !== tidFilter) continue;
        const start = s.start_ns;
        const end = s.start_ns + s.dur_ns;
        let durNs = s.dur_ns;
        if (hasScope) {{
          const ov0 = Math.max(start, scopeNs0);
          const ov1 = Math.min(end, scopeNs1);
          if (ov1 <= ov0) continue;
          durNs = ov1 - ov0;
        }}
        const durMs = durNs / 1e6;
        const name = String(s.name || "span");
        const cat = String(s.cat || "vla");
        const key = cat + "::" + name;
        let a = byKey.get(key);
        if (!a) {{
          a = {{
            name,
            cat,
            count: 0,
            total_ms: 0.0,
            min_ms: Infinity,
            max_ms: -Infinity,
            mean_ms: 0.0,
            m2: 0.0,
          }};
          byKey.set(key, a);
        }}
        a.count += 1;
        a.total_ms += durMs;
        if (durMs < a.min_ms) a.min_ms = durMs;
        if (durMs > a.max_ms) a.max_ms = durMs;
        const delta = durMs - a.mean_ms;
        a.mean_ms += delta / a.count;
        a.m2 += delta * (durMs - a.mean_ms);
      }}

      const out = [];
      for (const a of byKey.values()) {{
        const denom = Math.max(a.count - 1, 1);
        const std_ms = Math.sqrt(a.m2 / denom);
        out.push({{
          name: a.name,
          cat: a.cat,
          count: a.count,
          total_ms: a.total_ms,
          mean_ms: a.mean_ms,
          std_ms,
          min_ms: isFinite(a.min_ms) ? a.min_ms : 0.0,
          max_ms: isFinite(a.max_ms) ? a.max_ms : 0.0,
        }});
      }}
      return out;
    }}

    function currentScopeNs() {{
      if (!statsScope) return {{scopeNs0: null, scopeNs1: null, mode: "all"}};
      const mode = String(statsScope.value || "all");
      if (mode !== "sel") return {{scopeNs0: null, scopeNs1: null, mode: "all"}};
      const sel = selectionInfo();
      if (!sel) return {{scopeNs0: null, scopeNs1: null, mode: "sel"}};
      return {{scopeNs0: sel.ns0, scopeNs1: sel.ns1, mode: "sel"}};
    }}

    function compareStats(a, b, key) {{
      if (key === "name" || key === "cat") {{
        const av = String(a[key] || "");
        const bv = String(b[key] || "");
        return av.localeCompare(bv);
      }}
      const av = Number(a[key] || 0);
      const bv = Number(b[key] || 0);
      if (av === bv) return 0;
      return av < bv ? -1 : 1;
    }}

    function updateStatsSortIndicators() {{
      if (!statsTable) return;
      const ths = statsTable.querySelectorAll("th[data-key]");
      ths.forEach(th => {{
        const k = th.getAttribute("data-key");
        const arrow = th.querySelector(".arrow");
        if (!arrow) return;
        if (k === statsSortKey) {{
          arrow.textContent = statsSortDir === "asc" ? " ▲" : " ▼";
        }} else {{
          arrow.textContent = "";
        }}
      }});
    }}

    function renderStatsTable() {{
      if (!statsBody) return;
      const {{scopeNs0, scopeNs1, mode}} = currentScopeNs();
      const tidFilter = (statsTid && String(statsTid.value || \"all\") !== \"all\") ? Number(statsTid.value) : null;

      // Enable/disable selection mode when there is no selection.
      if (statsScope) {{
        const sel = selectionInfo();
        const optSel = statsScope.querySelector('option[value=\"sel\"]');
        if (optSel) optSel.disabled = !sel;
        if (!sel && statsScope.value === "sel") statsScope.value = "all";
      }}

      let rows = null;
      if (mode === "all") {{
        // Cache "all threads" only.
        if (tidFilter === null) {{
          if (!statsAll) statsAll = computeStats(null, null, null);
          rows = statsAll.slice();
        }} else {{
          rows = computeStats(null, null, tidFilter);
        }}
      }} else {{
        if (scopeNs0 === null || scopeNs1 === null) {{
          rows = [];
        }} else {{
          rows = computeStats(scopeNs0, scopeNs1, tidFilter);
        }}
      }}

      const q = statsFilter ? String(statsFilter.value || "").trim().toLowerCase() : "";
      if (q) {{
        rows = rows.filter(r => (r.name.toLowerCase().includes(q) || r.cat.toLowerCase().includes(q)));
      }}

      rows.sort((a,b) => {{
        const c = compareStats(a, b, statsSortKey);
        if (c !== 0) return statsSortDir === "asc" ? c : -c;
        // tiebreak
        const c2 = compareStats(a, b, "total_ms");
        return statsSortDir === "asc" ? c2 : -c2;
      }});

      updateStatsSortIndicators();

      const limit = 4000;
      const shown = Math.min(rows.length, limit);
      let html = "";
      for (let i = 0; i < shown; i++) {{
        const r = rows[i];
        const col = catColor(r.cat);
        html += "<tr>";
        html += `<td><span class=\"catdot\" style=\"background:${{col}}\"></span>${{escHtml(r.name)}}</td>`;
        html += `<td>${{escHtml(r.cat)}}</td>`;
        html += `<td class=\"right\">${{r.count}}</td>`;
        html += `<td class=\"right\">${{fmtMs(r.mean_ms)}}</td>`;
        html += `<td class=\"right\">${{fmtMs(r.std_ms)}}</td>`;
        html += `<td class=\"right\">${{fmtMs(r.min_ms)}}</td>`;
        html += `<td class=\"right\">${{fmtMs(r.max_ms)}}</td>`;
        html += `<td class=\"right\">${{fmtMs(r.total_ms)}}</td>`;
        html += "</tr>";
      }}
      statsBody.innerHTML = html;
      if (statsSummary) {{
        const scopeLabel = mode === "sel" ? "selection" : "all";
        const tidLabel = (tidFilter === null) ? "all" : String(tidFilter);
        const more = rows.length > shown ? `, showing ${{shown}}` : `, showing ${{shown}}`;
        statsSummary.textContent = `blocks=${{rows.length}}${{more}} (scope=${{scopeLabel}}, tid=${{tidLabel}})`;
      }}
    }}

    function requestStatsRender() {{
      statsDirty = true;
      if (statsPending) return;
      statsPending = true;
      setTimeout(() => {{
        statsPending = false;
        if (!statsDirty) return;
        statsDirty = false;
        renderStatsTable();
      }}, 0);
    }}

	    function applyCanvasSizes(resetView) {{
      const dpr = window.devicePixelRatio || 1;
      const w = scrollWrap.clientWidth;
      recomputeLaneLayout();
      const h = totalHeight;

      axisCanvas.style.width = w + "px";
      axisCanvas.style.height = axisH + "px";
      axisCanvas.width = Math.floor(w * dpr);
      axisCanvas.height = Math.floor(axisH * dpr);

      timelineCanvas.style.width = w + "px";
      timelineCanvas.style.height = h + "px";
      timelineCanvas.width = Math.floor(w * dpr);
      timelineCanvas.height = Math.floor(h * dpr);

      axisCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

	      if (resetView) {{
	        const usableW = Math.max(w - marginLeft - 20, 100);
	        scale = usableW / totalMs;
	        offsetX = 0;
	      }}
	      syncHScrollFromState();
	      requestRender();
	    }}

	    function resizeCanvases() {{
	      applyCanvasSizes(true);
	    }}

	    function syncHScrollFromState() {{
	      if (!hScrollWrap || !hScrollContent) return;
	      const w = scrollWrap.clientWidth;
	      const contentW = Math.max(w, Math.ceil(marginLeft + totalMs * scale + 20));
	      hScrollContent.style.width = contentW + "px";
	      const maxScroll = Math.max(0, contentW - w);
	      const desired = Math.min(Math.max(-offsetX, 0), maxScroll);
	      if (hScrollWrap.scrollLeft !== desired) {{
	        suppressHScroll = true;
	        hScrollWrap.scrollLeft = desired;
	        suppressHScroll = false;
	      }}
	      // Clamp offsetX to reachable range.
	      offsetX = -desired;
	    }}

    function xFromNs(ns) {{
      const tMs = (ns - minStartNs) / 1e6;
      return marginLeft + offsetX + tMs * scale;
    }}

    function nsFromX(x) {{
      const tMs = (x - marginLeft - offsetX) / scale;
      return minStartNs + tMs * 1e6;
    }}

    function clampTimelineX(x) {{
      const w = scrollWrap.clientWidth;
      return Math.min(Math.max(x, marginLeft), w - 10);
    }}

    function selectionInfo() {{
      if (selectStartNs === null || selectEndNs === null) return null;
      const ns0 = Math.min(selectStartNs, selectEndNs);
      const ns1 = Math.max(selectStartNs, selectEndNs);
      if (ns1 <= ns0) return null;

      const t0Ms = (ns0 - minStartNs) / 1e6;
      const t1Ms = (ns1 - minStartNs) / 1e6;
      const durMs = Math.abs(t1Ms - t0Ms);
      const startLabel = wallBaseNs ? fmtWall(wallBaseNs + t0Ms * 1e6) : (t0Ms.toFixed(3) + "ms");
      const endLabel = wallBaseNs ? fmtWall(wallBaseNs + t1Ms * 1e6) : (t1Ms.toFixed(3) + "ms");
      const x0 = clampTimelineX(xFromNs(ns0));
      const x1 = clampTimelineX(xFromNs(ns1));
      return {{ ns0, ns1, x0, x1, durMs, startLabel, endLabel }};
    }}

    function laneAtY(yAbs) {{
      // Linear scan is OK at current expected sizes; keep code simple.
      for (let lane = 0; lane < threads.length; lane++) {{
        const top = laneTops[lane];
        const bottom = top + laneHeights[lane];
        if (yAbs >= top && yAbs <= bottom) return lane;
      }}
      return null;
    }}

    function findSpanAt(clientX, clientY) {{
      const rect = timelineCanvas.getBoundingClientRect();
      const x = clientX - rect.left;
      const yAbs = clientY - rect.top + scrollWrap.scrollTop;
      if (x < marginLeft) return null;
      const lane = laneAtY(yAbs);
      if (lane === null) return null;
      const tNs = nsFromX(x);
      const arr = byLane[lane];
      let lo = 0, hi = arr.length - 1, best = -1;
      while (lo <= hi) {{
        const mid = (lo + hi) >> 1;
        if (arr[mid].start_ns <= tNs) {{ best = mid; lo = mid + 1; }} else {{ hi = mid - 1; }}
      }}
      const isExpanded = !!expandedLane[lane];
      const yLane = laneTops[lane];

      // NOTE: we must scan backward potentially far to find long "parent" spans
      // (e.g. get_obs_wholebody_compressed) that started long before the current inner span.
      const maxDur = laneMaxDurNs[lane] || 0;
      const boundStart = tNs - maxDur - 1;
      const maxIter = 5000;
      let iter = 0;
      for (let i = best; i >= 0; i--) {{
        if (iter++ > maxIter) break;
        const s = arr[i];
        if (s.start_ns < boundStart) break;
        if (!isExpanded && (s.depth || 0) > 0) continue;
        const end = s.end_ns || (s.start_ns + s.dur_ns);
        if (tNs < s.start_ns || tNs > end) continue;
        const y0 = isExpanded ? (yLane + lanePadTop + (s.depth || 0) * rowPitch) : (yLane + (laneHeights[lane] - barH) / 2);
        if (yAbs < y0 || yAbs > (y0 + barH)) continue;
        return s;
      }}
      return null;
    }}

    function renderAxis() {{
      const w = scrollWrap.clientWidth;
      axisCtx.clearRect(0, 0, w, axisH);
      axisCtx.fillStyle = "#fff";
      axisCtx.fillRect(0, 0, w, axisH);
      axisCtx.fillStyle = mutedColor;
      axisCtx.font = "12px " + css.getPropertyValue("--mono").trim();

      const tickCount = 10;
      const leftX = marginLeft;
      const rightX = w - 10;
      const minNs = nsFromX(leftX);
      const maxNs = nsFromX(rightX);
      const minMs = (minNs - minStartNs) / 1e6;
      const maxMs = (maxNs - minStartNs) / 1e6;
      const spanMs = Math.max(maxMs - minMs, 1e-6);

      axisCtx.strokeStyle = gridColor;
      axisCtx.beginPath();
      axisCtx.moveTo(marginLeft, axisH - 14);
      axisCtx.lineTo(rightX, axisH - 14);
      axisCtx.stroke();

      for (let i = 0; i <= tickCount; i++) {{
        const tMs = minMs + (spanMs * i / tickCount);
        const x = marginLeft + offsetX + tMs * scale;
        if (x < marginLeft) continue;
        axisCtx.strokeStyle = gridColor;
        axisCtx.beginPath();
        axisCtx.moveTo(x, 0);
        axisCtx.lineTo(x, axisH);
        axisCtx.stroke();

        let label = `${{tMs.toFixed(1)}}ms`;
        if (wallBaseNs) label = fmtWall(wallBaseNs + tMs * 1e6);
        axisCtx.fillStyle = mutedColor;
        axisCtx.textAlign = "center";
        axisCtx.fillText(label, x, axisH - 2);
      }}

      const sel = selectionInfo();
      if (sel) {{
        axisCtx.fillStyle = "rgba(30,144,255,0.10)";
        axisCtx.fillRect(sel.x0, 0, sel.x1 - sel.x0, axisH);
        axisCtx.strokeStyle = "rgba(30,144,255,0.85)";
        axisCtx.lineWidth = 1;
        axisCtx.beginPath();
        axisCtx.moveTo(sel.x0, 0);
        axisCtx.lineTo(sel.x0, axisH);
        axisCtx.moveTo(sel.x1, 0);
        axisCtx.lineTo(sel.x1, axisH);
        axisCtx.stroke();

        const label = sel.durMs.toFixed(3) + "ms";
        axisCtx.font = "12px " + css.getPropertyValue("--mono").trim();
        const pad = 4;
        const textW = axisCtx.measureText(label).width;
        const boxW = textW + pad * 2;
        const boxH = 18;
        const cx = (sel.x0 + sel.x1) / 2;
        const boxX = Math.min(Math.max(cx - boxW / 2, marginLeft), w - boxW - 6);
        const boxY = 22;
        axisCtx.fillStyle = "rgba(30,144,255,0.90)";
        axisCtx.fillRect(boxX, boxY, boxW, boxH);
        axisCtx.fillStyle = "#fff";
        axisCtx.textAlign = "left";
        axisCtx.fillText(label, boxX + pad, boxY + 13);
      }}

      if (cursorX !== null) {{
        const x = cursorX;
        axisCtx.strokeStyle = "#111";
        axisCtx.lineWidth = 1;
        axisCtx.beginPath();
        axisCtx.moveTo(x, 0);
        axisCtx.lineTo(x, axisH);
        axisCtx.stroke();

        const tMs = (x - marginLeft - offsetX) / scale;
        let label = `${{tMs.toFixed(3)}}ms`;
        if (wallBaseNs) label = fmtWall(wallBaseNs + tMs * 1e6);
        axisCtx.font = "12px " + css.getPropertyValue("--mono").trim();
        const pad = 4;
        const textW = axisCtx.measureText(label).width;
        const boxW = textW + pad * 2;
        const boxH = 18;
        const boxX = Math.min(Math.max(x - boxW / 2, marginLeft), w - boxW - 6);
        const boxY = 6;
        axisCtx.fillStyle = "rgba(20,20,20,0.90)";
        axisCtx.fillRect(boxX, boxY, boxW, boxH);
        axisCtx.fillStyle = "#fff";
        axisCtx.textAlign = "left";
        axisCtx.fillText(label, boxX + pad, boxY + 13);
      }}
    }}

    function drawSpanLabel(x0, y0, wPx, text, textColor) {{
      if (wPx < 28) return;
      ctx.save();
      ctx.beginPath();
      ctx.rect(x0 + 1, y0 + 1, Math.max(wPx - 2, 1), barH - 2);
      ctx.clip();
      ctx.font = "11px " + css.getPropertyValue("--mono").trim();
      const maxW = wPx - 6;
      let t = String(text || "");
      if (ctx.measureText(t).width > maxW) {{
        while (t.length > 0 && ctx.measureText(t + "…").width > maxW) t = t.slice(0, -1);
        t = t.length ? (t + "…") : "";
      }}
      ctx.fillStyle = textColor;
      ctx.fillText(t, x0 + 3, y0 + barH - 3);
      ctx.restore();
    }}

    function contrastTextColor(bgHex) {{
      try {{
        const hex = String(bgHex || "").replace("#", "");
        const r = parseInt(hex.substring(0, 2), 16);
        const g = parseInt(hex.substring(2, 4), 16);
        const b = parseInt(hex.substring(4, 6), 16);
        const lum = (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0;
        return lum < 0.55 ? "#fff" : "#111";
      }} catch (e) {{
        return "#111";
      }}
    }}

    function render() {{
      const w = scrollWrap.clientWidth;
      const h = parseFloat(timelineCanvas.style.height);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, w, h);

      const scrollTop = scrollWrap.scrollTop;
      const viewH = scrollWrap.clientHeight;
      let laneStart = 0;
      while (laneStart < threads.length && (laneTops[laneStart] + laneHeights[laneStart]) < (scrollTop - 30)) laneStart++;
      let laneEnd = laneStart;
      while (laneEnd < threads.length && laneTops[laneEnd] < (scrollTop + viewH + 30)) laneEnd++;

      const tickCount = 10;
      const leftX = marginLeft;
      const rightX = w - 10;
      const minNs = nsFromX(leftX);
      const maxNs = nsFromX(rightX);
      const minMs = (minNs - minStartNs) / 1e6;
      const maxMs = (maxNs - minStartNs) / 1e6;
      const spanMs = Math.max(maxMs - minMs, 1e-6);

      ctx.strokeStyle = gridColor;
      for (let i = 0; i <= tickCount; i++) {{
        const tMs = minMs + (spanMs * i / tickCount);
        const x = marginLeft + offsetX + tMs * scale;
        if (x < marginLeft) continue;
        ctx.beginPath();
        ctx.moveTo(x, scrollTop);
        ctx.lineTo(x, scrollTop + viewH);
        ctx.stroke();
      }}

      const sel = selectionInfo();
      if (sel) {{
        ctx.fillStyle = "rgba(30,144,255,0.08)";
        ctx.fillRect(sel.x0, scrollTop, sel.x1 - sel.x0, viewH);
        ctx.strokeStyle = "rgba(30,144,255,0.85)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(sel.x0, scrollTop);
        ctx.lineTo(sel.x0, scrollTop + viewH);
        ctx.moveTo(sel.x1, scrollTop);
        ctx.lineTo(sel.x1, scrollTop + viewH);
        ctx.stroke();
      }}

      if (cursorX !== null) {{
        ctx.strokeStyle = "#111";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(cursorX, scrollTop);
        ctx.lineTo(cursorX, scrollTop + viewH);
        ctx.stroke();
      }}

      // Lane labels.
      ctx.font = "12px " + css.getPropertyValue("--mono").trim();
      for (let lane = laneStart; lane < laneEnd; lane++) {{
        const yLane = laneTops[lane];
        const tid = threads[lane].tid;
        const tname = (threads[lane].thread_name || "").trim();
        const label = tname ? (tname + " (tid=" + tid + ")") : ("tid=" + tid);
        const hasChildren = (laneMaxDepth[lane] || 0) > 0;
        const arrow = hasChildren ? (expandedLane[lane] ? "▾" : "▸") : "";
        ctx.fillStyle = fgColor;
        ctx.textAlign = "left";
        ctx.fillText(arrow, 10, yLane + 14);
        ctx.fillText(label, 26, yLane + 14);

        ctx.strokeStyle = "#f5f5f5";
        ctx.beginPath();
        ctx.moveTo(marginLeft, yLane + laneHeights[lane] + 2);
        ctx.lineTo(w - 10, yLane + laneHeights[lane] + 2);
        ctx.stroke();
      }}

      // Spans.
      for (let lane = laneStart; lane < laneEnd; lane++) {{
        const yLane = laneTops[lane];
        const isExpanded = !!expandedLane[lane];
        for (const s of byLane[lane]) {{
          if (!isExpanded && (s.depth || 0) > 0) continue;
          const x0 = xFromNs(s.start_ns);
          const wPx0 = Math.max((s.dur_ns / 1e6) * scale, 0.8);
          const x1 = x0 + wPx0;
          const x0c = Math.max(x0, marginLeft);
          const x1c = Math.min(x1, rightX);
          const wPx = x1c - x0c;
          if (wPx <= 0) continue;
          const y0 = isExpanded ? (yLane + lanePadTop + (s.depth || 0) * rowPitch) : (yLane + (laneHeights[lane] - barH) / 2);
          const bg = catColor(s.cat || "vla");
          ctx.globalAlpha = 0.85;
          ctx.fillStyle = bg;
          ctx.fillRect(x0c, y0, wPx, barH);

          ctx.globalAlpha = 1.0;
          ctx.lineWidth = 1;
          ctx.strokeStyle = (!s.ok) ? "#000" : "rgba(0,0,0,0.25)";
          ctx.strokeRect(x0c + 0.5, y0 + 0.5, Math.max(wPx - 1, 0.5), barH - 1);

          const tc = contrastTextColor(bg);
          drawSpanLabel(x0c, y0, wPx, s.name, tc);
        }}
      }}
      ctx.globalAlpha = 1.0;

      renderAxis();
      let cursorText = "";
      if (cursorX !== null) {{
        const tMs = (cursorX - marginLeft - offsetX) / scale;
        cursorText = wallBaseNs ? (" cursor=" + fmtWall(wallBaseNs + tMs * 1e6)) : (" cursor=" + tMs.toFixed(3) + "ms");
        if (cursorLocked) cursorText += " (locked)";
      }}
      let selText = "";
      if (sel) {{
        selText = " sel=" + sel.durMs.toFixed(3) + "ms";
      }}
      status.textContent = `lanes=${{threads.length}} spans=${{spans.length}} zoom=${{(scale).toFixed(2)}}px/ms${{cursorText}}${{selText}}`;
    }}

    function showContextMenu(span, clientX, clientY) {{
      const startMs = (span.start_ns - minStartNs) / 1e6;
      const durMs = span.dur_ns / 1e6;
      const startLabel = wallBaseNs ? fmtWall(wallBaseNs + startMs * 1e6) : (startMs.toFixed(3) + "ms");
      const srcFile = span.src_file || "";
      const srcLine = span.src_line || 0;
      const srcFunc = span.src_func || "";
      const uri = vscodeUri(srcFile, srcLine);
      const srcText = (srcFile && srcLine) ? (String(srcFile) + ":" + String(srcLine)) : (srcFile ? String(srcFile) : "");

      let html = "";
      html += `<div class="row"><span class="k">name</span>: ${{escHtml(span.name)}}</div>`;
      html += `<div class="row"><span class="k">cat</span>: ${{escHtml(span.cat)}}</div>`;
      html += `<div class="row"><span class="k">start</span>: ${{escHtml(startLabel)}}</div>`;
      html += `<div class="row"><span class="k">dur</span>: ${{durMs.toFixed(3)}}ms</div>`;
      html += `<div class="row"><span class="k">lane</span>: ${{span.lane}} <span class="k">depth</span>: ${{(span.depth || 0)}}</div>`;
      html += `<div class="row"><span class="k">ok</span>: ${{span.ok}}</div>`;
      if (srcText) html += `<div class="row"><span class="k">src</span>: ${{escHtml(srcText)}}</div>`;
      if (srcFunc) html += `<div class="row"><span class="k">func</span>: ${{escHtml(srcFunc)}}</div>`;
      html += `<div class="actions">`;
      if (uri) html += `<button id="btnOpenVs">Open in VS Code</button>`;
      if (srcText) html += `<button id="btnCopySrc">Copy src</button>`;
      html += `<button id="btnCopyName">Copy name</button>`;
      html += `<button id="btnCopyJson">Copy span JSON</button>`;
      html += `</div>`;

      contextMenu.innerHTML = html;

      contextMenu.style.display = "block";
      const pad = 8;
      const rect = contextMenu.getBoundingClientRect();
      let x = clientX;
      let y = clientY;
      x = Math.min(x, window.innerWidth - rect.width - pad);
      y = Math.min(y, window.innerHeight - rect.height - pad);
      x = Math.max(x, pad);
      y = Math.max(y, pad);
      contextMenu.style.left = x + "px";
      contextMenu.style.top = y + "px";

      const btnOpen = document.getElementById("btnOpenVs");
      if (btnOpen && uri) btnOpen.onclick = () => window.open(uri, "_blank");
      const btnCopySrc = document.getElementById("btnCopySrc");
      if (btnCopySrc && srcText) btnCopySrc.onclick = () => copyText(srcText);
      const btnCopyName = document.getElementById("btnCopyName");
      if (btnCopyName) btnCopyName.onclick = () => copyText(span.name || "");
      const btnCopyJson = document.getElementById("btnCopyJson");
      if (btnCopyJson) btnCopyJson.onclick = () => copyText(JSON.stringify(span));
    }}

    // Interaction: wheel zoom; shift+wheel / trackpad horizontal scroll pans.
    timelineCanvas.addEventListener("wheel", (e) => {{
      const rect = timelineCanvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      if (x < marginLeft) return;
	      if (e.shiftKey || Math.abs(e.deltaX) > Math.abs(e.deltaY)) {{
	        e.preventDefault();
	        const dx = (Math.abs(e.deltaX) > 0) ? e.deltaX : e.deltaY;
	        offsetX -= dx;
	        syncHScrollFromState();
	        requestRender();
	        return;
	      }}

      e.preventDefault();
      const mouseX = x;
      const tMs = (mouseX - marginLeft - offsetX) / scale;
	      const zoom = Math.exp(-e.deltaY * 0.0015);
	      const newScale = Math.min(Math.max(scale * zoom, 0.02), 5000.0);
	      scale = newScale;
	      offsetX = mouseX - marginLeft - tMs * scale;
	      syncHScrollFromState();
	      requestRender();
	    }}, {{ passive: false }});

    // Measure tool: left-drag to select a time window.
    timelineCanvas.addEventListener("mousedown", (e) => {{
      if (e.button !== 0) return;
      const rect = timelineCanvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      if (x < marginLeft) return;
      isSelecting = true;
      const xc = clampTimelineX(x);
      selectDragStartX = xc;
      selectStartNs = nsFromX(xc);
      selectEndNs = selectStartNs;
      suppressNextClick = false;
      tooltip.style.display = "none";
      hideContextMenu();
      requestRender();
    }});
    window.addEventListener("mousemove", (e) => {{
      if (!isSelecting) return;
      const rect = timelineCanvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const xc = clampTimelineX(x);
      selectEndNs = nsFromX(xc);
      if (selectDragStartX !== null && Math.abs(xc - selectDragStartX) > 3) suppressNextClick = true;
      requestRender();
    }});
    window.addEventListener("mouseup", (e) => {{
      if (e.button !== 0) return;
      if (!isSelecting) return;
      isSelecting = false;
      if (selectDragStartX !== null) {{
        const rect = timelineCanvas.getBoundingClientRect();
        const endX = clampTimelineX(e.clientX - rect.left);
        if (Math.abs(endX - selectDragStartX) < 3) {{
          // Treat as a click; clear selection.
          selectStartNs = null;
          selectEndNs = null;
          suppressNextClick = false;
        }}
      }}
      selectDragStartX = null;
      requestRender();
      requestStatsRender();
    }});

    timelineCanvas.addEventListener("mousemove", (e) => {{
      if (isSelecting) return;
      const rect = timelineCanvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      if (!cursorLocked && x >= marginLeft) {{
        cursorX = x;
        requestRender();
      }}
      const s = findSpanAt(e.clientX, e.clientY);
      if (!s) {{
        tooltip.style.display = "none";
        return;
      }}
      const startMs = (s.start_ns - minStartNs) / 1e6;
      const durMs = s.dur_ns / 1e6;
      const startLabel = wallBaseNs ? fmtWall(wallBaseNs + startMs * 1e6) : (startMs.toFixed(3) + "ms");
      const srcText = (s.src_file && s.src_line) ? (String(s.src_file) + ":" + String(s.src_line)) : (s.src_file ? String(s.src_file) : "");

      let th = "";
      th += `<div><span class="k">name</span>: ${{escHtml(s.name)}}</div>`;
      th += `<div><span class="k">cat</span>: ${{escHtml(s.cat)}}</div>`;
      th += `<div><span class="k">start</span>: ${{escHtml(startLabel)}}</div>`;
      th += `<div><span class="k">dur</span>: ${{durMs.toFixed(3)}}ms</div>`;
      th += `<div><span class="k">lane</span>: ${{s.lane}} <span class="k">depth</span>: ${{(s.depth || 0)}}</div>`;
      th += `<div><span class="k">ok</span>: ${{s.ok}}</div>`;
      if (srcText) th += `<div><span class="k">src</span>: ${{escHtml(srcText)}}</div>`;

      tooltip.innerHTML = th;
      tooltip.style.left = (e.clientX + 12) + "px";
      tooltip.style.top = (e.clientY + 12) + "px";
      tooltip.style.display = "block";
    }});

    timelineCanvas.addEventListener("mouseleave", () => {{
      tooltip.style.display = "none";
      if (!cursorLocked) {{
        cursorX = null;
        requestRender();
      }}
    }});

    timelineCanvas.addEventListener("contextmenu", (e) => {{
      const s = findSpanAt(e.clientX, e.clientY);
      if (!s) return;
      e.preventDefault();
      tooltip.style.display = "none";
      showContextMenu(s, e.clientX, e.clientY);
    }});

    timelineCanvas.addEventListener("click", (e) => {{
      if (suppressNextClick) {{
        suppressNextClick = false;
        return;
      }}
      const rect = timelineCanvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const yAbs = e.clientY - rect.top + scrollWrap.scrollTop;
      if (x < marginLeft) {{
        const lane = laneAtY(yAbs);
        if (lane !== null && (laneMaxDepth[lane] || 0) > 0) {{
          expandedLane[lane] = !expandedLane[lane];
          hideContextMenu();
          applyCanvasSizes(false);
        }}
        return;
      }}
      if (!cursorLocked) {{
        cursorLocked = true;
        cursorX = x;
      }} else {{
        cursorLocked = false;
      }}
      hideContextMenu();
      requestRender();
    }});

	    scrollWrap.addEventListener("scroll", () => {{
	      hideContextMenu();
	      requestRender();
	    }});
	    if (hScrollWrap) {{
	      hScrollWrap.addEventListener("scroll", () => {{
	        if (suppressHScroll) return;
	        offsetX = -hScrollWrap.scrollLeft;
	        requestRender();
	      }});
	    }}
	    window.addEventListener("click", (e) => {{
	      if (contextMenu.style.display !== "none" && !contextMenu.contains(e.target)) hideContextMenu();
	    }});
    window.addEventListener("keydown", (e) => {{
      if (e.key === "Escape") {{
        hideContextMenu();
        selectStartNs = null;
        selectEndNs = null;
        selectDragStartX = null;
        suppressNextClick = false;
        requestRender();
        requestStatsRender();
      }}
    }});
    window.addEventListener("resize", () => resizeCanvases());

    // Stats UI hooks.
    if (statsTable) {{
      const ths = statsTable.querySelectorAll(\"th[data-key]\");
      ths.forEach(th => {{
        th.addEventListener(\"click\", () => {{
          const key = th.getAttribute(\"data-key\") || \"total_ms\";
          if (key === statsSortKey) {{
            statsSortDir = (statsSortDir === \"asc\") ? \"desc\" : \"asc\";
          }} else {{
            statsSortKey = key;
            statsSortDir = (key === \"name\" || key === \"cat\") ? \"asc\" : \"desc\";
          }}
          requestStatsRender();
        }});
      }});
    }}
    if (statsScope) statsScope.addEventListener(\"change\", () => requestStatsRender());
    if (statsTid) statsTid.addEventListener(\"change\", () => requestStatsRender());
    if (statsFilter) statsFilter.addEventListener(\"input\", () => requestStatsRender());

    // Resizable split between timeline and stats table.
    (function initSplitter() {{
      if (!splitter || !statsWrap) return;
      const key = \"vla_stats_height_px\";
      const minH = 120;
      const maxHMargin = 160; // keep some room for timeline
      const saved = Number(localStorage.getItem(key) || \"\");
      if (isFinite(saved) && saved > 0) {{
        statsWrap.style.height = saved + \"px\";
      }}

      let dragging = false;
      let startY = 0;
      let startH = 0;

      function clampHeight(h) {{
        const main = document.getElementById(\"main\");
        const mainH = main ? main.clientHeight : window.innerHeight;
        const maxH = Math.max(minH, mainH - maxHMargin);
        return Math.min(Math.max(h, minH), maxH);
      }}

      splitter.addEventListener(\"mousedown\", (e) => {{
        if (e.button !== 0) return;
        dragging = true;
        startY = e.clientY;
        startH = statsWrap.getBoundingClientRect().height;
        document.body.style.cursor = \"row-resize\";
        document.body.style.userSelect = \"none\";
        e.preventDefault();
      }});

      window.addEventListener(\"mousemove\", (e) => {{
        if (!dragging) return;
        const dy = e.clientY - startY;
        const nextH = clampHeight(startH - dy);
        statsWrap.style.height = nextH + \"px\";
        try {{ localStorage.setItem(key, String(Math.round(nextH))); }} catch (e2) {{}}
        applyCanvasSizes(false);
        requestRender();
      }});

      window.addEventListener(\"mouseup\", (e) => {{
        if (!dragging) return;
        dragging = false;
        document.body.style.cursor = \"\";
        document.body.style.userSelect = \"\";
      }});
    }})();

	    resizeCanvases();
	    syncHScrollFromState();
	    requestStatsRender();
	  }})();</script>
</body>
</html>
"""

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return len(spans_payload)
