"""Run report: score table and timeline as markdown and a self-contained HTML page."""

import html
import json
import os

from frida_task_sim.timeline import clock, describe

STYLE = """body{font-family:system-ui,sans-serif;margin:24px auto;max-width:1100px;padding:0 16px;background:#fafafa;color:#222}
table{border-collapse:collapse;width:100%}td,th{border-bottom:1px solid #ddd;padding:4px 8px;text-align:left}
.num{text-align:right}.off{color:#999}.pos{color:#1a7f37;font-weight:600}.neg{color:#c62828;font-weight:600}
.speech{color:#0b5cad}.operator{color:#9a6700}.step{color:#999}.bad{color:#c62828}.good{color:#1a7f37}
video{width:100%;background:#000}h1 small{font-weight:400;color:#666}
@media(prefers-color-scheme:dark){body{background:#161616;color:#ddd}td,th{border-color:#333}.speech{color:#7ab8ff}}"""


def _event_class(event: dict) -> str:
    kind = event["type"]
    if kind == "speech":
        return "speech" if event["who"] == "robot" else "operator"
    if kind == "step":
        return "step"
    bad = (
        kind == "knocked"
        or event.get("correct") is False
        or event.get("location") in ("floor", "air")
    )
    return "bad" if bad else "good"


def write(run_dir: str, scenario: dict, score: dict, events: list, status: str):
    """Write report.md and report.html into run_dir."""
    title = f"{scenario['task'].upper()} seed {scenario['seed']}"
    summary = f"{score['total']} / {score['max_in_sim']} reachable in sim ({score['max_rulebook']} on the rulebook sheet)"
    md = [
        f"# {title}",
        "",
        f"**Score: {summary}**",
        "",
        f"Status: {status}",
        "",
        "| Action | Points | Scored | Total | Details |",
        "|---|---:|---:|---:|---|",
    ]
    rows_html = ""
    for row in score["rows"]:
        scored = (
            f"{row['scored']}/{row['count']}" if row["simulated"] else "not simulated"
        )
        details = ", ".join(str(d) for d in row["details"])
        md.append(
            f"| {row['label']} | {row['points']} | {scored} | {row['total']} | {details} |"
        )
        cls = (
            "off"
            if not row["simulated"]
            else ("pos" if row["total"] > 0 else "neg" if row["total"] < 0 else "")
        )
        rows_html += (
            f"<tr class='{cls}'><td>{html.escape(row['label'])}</td><td class='num'>{row['points']}</td>"
            f"<td class='num'>{scored}</td><td class='num'>{row['total']}</td><td>{html.escape(details)}</td></tr>"
        )
    md += ["", "## Timeline", ""]
    timeline_html = ""
    for event in events:
        described = describe(event)
        if not described:
            continue
        md.append(f"- `{clock(event['t'])}` {described[1]}")
        timeline_html += f"<tr class='{_event_class(event)}'><td>{clock(event['t'])}</td><td>{html.escape(described[1])}</td></tr>"
    md += [
        "",
        "## Files",
        "",
        "- `overview.mp4`: top view with clock, score and speech",
        "- `scenario.json`: replay with `run_scenario.py --scenario <this file>`",
        "- `events.jsonl`, `transcript.jsonl`, `final.json`, `tm.log`, `stack_logs/`, `bt/`",
    ]
    with open(os.path.join(run_dir, "report.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    page = f"""<!doctype html><html><head><meta charset="utf-8"><title>{title}</title><style>{STYLE}</style></head><body>
<h1>{title} <small>{html.escape(status)}</small></h1><h2>Score: {summary}</h2>
<video controls src="overview.mp4"></video>
<h2>Scoresheet</h2><table><tr><th>Action</th><th class='num'>Points</th><th class='num'>Scored</th><th class='num'>Total</th><th>Details</th></tr>{rows_html}</table>
<h2>Timeline</h2><table>{timeline_html}</table>
<h2>Scenario</h2><pre>{html.escape(json.dumps(scenario, indent=1))}</pre></body></html>"""
    with open(os.path.join(run_dir, "report.html"), "w") as f:
        f.write(page)
