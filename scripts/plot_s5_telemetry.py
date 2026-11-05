#!/usr/bin/env python3
"""S5-09: offline CSV -> self-contained HTML/SVG. Standard library only; no hardware IO."""
import argparse
import csv
import html
import math
from pathlib import Path


def read_joint(path, joint):
    rows = []
    with open(path, encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"tick", "time_us", "joint", "status", "safety_state", "published",
                    "target_deg", "reference_deg", "command_deg", "measured_deg",
                    "feedback_age_us", "feedback_valid", "control_duration_us"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("unexpected telemetry CSV columns")
        for record in reader:
            if int(record["joint"]) != joint:
                continue
            row = {key: float(record[key]) for key in (
                "time_us", "target_deg", "reference_deg", "command_deg",
                "measured_deg", "feedback_age_us", "control_duration_us")}
            row["valid"] = int(record["feedback_valid"]) == 1
            row["published"] = int(record["published"]) == 1
            row["status"] = int(record["status"])
            rows.append(row)
    if not rows:
        raise ValueError(f"no samples for joint {joint}")
    base = rows[0]["time_us"]
    for row in rows:
        row["time_s"] = (row["time_us"] - base) / 1e6
    if any(not math.isfinite(v) for row in rows for v in (
            row["time_s"], row["target_deg"], row["reference_deg"],
            row["command_deg"], row["measured_deg"], row["feedback_age_us"],
            row["control_duration_us"])):
        raise ValueError("nonfinite telemetry values")
    return rows


def panel(rows, series, title, unit, top):
    left, right, width, height = 66, 870, 804, 210
    t0, t1 = rows[0]["time_s"], rows[-1]["time_s"]
    if t1 <= t0:
        t1 = t0 + 0.01
    values = [r[key] for key, _, pred in series for r in rows if pred(r)]
    low, high = (min(values), max(values)) if values else (0.0, 1.0)
    padding = max((high - low) * 0.12, 0.2)
    low, high = low - padding, high + padding

    def x(t):
        return left + width * (t - t0) / (t1 - t0)

    def y(v):
        return top + 35 + height * (high - v) / (high - low)

    parts = [f'<text x="{left}" y="{top + 16}" class="title">{html.escape(title)}</text>',
             f'<rect x="{left}" y="{top+35}" width="{width}" height="{height}" class="plot"/>']
    for i in range(5):
        value = low + (high - low) * i / 4
        yy = y(value)
        parts.append(f'<line x1="{left}" y1="{yy:.1f}" x2="{right}" y2="{yy:.1f}" class="grid"/>')
        parts.append(f'<text x="6" y="{yy+4:.1f}" class="axis">{value:.1f} {html.escape(unit)}</text>')
    for key, color, predicate in series:
        # Split on invalid rows: never draw a continuous line through missing data.
        segment = []
        def flush():
            if len(segment) >= 2:
                pts = " ".join(f"{x(r['time_s']):.1f},{y(r[key]):.1f}" for r in segment)
                parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>')
        for r in rows:
            if predicate(r):
                segment.append(r)
            else:
                flush()
                segment = []
        flush()
    for i in range(5):
        t = t0 + (t1-t0)*i/4
        parts.append(f'<text x="{x(t):.1f}" y="{top+height+60}" text-anchor="middle" class="axis">{t:.2f}s</text>')
    for i, (key, color, _) in enumerate(series):
        parts.append(f'<text x="{left+i*160}" y="{top+height+83}" fill="{color}" class="legend">{html.escape(key)}</text>')
    return "\n".join(parts)


def render(rows, joint):
    published = sum(r["published"] for r in rows)
    valid = sum(r["valid"] for r in rows)
    body = [
        panel(rows, [
            ("target_deg", "#2563eb", lambda r: r["published"]),
            ("reference_deg", "#a855f7", lambda r: r["published"]),
            ("command_deg", "#f59e0b", lambda r: r["published"]),
            ("measured_deg", "#10b981", lambda r: r["valid"]),
        ], "Target / reference / command / measured", "deg", 20),
        panel(rows, [
            ("feedback_age_us", "#0ea5e9", lambda r: r["valid"]),
        ], "Feedback age", "us", 355),
        panel(rows, [
            ("control_duration_us", "#ef4444", lambda r: True),
        ], "Control tick duration", "us", 690),
    ]
    return f"""<!doctype html>
<html lang="en"><meta charset="utf-8"><title>S5-09 joint {joint} telemetry</title>
<style>
body{{font:14px system-ui,sans-serif;background:#0b1220;color:#e5e7eb;max-width:950px;margin:30px auto}}
svg{{width:100%;height:auto;background:#111827;border-radius:12px}}
.title{{fill:#e5e7eb;font-size:17px;font-weight:600}}
.axis{{fill:#9ca3af;font-size:11px}}
.legend{{font-size:12px;font-weight:600}}
.plot{{fill:#111827;stroke:#374151}}
.grid{{stroke:#374151;stroke-dasharray:3 5}}
</style><h1>S5-09 — Joint {joint}</h1>
<p>{len(rows)} ticks · {published} published · {valid} valid feedback samples.
Unpublished commands and invalid feedback are intentionally omitted from the plotted series.</p>
<svg viewBox="0 0 900 1030" xmlns="http://www.w3.org/2000/svg">
{''.join(body)}
</svg></html>"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--joint", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("s5_telemetry.html"))
    args = parser.parse_args()
    if not 0 <= args.joint < 16:
        parser.error("--joint must be 0..15")
    rows = read_joint(args.csv, args.joint)
    args.output.write_text(render(rows, args.joint), encoding="utf-8")
    print(f"wrote {args.output} for joint {args.joint}: {len(rows)} ticks")


if __name__ == "__main__":
    main()
