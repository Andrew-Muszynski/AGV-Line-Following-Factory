#!/usr/bin/env python3
"""
color_sticker_test.py — Evaluate new floor stickers against the Alvik color
classifier, using the <Name>_color topic that AGV_Factory_color_pose.ino
publishes (~150 ms period; JSON r,g,b,h,s,v,color_label).

Requires the robot to run the AGV_Factory_color_pose sketch (v1.1+) and the
rosbridge websocket on the ROS2 laptop (same as everything else).

Workflow:
  1. Static capture — park a robot's color sensor over a surface and record:
       python color_sticker_test.py capture --robot Alvik1 --label new_blue --expect BLUE --seconds 8
     Repeat for each surface: new_red / new_yellow / new_blue, an old sticker
     of each color as control, black tape, and bare white board. If the table
     gets direct sun, capture sunny and shaded variants (e.g. new_blue_sun).
  2. Drive-over test — publishes a real command and reports what happens:
       python color_sticker_test.py drive --robot Alvik1 --command FORWARD_UNTIL_BLUE --label drive_blue --expect BLUE
     (The robot MOVES: keep a hand near it; 'stop' subcommand sends STOP.)
  3. Analysis — thresholds ported from the firmware, with margins:
       python color_sticker_test.py analyze

All samples append to color_test.csv (change with --csv).
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

DEFAULT_HOST = "192.0.2.14"
DEFAULT_PORT = 9090
DEFAULT_CSV = "color_test.csv"
CSV_FIELDS = ["wall_time", "label", "expect", "r", "g", "b", "h", "s", "v",
              "chroma", "onboard_label"]

# ---- classifier thresholds ported from AGV_Factory_color_pose.ino v2 ----
# (keep in sync with the sketch if retuned; the tape-context part of the
# yellow rejection needs line sensors and is evaluated onboard only)
RED_T = {"s": 0.40, "v": 0.04}                     # h > 340 or h < 20
YELLOW_T = {"h_lo": 25.0, "h_hi": 50.0, "s": 0.45, "v": 0.08, "chroma": 0.075}  # v2.1
BLUE_T = {"h_lo": 190.0, "h_hi": 260.0, "s": 0.60, "v": 0.05}


def offline_classify(h, s, v, chroma):
    if (h > 340.0 or h < 20.0) and s > RED_T["s"] and v > RED_T["v"]:
        return "RED"
    if (YELLOW_T["h_lo"] < h < YELLOW_T["h_hi"] and s > YELLOW_T["s"]
            and v > YELLOW_T["v"] and chroma > YELLOW_T["chroma"]):
        return "YELLOW"
    if BLUE_T["h_lo"] < h < BLUE_T["h_hi"] and s > BLUE_T["s"] and v > BLUE_T["v"]:
        return "BLUE"
    return "NONE"


def connect(host, port):
    import roslibpy
    client = roslibpy.Ros(host=host, port=port)
    client.run(timeout=10)
    print(f"connected to ws://{host}:{port}")
    return roslibpy, client


def open_csv(path):
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    if f.tell() == 0:
        writer.writeheader()
    return f, writer


def parse_color_msg(msg):
    try:
        d = json.loads(msg["data"])
        r, g, b = float(d["r"]), float(d["g"]), float(d["b"])
        return {
            "r": r, "g": g, "b": b,
            "h": float(d["h"]), "s": float(d["s"]), "v": float(d["v"]),
            "chroma": max(r, g, b) - min(r, g, b),
            "onboard_label": d.get("color_label", "?"),
        }
    except (ValueError, KeyError, TypeError):
        return None


def cmd_capture(args):
    roslibpy, client = connect(args.host, args.port)
    f, writer = open_csv(args.csv)
    rows = []

    def on_msg(msg):
        d = parse_color_msg(msg)
        if d is None:
            return
        d.update(wall_time=f"{time.time():.3f}", label=args.label, expect=args.expect)
        rows.append(d)
        writer.writerow(d)
        counts = Counter(r["onboard_label"] for r in rows)
        sys.stdout.write(f"\r{len(rows):4d} samples  onboard={dict(counts)}   ")
        sys.stdout.flush()

    topic = roslibpy.Topic(client, f"/{args.robot}_color", "std_msgs/String")
    topic.subscribe(on_msg)
    print(f"capturing '{args.label}' from /{args.robot}_color for {args.seconds:.0f}s "
          f"(expect {args.expect}) — keep the sensor over the surface...")
    time.sleep(args.seconds)
    topic.unsubscribe()
    client.terminate()
    f.close()
    print(f"\nsaved {len(rows)} samples to {args.csv}")
    if rows:
        summarize_label(args.label, args.expect, rows)


def cmd_drive(args):
    roslibpy, client = connect(args.host, args.port)
    f, writer = open_csv(args.csv)
    rows = []
    statuses = []
    t0 = time.time()

    def on_color(msg):
        d = parse_color_msg(msg)
        if d is None:
            return
        d.update(wall_time=f"{time.time():.3f}", label=args.label, expect=args.expect)
        rows.append(d)
        writer.writerow(d)

    def on_status(msg):
        text = str(msg.get("data", "")).strip()
        statuses.append((time.time() - t0, text))
        print(f"  t={time.time() - t0:6.2f}s  status: {text}")

    color_t = roslibpy.Topic(client, f"/{args.robot}_color", "std_msgs/String")
    status_t = roslibpy.Topic(client, f"/{args.robot}_status", "std_msgs/String")
    cmd_t = roslibpy.Topic(client, f"/{args.robot}_cmd", "std_msgs/String")
    color_t.subscribe(on_color)
    status_t.subscribe(on_status)
    cmd_t.advertise()
    time.sleep(0.5)

    print(f"sending {args.command} to {args.robot} — ROBOT WILL MOVE (Ctrl+C + "
          f"'stop' subcommand to abort)")
    cmd_t.publish(roslibpy.Message({"data": args.command}))
    deadline = time.time() + args.timeout
    done = False
    try:
        while time.time() < deadline and not done:
            time.sleep(0.1)
            done = any(s.startswith(("DETECTED", "ERROR")) or s == "IDLE"
                       for _, s in statuses[1:])  # skip echo of first IDLE
    except KeyboardInterrupt:
        cmd_t.publish(roslibpy.Message({"data": "STOP"}))
        print("\nSTOP sent")
    if not done:
        print(f"no completion within {args.timeout:.0f}s — sending STOP")
        cmd_t.publish(roslibpy.Message({"data": "STOP"}))
    color_t.unsubscribe()
    status_t.unsubscribe()
    cmd_t.unadvertise()
    client.terminate()
    f.close()
    detections = [(t, s) for t, s in statuses if s.startswith("DETECTED")]
    print(f"\nrun '{args.label}': {len(rows)} color samples, "
          f"detections: {detections or 'NONE'}")


def cmd_stop(args):
    roslibpy, client = connect(args.host, args.port)
    t = roslibpy.Topic(client, f"/{args.robot}_cmd", "std_msgs/String")
    t.advertise()
    time.sleep(0.3)
    t.publish(roslibpy.Message({"data": "STOP"}))
    time.sleep(0.3)
    t.unadvertise()
    client.terminate()
    print(f"STOP sent to {args.robot}")


def fmt_stats(vals):
    if not vals:
        return "-"
    med = statistics.median(vals)
    lo = sorted(vals)[max(0, int(len(vals) * 0.1))]
    hi = sorted(vals)[min(len(vals) - 1, int(len(vals) * 0.9))]
    return f"{med:6.3f} [{lo:6.3f},{hi:6.3f}]"


def summarize_label(label, expect, rows):
    n = len(rows)
    onboard = Counter(r["onboard_label"] for r in rows)
    offline = Counter(offline_classify(r["h"], r["s"], r["v"], r["chroma"]) for r in rows)
    print(f"\n=== {label}  (n={n}, expect {expect}) ===")
    print(f"  onboard classifier : " + ", ".join(
        f"{k}={100 * c / n:.0f}%" for k, c in onboard.most_common()))
    print(f"  offline thresholds : " + ", ".join(
        f"{k}={100 * c / n:.0f}%" for k, c in offline.most_common()))
    for field in ("h", "s", "v", "chroma"):
        print(f"  {field:6s}: {fmt_stats([r[field] for r in rows])}")

    if expect in ("RED", "YELLOW", "BLUE"):
        hit = 100 * onboard.get(expect, 0) / n
        verdict = "PASS" if hit >= 95 else ("MARGINAL" if hit >= 80 else "FAIL")
        print(f"  onboard hit rate for {expect}: {hit:.0f}%  -> {verdict}")
        s_vals = sorted(r["s"] for r in rows)
        v_vals = sorted(r["v"] for r in rows)
        s10 = s_vals[max(0, int(n * 0.1))]
        v10 = v_vals[max(0, int(n * 0.1))]
        t = {"RED": RED_T, "YELLOW": YELLOW_T, "BLUE": BLUE_T}[expect]
        print(f"  margin: s p10={s10:.3f} vs cutoff {t['s']:.2f}  "
              f"({'OK' if s10 > t['s'] * 1.15 else 'TIGHT — glare risk'})")
        print(f"  margin: v p10={v10:.3f} vs cutoff {t['v']:.2f}  "
              f"({'OK' if v10 > t['v'] * 1.5 else 'TIGHT'})")


def cmd_analyze(args):
    path = Path(args.csv)
    if not path.exists():
        raise SystemExit(f"{args.csv} not found — run some captures first")
    by_label = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for k in ("r", "g", "b", "h", "s", "v", "chroma"):
                row[k] = float(row[k])
            by_label.setdefault(row["label"], []).append(row)
    for label, rows in by_label.items():
        summarize_label(label, rows[0]["expect"], rows)
    # cross-talk check: no surface should ever read as a DIFFERENT color
    print("\n=== cross-talk (misreads as a wrong color are collision fuel) ===")
    clean = True
    for label, rows in by_label.items():
        expect = rows[0]["expect"]
        bad = Counter(r["onboard_label"] for r in rows
                      if r["onboard_label"] not in (expect, "NONE", "?"))
        if bad:
            clean = False
            print(f"  {label}: misread as {dict(bad)} of {len(rows)} samples")
    if clean:
        print("  none — no surface misreads as a wrong color")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--csv", default=DEFAULT_CSV)
    sub = p.add_subparsers(dest="mode", required=True)

    c = sub.add_parser("capture", help="record static readings over a surface")
    c.add_argument("--robot", required=True)
    c.add_argument("--label", required=True, help="e.g. new_blue, old_red, tape, board")
    c.add_argument("--expect", default="NONE",
                   choices=["RED", "YELLOW", "BLUE", "NONE"])
    c.add_argument("--seconds", type=float, default=8.0)
    c.set_defaults(fn=cmd_capture)

    d = sub.add_parser("drive", help="send a real command and log the run")
    d.add_argument("--robot", required=True)
    d.add_argument("--command", required=True,
                   help="e.g. FORWARD_UNTIL_BLUE (robot will move!)")
    d.add_argument("--label", required=True)
    d.add_argument("--expect", default="NONE",
                   choices=["RED", "YELLOW", "BLUE", "NONE"])
    d.add_argument("--timeout", type=float, default=20.0)
    d.set_defaults(fn=cmd_drive)

    s = sub.add_parser("stop", help="publish STOP to a robot")
    s.add_argument("--robot", required=True)
    s.set_defaults(fn=cmd_stop)

    a = sub.add_parser("analyze", help="summarize all captured runs")
    a.set_defaults(fn=cmd_analyze)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
