#!/usr/bin/env python3
"""Capture VLM benchmark samples: a camera frame paired with EXACT ground
truth taken from the AprilTag poses at the same moment.

WHY GROUND TRUTH MATTERS HERE
-----------------------------
Comparing VLMs by reading their answers side by side rewards whichever one
writes the most confident prose. This testbed can do better: every question
worth asking about robot count, position, heading and spatial relations is
something apriltag_localize.py already measures to a fraction of a degree.
So each sample stores the frame AND the measured state, and vlm_bench.py
scores answers against it automatically.

That turns "which model sounds smarter" into "which model is right, how
often, and how fast" -- and it makes a wrong answer detectable even when it
is fluent, which is the failure mode that matters for a control system.

WHAT IT CAPTURES
----------------
  <out>/<sample_id>/frame.jpg   one frame from the MJPEG preview stream
  <out>/<sample_id>/truth.json  every robot's measured x/y/yaw + derived
                                facts (counts, extremes, closest pair)

The frame is the ANNOTATED preview (apriltag_localize.py draws its overlay
in place before pushing to the stream). That is deliberate: the question
being asked is whether a model can reason about the state of the system as
an operator sees it, not whether it can do bare object detection. Isolating
raw perception would need a pre-overlay stream and is a separate experiment.

USAGE (on the Linux laptop, with the vision stack up)
-----
    python3 vlm_capture.py --samples 10 --interval 3 \\
        --stream http://192.168.0.162:8090/stream

apriltag_localize.py must be running with --stream <port>. Nothing is ever
commanded -- this only listens and looks.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import urllib.request
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# Heading convention, copied from camera_grid_navigate.py's module docstring:
#   0 = -y, 90 = +x, 180 = +y, 270 = -x, positive = CCW.
# On this table +y is toward the top of the camera image (node 1 is the
# bottom-left, depot row nearest the viewer), so:
CARDINALS = [(0.0, "south"), (90.0, "east"), (180.0, "north"), (270.0, "west")]


def cardinal(yaw_deg: float) -> str:
    """Nearest cardinal for a measured yaw. Returned as ground truth only
    when the yaw is unambiguous -- see truth_for()."""
    best, best_err = "south", 999.0
    for ref, name in CARDINALS:
        err = abs((yaw_deg - ref + 180.0) % 360.0 - 180.0)
        if err < best_err:
            best, best_err = name, err
    return best


def cardinal_margin(yaw_deg: float) -> float:
    """Degrees away from the nearest cardinal. A robot at 45deg is genuinely
    between two, and asking a model to name one would be scoring a coin
    flip -- truth_for() drops those rather than punishing a fair answer."""
    return min(abs((yaw_deg - ref + 180.0) % 360.0 - 180.0)
               for ref, _ in CARDINALS)


class PoseSnapshot(Node):
    """Listen-only: collects the newest vision pose per robot."""

    def __init__(self, robots: list[str]):
        super().__init__("vlm_capture")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.latest: dict[str, dict] = {}
        self.subs = {}
        for r in robots:
            self.subs[r] = self.create_subscription(
                String, f"{r}_vision_pose", self._cb(r), qos)

    def _cb(self, robot: str):
        def _inner(msg: String) -> None:
            try:
                d = json.loads(msg.data)
                self.latest[robot] = {
                    "x_in": float(d["x_in"]), "y_in": float(d["y_in"]),
                    "yaw_deg": float(d["yaw_deg"]),
                    "received_at": time.monotonic(),
                }
            except (ValueError, KeyError, TypeError):
                return
        return _inner

    def fresh(self, max_age_sec: float) -> dict[str, dict]:
        now = time.monotonic()
        return {r: p for r, p in self.latest.items()
                if now - p["received_at"] <= max_age_sec}


def truth_for(poses: dict[str, dict]) -> dict:
    """Derive every automatically-checkable fact from the measured poses."""
    names = sorted(poses)
    t: dict = {
        "robot_count": len(names),
        "robots": {n: {"x_in": round(poses[n]["x_in"], 2),
                       "y_in": round(poses[n]["y_in"], 2),
                       "yaw_deg": round(poses[n]["yaw_deg"], 2)}
                   for n in names},
        "facing": {},
        "extremes": {},
        "closest_pair": None,
        "north_of": {},
    }
    # Facing, but only where it is not a coin flip.
    for n in names:
        yaw = poses[n]["yaw_deg"]
        if cardinal_margin(yaw) <= 30.0:
            t["facing"][n] = cardinal(yaw)
    if names:
        t["extremes"] = {
            "east": max(names, key=lambda n: poses[n]["x_in"]),
            "west": min(names, key=lambda n: poses[n]["x_in"]),
            "north": max(names, key=lambda n: poses[n]["y_in"]),
            "south": min(names, key=lambda n: poses[n]["y_in"]),
        }
    if len(names) >= 2:
        best, best_d = None, 1e9
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                d = math.hypot(poses[a]["x_in"] - poses[b]["x_in"],
                               poses[a]["y_in"] - poses[b]["y_in"])
                if d < best_d:
                    best, best_d = sorted((a, b)), d
        t["closest_pair"] = {"robots": best, "distance_in": round(best_d, 2)}
        # Pairwise north/south, skipping pairs too close in y to be a fair
        # visual call.
        for i in range(len(names)):
            for j in range(len(names)):
                if i == j:
                    continue
                a, b = names[i], names[j]
                dy = poses[a]["y_in"] - poses[b]["y_in"]
                if abs(dy) >= 6.0:
                    t["north_of"][f"{a}|{b}"] = bool(dy > 0)
    return t


def grab_jpeg(stream_url: str, timeout: float = 10.0) -> bytes:
    """Pull exactly one JPEG out of a multipart MJPEG stream."""
    req = urllib.request.Request(stream_url)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = resp.read(4096)
            if not chunk:
                break
            buf += chunk
            start = buf.find(b"\xff\xd8")          # JPEG SOI
            end = buf.find(b"\xff\xd9", start + 2)  # JPEG EOI
            if start != -1 and end != -1:
                return buf[start:end + 2]
    raise RuntimeError(f"no complete JPEG from {stream_url} in {timeout:.0f}s")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Capture frame+ground-truth samples for the VLM bench.")
    ap.add_argument("--stream", required=True,
                    help="apriltag_localize.py MJPEG URL, e.g. "
                         "http://192.168.0.162:8090/stream")
    ap.add_argument("--robots", nargs="+",
                    default=["Alvik1", "Alvik2", "Alvik3", "Alvik4",
                             "Alvik5", "Alvik6", "tag7"])
    ap.add_argument("--samples", type=int, default=10)
    ap.add_argument("--interval", type=float, default=3.0,
                    help="seconds between samples (default 3)")
    ap.add_argument("--max-pose-age", type=float, default=1.0,
                    help="ignore poses older than this at capture (default 1s)")
    ap.add_argument("--out", default=None,
                    help="output directory (default vlm_samples/ beside this)")
    ap.add_argument("--discovery-timeout", type=float, default=15.0)
    args = ap.parse_args()

    out_root = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vlm_samples")
    os.makedirs(out_root, exist_ok=True)

    rclpy.init()
    node = PoseSnapshot(args.robots)
    try:
        # Two-stage wait, same reason as camera_grid_navigate's
        # wait_for_vision_ready: DDS discovery is not instant and racing it
        # against a data timeout burns the whole budget on discovery.
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.discovery_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.latest:
                break
        if not node.latest:
            raise SystemExit(
                "no vision poses arrived — is apriltag_localize.py running "
                "and seeing tags?")

        captured = 0
        for i in range(args.samples):
            deadline = time.monotonic() + args.interval
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            poses = node.fresh(args.max_pose_age)
            if not poses:
                print(f"  sample {i + 1}: skipped, no fresh poses")
                continue
            try:
                jpeg = grab_jpeg(args.stream)
            except Exception as exc:
                print(f"  sample {i + 1}: skipped, {exc}")
                continue
            sample_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
            d = os.path.join(out_root, sample_id)
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "frame.jpg"), "wb") as fh:
                fh.write(jpeg)
            truth = truth_for(poses)
            truth["sample_id"] = sample_id
            truth["captured_utc"] = datetime.now(timezone.utc).isoformat(
                timespec="seconds")
            with open(os.path.join(d, "truth.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(truth, fh, indent=2, sort_keys=True)
                fh.write("\n")
            captured += 1
            print(f"  sample {i + 1}: {sample_id} — "
                  f"{truth['robot_count']} robot(s), {len(jpeg)} bytes")
        print(f"\ncaptured {captured}/{args.samples} into {out_root}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
