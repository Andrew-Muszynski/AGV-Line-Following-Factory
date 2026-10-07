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

USAGE (wherever the frame URL resolves, with the vision stack up)
-----
Posed arrangements, one shot per keypress -- the usual case when six robots
are being placed on marked nodes by hand:

    python3 vlm_capture.py --manual --samples 4 --label A-q95 \\
        --rosbridge 192.168.0.212:9090 \\
        --robots Alvik1 Alvik2 Alvik3 Alvik4 Alvik5 Alvik6 \\
        --stream http://127.0.0.1:8081/full.jpg

Timed, for a live run where nobody is at the table:

    python3 vlm_capture.py --samples 10 --interval 3 \\
        --stream http://127.0.0.1:8081/full.jpg

An empty table, for negative examples (no poses exist, so no pose source):

    python3 vlm_capture.py --empty --label empty-table --samples 20 \\
        --stream http://127.0.0.1:8081/full.jpg

PORTS
-----
apriltag_localize.py must be running with --stream. Bare --stream is port
8081 -- its argparse const -- the same port vlm_chat.py and the solver
HTML's Live mode expect. This docstring said 8090 until 2026-09-10 and
nothing has ever served on that port; use 8081.

Prefer /full.jpg over /stream: /stream is downscaled to 960px at q70, which
costs three quarters of the image tokens and blurs the pose HUD the model is
meant to read.

If apriltag_localize runs inside WSL2, its listener lives in WSL's network
namespace and Windows cannot reach it -- run this script inside WSL against
127.0.0.1 rather than hunting for an address that works.

Nothing is ever commanded; this only listens and looks.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import urllib.request
from datetime import datetime, timezone

# rclpy is optional. It exists only on the ROS2 machine, and the poses are
# equally reachable over the rosbridge websocket apriltag_localize.py already
# publishes them through -- see --rosbridge below.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    HAVE_RCLPY = True
except ImportError:                     # pragma: no cover - platform choice
    HAVE_RCLPY = False
    Node = object                       # lets PoseSnapshot below still parse

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

    def pump(self, timeout_sec: float = 0.05) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def close(self) -> None:
        self.destroy_node()
        rclpy.shutdown()


class RosbridgePoseSnapshot:
    """The same listener, over a rosbridge websocket rather than rclpy.

    Identical topics and payload -- /<Name>_vision_pose carrying a
    std_msgs/String of JSON -- because this is the route
    apriltag_localize.py already publishes through. Messages arrive on
    roslibpy's own thread, so pump() only has to yield time.
    """

    def __init__(self, robots: list[str], host: str, port: int):
        import roslibpy
        self.latest: dict[str, dict] = {}
        self._client = roslibpy.Ros(host=host, port=port)
        self._client.run()
        if not self._client.is_connected:
            raise SystemExit(f"cannot reach rosbridge at ws://{host}:{port}")
        print(f"rosbridge: connected to ws://{host}:{port}")
        self._topics = []
        for r in robots:
            topic = roslibpy.Topic(self._client, f"/{r}_vision_pose",
                                   "std_msgs/String")
            topic.subscribe(self._cb(r))
            self._topics.append(topic)

    def _cb(self, robot: str):
        def _inner(msg) -> None:
            try:
                d = json.loads(msg["data"])
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

    def pump(self, timeout_sec: float = 0.05) -> None:
        time.sleep(timeout_sec)

    def close(self) -> None:
        for topic in self._topics:
            try:
                topic.unsubscribe()
            except Exception:
                pass
        self._client.terminate()


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


def jpeg_size(data: bytes) -> tuple[int, int] | None:
    """(width, height) from a JPEG's SOF marker, without decoding it."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3):
            h = int.from_bytes(data[i + 5:i + 7], "big")
            w = int.from_bytes(data[i + 7:i + 9], "big")
            return w, h
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        i += 2 + int.from_bytes(data[i + 2:i + 4], "big")
    return None


_LAST_SHOW_NODES: list = [None]     # X-Show-Nodes from the most recent grab


def grab_jpeg(stream_url: str, timeout: float = 10.0) -> bytes:
    """One JPEG, from either /full.jpg or the multipart /stream.

    /full.jpg answers with a single image/jpeg at the camera's native size
    and q95; /stream is halved to 960px at q70 because it is a monitor view
    for a person. Prefer /full.jpg for captures: the pose HUD is there to be
    read by the model, and at 960/q70 that text is half-size and compressed.

    The marker scan below handles both -- a plain JPEG is simply one frame
    with nothing around it.
    """
    req = urllib.request.Request(stream_url)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        # apriltag_localize states whether the node overlay was drawn on
        # this exact frame. Older builds do not send it, hence the None.
        _hdr = resp.headers.get("X-Show-Nodes")
        _LAST_SHOW_NODES[0] = (None if _hdr is None else _hdr == "1")
        if (resp.headers.get("Content-Type") or "").startswith("image/"):
            return resp.read()
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
                    help="apriltag_localize.py frame URL. Prefer the "
                         "native-resolution still, e.g. "
                         "http://127.0.0.1:8081/full.jpg -- /stream is "
                         "downscaled to 960px at q70 for bandwidth, which "
                         "costs the model three quarters of its image tokens "
                         "and blurs the pose HUD it is meant to read.")
    ap.add_argument("--robots", nargs="+",
                    default=["Alvik1", "Alvik2", "Alvik3", "Alvik4",
                             "Alvik5", "Alvik6", "tag7"])
    ap.add_argument("--samples", type=int, default=10)
    ap.add_argument("--rosbridge", default=None, metavar="HOST[:PORT]",
                    nargs="?", const="192.168.0.212:9090",
                    help="subscribe over a rosbridge websocket instead of "
                         "rclpy, so this runs anywhere without ROS2 "
                         "installed. Bare --rosbridge uses the lab default "
                         "192.168.0.212:9090, matching apriltag_localize.py.")
    ap.add_argument("--manual", action="store_true",
                    help="wait for Enter before each capture instead of "
                         "sleeping --interval. Placing six robots on marked "
                         "nodes and verifying each one does not fit a fixed "
                         "interval. Prints every robot's measured pose and "
                         "derived facing right after each shot, so a "
                         "misplacement is caught at the table instead of in "
                         "backfill_truth an hour later. q finishes early.")
    ap.add_argument("--empty", action="store_true",
                    help="capture an EMPTY table -- no robots, no pose "
                         "source, no rosbridge. Writes robot_count 0 truth, "
                         "which build_questions turns into counting, "
                         "occupancy and control questions whose true answers "
                         "are 0/none/no. Without this the run aborts, because "
                         "an empty table produces no poses and the pose wait "
                         "treats that as a broken vision pipeline. Pair with "
                         "--label empty-table.")
    ap.add_argument("--label", default=None,
                    help="tag every sample from this session, e.g. "
                         "'posed-4robots' or 'run-6robot-dynamics'. Stored in "
                         "truth.json so posed and live-run samples can be "
                         "reported separately later. Folder names cannot "
                         "carry this -- sample dirs are timestamps and all "
                         "sessions share one pool.")
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

    if args.empty:
        # An empty table has no poses, so there is nothing to subscribe to and
        # the two waits below would both fail by design: discovery never sees
        # a pose, and every sample would be skipped as "no fresh poses".
        # truth_for({}) is already correct for this case -- robot_count 0,
        # empty facing/extremes, closest_pair None -- and build_questions
        # generates 7 scoreable questions from it, including the control
        # "Are there exactly 0 tracked robots?" with a true answer of yes.
        #
        # These are the negative examples the benchmark has none of. Measured
        # 2026-09-10, adding the image DROPPED the controls from 100% to 99%
        # and ws:which from 100% to 76%: the picture argues the model out of
        # arithmetic it had right. An empty table is the cleanest possible
        # test of whether a model will assert absence.
        node = None
    elif args.rosbridge:
        host, _, port = args.rosbridge.partition(":")
        node = RosbridgePoseSnapshot(args.robots, host, int(port or 9090))
    elif HAVE_RCLPY:
        rclpy.init()
        node = PoseSnapshot(args.robots)
    else:
        raise SystemExit(
            "rclpy is not installed here, so there is no ROS2 to listen to.\n"
            "Use --rosbridge to subscribe over the same websocket "
            "apriltag_localize.py already publishes through, e.g.\n"
            "    --rosbridge 192.168.0.212:9090\n"
            "or run this on the ROS2 machine instead.")
    try:
        # Two-stage wait, same reason as camera_grid_navigate's
        # wait_for_vision_ready: DDS discovery is not instant and racing it
        # against a data timeout burns the whole budget on discovery.
        if node is not None:
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.discovery_timeout:
                node.pump(0.05)
                if node.latest:
                    break
            if not node.latest:
                raise SystemExit(
                    "no vision poses arrived — is apriltag_localize.py "
                    "running and seeing tags?")

        captured = 0
        # Frame sizes seen in THIS invocation. The node overlay adds ~160
        # circles and labels, which is worth ~85KB on a 1920x1080 q95 frame
        # (measured 2026-09-11: raw 438-444KB, overlay 521-523KB). A capture
        # takes several seconds, so pressing 'o' while one is running splits
        # the invocation across two overlay states -- which happened on the
        # very first block and is invisible in the pose read-back, because
        # the poses are identical either way. Compare and shout.
        sizes: list[tuple[str, int]] = []
        for i in range(args.samples):
            if args.manual:
                # Placing six robots by hand and confirming each one takes as
                # long as it takes; a fixed interval either rushes it or
                # wastes the wait. Enter captures, q stops.
                try:
                    key = input(f"  [{i + 1}/{args.samples}] Enter to "
                                f"capture (q to finish): ").strip().lower()
                except EOFError:
                    break
                if key in ("q", "quit"):
                    print("  stopping early")
                    break
                # Poses go stale while input() blocks. roslibpy delivers on
                # its own thread so latest keeps updating, but rclpy only
                # receives inside spin_once -- so refresh either way before
                # snapshotting, or fresh() rejects everything as too old.
                if node is not None:
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < max(args.max_pose_age, 0.5):
                        node.pump(0.05)
            else:
                deadline = time.monotonic() + args.interval
                while time.monotonic() < deadline:
                    if node is None:
                        time.sleep(0.05)
                    else:
                        node.pump(0.05)
            poses = {} if node is None else node.fresh(args.max_pose_age)
            if not poses and node is not None:
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
            dims = jpeg_size(jpeg)
            if dims and dims[0] < 1280:
                print(f"  NOTE: frame is {dims[0]}x{dims[1]} -- that is the "
                      "downscaled /stream view. Use /full.jpg instead.")
            with open(os.path.join(d, "frame.jpg"), "wb") as fh:
                fh.write(jpeg)
            truth = truth_for(poses)
            # Record what the image ACTUALLY is, rather than trusting whoever
            # ran the capture to label it. One pool now holds 1920x1080 frames
            # from a q75 bridge and from a q95 one, and the 960px archive was
            # split off into its own directory precisely because nothing in
            # the sample said how big it was. Dimensions and byte size are
            # objective and let any later analysis group by them.
            #
            # The BRIDGE quality is not recoverable from the file --
            # apriltag_localize re-encodes /full.jpg at its own quality
            # regardless -- so provenance still needs --label. Byte size is a
            # decent proxy but confounds with scene complexity: an empty table
            # is smaller than six robots at the same settings.
            if dims:
                truth["frame_width"], truth["frame_height"] = dims
            truth["frame_bytes"] = len(jpeg)
            if _LAST_SHOW_NODES[0] is not None:
                truth["overlay"] = _LAST_SHOW_NODES[0]
                # The label is a human promise; the header is what actually
                # happened. When they disagree, say so at the table -- an
                # RCBD treatment label that contradicts its own pixels is
                # the one error this capture flow cannot survive silently.
                lab = (args.label or "").lower()
                claims = ("overlay" if "overlay" in lab
                          else "raw" if "raw" in lab else None)
                if claims is not None:
                    actual = "overlay" if _LAST_SHOW_NODES[0] else "raw"
                    if claims != actual:
                        print(f"      !! LABEL SAYS {claims.upper()} BUT THE "
                              f"FRAME IS {actual.upper()}")
                        print(f"      !! press 'o' in the apriltag window "
                              f"and re-run this label")
            # Room lighting as a MEASURED covariate rather than something
            # anyone has to reproduce by eye. The lab lights are on an
            # occupancy sensor with only up/down buttons, so exact
            # repetition between blocks is not achievable -- but an RCBD
            # absorbs block-level nuisance by design, and recording the
            # number turns "the lighting was similar" into something the
            # methods chapter can demonstrate (balance across treatments)
            # instead of assert. It also catches the sensor cutting the
            # lights mid-session, which no settle delay would.
            #
            # Same central-crop statistics camera_focus_check.py uses, so a
            # value here is directly comparable to what was on screen while
            # the lighting was being tuned.
            try:
                import cv2, numpy as np
                img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                   cv2.IMREAD_COLOR)
                if img is not None:
                    h, w = img.shape[:2]
                    ch, cw = int(h * 0.6), int(w * 0.6)
                    crop = img[(h - ch) // 2:(h + ch) // 2,
                               (w - cw) // 2:(w + cw) // 2]
                    g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                    truth["exposure_mean"] = round(float(g.mean()), 2)
                    truth["clip_low_pct"] = round(
                        float((g <= 2).sum()) * 100.0 / g.size, 3)
                    truth["clip_high_pct"] = round(
                        float((g >= 253).sum()) * 100.0 / g.size, 3)
                    truth["sharpness"] = round(
                        float(cv2.Laplacian(g, cv2.CV_64F).var()), 1)
            except Exception:
                pass          # a missing metric must never lose a capture
            if args.label:
                truth["label"] = args.label
            truth["sample_id"] = sample_id
            truth["captured_utc"] = datetime.now(timezone.utc).isoformat(
                timespec="seconds")
            with open(os.path.join(d, "truth.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(truth, fh, indent=2, sort_keys=True)
                fh.write("\n")
            captured += 1
            sizes.append((sample_id, len(jpeg)))
            if len(sizes) > 1:
                base = sizes[0][1]
                if abs(len(jpeg) - base) > 0.08 * base:
                    print(f"      !! FRAME SIZE JUMPED {base:,} -> "
                          f"{len(jpeg):,} bytes within one capture.")
                    print(f"      !! The overlay was almost certainly "
                          f"toggled mid-capture, so these frames are NOT "
                          f"all in the same state.")
                    print(f"      !! Re-run this label with the overlay "
                          f"left alone, and discard the odd frame.")
            print(f"  sample {i + 1}: {sample_id} — "
                  f"{truth['robot_count']} robot(s), {len(jpeg)} bytes"
                  + (f", {dims[0]}x{dims[1]}" if dims else ""))
            # Always, not just in --manual: the read-back is the only thing
            # that catches a misplaced robot at the table instead of in
            # backfill_truth an hour later, and a posed arrangement is only
            # worth capturing if truth.json agrees with the frame.
            if truth["robot_count"]:
                # Read back what was actually measured. The whole point of a
                # posed arrangement is that truth.json agrees with the frame,
                # and the cheapest place to notice it does not is here.
                for r in sorted(truth["robots"]):
                    p = truth["robots"][r]
                    f = truth["facing"].get(r, "AMBIGUOUS (>30deg off a "
                                               "cardinal, facing dropped)")
                    print(f"      {r:8} x={p['x_in']:6.1f} y={p['y_in']:6.1f} "
                          f"yaw={p['yaw_deg']:6.1f}  {f}")
                miss = [r for r in truth["robots"] if r not in truth["facing"]]
                if miss:
                    print(f"      !! no facing recorded for {', '.join(miss)}"
                          f" -- square them up and recapture")
        print(f"\ncaptured {captured}/{args.samples} into {out_root}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()


if __name__ == "__main__":
    main()
