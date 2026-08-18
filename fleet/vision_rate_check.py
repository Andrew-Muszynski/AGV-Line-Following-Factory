#!/usr/bin/env python3
"""Standalone vision throughput diagnostic -- subscribes to one or more
robots' real /<Name>_vision_pose topics over rosbridge (same route
camera_grid_navigate.py uses) and reports actual inter-message timing over a
run window per robot: mean/min/max gap, effective Hz, and how many gaps
exceeded a given threshold.

Built 2026-08-03 after two "vision lost" mid-turn failures on the real
Linux-laptop-hosted stack, despite the live camera preview never flickering --
camera_grid_navigate.py's fresh_pose()/yaw-jump-reject logic could report
"lost" purely from a publish-side timing gap even while detection itself
never stopped, so the actual question is "what is the REAL delivered rate to
this topic," not "does the camera look fine." Deliberately separate from
apriltag_localize.py's own --print-interval output (which is for tuning the
capture/detect/publish pipeline itself) -- this measures what a CONSUMER
actually receives, at the exact point where the "vision lost" decision is
made, over rosbridge, not the publisher's own internal accounting.

Multi-robot support added same day: a single robot's healthy rate doesn't
prove the pipeline holds up under full table load (6 robots = 6x the
AprilTag detection work per frame) -- run with --robot repeated, or
--all-robots for the standard Alvik1-6 roster, to see whether per-robot rate
degrades as more tags are being tracked simultaneously.

Usage:
    python3 vision_rate_check.py --robot Alvik1 --rosbridge 192.168.0.212:9090 --duration 20
    python3 vision_rate_check.py --robot Alvik1 --robot Alvik2 --rosbridge 192.168.0.212:9090
    python3 vision_rate_check.py --all-robots --rosbridge 192.168.0.212:9090 --duration 30
"""
import argparse
import json
import sys
import time

ALL_ROBOTS = ["Alvik1", "Alvik2", "Alvik3", "Alvik4", "Alvik5", "Alvik6"]


class RobotTracker:
    def __init__(self, name: str, t_start: float):
        self.name = name
        self.t_start = t_start
        self.gaps: list[tuple[float, float]] = []  # (elapsed_since_start_sec, gap_ms)
        self.seq_gaps_dropped = 0
        self.last_t: float | None = None
        self.last_seq: int | None = None
        self.count = 0

    def on_message(self, msg: dict) -> None:
        now = time.monotonic()
        try:
            d = json.loads(msg["data"])
        except (KeyError, ValueError, TypeError):
            return
        self.count += 1
        if self.last_t is not None:
            self.gaps.append((now - self.t_start, (now - self.last_t) * 1000.0))
        self.last_t = now
        seq = d.get("seq")
        if seq is not None and self.last_seq is not None and seq > self.last_seq + 1:
            self.seq_gaps_dropped += (seq - self.last_seq - 1)
        if seq is not None:
            self.last_seq = seq

    def report(self, elapsed: float, gap_warn_ms: float) -> None:
        topic_name = f"/{self.name}_vision_pose"
        if self.count == 0:
            print(f"\n{topic_name}: NO messages received in {elapsed:.1f}s -- "
                  "is this robot's tag visible and is apriltag_localize.py "
                  "actually publishing this topic?")
            return
        print(f"\n{topic_name}: {self.count} messages in {elapsed:.1f}s "
              f"= {self.count / elapsed:.1f} msg/s (rosbridge-delivered rate)")
        if self.gaps:
            gap_vals = sorted(g for _, g in self.gaps)
            n = len(gap_vals)
            mean_gap = sum(gap_vals) / n
            print(f"  inter-message gap: mean={mean_gap:.1f}ms "
                  f"min={gap_vals[0]:.1f}ms max={gap_vals[-1]:.1f}ms "
                  f"median={gap_vals[n // 2]:.1f}ms")
            warn = [(t, g) for t, g in self.gaps if g > gap_warn_ms]
            if warn:
                worst = sorted(warn, key=lambda tg: tg[1], reverse=True)[:5]
                print(f"  {len(warn)}/{n} gaps exceeded {gap_warn_ms:.0f}ms "
                      "-- worst (at t=+Ns since run start): "
                      + ", ".join(f"{g:.0f}ms@t+{t:.1f}s" for t, g in worst))
            else:
                print(f"  no gaps exceeded {gap_warn_ms:.0f}ms")
        if self.seq_gaps_dropped:
            print(f"  sequence numbers show {self.seq_gaps_dropped} apparently-"
                  "dropped frame(s) between publish and this subscriber "
                  "(rosbridge/network-side loss, not a detection failure)")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", action="append", default=[],
                     help="robot name, e.g. Alvik1 -- repeat for multiple "
                          "robots. Subscribes to /<robot>_vision_pose for each.")
    ap.add_argument("--all-robots", action="store_true",
                     help=f"shorthand for --robot on all of {', '.join(ALL_ROBOTS)}")
    ap.add_argument("--rosbridge", required=True, metavar="HOST:PORT",
                     help="rosbridge websocket host:port, e.g. 192.168.0.212:9090")
    ap.add_argument("--duration", type=float, default=20.0,
                     help="seconds to collect data before reporting (default 20)")
    ap.add_argument("--gap-warn-ms", type=float, default=100.0,
                     help="flag any single inter-message gap above this many "
                          "ms (default 100 -- roughly 2x a 60Hz publish "
                          "period's worth of slack)")
    args = ap.parse_args()

    robots = list(dict.fromkeys(args.robot + (ALL_ROBOTS if args.all_robots else [])))
    if not robots:
        ap.error("pass --robot NAME (repeatable) or --all-robots")

    try:
        import roslibpy
    except ImportError:
        print("roslibpy not installed -- pip install roslibpy", file=sys.stderr)
        raise SystemExit(1)

    host, port_str = args.rosbridge.rsplit(":", 1)
    port = int(port_str)

    client = roslibpy.Ros(host=host, port=port)
    try:
        client.run(timeout=5)
    except Exception as exc:
        print(f"could not connect to ws://{host}:{port}: {exc}", file=sys.stderr)
        raise SystemExit(1)
    print(f"connected to ws://{host}:{port}, subscribing to "
          f"{len(robots)} robot(s) ({', '.join(robots)}) for {args.duration:.0f}s...")

    t_start = time.monotonic()
    trackers = {name: RobotTracker(name, t_start) for name in robots}
    topics = []
    for name, tracker in trackers.items():
        topic = roslibpy.Topic(client, f"/{name}_vision_pose", "std_msgs/String")
        topic.subscribe(tracker.on_message)
        topics.append(topic)

    try:
        while time.monotonic() - t_start < args.duration:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        for topic in topics:
            topic.unsubscribe()
        client.terminate()

    elapsed = time.monotonic() - t_start
    for name in robots:
        trackers[name].report(elapsed, args.gap_warn_ms)

    heard = [name for name in robots if trackers[name].count > 0]
    if len(heard) > 1:
        combined_rate = sum(trackers[n].count for n in heard) / elapsed
        print(f"\ncombined: {len(heard)}/{len(robots)} robots publishing, "
              f"{combined_rate:.1f} msg/s total across all of them")


if __name__ == "__main__":
    main()
