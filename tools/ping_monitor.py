#!/usr/bin/env python3
"""Continuous ICMP probe of every robot, timestamped to line up with
supervisor_live.log, to settle what is causing ERROR WHEEL_CMD_TIMEOUT
gap=1001ms.

WHY THIS EXISTS (2026-09-29)
----------------------------
The firmware aborts a route when it spends WHEEL_CMD_ABORT_MS=1000 in
STATE_WHEEL_FOLLOW without a wheel setpoint arriving. Across 10 logged
occurrences the evidence says the loss is in the transport, not at either
end:

  * the navigator was publishing 0-120ms before the abort
  * the firmware loop was healthy -- it catches the 1000ms crossing within
    1ms, so it was ticking, just receiving nothing
  * 596 completed turns show a worst control-loop gap of 85ms, zero above
    300ms -- the Python side never paused long enough to explain it

Three hypotheses were eliminated by experiment: relay bandwidth on the WSL2
virtual NIC (moved ~16MB/s off it, no change), CPU starvation from AprilTag
detection (11-25 percent utilisation across the whole --threads sweep), and
a navigator process stall (the 85ms ceiling above).

What survives is that the losses are SIMULTANEOUS ACROSS ROBOTS -- five
within 28.8ms, two within 8.0ms. Independent robots do not drop packets in
lockstep unless they share whatever failed, and they share exactly two
things: the micro-ROS agent, and the WiFi link.

THIS SCRIPT DISCRIMINATES BETWEEN THOSE TWO. ICMP does not traverse the
agent at all -- it is handled by each robot's network stack. So:

  * ping loss coincides with the aborts  -> RF / AP problem. The agent is
    innocent, and this also explains why moving vision off the VM changed
    nothing: the radio link was never involved.
  * pings stay clean while wheel commands vanish -> the loss is above the
    IP layer, i.e. the micro-ROS agent or the XRCE-DDS session, and moving
    the agent back to the Linux laptop becomes a fix rather than a guess.

Run it in its own terminal alongside a normal fleet run; it changes nothing
about the stack. Prefer WSL2, so probes leave by the same interface the
agent uses.

  python3 tools/ping_monitor.py monitor Alvik1=192.168.0.x Alvik2=... \
      --out ~/ping_run.jsonl

Then, after the run:

  python3 tools/ping_monitor.py correlate ~/ping_run.jsonl ~/supervisor_live.log

Timestamps are time.time() (Unix epoch), which is exactly what the ROS
logger prints in brackets, so the two logs cross-reference directly with no
clock conversion.

Unprivileged ICMP needs the ping group range opened (most distros already
do; WSL2 Ubuntu sometimes does not):

  sudo sysctl -w net.ipv4.ping_group_range="0 2147483647"

Otherwise run this one script with sudo -- it does nothing else privileged.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import select
import socket
import struct
import sys
import time

ICMP_ECHO_REQUEST = 8
ICMP_ECHO_REPLY = 0


def _checksum(data: bytes) -> int:
    """Standard RFC 1071 one's-complement sum.

    Computed even on SOCK_DGRAM sockets, where the kernel would fill it in
    anyway -- costs nothing and keeps the SOCK_RAW fallback correct.
    """
    if len(data) % 2:
        data += b"\x00"
    s = 0
    for i in range(0, len(data), 2):
        s += (data[i] << 8) + data[i + 1]
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF


def open_icmp_socket() -> socket.socket:
    """Unprivileged ICMP if the kernel allows it, raw as a fallback.

    SOCK_DGRAM is preferred: the kernel rewrites the echo id on those
    sockets, which is exactly why the sequence number is carried in the
    PAYLOAD below rather than the header -- payloads come back untouched.
    """
    try:
        return socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_ICMP)
    except PermissionError:
        pass
    try:
        return socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    except PermissionError:
        sys.exit(
            "Cannot open an ICMP socket.\n"
            "  Either:  sudo sysctl -w net.ipv4.ping_group_range=\"0 2147483647\"\n"
            "  Or run just this script with sudo.")


def _strip_ip_header(pkt: bytes) -> bytes:
    """SOCK_RAW hands back the IP header; SOCK_DGRAM does not."""
    if len(pkt) >= 20 and (pkt[0] >> 4) == 4:
        ihl = (pkt[0] & 0x0F) * 4
        if 20 <= ihl <= len(pkt):
            return pkt[ihl:]
    return pkt


def monitor(args: argparse.Namespace) -> int:
    hosts: dict[str, str] = {}
    for spec in args.hosts:
        if "=" not in spec:
            sys.exit(f"--hosts entries must be NAME=IP, got {spec!r}")
        name, ip = spec.split("=", 1)
        hosts[name] = ip
    names = list(hosts)

    sock = open_icmp_socket()
    sock.setblocking(False)

    out = open(args.out, "a", buffering=1) if args.out else None
    if out:
        out.write(json.dumps({
            "event": "start", "t": time.time(), "hosts": hosts,
            "interval_sec": args.interval,
        }) + "\n")

    # seq -> (robot, send_epoch). Entries older than the timeout are swept
    # into losses, so this cannot grow without bound.
    inflight: dict[int, tuple[str, float]] = {}
    seq = 0

    # per-robot consecutive-loss run length, which is what actually matters:
    # the firmware aborts after 1000ms of silence, so a single dropped probe
    # is noise and a run covering >1s is the event we are hunting.
    outage_since: dict[str, float] = {}
    # Time of the last reply actually received, per robot. The outage has to
    # be measured from THIS, not from the lost probe's send time: a probe is
    # only swept into "lost" at send+timeout, by which point later probes may
    # already have been answered, so measuring from the send time reports
    # ~timeout ms of "outage" for every isolated single drop. That made 12
    # scattered drops on one robot look like 12 one-second blackouts.
    last_ok: dict[str, float] = {}
    stats = {n: {"sent": 0, "recv": 0, "lost": 0, "rtt": []} for n in names}
    events: list[dict] = []

    print(f"probing {len(names)} robots every {args.interval*1000:.0f}ms "
          f"-- Ctrl+C to stop")
    print(f"  {', '.join(f'{n}={ip}' for n, ip in hosts.items())}")
    if out:
        print(f"  writing {args.out}")
    print()

    for _n in names:
        last_ok[_n] = time.time()
    next_send = time.monotonic()
    next_report = time.monotonic() + args.report
    t_end = time.monotonic() + args.duration if args.duration else None

    try:
        while True:
            now_m = time.monotonic()
            if t_end and now_m >= t_end:
                break

            if now_m >= next_send:
                next_send += args.interval
                if now_m - next_send > args.interval:   # fell behind, resync
                    next_send = now_m + args.interval
                for name in names:
                    seq += 1
                    t = time.time()
                    payload = struct.pack("!Qd", seq, t) + b"agv-ping"
                    hdr = struct.pack("!BBHHH", ICMP_ECHO_REQUEST, 0, 0,
                                      os.getpid() & 0xFFFF, seq & 0xFFFF)
                    chk = _checksum(hdr + payload)
                    hdr = struct.pack("!BBHHH", ICMP_ECHO_REQUEST, 0, chk,
                                      os.getpid() & 0xFFFF, seq & 0xFFFF)
                    try:
                        sock.sendto(hdr + payload, (hosts[name], 0))
                    except OSError:
                        # host unreachable at the ARP layer counts as loss,
                        # not as a crash
                        continue
                    inflight[seq] = (name, t)
                    stats[name]["sent"] += 1

            # drain replies
            while True:
                r, _, _ = select.select([sock], [], [], 0)
                if not r:
                    break
                try:
                    pkt, _addr = sock.recvfrom(2048)
                except OSError:
                    break
                pkt = _strip_ip_header(pkt)
                if len(pkt) < 8 + 16 or pkt[0] != ICMP_ECHO_REPLY:
                    continue
                try:
                    rseq, sent_t = struct.unpack("!Qd", pkt[8:24])
                except struct.error:
                    continue
                hit = inflight.pop(rseq, None)
                if hit is None:
                    continue
                name, t0 = hit
                rtt = (time.time() - t0) * 1000.0
                stats[name]["recv"] += 1
                stats[name]["rtt"].append(rtt)
                now_w = time.time()
                if name in outage_since:
                    outage_since.pop(name)
                    # dark time = last reply -> this reply, so a single drop
                    # reports ~one probe interval, not ~one timeout
                    prev = last_ok.get(name)
                    dur = (now_w - prev) * 1000.0 if prev else 0.0
                    ev = {"event": "recovered", "t": now_w, "robot": name,
                          "dark_ms": round(dur, 1)}
                    events.append(ev)
                    if out:
                        out.write(json.dumps(ev) + "\n")
                    if dur >= args.outage_ms:
                        print(f"  [{now_w:.3f}] {name} BACK after "
                              f"{dur:.0f}ms dark")
                last_ok[name] = now_w

            # sweep timed-out probes into losses
            cutoff = time.time() - args.timeout
            for s in [s for s, (_, t0) in inflight.items() if t0 < cutoff]:
                name, t0 = inflight.pop(s)
                stats[name]["lost"] += 1
                outage_since.setdefault(name, t0)
                ev = {"event": "loss", "t": t0, "robot": name}
                if out:
                    out.write(json.dumps(ev) + "\n")

            # THE signature: several robots dark at the same instant. One
            # robot losing packets is a robot problem; all of them at once
            # is the shared medium or the shared agent.
            if len(outage_since) >= 2:
                oldest = min(outage_since.values())
                if (time.time() - oldest) * 1000.0 >= args.outage_ms:
                    key = tuple(sorted(outage_since))
                    if not events or events[-1].get("robots") != list(key):
                        ev = {"event": "simultaneous", "t": oldest,
                              "robots": list(key), "n": len(key)}
                        events.append(ev)
                        if out:
                            out.write(json.dumps(ev) + "\n")
                        print(f"  [{oldest:.3f}] *** {len(key)} ROBOTS DARK "
                              f"TOGETHER: {', '.join(key)}")

            if time.monotonic() >= next_report:
                next_report += args.report
                parts = []
                for n in names:
                    st_ = stats[n]
                    tot = st_["recv"] + st_["lost"]
                    lossp = 100.0 * st_["lost"] / tot if tot else 0.0
                    med = (sorted(st_["rtt"])[len(st_["rtt"]) // 2]
                           if st_["rtt"] else float("nan"))
                    parts.append(f"{n} {med:5.1f}ms {lossp:4.1f}%")
                print(f"[{time.time():.3f}] " + " | ".join(parts))

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\nstopping...")

    print("\n" + "=" * 70)
    print("summary")
    for n in names:
        st_ = stats[n]
        tot = st_["recv"] + st_["lost"]
        rtts = sorted(st_["rtt"])
        med = rtts[len(rtts) // 2] if rtts else float("nan")
        p95 = rtts[int(len(rtts) * 0.95)] if rtts else float("nan")
        lossp = 100.0 * st_["lost"] / tot if tot else 0.0
        print(f"  {n:8} sent {st_['sent']:6}  lost {st_['lost']:5} "
              f"({lossp:5.2f}%)  rtt median {med:6.1f}ms  p95 {p95:6.1f}ms")
    dark = [e for e in events if e["event"] == "recovered"]
    worst = max((e["dark_ms"] for e in dark), default=0.0)
    print(f"\n  longest single dark period: {worst:.0f}ms "
          f"(the firmware aborts at 1000ms)")
    over = [e for e in dark if e["dark_ms"] >= 1000.0]
    print(f"  dark periods that would have starved the watchdog: {len(over)}")
    for e in over:
        print(f"    [{e['t']:.3f}] {e['robot']} dark {e['dark_ms']:.0f}ms")

    sim = [e for e in events if e["event"] == "simultaneous"]
    print(f"\n  multi-robot simultaneous outages (>= {args.outage_ms:.0f}ms): "
          f"{len(sim)}")
    for e in sim:
        print(f"    [{e['t']:.3f}] {e['n']} robots: {', '.join(e['robots'])}")
    if out:
        out.write(json.dumps({"event": "end", "t": time.time()}) + "\n")
        out.close()
        print(f"\n  wrote {args.out}")
        print(f"  correlate with:  python3 {sys.argv[0]} correlate "
              f"{args.out} ~/supervisor_live.log")
    return 0


TS_RE = re.compile(r"\[(\d+\.\d+)\]")
ROBOT_RE = re.compile(r"\[(?:camera_grid_navigate_)?(\w*[Aa]lvik\d)")


def correlate(args: argparse.Namespace) -> int:
    """Line the ping outages up against the firmware's aborts.

    The whole question is whether the robot was reachable at IP level during
    the second the firmware spent waiting. If it was, the packets were lost
    above IP -- the agent. If it was not, the radio link dropped and the
    agent is innocent.
    """
    losses: list[tuple[float, str]] = []
    sims: list[dict] = []
    t_first = t_last = None
    with open(args.pings) as f:
        for line in f:
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = e.get("t")
            if t is not None:
                t_first = t if t_first is None else min(t_first, t)
                t_last = t if t_last is None else max(t_last, t)
            if e.get("event") == "loss":
                losses.append((e["t"], e["robot"]))
            elif e.get("event") == "simultaneous":
                sims.append(e)
    if t_first is None:
        sys.exit(f"{args.pings} has no timestamped records")

    # Only aborts this ping log actually covers. supervisor_live.log is
    # appended across every session ever run, so without this bound the
    # older sessions come back as "no ping loss nearby" -- which reads as
    # evidence for the agent when it only means nothing was recorded.
    raw: list[tuple[float, str]] = []
    skipped = 0
    with open(args.supervisor, encoding="utf-8", errors="replace") as f:
        for line in f:
            if "WHEEL_CMD_TIMEOUT" not in line:
                continue
            m = TS_RE.search(line)
            if not m:
                continue
            t = float(m.group(1))
            if not (t_first - args.window <= t <= t_last + args.window):
                skipped += 1
                continue
            r = ROBOT_RE.search(line)
            raw.append((t, (r.group(1) if r else "?").capitalize()))

    # One stall produces TWO lines -- the navigator's [DIAG] and the
    # supervisor's reported-error. Collapse them, or every count doubles
    # (the same trap that made `grep -c` report 21 for 10 real events).
    raw.sort()
    timeouts: list[tuple[float, str]] = []
    for t, robot in raw:
        if any(r == robot and abs(t - pt) <= 0.25 for pt, r in timeouts):
            continue
        timeouts.append((t, robot))

    print(f"ping log covers {t_last - t_first:.0f}s "
          f"({t_first:.3f} .. {t_last:.3f})")
    if skipped:
        print(f"  ignoring {skipped} timeout line(s) outside that window")
    if not timeouts:
        print("\nNo WHEEL_CMD_TIMEOUT inside the ping window -- a clean run.")
        print(f"({len(losses)} ping losses recorded, "
              f"{len(sims)} simultaneous outages)")
        return 0

    print(f"{len(timeouts)} distinct stall(s), {len(losses)} ping "
          f"loss(es), window +/-{args.window}s\n")
    print(f"{'timeout at':>16} {'robot':10} {'ping losses nearby':>19}  verdict")
    verdicts = []
    for t, robot in timeouts:
        near = [(lt, ln) for lt, ln in losses if abs(lt - t) <= args.window]
        same = [x for x in near if x[1].lower() == robot.lower()]
        v = "RF / radio link" if same else ("agent / above IP" if not near
                                            else "other robot only")
        verdicts.append(v)
        print(f"{t:>16.3f} {robot:10} {len(near):>6} ({len(same)} same robot)"
              f"  {v}")

    print()
    rf = sum(1 for v in verdicts if v.startswith("RF"))
    ag = sum(1 for v in verdicts if v.startswith("agent"))
    print(f"  reachable at IP level through the abort (agent suspect): {ag}")
    print(f"  unreachable at IP level through the abort (RF suspect):  {rf}")
    if ag and not rf:
        print("\n  => The robots answered ICMP while wheel commands vanished.")
        print("     The radio link was up; the loss is above IP. Move the")
        print("     micro-ROS agent back to the Linux laptop and re-measure.")
    elif rf and not ag:
        print("\n  => The robots were unreachable at IP level during the")
        print("     aborts. This is the RF link, not the agent -- which is")
        print("     also why moving vision off the WSL2 NIC changed nothing.")
    elif rf and ag:
        print("\n  => Mixed. Both paths contribute; split the events by")
        print("     session before drawing a conclusion.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("monitor", help="probe the robots during a run")
    m.add_argument("hosts", nargs="+", metavar="NAME=IP",
                   help="e.g. Alvik1=192.168.0.21 Alvik2=192.168.0.22 ...")
    m.add_argument("--interval", type=float, default=0.1,
                   help="seconds between probe rounds (default 0.1 = 10Hz; "
                        "the firmware aborts after 1.0s of silence, so this "
                        "gives ~10 samples inside the window that matters)")
    m.add_argument("--timeout", type=float, default=1.0,
                   help="seconds before an unanswered probe counts as lost "
                        "(default 1.0, matching WHEEL_CMD_ABORT_MS)")
    m.add_argument("--outage-ms", type=float, default=300.0,
                   help="consecutive loss this long is printed as an event "
                        "(default 300, matching WHEEL_CMD_BRAKE_MS)")
    m.add_argument("--report", type=float, default=5.0,
                   help="seconds between console summary lines (default 5)")
    m.add_argument("--duration", type=float, default=0.0,
                   help="stop after this many seconds (default 0 = until "
                        "Ctrl+C)")
    m.add_argument("--out", default="", metavar="PATH",
                   help="append JSONL here for the correlate step")
    m.set_defaults(func=monitor)

    c = sub.add_parser("correlate", help="line pings up against the aborts")
    c.add_argument("pings", help="the JSONL written by monitor --out")
    c.add_argument("supervisor", help="path to supervisor_live.log")
    c.add_argument("--window", type=float, default=1.5,
                   help="seconds either side of an abort to look for ping "
                        "loss (default 1.5 -- the firmware's 1.0s window "
                        "plus margin)")
    c.set_defaults(func=correlate)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
