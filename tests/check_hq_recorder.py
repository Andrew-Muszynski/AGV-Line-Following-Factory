#!/usr/bin/env python3
"""Full-quality recorder: real ffmpeg, synthetic 1080p frames at 60 fps,
driven through its HTTP API the way video_recorder.py drives it.

    python3 tests/check_hq_recorder.py
"""
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np                        # noqa: E402

import hq_recorder as H                    # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


def post(port, path, body):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}",
                                 data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


ff = H.find_ffmpeg()
if not ff:
    print("ffmpeg not found -- skipped")
    sys.exit(0)
out = tempfile.mkdtemp(prefix="hqtest_")
rec = H.HqRecorder(ff, out)
print(f"ffmpeg {ff}\nencoder {rec.encoder}\nout {out}")
port = 18767
rec.serve(port)

print("1. idle costs nothing")
frame = np.zeros((1080, 1920, 3), np.uint8)
t = time.perf_counter()
for _ in range(1000):
    if rec.active:
        rec.offer(frame)
check("idle: 1000 frames checked", not rec.active,
      f"{(time.perf_counter() - t) * 1e3:.2f} ms total")

print("\n2. a 5 s recording at 60 fps")
r = post(port, "/api/record", {"name": "test run",
                               "file_core": "20261103-150000_test_run"})
check("started", r["recording"]["state"] == "recording", r["recording"]["file"])
worst = 0.0
t0 = time.perf_counter()
n = 300
for i in range(n):
    frame = np.full((1080, 1920, 3), (i * 4) % 256, np.uint8)
    frame[500:580, (i * 6) % 1800:(i * 6) % 1800 + 120] = (0, 0, 255)
    t = time.perf_counter()
    rec.offer(frame)
    worst = max(worst, time.perf_counter() - t)
    time.sleep(max(0.0, t0 + (i + 1) / 60 - time.perf_counter()))
check("offer() never blocks the camera loop", worst < 0.005,
      f"worst {worst * 1e3:.2f} ms")
st = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status"))
check("status reports the recording", st["recording"] is not None)
post(port, "/api/stop", {"file": r["recording"]["file"]})
for _ in range(300):
    if rec.status()["recording"] is None:
        break
    time.sleep(0.1)
last = rec.status()["last"]
print("   ", last)
check("saved", last["state"] == "saved" and not last["error"], str(last["error"]))
check("no frames dropped", last["frames_dropped"] == 0,
      f"{last['frames_dropped']} of {last['frames_offered'] + last['frames_dropped']}")
path = os.path.join(out, last["file"])
check("final mp4 exists, .part removed",
      os.path.exists(path) and not any(f.endswith(".part.mp4")
                                       for f in os.listdir(out)))
probe = subprocess.run(
    [ff.replace("ffmpeg.exe", "ffprobe.exe").replace("bin/ffmpeg", "bin/ffprobe"),
     "-v", "error", "-show_entries",
     "stream=codec_name,width,height,r_frame_rate:format=duration",
     "-of", "json", path], capture_output=True, text=True)
info = json.loads(probe.stdout or "{}")
s0 = (info.get("streams") or [{}])[0]
dur = float((info.get("format") or {}).get("duration") or 0)
print("   ", s0, f"duration {dur:.2f}s")
check("1920x1080 H.264", (s0.get("width"), s0.get("height"),
                          s0.get("codec_name")) == (1920, 1080, "h264"))
check("60 fps", s0.get("r_frame_rate") == "60/1")
check("real time (5 s of frames -> about 5 s)", 4.5 < dur < 5.6, f"{dur:.2f}s")
with open(path, "rb") as fh:
    head = fh.read(4096)
check("index at the front (plays while downloading)",
      head.find(b"moov") != -1 and (head.find(b"mdat") == -1
                                    or head.find(b"moov") < head.find(b"mdat")))

print("\n3. errors")
try:
    post(port, "/api/stop", {"file": "nope.mp4"})
    check("stopping when idle is harmless", True)
except Exception as exc:
    check("stopping when idle is harmless", False, str(exc))
r = post(port, "/api/record", {"name": "x", "file_core": "20261103-150000_test_run"})
check("a name already used gets a suffix, not an overwrite",
      r["recording"]["file"].endswith("_hq_2.mp4"), r["recording"]["file"])
try:
    post(port, "/api/record", {"name": "y"})
    check("a second start while recording is refused", False)
except urllib.error.HTTPError as exc:
    check("a second start while recording is refused", exc.code == 409)
rec.stop(wait=True)
check("stopped with no frames: reported, not a crash",
      rec.status()["last"]["state"] == "failed",
      rec.status()["last"]["error"])

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
