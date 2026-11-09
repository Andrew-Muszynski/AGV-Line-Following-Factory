#!/usr/bin/env python3
"""hq_recorder.py -- full-quality (native 1920x1080, 60 fps) run video,
taken inside camera_bridge_windows.py from the frames it already captures.

WHY IN THE BRIDGE. The bridge is the only process that can open the camera
on Windows. Everything downstream sees less: the localizer gets a table
crop, and the run videos (video_recorder.py) are its annotated 960x540
preview at 10 fps. Operator, 2026-11-03: a native-resolution recording for
presenting the testbed; not every run, so it is off unless asked for.

LOCALIZATION COMES FIRST. offer() is the only call on the bridge's hot
path: a non-blocking queue put of the frame cap.read() just returned (a
fresh array the bridge never writes to). A separate thread feeds the
frames to ffmpeg, encoding on the GPU (h264_nvenc; libx264 if there is no
NVENC). If the encoder falls behind, recording frames are DROPPED and
counted -- the localizer's frames are never delayed. The ffmpeg pipe was
measured stalling up to 183ms at start-up, which is why it is not written
from the hot path.

TIMING. Frames are stamped with the wall clock as ffmpeg reads them and
written as constant 60 fps, so a dropped frame is a repeated frame, and
the video stays in real time.

CRASH SAFETY. ffmpeg writes a fragmented <core>_hq.part.mp4 (playable up
to the last fragment if the bridge dies), remuxed to <core>_hq.mp4 with
the index at the front on Stop.

CONTROL. A small HTTP API on 127.0.0.1, the same shape as the overhead
camera's recorder (overhead_cam/overhead_recorder.py), so video_recorder.py
drives it with the same client:
    POST /api/record {"name", "file_core"}   -> {"recording": {...}}
    POST /api/stop   {"file"}                -> {"stopped": {...}}
    GET  /api/status
"""
from __future__ import annotations

import glob
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

FPS = 60
BITRATE = "20M"            # ~150 MB a minute at 1080p60
MAX_BITRATE = "30M"
#: ~0.75 s of 1080p frames (6.2 MB each). Past this the encoder is behind
#: and frames are dropped instead of queued.
QUEUE_MAX = 45
MAX_SEC = 30 * 60          # same cap as the run videos
TAG = "hq"


def find_ffmpeg(explicit: str | None = None) -> str | None:
    """ffmpeg.exe: explicit path, PATH, then winget's install folder (a
    shell opened before the install has no PATH entry for it)."""
    if explicit:
        return explicit if os.path.isfile(explicit) else None
    found = shutil.which("ffmpeg")
    if found:
        return found
    base = os.environ.get("LOCALAPPDATA", "")
    hits = sorted(glob.glob(os.path.join(
        base, "Microsoft", "WinGet", "Packages", "*FFmpeg*", "*", "bin",
        "ffmpeg.exe")))
    return hits[-1] if hits else None


def has_nvenc(ffmpeg: str) -> bool:
    try:
        out = subprocess.run([ffmpeg, "-hide_banner", "-encoders"],
                             capture_output=True, text=True, timeout=10)
        return "h264_nvenc" in out.stdout
    except (OSError, subprocess.SubprocessError):
        return False


def _slug(text: str, limit: int = 60) -> str:
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", str(text or "")).strip("._-")
    return s[:limit].rstrip("._-")


class HqRecorder:
    def __init__(self, ffmpeg: str | None, video_dir: str, fps: int = FPS,
                 bitrate: str = BITRATE, queue_max: int = QUEUE_MAX,
                 max_sec: float = MAX_SEC, encoder: str | None = None):
        self.ffmpeg = ffmpeg
        self.video_dir = os.path.abspath(video_dir)
        self.fps = int(fps)
        self.bitrate = bitrate
        self.max_sec = float(max_sec)
        self.encoder = encoder or (
            "h264_nvenc" if ffmpeg and has_nvenc(ffmpeg) else "libx264")
        self._q: queue.Queue = queue.Queue(maxsize=queue_max)
        self._lock = threading.Lock()
        self._s: dict | None = None       # the recording in progress
        self._last: dict | None = None
        # Read on the hot path without the lock: a plain bool.
        self.active = False
        os.makedirs(self.video_dir, exist_ok=True)

    # ---- hot path --------------------------------------------------------

    def offer(self, frame) -> None:
        """Hand over a captured frame. Never blocks."""
        s = self._s
        if s is None or not self.active:
            return
        try:
            self._q.put_nowait(frame)
            s["offered"] += 1
        except queue.Full:
            s["dropped"] += 1

    # ---- control ---------------------------------------------------------

    def start(self, name: str = "", file_core: str = "") -> dict:
        if not self.ffmpeg:
            raise RuntimeError("ffmpeg not found on this laptop "
                               "(install: winget install Gyan.FFmpeg)")
        with self._lock:
            if self._s is not None:
                raise RuntimeError(f"already recording {self._s['file']}")
            core = _slug(file_core) or (
                datetime.now().strftime("%Y%m%d-%H%M%S")
                + (f"_{_slug(name)}" if _slug(name) else ""))
            final = f"{core}_{TAG}.mp4"
            n = 2
            while os.path.exists(os.path.join(self.video_dir, final)):
                final = f"{core}_{TAG}_{n}.mp4"
                n += 1
            while not self._q.empty():
                self._q.get_nowait()
            s = {"name": name, "file": final,
                 "part": os.path.join(self.video_dir,
                                      final[:-4] + ".part.mp4"),
                 "t0": time.monotonic(), "first_frame_sec": None,
                 "offered": 0, "dropped": 0, "written": 0,
                 "size": None, "error": None, "state": "recording",
                 "stop": threading.Event()}
            s["thread"] = threading.Thread(target=self._run, args=(s,),
                                           daemon=True, name="hq-writer")
            self._s = s
            self.active = True
            s["thread"].start()
        return {"recording": self._public(s)}

    def stop(self, file: str | None = None, wait: bool = False) -> dict:
        with self._lock:
            s = self._s
            if s is None:
                return {"stopped": None, "last": self._last}
            if file and file != s["file"]:
                raise RuntimeError(f"not recording {file} "
                                   f"(recording {s['file']})")
            self.active = False
            s["stop"].set()
        if wait:
            s["thread"].join(timeout=60)
        return {"stopped": self._public(s)}

    def status(self) -> dict:
        s = self._s
        return {"ffmpeg": self.ffmpeg, "encoder": self.encoder,
                "video_dir": self.video_dir, "fps": self.fps,
                "recording": self._public(s) if s else None,
                "last": self._last}

    def _public(self, s: dict) -> dict:
        return {"file": s["file"], "name": s["name"], "state": s["state"],
                "elapsed_sec": round(time.monotonic() - s["t0"], 3),
                "first_frame_sec": s["first_frame_sec"],
                "frames_offered": s["offered"], "frames_written": s["written"],
                "frames_dropped": s["dropped"], "size": s["size"],
                "error": s["error"]}

    # ---- writer thread ---------------------------------------------------

    def _cmd(self, w: int, h: int) -> list[str]:
        enc = (["-c:v", "h264_nvenc", "-preset", "p4", "-rc", "vbr",
                "-b:v", self.bitrate, "-maxrate", MAX_BITRATE]
               if self.encoder == "h264_nvenc" else
               ["-c:v", "libx264", "-preset", "veryfast", "-crf", "20"])
        return [self.ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
                "-use_wallclock_as_timestamps", "1",
                "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
                "-i", "-", *enc, "-pix_fmt", "yuv420p",
                "-fps_mode", "cfr", "-r", str(self.fps),
                "-movflags", "+frag_keyframe+empty_moov+default_base_moof"]

    def _run(self, s: dict) -> None:
        proc = None
        err_path = s["part"][:-9] + ".ffmpeg.log"
        err = None
        try:
            while not s["stop"].is_set():
                if time.monotonic() - s["t0"] > self.max_sec:
                    s["error"] = f"stopped at the {self.max_sec / 60:.0f}-minute cap"
                    break
                try:
                    frame = self._q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if proc is None:
                    h, w = frame.shape[:2]
                    err = open(err_path, "wb")
                    flags = 0
                    if sys.platform == "win32":
                        flags = (subprocess.CREATE_NO_WINDOW
                                 | subprocess.BELOW_NORMAL_PRIORITY_CLASS)
                    proc = subprocess.Popen(
                        self._cmd(w, h) + [s["part"]],
                        stdin=subprocess.PIPE, stderr=err,
                        creationflags=flags)
                    s["first_frame_sec"] = round(
                        time.monotonic() - s["t0"], 3)
                proc.stdin.write(memoryview(frame).cast("B")
                                 if frame.flags.c_contiguous
                                 else frame.tobytes())
                s["written"] += 1
        except Exception as exc:
            s["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            self.active = False
            s["state"] = "saving"
            self._close(s, proc, err, err_path)

    def _close(self, s, proc, err, err_path) -> None:
        final = os.path.join(self.video_dir, s["file"])
        try:
            if proc is not None:
                try:
                    proc.stdin.close()
                except OSError:
                    pass
                try:
                    proc.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    s["error"] = s["error"] or "ffmpeg did not finish"
                if proc.returncode not in (0, None) and not s["error"]:
                    s["error"] = f"ffmpeg exited {proc.returncode}"
            if err is not None:
                err.close()
            if proc is None:
                s["error"] = s["error"] or "no frames arrived"
            elif os.path.exists(s["part"]):
                rc = subprocess.run(
                    [self.ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
                     "-i", s["part"], "-c", "copy", "-movflags", "+faststart",
                     final], capture_output=True,
                    creationflags=(subprocess.CREATE_NO_WINDOW
                                   if sys.platform == "win32" else 0))
                if rc.returncode == 0 and os.path.getsize(final) > 0:
                    os.remove(s["part"])
                else:
                    # Keep the fragmented file: it plays as it is.
                    final = s["part"]
                    s["file"] = os.path.basename(final)
                    s["error"] = s["error"] or "remux failed; kept .part.mp4"
            if os.path.exists(final):
                s["size"] = os.path.getsize(final)
            if os.path.exists(err_path) and os.path.getsize(err_path) == 0:
                os.remove(err_path)
        except Exception as exc:
            s["error"] = s["error"] or f"{type(exc).__name__}: {exc}"
        finally:
            s["state"] = "failed" if (s["error"] and not s["size"]) else "saved"
            with self._lock:
                self._last = self._public(s)
                if self._s is s:
                    self._s = None

    # ---- HTTP ------------------------------------------------------------

    def serve(self, port: int, host: str = "127.0.0.1") -> ThreadingHTTPServer:
        rec = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):
                pass

            def _send(self, code: int, body: dict) -> None:
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path.rstrip("/") == "/api/status":
                    self._send(200, rec.status())
                else:
                    self._send(404, {"error": "not found"})

            def do_POST(self):
                try:
                    n = int(self.headers.get("Content-Length") or 0)
                    body = json.loads(self.rfile.read(n) or b"{}")
                    if self.path == "/api/record":
                        self._send(200, rec.start(body.get("name", ""),
                                                  body.get("file_core", "")))
                    elif self.path == "/api/stop":
                        self._send(200, rec.stop(body.get("file")))
                    else:
                        self._send(404, {"error": "not found"})
                except RuntimeError as exc:
                    self._send(409, {"error": str(exc)})
                except Exception as exc:
                    self._send(500, {"error": f"{type(exc).__name__}: {exc}"})

        srv = ThreadingHTTPServer((host, port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True,
                         name="hq-http").start()
        return srv
