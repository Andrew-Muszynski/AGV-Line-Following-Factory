#!/usr/bin/env python3
"""Overhead camera recorder: name, record, preview and play back testbed videos.

Runs on the secondary-camera laptop (industrious@192.168.0.73), which is not
part of the fleet stack. Open it from any machine on the lab network:

    python3 overhead_recorder.py              # then http://192.168.0.73:8090/

Needs only Python 3.8+ and ffmpeg (sudo apt install ffmpeg); no pip packages.
Videos stay on this laptop, in ~/Videos/testbed/ by default.

Recording uses the same encode as the command that worked over ssh:
v4l2 MJPEG 1920x1080 at 30 fps -> libx264 veryfast CRF 23, yuv420p. Two
things differ, both for safety:

  * While recording, the file is a FRAGMENTED MP4 (<name>.part.mp4), which
    stays playable up to the last 2 s even if ffmpeg, the laptop or the
    power dies. On Stop it is remuxed (no re-encode, a few seconds) to a
    normal +faststart MP4 that seeks instantly in a browser. A .part.mp4
    left behind by a crash is recovered the same way at the next start.
  * The same ffmpeg also writes a small 960px, 10 fps JPEG preview, so the
    page can show what is being recorded. The camera can be opened by only
    one process, so outside a recording the preview runs only while someone
    is watching it, and stops 10 s after the last viewer leaves.

Recordings made by hand over ssh into the same folder show up in the
library too.

There is no login: anyone on the lab network who can reach port 8090 can
record, rename and delete. Use --host 127.0.0.1 plus an ssh tunnel if that
is ever a concern.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
PREVIEW_IDLE_SEC = 10.0
SAFE_FILE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._+\-]*\.mp4$")
STAMP = re.compile(r"^(\d{8}-\d{6})_")
STAMP_ONLY = re.compile(r"^\d{8}-\d{6}$")


def slug(text: str, limit: int = 80) -> str:
    """File-name-safe form of a name: '6 robots encoder+camera' -> '6_robots_encoder+camera'."""
    s = re.sub(r"[^A-Za-z0-9.+\-]+", "_", text or "").strip("._-")
    return s[:limit].rstrip("._-")


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _die_with_parent() -> None:
    """Runs in the ffmpeg child: if this server dies (kill -9, crash), the
    kernel sends ffmpeg SIGINT, which it treats like 'q' -- it closes the
    file and lets go of the camera instead of recording on, orphaned.

    PR_SET_PDEATHSIG fires when the THREAD that started the child exits,
    not the process. That is why every ffmpeg is started by _Launcher's one
    long-lived thread: started from an HTTP request thread, ffmpeg was told
    to stop as soon as the Record request returned -- the 2026-10-19 test
    recording held 0.6 s."""
    try:
        import ctypes
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGINT)  # PR_SET_PDEATHSIG
    except Exception:
        pass


class _Launcher:
    """Starts every ffmpeg from one thread that lives as long as the server
    (see _die_with_parent for why)."""

    def __init__(self):
        self._q: queue.Queue = queue.Queue()
        threading.Thread(target=self._run, daemon=True,
                         name="ffmpeg-launcher").start()

    def _run(self):
        while True:
            kwargs, box, done = self._q.get()
            try:
                box["proc"] = subprocess.Popen(**kwargs)
            except Exception as exc:
                box["error"] = exc
            done.set()

    def popen(self, **kwargs) -> subprocess.Popen:
        box, done = {}, threading.Event()
        self._q.put((kwargs, box, done))
        done.wait()
        if "error" in box:
            raise box["error"]
        return box["proc"]


_LAUNCHER = _Launcher()


class FFmpeg:
    """One ffmpeg process. stdout is always drained (a full pipe would stall
    the recording) and split into JPEG frames for the preview; stderr is
    kept as a short tail for error messages."""

    def __init__(self, cmd: list, on_frame, kind: str):
        self.kind = kind
        self.cmd = cmd
        self.tail = deque(maxlen=25)
        self.p = _LAUNCHER.popen(args=cmd, stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, bufsize=0,
                                 preexec_fn=_die_with_parent
                                 if sys.platform.startswith("linux") else None)
        self._on_frame = on_frame
        threading.Thread(target=self._read_frames, daemon=True).start()
        threading.Thread(target=self._read_log, daemon=True).start()

    def _read_frames(self):
        buf = b""
        while True:
            chunk = self.p.stdout.read(65536)
            if not chunk:
                return
            buf += chunk
            while True:
                start = buf.find(b"\xff\xd8")
                if start < 0:
                    buf = buf[-1:]
                    break
                end = buf.find(b"\xff\xd9", start + 2)
                if end < 0:
                    buf = buf[start:]
                    break
                self._on_frame(buf[start:end + 2])
                buf = buf[end + 2:]
            if len(buf) > 16_000_000:      # never a real frame; resync
                buf = b""

    def _read_log(self):
        for line in iter(self.p.stderr.readline, b""):
            text = line.decode("utf-8", "replace").rstrip()
            if text:
                self.tail.append(text)

    def alive(self) -> bool:
        return self.p.poll() is None

    def error_text(self) -> str:
        code = self.p.returncode
        how = (f"ffmpeg was ended by signal {-code}" if code is not None and code < 0
               else f"ffmpeg exited with code {code}")
        # The tail often holds only harmless warnings; the exit status is
        # what says what happened.
        return "\n".join([how] + list(self.tail))

    def stop(self, graceful: bool) -> None:
        """graceful: ask ffmpeg to finish the file ('q' on stdin, then
        SIGINT). Otherwise just end it -- the preview writes no file."""
        if self.alive() and graceful:
            try:
                self.p.stdin.write(b"q")
                self.p.stdin.flush()
            except OSError:
                pass
            try:
                self.p.wait(10)
            except subprocess.TimeoutExpired:
                if hasattr(signal, "SIGINT") and os.name == "posix":
                    self.p.send_signal(signal.SIGINT)
        if self.alive():
            self.p.terminate()
        try:
            self.p.wait(10)
        except subprocess.TimeoutExpired:
            self.p.kill()
            self.p.wait(5)
        for pipe in (self.p.stdin, self.p.stdout, self.p.stderr):
            try:
                pipe.close()
            except OSError:
                pass


class Recorder:
    def __init__(self, args):
        self.a = args
        self.dir = os.path.abspath(os.path.expanduser(args.dir))
        self.thumbs = os.path.join(self.dir, ".thumbs")
        os.makedirs(self.thumbs, exist_ok=True)
        self.lock = threading.RLock()
        self.proc: FFmpeg | None = None
        self.mode = "idle"                 # idle | preview | recording
        self.rec: dict | None = None
        self.finalizing: dict = {}         # file -> title
        self.error: str | None = None
        self.version = 0                   # bumps whenever the library changes
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.seq = 0
        self.clients = 0
        self.last_client = 0.0
        self._ff_t0: float | None = None     # recording ffmpeg launched at
        self._ff_sec: float | None = None    # ...and its first frame after
        self._thumb_lock = threading.Lock()
        self._recover()
        threading.Thread(target=self._watchdog, daemon=True).start()

    # ---- ffmpeg command lines ------------------------------------------

    def _input(self) -> list:
        if self.a.input:
            return shlex.split(self.a.input)
        return ["-f", "v4l2", "-input_format", "mjpeg",
                "-video_size", self.a.size, "-framerate", str(self.a.fps),
                "-i", self.a.device]

    def _preview_out(self) -> list:
        return ["-map", "0:v", "-an",
                "-vf", f"fps={self.a.preview_fps},scale={self.a.preview_width}:-2",
                "-c:v", "mjpeg", "-q:v", "7", "-f", "image2pipe", "pipe:1"]

    def _record_cmd(self, part: str) -> list:
        return ([self.a.ffmpeg, "-hide_banner", "-loglevel", "warning", "-y"]
                + self._input()
                + ["-map", "0:v", "-an", "-c:v", "libx264",
                   "-preset", self.a.preset, "-crf", str(self.a.crf),
                   "-pix_fmt", "yuv420p",
                   # A keyframe -- and so a fragment -- every 2 s: a crash
                   # loses at most the last 2 s.
                   "-g", str(int(self.a.fps) * 2),
                   "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
                   part]
                + self._preview_out())

    def _preview_cmd(self) -> list:
        return ([self.a.ffmpeg, "-hide_banner", "-loglevel", "warning"]
                + self._input() + self._preview_out())

    # ---- preview frames ------------------------------------------------

    def _on_frame(self, jpeg: bytes) -> None:
        if self._ff_t0 is not None and self._ff_sec is None:
            # Time from launching the recording ffmpeg to its first frame:
            # the recorded file's 0:00. The main app subtracts it when lining
            # this video up with the main camera's. Timed from launch, not
            # from when start() returns -- the first frame usually arrives
            # during start()'s 1.5 s camera check.
            self._ff_sec = round(time.monotonic() - self._ff_t0, 3)
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def add_client(self) -> None:
        with self.lock:
            self.clients += 1
            self.last_client = time.monotonic()
            if self.mode == "idle":
                self._start_preview()

    def remove_client(self) -> None:
        with self.lock:
            self.clients = max(0, self.clients - 1)
            self.last_client = time.monotonic()

    def _start_preview(self) -> None:
        try:
            self.proc = FFmpeg(self._preview_cmd(), self._on_frame, "preview")
            self.mode = "preview"
        except OSError as exc:
            self.error = f"could not start ffmpeg: {exc}"

    def _stop_proc(self, graceful: bool) -> None:
        proc, self.proc = self.proc, None
        if proc is not None:
            proc.stop(graceful)

    # ---- recording -----------------------------------------------------

    def start(self, name: str, stamp: str = "", camera: str = "",
              file_core: str = "") -> dict:
        """stamp + camera come from the main app (app.py on the Windows
        laptop) when it starts both cameras for one run: the two files then
        share the date-time prefix and the name, and differ only in the
        camera tag, e.g. ..._R037_S1_6r_CBS_main.mp4 / ..._second.mp4."""
        with self.lock:
            if self.mode == "recording":
                raise RuntimeError(f"already recording \"{self.rec['title']}\"")
            # One process at a time owns the camera: the preview gives it up.
            self._stop_proc(graceful=False)
            self.mode = "idle"
            started = datetime.now()
            title = (name or "").strip() or f"Recording {started:%Y-%m-%d %H:%M}"
            tag = f"_{slug(camera)}" if slug(camera) else ""
            if slug(file_core):
                # The main app's exact file name minus its camera tag, e.g.
                # DOE_6r_cbs_3: both cameras' files then match exactly.
                prefix = f"{stamp}_" if STAMP_ONLY.match(stamp or "") else ""
                base = self._unique(f"{prefix}{slug(file_core)}{tag}")
            else:
                prefix = stamp if STAMP_ONLY.match(stamp or "") else f"{started:%Y%m%d-%H%M%S}"
                base = self._unique(f"{prefix}_{slug(title) or 'recording'}{tag}")
            part = os.path.join(self.dir, base + ".part.mp4")
            self._write_meta(base + ".mp4", {"title": title, "started": now_iso(),
                                             "status": "recording",
                                             "camera": slug(camera) or None})
            t0 = time.monotonic()
            self._ff_sec, self._ff_t0 = None, t0
            try:
                proc = FFmpeg(self._record_cmd(part), self._on_frame, "record")
            except OSError as exc:
                self._drop_meta(base + ".mp4")
                raise RuntimeError(f"could not start ffmpeg: {exc}")
            # A busy or missing camera fails within a second; say so now
            # rather than showing a recording that is not happening.
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and proc.alive():
                time.sleep(0.1)
            if not proc.alive():
                err = proc.error_text()
                proc.stop(graceful=False)
                for path in (part,):
                    if os.path.exists(path):
                        os.remove(path)
                self._drop_meta(base + ".mp4")
                self.error = err
                if self.clients:
                    self._start_preview()
                raise RuntimeError("ffmpeg could not record:\n" + err)
            self.proc = proc
            self.mode = "recording"
            self.error = None
            self.rec = {"title": title, "file": base + ".mp4", "part": part,
                        "t0": t0, "started": now_iso()}
            self.version += 1
            return self.status()

    def stop(self, reason: str = "stopped", file: str = "") -> dict:
        """file: stop only if that is what is recording -- the main app
        must never end a recording somebody started here by hand."""
        with self.lock:
            if self.mode != "recording":
                return self.status()
            if file and self.rec["file"] != file:
                return self.status()
            rec = self.rec
            self._stop_proc(graceful=True)      # 'q': ffmpeg closes the file
            self.mode, self.rec = "idle", None
            self.finalizing[rec["file"]] = rec["title"]
            self.version += 1
            if self.clients:
                self._start_preview()
        threading.Thread(target=self._finalize,
                         args=(rec, reason, time.monotonic() - rec["t0"]),
                         daemon=True).start()
        return self.status()

    def _finalize(self, rec: dict, reason: str, wall_sec: float) -> None:
        final = os.path.join(self.dir, rec["file"])
        ok = self._remux(rec["part"], final)
        meta = self._read_meta(rec["file"])
        meta.update({"status": "ready" if ok else "failed", "ended": now_iso(),
                     "stop_reason": reason})
        if ok:
            meta["duration_sec"] = self._duration(final) or round(wall_sec, 1)
            meta["bytes"] = os.path.getsize(final)
        else:
            meta["error"] = "the recording file could not be finalized"
        self._write_meta(rec["file"], meta)
        if ok:
            self._thumb(rec["file"])
        with self.lock:
            self.finalizing.pop(rec["file"], None)
            self.version += 1

    def _remux(self, part: str, final: str) -> bool:
        """Fragmented .part.mp4 -> normal +faststart .mp4, no re-encode.
        If that fails the fragmented file is kept as the final one: it
        still plays, it just seeks more slowly."""
        if not os.path.exists(part) or os.path.getsize(part) == 0:
            return False
        r = subprocess.run([self.a.ffmpeg, "-hide_banner", "-loglevel", "error",
                            "-y", "-i", part, "-c", "copy",
                            "-movflags", "+faststart", final],
                           stdin=subprocess.DEVNULL, capture_output=True)
        if r.returncode == 0 and os.path.exists(final) and os.path.getsize(final) > 0:
            os.remove(part)
        else:
            os.replace(part, final)
        return True

    def _recover(self) -> None:
        """A .part.mp4 at startup is a recording a crash cut short."""
        for fn in os.listdir(self.dir):
            if fn.endswith(".part.mp4"):
                file = fn[: -len(".part.mp4")] + ".mp4"
                meta = self._read_meta(file)
                meta.setdefault("title", file[:-4])
                self._write_meta(file, meta)
                self._finalize({"file": file, "part": os.path.join(self.dir, fn),
                                "title": meta["title"]},
                               "recovered after an unexpected stop", 0.0)
                print(f"recovered {fn}")

    def _watchdog(self) -> None:
        while True:
            time.sleep(1.0)
            with self.lock:
                now = time.monotonic()
                proc = self.proc
                if self.mode == "preview":
                    if proc is None or not proc.alive():
                        self.error = proc.error_text() if proc else None
                        self.proc, self.mode = None, "idle"
                    elif self.clients == 0 and now - self.last_client > PREVIEW_IDLE_SEC:
                        self._stop_proc(graceful=False)
                        self.mode = "idle"
                elif self.mode == "recording":
                    if proc is None or not proc.alive():
                        self.error = ("ffmpeg stopped on its own; what was "
                                      "recorded is kept.\n"
                                      + (proc.error_text() if proc else ""))
                        threading.Thread(target=self.stop,
                                         args=("ffmpeg exited",), daemon=True).start()
                    elif now - self.rec["t0"] > self.a.max_minutes * 60:
                        threading.Thread(target=self.stop,
                                         args=(f"reached the {self.a.max_minutes} min cap",),
                                         daemon=True).start()

    # ---- library -------------------------------------------------------

    def _meta_path(self, file: str) -> str:
        return os.path.join(self.dir, file[:-4] + ".json")

    def _read_meta(self, file: str) -> dict:
        try:
            with open(self._meta_path(file), "r", encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {}

    def _write_meta(self, file: str, meta: dict) -> None:
        tmp = self._meta_path(file) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=1)
        os.replace(tmp, self._meta_path(file))

    def _drop_meta(self, file: str) -> None:
        try:
            os.remove(self._meta_path(file))
        except OSError:
            pass

    def _unique(self, base: str) -> str:
        cand, i = base, 2
        while any(os.path.exists(os.path.join(self.dir, cand + ext))
                  for ext in (".mp4", ".part.mp4", ".json")):
            cand, i = f"{base}-{i}", i + 1
        return cand

    def _duration(self, path: str) -> float | None:
        # ffmpeg itself prints "Duration: HH:MM:SS.ss"; no ffprobe needed.
        r = subprocess.run([self.a.ffmpeg, "-hide_banner", "-i", path],
                           stdin=subprocess.DEVNULL, capture_output=True)
        m = re.search(rb"Duration: (\d+):(\d+):(\d+(?:\.\d+)?)", r.stderr)
        if not m:
            return None
        h, mi, s = int(m.group(1)), int(m.group(2)), float(m.group(3))
        return round(h * 3600 + mi * 60 + s, 1)

    def _thumb(self, file: str) -> str | None:
        out = os.path.join(self.thumbs, file[:-4] + ".jpg")
        if os.path.exists(out):
            return out
        src = os.path.join(self.dir, file)
        if not os.path.exists(src):
            return None
        with self._thumb_lock:
            dur = self._read_meta(file).get("duration_sec") or 0
            at = min(5.0, dur / 2) if dur else 1.0
            subprocess.run([self.a.ffmpeg, "-hide_banner", "-loglevel", "error",
                            "-y", "-ss", f"{at:.1f}", "-i", src, "-frames:v", "1",
                            "-vf", "scale=480:-2", "-q:v", "4", out],
                           stdin=subprocess.DEVNULL, capture_output=True)
        return out if os.path.exists(out) else None

    def safe_path(self, file: str) -> str | None:
        file = os.path.basename(unquote(file or ""))
        if not SAFE_FILE.match(file) or file.endswith(".part.mp4"):
            return None
        path = os.path.join(self.dir, file)
        return path if os.path.isfile(path) else None

    def library(self) -> list:
        items = []
        with self.lock:
            busy = dict(self.finalizing)
            live = dict(self.rec) if self.rec else None
            if live:
                busy[live["file"]] = live["title"]
        for fn in os.listdir(self.dir):
            if not fn.endswith(".mp4") or fn.endswith(".part.mp4") or fn in busy:
                continue
            if not SAFE_FILE.match(fn):
                continue
            path = os.path.join(self.dir, fn)
            meta = self._read_meta(fn)
            st = os.stat(path)
            if "duration_sec" not in meta or meta.get("bytes") != st.st_size:
                # A file recorded by hand over ssh, or not seen before.
                meta.setdefault("title", fn[:-4])
                meta.setdefault("started", datetime.fromtimestamp(
                    st.st_mtime).astimezone().isoformat(timespec="seconds"))
                meta.setdefault("status", "ready")
                meta["duration_sec"] = self._duration(path)
                meta["bytes"] = st.st_size
                self._write_meta(fn, meta)
            items.append({"file": fn, "title": meta.get("title", fn[:-4]),
                          "camera": meta.get("camera"),
                          "started": meta.get("started"),
                          "duration_sec": meta.get("duration_sec"),
                          "bytes": st.st_size,
                          "status": meta.get("status", "ready"),
                          "stop_reason": meta.get("stop_reason"),
                          "error": meta.get("error")})
        for fn, title in busy.items():
            now_live = live is not None and live["file"] == fn
            items.append({"file": fn, "title": title,
                          "started": live["started"] if now_live
                          else self._read_meta(fn).get("started"),
                          "duration_sec": None, "bytes": None,
                          "status": "recording" if now_live else "finalizing"})
        items.sort(key=lambda v: v.get("started") or "", reverse=True)
        return items

    def rename(self, file: str, name: str) -> dict:
        title = (name or "").strip()
        if not title:
            raise ValueError("a recording needs a name")
        path = self.safe_path(file)
        if path is None:
            raise FileNotFoundError(file)
        m = STAMP.match(file)
        stamp = m.group(1) if m else datetime.fromtimestamp(
            os.path.getmtime(path)).strftime("%Y%m%d-%H%M%S")
        new_file = file
        wanted = f"{stamp}_{slug(title) or 'recording'}"
        if wanted + ".mp4" != file:
            new_file = self._unique(wanted) + ".mp4"
        meta = self._read_meta(file)
        meta["title"] = title
        with self.lock:
            if new_file != file:
                os.replace(path, os.path.join(self.dir, new_file))
                self._drop_meta(file)
                old_thumb = os.path.join(self.thumbs, file[:-4] + ".jpg")
                if os.path.exists(old_thumb):
                    os.replace(old_thumb, os.path.join(self.thumbs, new_file[:-4] + ".jpg"))
            self._write_meta(new_file, meta)
            self.version += 1
        return {"file": new_file, "title": title}

    def delete(self, file: str) -> None:
        path = self.safe_path(file)
        if path is None:
            raise FileNotFoundError(file)
        with self.lock:
            os.remove(path)
            self._drop_meta(file)
            thumb = os.path.join(self.thumbs, file[:-4] + ".jpg")
            if os.path.exists(thumb):
                os.remove(thumb)
            self.version += 1

    def status(self) -> dict:
        with self.lock:
            rec = None
            if self.rec:
                part = self.rec["part"]
                rec = {"title": self.rec["title"], "file": self.rec["file"],
                       "elapsed_sec": round(time.monotonic() - self.rec["t0"], 3),
                       "first_frame_sec": self._ff_sec,
                       "bytes": os.path.getsize(part) if os.path.exists(part) else 0}
            usage = shutil.disk_usage(self.dir)
            return {"mode": self.mode, "recording": rec,
                    "finalizing": list(self.finalizing.values()),
                    "error": self.error, "version": self.version,
                    "viewers": self.clients, "dir": self.dir,
                    "disk_free_gb": round(usage.free / 1e9, 1),
                    "host": os.uname().nodename if hasattr(os, "uname") else "",
                    "camera": self.a.input or f"{self.a.device} {self.a.size} @ {self.a.fps} fps",
                    "encode": f"libx264 {self.a.preset} CRF {self.a.crf}"}

    def shutdown(self) -> None:
        if self.mode == "recording":
            self.stop("recorder shut down")
            time.sleep(0.5)
            while self.finalizing:
                time.sleep(0.2)
        with self.lock:
            self._stop_proc(graceful=False)


# ---------------------------------------------------------------------------
# HTTP


def make_handler(rec: Recorder):
    page = open(os.path.join(HERE, "overhead_recorder.html"), "rb").read()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            try:
                return json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return {}

        def do_GET(self):  # noqa: N802
            url = urlparse(self.path)
            path = url.path
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif path == "/api/status":
                self._json(rec.status())
            elif path == "/api/recordings":
                self._json({"recordings": rec.library(), "version": rec.version})
            elif path == "/stream":
                self._stream()
            elif path.startswith("/videos/"):
                self._video(path[len("/videos/"):],
                            parse_qs(url.query).get("download") == ["1"])
            elif path.startswith("/thumbs/"):
                file = os.path.basename(unquote(path[len("/thumbs/"):]))
                src = rec.safe_path(file[:-4] + ".mp4") if file.endswith(".jpg") else None
                thumb = rec._thumb(os.path.basename(src)) if src else None
                if not thumb:
                    self.send_error(404)
                    return
                data = open(thumb, "rb").read()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "max-age=3600")
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)

        def do_POST(self):  # noqa: N802
            path = urlparse(self.path).path
            body = self._body()
            try:
                if path == "/api/record":
                    self._json(rec.start(str(body.get("name") or ""),
                                         str(body.get("stamp") or ""),
                                         str(body.get("camera") or ""),
                                         str(body.get("file_core") or "")))
                elif path == "/api/stop":
                    self._json(rec.stop(file=str(body.get("file") or "")))
                elif path == "/api/rename":
                    self._json(rec.rename(str(body.get("file") or ""),
                                          str(body.get("name") or "")))
                elif path == "/api/delete":
                    rec.delete(str(body.get("file") or ""))
                    self._json({"deleted": body.get("file")})
                else:
                    self.send_error(404)
            except RuntimeError as exc:
                self._json({"error": str(exc)}, 409)
            except ValueError as exc:
                self._json({"error": str(exc)}, 400)
            except FileNotFoundError:
                self._json({"error": "no such recording"}, 404)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            rec.add_client()
            seq = -1
            try:
                while True:
                    with rec.cond:
                        rec.cond.wait_for(lambda: rec.seq != seq, timeout=1.0)
                        jpeg, seq = rec.jpeg, rec.seq
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                     + jpeg + b"\r\n")
                    with rec.lock:
                        rec.last_client = time.monotonic()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                rec.remove_client()
                self.close_connection = True

        def _video(self, file: str, download: bool):
            path = rec.safe_path(file)
            if path is None:
                self.send_error(404)
                return
            size = os.path.getsize(path)
            start, end = 0, size - 1
            m = re.match(r"bytes=(\d*)-(\d*)$", self.headers.get("Range") or "")
            if m and (m.group(1) or m.group(2)):
                if m.group(1):
                    start = int(m.group(1))
                    end = int(m.group(2)) if m.group(2) else size - 1
                else:                       # suffix range: last N bytes
                    start = max(0, size - int(m.group(2)))
                end = min(end, size - 1)
                if start > end:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                self.send_response(206)
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            else:
                self.send_response(200)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(end - start + 1))
            if download:
                title = rec._read_meta(os.path.basename(path)).get("title") or file[:-4]
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{slug(title) or file[:-4]}.mp4"')
            self.end_headers()
            try:
                with open(path, "rb") as fh:
                    fh.seek(start)
                    left = end - start + 1
                    while left > 0:
                        chunk = fh.read(min(1 << 20, left))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        left -= len(chunk)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

    return Handler


class QuietServer(ThreadingHTTPServer):
    def handle_error(self, request, client_address):
        # A browser closing a tab mid-request is routine, not an error.
        if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--dir", default="~/Videos/testbed")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--preset", default="veryfast")
    ap.add_argument("--crf", type=int, default=23)
    ap.add_argument("--preview-width", type=int, default=960)
    ap.add_argument("--preview-fps", type=int, default=10)
    ap.add_argument("--max-minutes", type=int, default=180,
                    help="stop a recording nobody stopped after this long")
    ap.add_argument("--ffmpeg", default=shutil.which("ffmpeg") or "ffmpeg")
    ap.add_argument("--input", default="",
                    help="replace the camera input arguments, e.g. for a test "
                         "pattern: \"-re -f lavfi -i testsrc2=size=1920x1080:rate=30\"")
    args = ap.parse_args()
    if not shutil.which(args.ffmpeg) and not os.path.exists(args.ffmpeg):
        sys.exit("ffmpeg not found. Install it with: sudo apt install ffmpeg")
    rec = Recorder(args)
    server = QuietServer((args.host, args.port), make_handler(rec))
    server.daemon_threads = True

    def bye(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, bye)
    print(f"overhead recorder: http://{args.host}:{args.port}/  videos in {rec.dir}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        rec.shutdown()
        print("stopped; any recording in progress was saved")


if __name__ == "__main__":
    main()
