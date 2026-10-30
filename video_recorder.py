"""Per-run video recording of the annotated overhead camera stream.

Each run is saved as its own named video, so a past run can be watched back
from the Recordings page (page 6) next to its metrics and events.

Source: the MJPEG preview that apriltag_localize.py already serves at
http://localhost:8081/stream (WSL2, forwarded to Windows localhost). That is
the same annotated view Run & Monitor shows -- tags, poses and the node
overlay included -- so nothing new runs in the vision process and the
detector pays nothing for a recording. Override with AGV_STREAM_URL.

Encoder: H.264 through Windows Media Foundation (cv2.CAP_MSMF). Measured on
this laptop on 2026-10-17 at the stream's 960x540: 3.5 ms per frame,
Baseline profile, plays in every browser. The alternatives that also play
in a browser either were not available (OpenCV's FFmpeg H.264 needs the
OpenH264 DLL, which is not installed) or were far too slow (VP8 153 ms per
frame, VP9 600 ms -- a 10 fps stream needs under 100). OpenCV fixes the
Media Foundation bitrate at fps x width x height, about 5.4 Mbit/s here:
roughly 160 MB for a 4-minute run.

Timing: the stream is about 10 fps but jittery, so frames are written on a
fixed 10 fps clock (repeating the last frame, dropping extras). Video time
therefore equals wall time since the recording started, which is what lets
the page seek to an event: an event at elapsed_ms plays at
run_t0_offset_sec + elapsed_ms / 1000.

Triggers (wired in app.py): a recording starts on the earlier of the
browser's PATCH status=running (the Start button) and the supervisor's
RUN_STARTED event, and stops TAIL_SEC after the first terminal signal
(RUN_COMPLETED / RUN_FAILED / RUN_ABORTED, or the matching PATCH). Only one
recording runs at a time -- there is one camera.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import threading
import time
import urllib.request
from datetime import datetime, timedelta

import database

try:
    import cv2
    import numpy as np
except ImportError:          # the metrics backend must still start without it
    cv2 = None
    np = None

DEFAULT_STREAM_URL = "http://localhost:8081/stream"
#: overhead_cam/overhead_recorder.py on the second camera's laptop
#: (industrious@192.168.0.73). Override with AGV_SECOND_CAMERA_URL; set it
#: to an empty string to never contact it.
DEFAULT_SECOND_CAMERA_URL = "http://192.168.0.73:8090"
#: Camera tags in the file names: the two videos of one run differ only here.
MAIN_TAG, SECOND_TAG = "main", "second"
FPS = 10.0
#: Seconds kept after the run's terminal signal: the last robot parking.
TAIL_SEC = 5.0
#: Hard cap. A run whose terminal event never arrives (Start pressed, the
#: mission never loaded) must not record for the rest of the day.
MAX_SEC = 30 * 60
#: How long to wait for the first camera frame before opening the file at
#: the default size with a "waiting" card.
FIRST_FRAME_WAIT_SEC = 4.0
#: No new camera frame for this long marks the strip STALLED.
STALL_SEC = 2.0
DEFAULT_SIZE = (960, 540)
#: Height of the caption strip added under the camera image.
STRIP_PX = 28

TRAFFIC_LABELS = {"page": "Page plan", "page_wait": "Page plan",
                  "astar": "A*", "wait": "WAIT", "pp": "PP", "cbs": "CBS"}

#: Events the player marks on its timeline, by category.
MARKER_EVENTS = {
    "safety": ("COLLISION", "COLLISION_AVOIDED", "DEADLOCK_DETECTED",
               "DEADLOCK_RESOLUTION_STARTED", "DEADLOCK_RESOLVED",
               "DEADLOCK_RESOLUTION_DECLINED", "DEADLOCK_RESOLUTION_FAILED",
               "COMMAND_FAILED", "ROBOT_OFFLINE"),
    "progress": ("RUN_STARTED", "LAUNCH_COMPLETED", "PROCESSING_STARTED",
                 "PROCESSING_COMPLETED", "DROPOFF_COMPLETED",
                 "DEPOT_ARRIVED", "RUN_COMPLETED", "RUN_FAILED",
                 "RUN_ABORTED"),
}


def slug(text: str, limit: int = 60) -> str:
    """File-name-safe form of a user label: R037 S1_6r PP -> R037_S1_6r_PP."""
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", str(text or "")).strip("._-")
    return s[:limit].rstrip("._-")


def plan_stem(plan_file: str) -> str:
    """'generated_plans/DOE_6r_cbs.json' -> 'DOE_6r_cbs'."""
    name = os.path.basename(str(plan_file or "")).strip()
    return name[:-5] if name.lower().endswith(".json") else name


def video_names(started: datetime, label: str | None, run: dict | None,
                run_id: str | None) -> tuple[str, str]:
    """(title, file base name) for a new recording.

    The file name always starts with the local start time, so the folder
    sorts chronologically and two recordings can never collide on a label.
    """
    ts = started.strftime("%Y%m%d-%H%M%S")
    label = (label or "").strip()
    if label and slug(label):
        return label, f"{ts}_{slug(label)}"
    run = run or {}
    n = run.get("robot_count")
    method = run.get("traffic_method") or ""
    parts = [f"{n}r" if n else None, slug(method) or None,
             (run_id or "")[:8] or "manual"]
    base = f"{ts}_" + "_".join(p for p in parts if p)
    title_bits = [f"{n} robots" if n else None,
                  TRAFFIC_LABELS.get(method, method) or None,
                  started.strftime("%Y-%m-%d %H:%M")]
    if not run_id:
        title_bits.insert(0, "Manual recording")
    return " · ".join(b for b in title_bits if b), base


# ---------------------------------------------------------------------------
# second camera


class SecondCamera:
    """HTTP client for the second camera's recorder. It records on its own
    laptop; this side only starts and stops it and keeps its file name."""

    def __init__(self, url: str, timeout: float = 8.0):
        self.url = url.rstrip("/")
        self.timeout = timeout

    def _post(self, path: str, body: dict) -> dict:
        req = urllib.request.Request(
            self.url + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.load(resp)
        except urllib.error.HTTPError as exc:
            try:
                msg = json.load(exc).get("error") or str(exc)
            except ValueError:
                msg = str(exc)
            raise RuntimeError(msg) from None
        except (OSError, ValueError) as exc:
            raise RuntimeError(f"unreachable at {self.url} ({exc})") from None

    def start(self, name: str, stamp: str, file_core: str = "") -> dict:
        return self._post("/api/record", {"name": name, "stamp": stamp,
                                          "camera": SECOND_TAG,
                                          "file_core": file_core})

    def stop(self, file: str) -> dict:
        return self._post("/api/stop", {"file": file})


# ---------------------------------------------------------------------------
# camera source


class MjpegReader:
    """Keeps the newest JPEG of a multipart/x-mixed-replace stream.

    Reconnects every second while running, so a recording that starts
    before the vision process, or outlives a restart of it, picks the
    stream up as soon as it is back.
    """

    def __init__(self, url: str, timeout: float = 3.0):
        self.url = url
        self.timeout = timeout
        self.error: str | None = None
        self.connects = 0
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._at: float | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="video-mjpeg-reader")

    def start(self) -> "MjpegReader":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def latest(self) -> tuple[bytes | None, int, float | None]:
        with self._lock:
            return self._jpeg, self._seq, self._at

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url,
                                            timeout=self.timeout) as resp:
                    self.connects += 1
                    self.error = None
                    self._read_parts(resp)
            except Exception as exc:          # refused, timed out, reset
                self.error = f"{type(exc).__name__}: {exc}"
            self._stop.wait(1.0)

    def _read_parts(self, resp) -> None:
        # apriltag_localize.MjpegServer writes, per frame:
        #   --frame CRLF Content-Type: image/jpeg CRLF
        #   Content-Length: N CRLF CRLF <N bytes> CRLF
        while not self._stop.is_set():
            line = resp.readline()
            if not line:
                return
            if not line.lower().startswith(b"content-length:"):
                continue
            n = int(line.split(b":", 1)[1].strip())
            while True:
                hdr = resp.readline()
                if not hdr:
                    return
                if hdr in (b"\r\n", b"\n"):
                    break
            data = resp.read(n)
            if len(data) < n:
                return
            with self._lock:
                self._jpeg = data
                self._seq += 1
                self._at = time.monotonic()


# ---------------------------------------------------------------------------
# encoder


class _CvWriter:
    def __init__(self, writer, path: str, codec: str, playable: bool):
        self._w = writer
        self.path = path
        self.codec = codec
        self.browser_playable = playable

    def write(self, frame) -> None:
        self._w.write(frame)

    def release(self) -> None:
        self._w.release()


def open_writer(base_path: str, fps: float, size: tuple[int, int]) -> _CvWriter:
    """First encoder that opens, best first. Raises if none does."""
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is not installed for this Python")
    attempts = []
    if hasattr(cv2, "CAP_MSMF"):
        attempts.append((cv2.CAP_MSMF, "H264", ".mp4", "h264", True))
    attempts += [(cv2.CAP_FFMPEG, "avc1", ".mp4", "h264", True),
                 # Last resort: always encodes, but no browser plays it --
                 # the page offers it as a download instead.
                 (cv2.CAP_ANY, "MJPG", ".avi", "mjpeg", False)]
    for api, tag, ext, codec, playable in attempts:
        path = base_path + ext
        w = cv2.VideoWriter(path, api, cv2.VideoWriter_fourcc(*tag), fps, size)
        if w.isOpened():
            return _CvWriter(w, path, codec, playable)
        w.release()
        if os.path.exists(path) and os.path.getsize(path) == 0:
            os.remove(path)
    raise RuntimeError(f"no video encoder would open {size[0]}x{size[1]}")


# ---------------------------------------------------------------------------
# frames


def _fit(img, size):
    """img at exactly size (w, h); letterboxed if the stream changed shape."""
    w, h = size
    ih, iw = img.shape[:2]
    if (iw, ih) == (w, h):
        return img
    scale = min(w / iw, h / ih)
    nw, nh = max(1, int(iw * scale)), max(1, int(ih * scale))
    out = np.zeros((h, w, 3), np.uint8)
    x, y = (w - nw) // 2, (h - nh) // 2
    out[y:y + nh, x:x + nw] = cv2.resize(img, (nw, nh),
                                         interpolation=cv2.INTER_AREA)
    return out


def _placeholder(size, text):
    w, h = size
    img = np.full((h, w, 3), 32, np.uint8)
    cv2.putText(img, text, (24, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (200, 200, 200), 1, cv2.LINE_AA)
    return img


def _fmt_clock(sec: float) -> str:
    sign = "-" if sec < 0 else "+"
    sec = abs(sec)
    m = int(sec // 60)
    return f"{sign}{m:02d}:{sec - 60 * m:04.1f}"


def _with_strip(img, left: str, right: str, warn: str | None):
    """Camera image with a caption strip under it (title, clocks, warning).
    Under, not over: the localizer's own HUD already uses the image."""
    h, w = img.shape[:2]
    out = np.empty((h + STRIP_PX, w, 3), np.uint8)
    out[:h] = img
    out[h:] = (24, 22, 20)
    font, scale, y = cv2.FONT_HERSHEY_SIMPLEX, 0.5, h + 19
    # Hershey fonts are ASCII-only; the title's middle dots would print as ??
    left = left.replace("·", "|").encode("ascii", "replace").decode()
    cv2.putText(out, left[:80], (8, y), font, scale, (235, 235, 235), 1,
                cv2.LINE_AA)
    (tw, _), _ = cv2.getTextSize(right, font, scale, 1)
    cv2.putText(out, right, (w - tw - 8, y), font, scale, (235, 235, 235), 1,
                cv2.LINE_AA)
    if warn:
        (ww, _), _ = cv2.getTextSize(warn, font, scale, 2)
        cv2.putText(out, warn, ((w - ww) // 2, y), font, scale,
                    (60, 60, 255), 2, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# recorder


class _Session:
    def __init__(self, video_id, run_id, title, base_path, started_by,
                 started_local):
        self.video_id = video_id
        self.run_id = run_id
        self.title = title
        self.base_path = base_path
        self.started_by = started_by
        self.started_local = started_local
        self.t0 = time.monotonic()
        self.stop_evt = threading.Event()
        self.stop_at: float | None = None
        self.stop_reason: str | None = None
        self.offset_sec: float | None = None
        self.offset_source: str | None = None
        self.frames = 0
        self.camera_frames = 0
        self.stall_count = 0
        self.max_stall_sec = 0.0
        self.size: tuple[int, int] | None = None
        self.writer: _CvWriter | None = None
        self.error: str | None = None
        self.last_frame_at: float | None = None
        self.thread: threading.Thread | None = None
        self.reader = None
        # The second camera's half of this recording.
        self.second_file: str | None = None
        self.second_offset: float | None = None
        self.second_error: str | None = None
        self.second_state = "off"        # off | starting | recording | stopped | failed


class VideoRecorder:
    """One camera, one recording at a time. Thread-safe: the public
    methods are called from Flask request threads."""

    def __init__(self, db_path: str, video_dir: str,
                 stream_url: str = DEFAULT_STREAM_URL, fps: float = FPS,
                 tail_sec: float = TAIL_SEC, max_sec: float = MAX_SEC,
                 first_frame_wait_sec: float = FIRST_FRAME_WAIT_SEC,
                 source_factory=None, writer_factory=None,
                 second_camera: SecondCamera | None = None):
        self.db_path = db_path
        self.video_dir = video_dir
        self.stream_url = stream_url
        self.fps = float(fps)
        self.tail_sec = float(tail_sec)
        self.max_sec = float(max_sec)
        self.first_frame_wait_sec = float(first_frame_wait_sec)
        self._source_factory = source_factory or (
            lambda url: MjpegReader(url).start())
        self._writer_factory = writer_factory or open_writer
        self.second_camera = second_camera
        self._lock = threading.Lock()
        self._cur: _Session | None = None
        self._labels: dict[str, str] = {}
        self._plans: dict[str, str] = {}     # run_id -> saved plan it ran
        self._armed: dict | None = None    # the last mission sent
        self._last: dict | None = None
        os.makedirs(video_dir, exist_ok=True)
        self._settings_path = os.path.join(video_dir, "recorder_settings.json")
        settings = self._load_settings()
        self.auto = bool(settings.get("auto", True))
        self.second_on = bool(settings.get("second", True))
        self._recover_interrupted()

    @classmethod
    def for_app(cls, db_path: str) -> "VideoRecorder":
        """The production recorder: videos next to the metrics database."""
        return cls(db_path,
                   os.path.join(os.path.dirname(os.path.abspath(db_path)),
                                "videos"),
                   stream_url=os.environ.get("AGV_STREAM_URL",
                                             DEFAULT_STREAM_URL),
                   second_camera=SecondCamera(url) if (url := os.environ.get(
                       "AGV_SECOND_CAMERA_URL", DEFAULT_SECOND_CAMERA_URL))
                   else None)

    # ---- settings / housekeeping ---------------------------------------

    def _load_settings(self) -> dict:
        try:
            with open(self._settings_path, "r", encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {}

    def _save_settings(self) -> None:
        try:
            with open(self._settings_path, "w", encoding="utf-8") as fh:
                json.dump({"auto": self.auto, "second": self.second_on}, fh)
        except OSError:
            pass

    def set_auto(self, on: bool) -> None:
        self.auto = bool(on)
        self._save_settings()

    def set_second(self, on: bool) -> None:
        self.second_on = bool(on)
        self._save_settings()

    def _db(self):
        return database.connect(self.db_path)

    def _recover_interrupted(self) -> None:
        """A row still 'recording' at startup belongs to a process that died
        mid-run. Its MP4 has no index (Media Foundation writes it on close),
        so it is marked interrupted rather than offered as playable."""
        conn = self._db()
        try:
            rows = conn.execute("SELECT video_id, file_name FROM run_videos "
                                "WHERE status = 'recording'").fetchall()
            for row in rows:
                path = self.find_file(row["file_name"])
                database.update_video(conn, row["video_id"], {
                    "status": "interrupted",
                    "error": "the app stopped while this was recording",
                    "bytes": os.path.getsize(path) if path else None})
        finally:
            conn.close()

    def find_file(self, file_name: str | None) -> str | None:
        """Absolute path of a stored video, or None if it is missing or the
        name would escape the video folder. A row created at start holds
        the base name; the encoder adds the extension."""
        if not file_name:
            return None
        root = os.path.realpath(self.video_dir)
        for name in (file_name, file_name + ".mp4", file_name + ".avi"):
            path = os.path.realpath(os.path.join(root, name))
            if os.path.dirname(path) == root and os.path.isfile(path):
                return path
        return None

    # ---- triggers -------------------------------------------------------

    def arm(self, run_id: str, label: str | None,
            plan_file: str | None = None) -> None:
        """A mission was sent: remember its video name (typed, or else the
        saved plan it was loaded from), and close out a recording still
        waiting on an earlier run's terminal event."""
        with self._lock:
            if label and label.strip():
                self._labels[run_id] = label.strip()
            if plan_file and plan_stem(plan_file):
                self._plans[run_id] = plan_stem(plan_file)
            _title, core, _stamped = self._names(
                self._labels.get(run_id), self._plans.get(run_id), {}, run_id,
                datetime.now())
            self._armed = {"run_id": run_id,
                           "label": self._labels.get(run_id),
                           "plan": self._plans.get(run_id),
                           "next_name": core}
            cur = self._cur
            stale = (cur is not None and cur.started_by == "auto"
                     and cur.run_id != run_id)
        if stale:
            self.stop("next mission sent")

    @staticmethod
    def _anchor(cur: _Session, source: str, lag_sec: float) -> None:
        """Set the video time at which the run's elapsed clock reads 0.
        The supervisor's RUN_STARTED is the clock elapsed_ms counts from,
        so it replaces a browser anchor but never the reverse. Negative is
        legitimate: the run started just before the recording did."""
        if cur.offset_source == "supervisor" and source != "supervisor":
            return
        cur.offset_sec = round(time.monotonic() - cur.t0
                               - max(0.0, lag_sec), 3)
        cur.offset_source = source

    def on_run_started(self, run_id: str, source: str,
                       lag_sec: float = 0.0) -> None:
        with self._lock:
            cur = self._cur
            if cur is not None and (cur.run_id == run_id or
                                    (cur.started_by == "manual"
                                     and cur.run_id is None)):
                # Already recording this run (or a manual recording waiting
                # for one): only the time anchor improves.
                attach = cur.run_id is None
                cur.run_id = run_id
                self._anchor(cur, source, lag_sec)
            elif cur is not None and cur.started_by == "manual":
                return          # the user is recording by hand; leave it
            elif not self.auto:
                return
            else:
                attach = None
        if attach is not None:
            if attach:
                self._attach_run(cur)
            return
        if cur is not None:
            self.stop(f"run {run_id[:8]} started", wait=True)
        try:
            self.start(run_id=run_id, started_by="auto",
                       offset_source=source, lag_sec=lag_sec)
        except RuntimeError:
            # The other trigger (browser vs supervisor) won the race and is
            # already recording this run; still take this one's anchor.
            with self._lock:
                if self._cur is not None and self._cur.run_id == run_id:
                    self._anchor(self._cur, source, lag_sec)

    def on_run_ended(self, run_id: str, reason: str) -> None:
        with self._lock:
            cur = self._cur
            if cur is None or cur.run_id != run_id \
                    or cur.started_by != "auto":
                return
            if cur.stop_at is None:
                cur.stop_at = time.monotonic() + self.tail_sec
                cur.stop_reason = reason

    # ---- manual control -------------------------------------------------

    def start(self, run_id: str | None = None, label: str | None = None,
              started_by: str = "manual", offset_source: str | None = None,
              lag_sec: float = 0.0) -> dict:
        if cv2 is None and self._writer_factory is open_writer:
            raise RuntimeError("OpenCV (cv2) is not installed for the "
                               "Python running app.py")
        started = datetime.now()
        conn = self._db()
        try:
            run = dict(database.get_run(conn, run_id) or {}) if run_id else {}
            with self._lock:
                if self._cur is not None:
                    raise RuntimeError("already recording "
                                       f"'{self._cur.title}'")
                label = label or (self._labels.pop(run_id, None)
                                  if run_id else None)
                plan = self._plans.pop(run_id, None) if run_id else None
                if run_id and self._armed \
                        and self._armed["run_id"] == run_id:
                    self._armed = None
                title, core, _stamped = self._names(label, plan, run, run_id,
                                                    started)
                base = self._unique_base(f"{core}_{MAIN_TAG}")
                video_id = database.create_video(conn, {
                    "run_id": run_id or None, "title": title,
                    "file_name": base, "started_by": started_by,
                    "fps": self.fps, "stream_url": self.stream_url})
                s = _Session(video_id, run_id, title,
                             os.path.join(self.video_dir, base), started_by,
                             started)
                if offset_source:
                    s.offset_sec = -round(max(0.0, lag_sec), 3)
                    s.offset_source = offset_source
                s.reader = self._source_factory(self.stream_url)
                s.thread = threading.Thread(target=self._loop, args=(s,),
                                            daemon=True,
                                            name=f"video-{video_id}")
                self._cur = s
                s.thread.start()
                if self.second_camera is not None and self.second_on:
                    s.second_state = "starting"
                    # The second camera gets this file's exact name minus the
                    # camera tag, so the pair always matches.
                    pair = base.rsplit(f"_{MAIN_TAG}", 1)[0]
                    threading.Thread(target=self._second_start,
                                     args=(s, "", pair), daemon=True).start()
        finally:
            conn.close()
        return self.status()

    def _second_start(self, s: _Session, stamp: str, core: str = "") -> None:
        """Start the second camera with this recording's name and date-time
        prefix, and work out where its video sits on this one's clock.

        Its reply says how long it has been recording (elapsed_sec) and how
        long its first frame took (first_frame_sec, the second file's 0:00).
        At the moment the reply arrives this video is at t_here, so the
        second video's 0:00 plays at t_here - (elapsed - first_frame) of
        this one. Error: about half a LAN round trip plus a frame or two;
        the Recordings page has a nudge for the rest."""
        try:
            out = self.second_camera.start(s.title, stamp, core)
            t_here = time.monotonic() - s.t0
            rec = out.get("recording") or {}
            s.second_file = rec.get("file")
            elapsed = float(rec.get("elapsed_sec") or 0.0)
            first = rec.get("first_frame_sec")
            s.second_offset = round(
                t_here - (elapsed - float(first if first is not None else 0.0)), 3)
            s.second_state = "recording"
            stopped = s.stop_evt.is_set() or (s.thread and not s.thread.is_alive())
        except Exception as exc:          # never let it touch the main video
            s.second_error = str(exc)
            s.second_state = "failed"
            return
        if stopped:                       # the run ended while it started
            self._second_stop(s)

    def _second_stop(self, s: _Session) -> None:
        if s.second_state != "recording" or not s.second_file:
            return
        try:
            self.second_camera.stop(s.second_file)
            s.second_state = "stopped"
        except Exception as exc:
            s.second_error = f"stop failed: {exc}"
            s.second_state = "failed"

    def stop(self, reason: str = "stopped by hand", wait: bool = False) -> dict:
        with self._lock:
            s = self._cur
            if s is not None:
                s.stop_reason = s.stop_reason or reason
                s.stop_evt.set()
        if s is not None and wait and s.thread is not None:
            s.thread.join(timeout=15)
        return self.status()

    def shutdown(self) -> None:
        self.stop("app shutting down", wait=True)

    def _attach_run(self, s: _Session) -> None:
        conn = self._db()
        try:
            database.update_video(conn, s.video_id, {"run_id": s.run_id})
        finally:
            conn.close()

    def _names(self, label, plan, run, run_id, started):
        """(title, file core, stamped) for a new recording.

        * A saved plan (e.g. DOE_6r_cbs.json) and nothing typed: the plan's
          name plus the next repeat number -- DOE_6r_cbs_1, DOE_6r_cbs_2, ...
          -- so a run sheet row maps to a video with nothing to type.
        * A typed name: used as typed; a repeat gets _2, _3, ...
        * Neither: date-time, robot count and planner, as before."""
        if label and slug(label):
            core = slug(label)
            if self._taken(core):
                core = f"{core}_{self._next_number(core, start=2)}"
            return label, core, False
        if plan and slug(plan):
            core = f"{slug(plan)}_{self._next_number(slug(plan))}"
            return core, core, False
        title, base = video_names(started, None, run, run_id)
        return title, base, True

    def _existing_cores(self) -> set:
        """File cores (name minus _main and extension) already used, from
        the index and the folder both."""
        names = set(os.listdir(self.video_dir))
        conn = self._db()
        try:
            names |= {r["file_name"] for r in
                      conn.execute("SELECT file_name FROM run_videos")}
        finally:
            conn.close()
        cores = set()
        for n in names:
            m = re.match(rf"^(.*)_{MAIN_TAG}(?:\.\w+)?$", n or "")
            if m:
                cores.add(m.group(1))
        return cores

    def _taken(self, core: str) -> bool:
        return core in self._existing_cores()

    def _next_number(self, prefix: str, start: int = 1) -> int:
        nums = [int(m.group(1)) for c in self._existing_cores()
                if (m := re.fullmatch(rf"{re.escape(prefix)}_(\d+)", c))]
        return max([start - 1, *nums]) + 1

    def _unique_base(self, base: str) -> str:
        taken = set(os.listdir(self.video_dir))
        cand, i = base, 2
        while any(n == cand or n.startswith(cand + ".") for n in taken):
            cand, i = f"{base}-{i}", i + 1
        return cand

    # ---- status ---------------------------------------------------------

    def status(self) -> dict:
        try:
            free_gb = round(shutil.disk_usage(self.video_dir).free / 1e9, 1)
        except OSError:
            free_gb = None
        out = {"available": cv2 is not None
               or self._writer_factory is not open_writer,
               "auto": self.auto, "stream_url": self.stream_url,
               "video_dir": os.path.abspath(self.video_dir),
               "disk_free_gb": free_gb, "fps": self.fps,
               "recording": None, "last": self._last,
               "armed": self._armed,
               "second": {"configured": self.second_camera is not None,
                          "on": self.second_on,
                          "url": self.second_camera.url
                          if self.second_camera else None}}
        with self._lock:
            s = self._cur
            if s is not None:
                now = time.monotonic()
                age = None if s.last_frame_at is None \
                    else round(now - s.last_frame_at, 1)
                out["recording"] = {
                    "video_id": s.video_id, "run_id": s.run_id,
                    "title": s.title, "started_by": s.started_by,
                    "elapsed_sec": round(now - s.t0, 1),
                    "frames": s.frames, "camera_frames": s.camera_frames,
                    "last_frame_age_sec": age,
                    "stalled": age is None or age > STALL_SEC,
                    "stream_error": getattr(s.reader, "error", None),
                    "stopping_in_sec": None if s.stop_at is None
                    else round(max(0.0, s.stop_at - now), 1),
                    "offset_sec": s.offset_sec,
                    "second_state": s.second_state,
                    "second_file": s.second_file,
                    "second_error": s.second_error,
                }
        return out

    # ---- the recording thread -------------------------------------------

    def _loop(self, s: _Session) -> None:
        fps = self.fps
        cached_seq, base_img = -1, None
        first_at = None
        stall_from = None
        try:
            while True:
                now = time.monotonic()
                if s.stop_evt.is_set():
                    break
                if s.stop_at is not None and now >= s.stop_at:
                    break
                if now - s.t0 >= self.max_sec:
                    s.stop_reason = s.stop_reason or (
                        f"reached the {self.max_sec / 60:.0f} min cap")
                    break
                jpeg, seq, at = s.reader.latest()
                img = None
                if jpeg is not None and seq != cached_seq:
                    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8),
                                       cv2.IMREAD_COLOR)
                    cached_seq = seq
                if s.writer is None:
                    if img is None and now - s.t0 < self.first_frame_wait_sec:
                        s.stop_evt.wait(0.02)
                        continue
                    if img is not None:
                        h, w = img.shape[:2]
                        s.size = (w - w % 2, h - h % 2)
                    else:
                        s.size = DEFAULT_SIZE
                    s.writer = self._writer_factory(
                        s.base_path, fps, (s.size[0], s.size[1] + STRIP_PX))
                if img is not None:
                    base_img = _fit(img, s.size)
                    s.camera_frames += 1
                    s.last_frame_at = at
                    if first_at is None:
                        first_at = at
                # A stall episode is a gap longer than STALL_SEC; the count
                # and the longest one go in the row.
                age = None if s.last_frame_at is None else now - s.last_frame_at
                stalled = age is not None and age > STALL_SEC
                if stalled:
                    if stall_from is None:
                        stall_from = s.last_frame_at
                        s.stall_count += 1
                    s.max_stall_sec = max(s.max_stall_sec, now - stall_from)
                else:
                    stall_from = None
                due = int((now - s.t0) * fps) + 1
                while s.frames < due:
                    t_video = s.frames / fps
                    if base_img is None or s.t0 + t_video < first_at:
                        frame = _placeholder(
                            s.size, "Waiting for the camera stream "
                            f"({self.stream_url})")
                        warn = "NO CAMERA FRAMES YET"
                    else:
                        frame = base_img
                        warn = "CAMERA STREAM STALLED" if stalled else None
                    wall = (s.started_local
                            + timedelta(seconds=t_video)).strftime("%H:%M:%S")
                    clock = f"run {_fmt_clock(t_video - s.offset_sec)}" \
                        if s.offset_sec is not None \
                        else f"video {_fmt_clock(t_video)}"
                    s.writer.write(_with_strip(frame, s.title,
                                               f"{wall}   {clock}", warn))
                    s.frames += 1
                s.stop_evt.wait(max(0.0, s.t0 + s.frames / fps
                                    - time.monotonic()))
        except Exception as exc:
            s.error = f"{type(exc).__name__}: {exc}"
        finally:
            # Both cameras stop together, before this side closes its file.
            threading.Thread(target=self._second_stop, args=(s,),
                             daemon=True).start()
            try:
                s.reader.stop()
            except Exception:
                pass
            if s.writer is not None:
                try:
                    s.writer.release()
                except Exception as exc:
                    s.error = s.error or f"encoder close failed: {exc}"
            self._finish(s)

    def _finish(self, s: _Session) -> None:
        path = s.writer.path if s.writer is not None else None
        size = os.path.getsize(path) if path and os.path.exists(path) else 0
        status, error = "ready", s.error
        if s.camera_frames == 0:
            # Nothing but "waiting" cards: not a recording of anything.
            status = "failed"
            error = error or ("no frames arrived from the camera stream "
                              f"at {self.stream_url} -- is the localizer "
                              "running with --stream?")
            if path and os.path.exists(path):
                os.remove(path)
            size = 0
        elif error or size == 0:
            status = "failed"
            error = error or "the encoder wrote an empty file"
        patch = {
            "status": status, "ended_at_utc": database.utc_now_iso(),
            "duration_sec": round(s.frames / self.fps, 2),
            "frame_count": s.frames, "camera_frames": s.camera_frames,
            "stall_count": s.stall_count,
            "max_stall_sec": round(s.max_stall_sec, 1),
            "width": s.size[0] if s.size else None,
            "height": s.size[1] + STRIP_PX if s.size else None,
            "codec": s.writer.codec if s.writer else None,
            "browser_playable": int(bool(s.writer and
                                         s.writer.browser_playable)),
            "bytes": size, "run_t0_offset_sec": s.offset_sec,
            "offset_source": s.offset_source, "stop_reason": s.stop_reason,
            "error": error, "run_id": s.run_id,
            "second_camera_url": self.second_camera.url
            if self.second_camera and s.second_state != "off" else None,
            "second_camera_file": s.second_file,
            "second_offset_sec": s.second_offset,
            "second_error": s.second_error,
        }
        if path and status == "ready":
            patch["file_name"] = os.path.basename(path)
        try:
            conn = self._db()
            try:
                database.update_video(conn, s.video_id, patch)
            finally:
                conn.close()
        finally:
            with self._lock:
                if self._cur is s:
                    self._cur = None
                self._last = {"video_id": s.video_id, "title": s.title,
                              "status": status, "error": error,
                              "duration_sec": patch["duration_sec"]}
