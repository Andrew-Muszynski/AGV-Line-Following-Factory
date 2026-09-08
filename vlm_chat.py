"""VLM chat backend: answer questions about the live testbed.

DESIGN
------
Every question is answered from THREE things assembled server-side at ask
time:

  1. the CURRENT camera frame, pulled fresh from apriltag_localize.py's
     MJPEG preview stream
  2. the CURRENT measured state, sent up by the browser (it already holds
     exactly what the operator is looking at, via rosbridge)
  3. the recent conversation, as text

Fresh frame and fresh state on every turn, deliberately. A cached frame
would answer about the past, and on a testbed where a robot can leave the
table in under a second that is worse than not answering.

WHY THE BROWSER SUPPLIES THE STATE
----------------------------------
Flask has the metrics database but no live ROS connection; the browser
already has one. Sending state up with the question keeps this process free
of a rclpy/rosbridge dependency and guarantees the model is reasoning about
the same numbers the operator can see on screen.

WHY THE SERVER FETCHES THE FRAME
--------------------------------
The browser cannot: the MJPEG stream is a different origin with no CORS
headers, so drawing <img id="live-feed"> to a canvas taints it and
toDataURL() throws. Fetching server-side sidesteps that entirely.

MEASURED vs OBSERVED
--------------------
The system prompt tells the model that positions and headings given to it
are MEASURED to a fraction of a degree and must never be contradicted from
the image -- it is there to describe what the numbers do not cover. A model
"correcting" an AprilTag reading by eyeballing a photo is the failure mode
that would make this untrustworthy, so it is ruled out in the prompt and
surfaced in the UI.

CONFIG (environment)
    VLM_OLLAMA_URL   default http://192.168.0.162:11434
    VLM_MODEL        default qwen3.5:4b
    VLM_STREAM_URL   default http://127.0.0.1:8081/stream
"""
from __future__ import annotations

import base64
import json
import os
import time
import urllib.request

DEFAULT_OLLAMA_URL = "http://192.168.0.162:11434"
DEFAULT_MODEL = "qwen3.5:4b"
# Port 8081 matches apriltag_localize.py --stream and the URL the page
# itself uses for #live-feed (see cameraStreamUrl() in solver.js).
DEFAULT_STREAM_URL = "http://127.0.0.1:8081/stream"

# A frame older than this is refetched. Several questions in a row about the
# same moment should not each pay a stream connection, but anything beyond a
# second or so on this testbed is a different moment.
FRAME_CACHE_SEC = 1.0

SYSTEM_PROMPT = """You are a monitoring assistant for an AGV robot testbed: \
Arduino Alvik robots on a taped 8x8 grid with 49 workstation bays, tracked \
by an overhead camera using AprilTags.

You are given (a) the live annotated camera view and (b) the system's own \
MEASURED state.

Rules:
- The measured state is ground truth. Positions come from AprilTag tracking \
and are accurate to a fraction of an inch and under a degree. NEVER \
contradict a measured number based on how the image looks, and never \
re-estimate a value you were given.
- Use the image for what the numbers do not cover: obstructions, anything \
out of place, whether a robot looks physically wrong, what the scene looks \
like overall.
- If the answer is not in the state and not visible, say so plainly. Do not \
guess.
- Be brief. One or two sentences unless asked for detail.
- Coordinates: +x is east (right), +y is north (up in the image). Yaw \
0=south, 90=east, 180=north, 270=west."""


class FrameSource:
    """Pulls single JPEGs out of the MJPEG preview stream, with a short
    cache so a burst of questions does not reconnect per question."""

    def __init__(self, url: str):
        self.url = url
        self._jpeg: bytes | None = None
        self._at = 0.0
        self._error: str | None = None

    def get(self, max_age_sec: float = FRAME_CACHE_SEC
            ) -> tuple[bytes | None, str | None]:
        if self._jpeg is not None and time.monotonic() - self._at <= max_age_sec:
            return self._jpeg, None
        try:
            jpeg = _grab_jpeg(self.url)
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            return None, self._error
        self._jpeg, self._at, self._error = jpeg, time.monotonic(), None
        return jpeg, None


def _grab_jpeg(stream_url: str, timeout: float = 8.0) -> bytes:
    """One complete JPEG from a multipart MJPEG stream."""
    with urllib.request.urlopen(stream_url, timeout=timeout) as resp:
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = resp.read(4096)
            if not chunk:
                break
            buf += chunk
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start + 2)
            if start != -1 and end != -1:
                return buf[start:end + 2]
            if len(buf) > 8_000_000:
                raise RuntimeError("no JPEG boundary in 8MB of stream")
    raise RuntimeError(f"no complete JPEG within {timeout:.0f}s")


def state_text(state: dict) -> str:
    """Render the browser's state snapshot as prose for the prompt.

    Tolerant by design: the browser sends what it has, and a partially
    connected fleet is normal. Missing pieces are omitted rather than
    reported as zero -- the model must not be told there are 0 robots when
    the truth is that nothing has been received yet."""
    lines: list[str] = []
    robots = state.get("robots") or {}
    if robots:
        lines.append(f"Robots currently tracked: {len(robots)}")
        for name in sorted(robots):
            r = robots[name] or {}
            bits = []
            if r.get("x_in") is not None and r.get("y_in") is not None:
                bits.append(f"x={float(r['x_in']):.1f}in, "
                            f"y={float(r['y_in']):.1f}in")
            if r.get("yaw_deg") is not None:
                bits.append(f"yaw={float(r['yaw_deg']):.1f}deg")
            if r.get("battery_pct") is not None:
                bits.append(f"battery={r['battery_pct']}%")
            if r.get("source"):
                bits.append(str(r["source"]))
            lines.append(f"- {name}: " + (", ".join(bits) or "no pose yet"))
    else:
        lines.append("No robot poses have been received yet.")

    run = state.get("run") or {}
    if run:
        parts = [f"{k}={v}" for k, v in sorted(run.items()) if v is not None]
        if parts:
            lines.append("Run: " + ", ".join(parts))
    events = state.get("recent_events") or []
    if events:
        lines.append("Recent supervisor events (newest last):")
        for e in events[-8:]:
            lines.append(f"- {e}")
    return "\n".join(lines)


class VlmChat:
    def __init__(self, ollama_url: str | None = None, model: str | None = None,
                 stream_url: str | None = None):
        self.url = (ollama_url or os.environ.get("VLM_OLLAMA_URL")
                    or DEFAULT_OLLAMA_URL).rstrip("/")
        self.model = model or os.environ.get("VLM_MODEL") or DEFAULT_MODEL
        self.frames = FrameSource(stream_url
                                  or os.environ.get("VLM_STREAM_URL")
                                  or DEFAULT_STREAM_URL)

    # ---- status ---------------------------------------------------------
    def status(self) -> dict:
        """Everything the UI needs to explain itself when something is off:
        which model, whether Ollama has it, whether the camera is reachable."""
        out = {"model": self.model, "ollama_url": self.url,
               "stream_url": self.frames.url, "ollama_ok": False,
               "model_present": False, "camera_ok": False, "error": None}
        try:
            with urllib.request.urlopen(f"{self.url}/api/tags",
                                        timeout=5) as resp:
                tags = json.loads(resp.read())
            out["ollama_ok"] = True
            names = [m.get("name", "") for m in tags.get("models", [])]
            out["available_models"] = sorted(names)
            out["model_present"] = any(
                n == self.model or n.startswith(self.model + ":")
                for n in names)
        except Exception as exc:
            out["error"] = f"Ollama unreachable: {exc}"
            return out
        jpeg, err = self.frames.get(max_age_sec=5.0)
        out["camera_ok"] = jpeg is not None
        if err:
            out["error"] = f"camera stream: {err}"
        return out

    # ---- one turn -------------------------------------------------------
    def ask(self, question: str, state: dict | None = None,
            history: list | None = None, timeout: float = 120.0) -> dict:
        state = state or {}
        history = history or []
        jpeg, frame_err = self.frames.get()

        context = state_text(state)
        if frame_err:
            # Answer anyway, but say the image is missing rather than let the
            # model imply it looked. Text-only questions still work.
            context += (f"\n\n(NOTE: the camera frame is unavailable right "
                        f"now — {frame_err}. Answer from the measured state "
                        "only, and say you cannot see the table.)")

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        for turn in history[-6:]:
            role = turn.get("role")
            content = str(turn.get("content") or "")
            if role in ("user", "assistant") and content:
                messages.append({"role": role, "content": content})
        user_msg: dict = {
            "role": "user",
            "content": f"{context}\n\nQuestion: {question}",
        }
        if jpeg is not None:
            user_msg["images"] = [base64.b64encode(jpeg).decode()]
        messages.append(user_msg)

        body = json.dumps({
            "model": self.model, "messages": messages, "stream": False,
            "keep_alive": "30m",
            "options": {"temperature": 0.2},
        }).encode()
        req = urllib.request.Request(
            f"{self.url}/api/chat", data=body,
            headers={"Content-Type": "application/json"})
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                data = json.loads(resp.read())
        except Exception as exc:
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}",
                    "model": self.model,
                    "seconds": round(time.monotonic() - t0, 2)}
        return {
            "ok": True,
            "answer": (data.get("message") or {}).get("content", "").strip(),
            "model": self.model,
            "seconds": round(time.monotonic() - t0, 2),
            "saw_frame": jpeg is not None,
            "frame_bytes": len(jpeg) if jpeg else 0,
        }
