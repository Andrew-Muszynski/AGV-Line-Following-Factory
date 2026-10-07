#!/usr/bin/env python3
"""One question, one model, full timing detail. A benchmark tells you a model
is slow; this tells you why.

Every slowdown found on this testbed so far has turned out to be output
length rather than anything about the network or the GPU:

  qwen3-vl:30b   p99 of 27s against a 1.35s median
  qwen3-vl:30b   3.5x slower with --no-image (489 output tokens vs 91)
  qwen3-vl:4b    120s timeouts locally while the GPU sat at 95% utilization

The benchmark records only wall-clock seconds, so none of those could be
diagnosed from its results. This prints the token counts and Ollama's own
phase timings, and shows whether a `thinking` field came back -- which is
how you tell a model that ignores "think": false from one that honours it.

    python tools/vlm_probe.py --model qwen3-vl:4b
    python tools/vlm_probe.py --model qwen3-vl:30b --url http://127.0.0.1:21435
    python tools/vlm_probe.py --model qwen3-vl:4b --num-predict 32
"""
from __future__ import annotations

import argparse
import base64
import glob
import json
import os
import sys
import time
import urllib.request

# Loopback, not this laptop's own LAN address -- see the note in vlm_bench.py:
# a hard-coded LAN IP silently stops resolving whenever the testbed changes
# network. Override with OLLAMA_URL or --url for a server on another host.
DEFAULT_URL = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--model", required=True)
    ap.add_argument("--url", default=DEFAULT_URL,
                    help="default: %(default)s (NOT 127.0.0.1, which on this "
                         "machine reaches a different Ollama)")
    ap.add_argument("--question",
                    default="How many robots are on the table? "
                            "Answer with a number.")
    ap.add_argument("--no-image", action="store_true")
    ap.add_argument("--num-predict", type=int, default=None,
                    help="cap output tokens. Test this here before it goes "
                         "anywhere near a benchmark run.")
    ap.add_argument("--num-ctx", type=int, default=None,
                    help="context window. Ollama picks this itself and can "
                         "pick one whose KV cache pushes the model off an "
                         "8GB card into shared system memory. Real prompts "
                         "here are about 805 tokens, so a smaller window "
                         "frees VRAM without truncating anything -- check "
                         "the answer is unchanged before believing that.")
    ap.add_argument("--think", action="store_true",
                    help="leave reasoning ON (default sends think: false)")
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()

    options: dict = {"temperature": 0.0, "seed": 42}
    if args.num_predict is not None:
        options["num_predict"] = args.num_predict
    if args.num_ctx is not None:
        options["num_ctx"] = args.num_ctx

    msg: dict = {"role": "user", "content": args.question}
    if not args.no_image:
        frames = sorted(glob.glob(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "vlm_samples", "*", "frame.jpg")))
        if not frames:
            sys.exit("no sample frames found")
        with open(frames[0], "rb") as fh:
            msg["images"] = [base64.b64encode(fh.read()).decode()]

    body = json.dumps({
        "model": args.model,
        "messages": [msg],
        "stream": False,
        "keep_alive": "30m",
        "think": bool(args.think),
        "options": options,
    }).encode()

    print(f"model    {args.model}")
    print(f"endpoint {args.url}")
    print(f"image    {'no' if args.no_image else 'yes'}"
          f"    think {bool(args.think)}"
          f"    num_predict {args.num_predict}"
          f"    num_ctx {args.num_ctx}")
    print()

    req = urllib.request.Request(
        f"{args.url}/api/chat", data=body,
        headers={"Content-Type": "application/json"})
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=args.timeout) as resp:
            out = json.loads(resp.read())
    except Exception as exc:
        # An HTTPError body is where Ollama says what it disliked about the
        # request -- exactly what was missing when --max-output-tokens failed.
        detail = ""
        if hasattr(exc, "read"):
            try:
                detail = exc.read().decode("utf-8", errors="replace").strip()
            except Exception:
                detail = ""
        print(f"FAILED after {time.monotonic() - t0:.1f}s: {exc!r}")
        if detail:
            print(f"server said: {detail}")
        sys.exit(1)

    wall = time.monotonic() - t0
    m = out.get("message") or {}
    thinking = m.get("thinking") or ""
    content = m.get("content") or ""

    def ns(key: str) -> str:
        v = out.get(key)
        return f"{v / 1e9:.2f}s" if isinstance(v, int) else "-"

    print(f"wall            {wall:.2f}s")
    print(f"  load          {ns('load_duration')}")
    print(f"  prompt eval   {ns('prompt_eval_duration')}"
          f"   {out.get('prompt_eval_count')} tokens in")
    print(f"  generation    {ns('eval_duration')}"
          f"   {out.get('eval_count')} tokens out")
    print(f"  total (server){ns('total_duration')}")
    ev, ed = out.get("eval_count"), out.get("eval_duration")
    if isinstance(ev, int) and isinstance(ed, int) and ed:
        print(f"  throughput    {ev / (ed / 1e9):.1f} tokens/s")
    print()
    print(f"thinking field  {len(thinking)} chars"
          + ("   <-- think:false NOT honoured" if thinking and not args.think
             else ""))
    if thinking:
        print(f"   {thinking[:300]!r}")
    print(f"content         {len(content)} chars")
    print(f"   {content[:300]!r}")


if __name__ == "__main__":
    main()
