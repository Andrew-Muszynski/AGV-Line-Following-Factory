#!/usr/bin/env python3
"""
ollama_advisor.py — LLM advisor backend using a local Ollama server.

Free, fully local replacement for haiku_advisor.py (the Anthropic API bills
separately from a Claude subscription, so it is not used). Same contract:
propose() returns {'dwell_ms': int, 'rationale': str} or raises, and whatever
comes back is re-validated by the deterministic gate in dispatch_advisor.py
before it can reach a robot — this module makes no safety decisions.

Setup on the machine running fleetSupervisor.py (one time):

    curl -fsSL https://ollama.com/install.sh | sh
    ollama pull llama3.2:3b

Ollama then runs as a systemd service on http://127.0.0.1:11434. Override
with OLLAMA_URL / OLLAMA_MODEL environment variables if needed (llama3.2:1b
is the fallback if 3b responds too slowly on the laptop's CPU).

Pure stdlib (urllib) — no extra pip packages in the ROS venv.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Optional

DEFAULT_URL = "http://127.0.0.1:11434"
# gemma4:e2b-it-qat, not llama3.2:3b (the default through 2026-09-23). The
# e2b variant rather than e4b because this model sits INSIDE the dispatch
# loop -- it is consulted on a live conflict while robots hold their
# reservations, so per-call latency is a cost paid by the fleet, not just by
# the caller. Set OLLAMA_MODEL to override per run.
DEFAULT_MODEL = "gemma4:e2b-it-qat"
# 2026-07-18 hardware run: CPU inference on the supervisor laptop took 21-32s
# per consultation and starved the micro-ROS agent — robots dropped their
# sessions mid-mission. Host Ollama on the GPU laptop instead
# (OLLAMA_URL=http://<gpu-laptop>:11434 -- 192.168.0.162 in the lab. The
# address changes whenever that laptop moves network, so confirm with
# `ipconfig` rather than trusting this comment.)
# and cap each call so a slow backend can only cost one bounded stall; the
# supervisor's fallback hold covers a timeout exactly like a gate rejection.
# Override: OLLAMA_TIMEOUT_SEC.
REQUEST_TIMEOUT_SEC = float(os.environ.get("OLLAMA_TIMEOUT_SEC", "10"))
WARMUP_TIMEOUT_SEC = 90.0    # first load pulls the model off disk into RAM
KEEP_ALIVE = "30m"           # keep the model resident for the whole mission

# Bare JSON schema (Ollama's `format` field) — same shape haiku_advisor used.
OUTPUT_SCHEMA = {
    "type": "object",
    "properties": {
        "dwell_ms": {
            "type": "integer",
            "description": "Must exactly equal one dwell_ms in the supplied "
                           "deterministically safe candidate_dwells list.",
        },
        "rationale": {
            "type": "string",
            "description": "One or two sentences citing the tradeoff you used — "
                           "e.g. lowest fleet_makespan_sec, or a small "
                           "extra_fleet_sec_vs_best to reach later_holds_for_me "
                           "= 0.",
        },
    },
    "required": ["dwell_ms", "rationale"],
    "additionalProperties": False,
}

SYSTEM_PROMPT = (
    "You advise a fleet supervisor for small AGVs on an 8x8 grid. One robot "
    "is idle and its immediate movement is predicted to conflict with "
    "another robot. Resource ownership and temporal priority have already "
    "been decided deterministically and cannot be overridden. Every candidate "
    "you are given is already collision-safe, so your job is the TRADEOFF: "
    "pick the wait horizon that keeps the whole fleet moving best. Read each "
    "candidate's fleet_makespan_sec (lower = less total delay), "
    "extra_fleet_sec_vs_best, and later_holds_for_me (future stops this robot "
    "still faces). Prefer the lowest makespan; but if paying a small "
    "extra_fleet_sec_vs_best drops later_holds_for_me to 0, prefer that — one "
    "slightly longer wait beats two stop-and-go halts. Select exactly one "
    "listed dwell_ms with conflict_free true; never invent a duration. The "
    "deterministic gate re-evaluates before movement. Respond with JSON only."
)


class OllamaAdvisor:
    """propose() returns {'dwell_ms': int, 'rationale': str} or raises."""

    def __init__(self, model: str | None = None, url: str | None = None):
        self.url = (url or os.environ.get("OLLAMA_URL") or DEFAULT_URL).rstrip("/")
        self.model = model or os.environ.get("OLLAMA_MODEL") or DEFAULT_MODEL
        self._check_server()
        self._warm_up()

    # -- setup checks (fail loudly at construction, not mid-run) ------------

    def _check_server(self) -> None:
        try:
            with urllib.request.urlopen(f"{self.url}/api/tags", timeout=5) as resp:
                tags = json.loads(resp.read())
        except (urllib.error.URLError, OSError) as exc:
            raise RuntimeError(
                f"Ollama not reachable at {self.url} — install it with "
                f"'curl -fsSL https://ollama.com/install.sh | sh' and pull the "
                f"model with 'ollama pull {self.model}'") from exc
        names = [m.get("name", "") for m in tags.get("models", [])]
        if not any(n == self.model or n.startswith(self.model + ":") for n in names):
            raise RuntimeError(
                f"model '{self.model}' not found on the Ollama server at "
                f"{self.url} (have: {names or 'none'}) — run "
                f"'ollama pull {self.model}'")

    def _warm_up(self) -> None:
        # An empty /api/generate call loads the model into RAM so the first
        # real conflict consultation doesn't pay the disk-load latency.
        self._post("/api/generate",
                   {"model": self.model, "keep_alive": KEEP_ALIVE},
                   timeout=WARMUP_TIMEOUT_SEC)

    # -- advisor interface ---------------------------------------------------

    def propose(self, payload: dict, rejection: Optional[str] = None,
                prior: Optional[dict] = None) -> dict:
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": json.dumps(payload)},
        ]
        if rejection and prior:
            # Second attempt: show the model its rejected proposal and the
            # deterministic gate's reason.
            messages.append({"role": "assistant", "content": json.dumps(prior)})
            messages.append({"role": "user", "content": json.dumps({
                "rejected": True,
                "gate_reason": rejection,
                "instruction": "Your previous dwell_ms was rejected by the "
                               "deterministic collision check. Propose a "
                               "different value.",
            })})

        resp = self._post("/api/chat", {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "format": OUTPUT_SCHEMA,  # schema-constrained decoding
            "options": {"temperature": 0.2, "num_predict": 250},
            "keep_alive": KEEP_ALIVE,
        }, timeout=REQUEST_TIMEOUT_SEC)

        proposal = json.loads(resp["message"]["content"])
        dwell_ms = proposal.get("dwell_ms")
        if not isinstance(dwell_ms, int) or isinstance(dwell_ms, bool):
            raise ValueError("Ollama dwell_ms must be an integer")
        allowed = {
            candidate.get("dwell_ms")
            for candidate in payload.get("candidate_dwells", [])
            if candidate.get("conflict_free", True)
        }
        if not allowed:
            raise ValueError("Ollama received no deterministically safe candidates")
        if dwell_ms not in allowed:
            raise ValueError(
                f"Ollama selected {dwell_ms}, not a safe candidate "
                f"{sorted(allowed)}")
        return {
            "dwell_ms": dwell_ms,
            "rationale": str(proposal.get("rationale", "")),
        }

    def break_tie(self, payload: dict) -> Optional[str]:
        """Choose which of two genuinely-tied robots goes first.

        Returns one of payload['allowed_owners'] or None. The advisor validates
        the answer and caches it per pair, so a wrong/None answer is safe: it
        just falls back to the deterministic sort. Never raises out."""
        allowed = list(payload.get("allowed_owners", []))
        if len(allowed) != 2:
            return None
        schema = {
            "type": "object",
            "properties": {
                "owner": {"type": "string", "enum": allowed,
                          "description": "Which robot goes first."},
                "rationale": {"type": "string",
                              "description": "One sentence: why this robot."},
            },
            "required": ["owner"],
            "additionalProperties": False,
        }
        system = (
            "You advise a fleet supervisor for small AGVs. Two robots contend "
            "for the same resource at nearly the same instant and no rule "
            "separates them. Pick which robot goes FIRST to keep the whole "
            "fleet moving — generally the one with more remaining work, so the "
            "longer path is not delayed. Answer with one of the two allowed "
            "owner names only. Respond with JSON only.")
        try:
            resp = self._post("/api/chat", {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": system},
                    {"role": "user", "content": json.dumps(payload)},
                ],
                "stream": False,
                "format": schema,
                "options": {"temperature": 0.2, "num_predict": 120},
                "keep_alive": KEEP_ALIVE,
            }, timeout=REQUEST_TIMEOUT_SEC)
            owner = json.loads(resp["message"]["content"]).get("owner")
        except (urllib.error.URLError, OSError, ValueError, KeyError):
            return None
        return owner if owner in allowed else None

    # -- transport -----------------------------------------------------------

    def _post(self, path: str, body: dict, timeout: float) -> dict:
        req = urllib.request.Request(
            f"{self.url}{path}",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read())
