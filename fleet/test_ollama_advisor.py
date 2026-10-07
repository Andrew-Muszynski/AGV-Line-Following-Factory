#!/usr/bin/env python3
"""Contract tests for ollama_advisor.py — no Ollama server needed.

Verifies the request the advisor composes (schema-constrained, non-streaming,
retry structure) and how it parses/coerces the response. The deterministic
gate in dispatch_advisor.py is what makes proposals safe; these tests only
pin the transport contract.
"""

import json
import urllib.error
import urllib.request

import pytest

from ollama_advisor import OllamaAdvisor, OUTPUT_SCHEMA, SYSTEM_PROMPT


def make_advisor():
    adv = OllamaAdvisor.__new__(OllamaAdvisor)  # skip network __init__
    adv.url = "http://test:11434"
    adv.model = "test-model"
    return adv


def test_propose_sends_schema_and_parses():
    adv = make_advisor()
    captured = {}

    def fake_post(path, body, timeout):
        captured["path"] = path
        captured["body"] = body
        return {"message": {"content": json.dumps(
            {"dwell_ms": 3400, "rationale": "shortest conflict-free"})}}

    adv._post = fake_post
    payload = {
        "conflict": "node 12",
        "candidate_dwells": [{"dwell_ms": 3400, "conflict_free": True}],
    }
    out = adv.propose(payload)

    assert out == {"dwell_ms": 3400, "rationale": "shortest conflict-free"}
    assert captured["path"] == "/api/chat"
    body = captured["body"]
    assert body["model"] == "test-model"
    assert body["stream"] is False
    assert body["format"] == OUTPUT_SCHEMA
    assert body["messages"][0] == {"role": "system", "content": SYSTEM_PROMPT}
    assert json.loads(body["messages"][1]["content"]) == payload


def test_propose_retry_includes_rejected_prior():
    adv = make_advisor()
    captured = {}

    def fake_post(path, body, timeout):
        captured["body"] = body
        return {"message": {"content": json.dumps(
            {"dwell_ms": 5200, "rationale": "longer hold"})}}

    adv._post = fake_post
    prior = {"dwell_ms": 2600, "rationale": "first try"}
    adv.propose({
        "conflict": "edge 4-5",
        "candidate_dwells": [{"dwell_ms": 5200, "conflict_free": True}],
    },
                rejection="still conflicts at node 5", prior=prior)

    msgs = captured["body"]["messages"]
    assert len(msgs) == 4  # system, payload, assistant echo, rejection
    assert msgs[2]["role"] == "assistant"
    assert json.loads(msgs[2]["content"]) == prior
    reject = json.loads(msgs[3]["content"])
    assert reject["rejected"] is True
    assert reject["gate_reason"] == "still conflicts at node 5"


def test_propose_rejects_non_integer_selection():
    adv = make_advisor()
    adv._post = lambda path, body, timeout: {"message": {"content": json.dumps(
        {"dwell_ms": "2500"})}}
    with pytest.raises(ValueError, match="must be an integer"):
        adv.propose({
            "candidate_dwells": [
                {"dwell_ms": 2500, "conflict_free": True}],
        })


def test_propose_rejects_unlisted_duration():
    adv = make_advisor()
    adv._post = lambda path, body, timeout: {"message": {"content": json.dumps(
        {"dwell_ms": 2600, "rationale": "invented"})}}
    with pytest.raises(ValueError, match="not a safe candidate"):
        adv.propose({
            "candidate_dwells": [
                {"dwell_ms": 2500, "conflict_free": True}],
        })


def test_propose_rejects_when_no_safe_candidates_exist():
    adv = make_advisor()
    adv._post = lambda path, body, timeout: {"message": {"content": json.dumps(
        {"dwell_ms": 2500, "rationale": "no list"})}}
    with pytest.raises(ValueError, match="no deterministically safe candidates"):
        adv.propose({"candidate_dwells": []})


class _FakeResp:
    def __init__(self, data):
        self._data = json.dumps(data).encode()

    def read(self):
        return self._data

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def test_init_raises_when_server_unreachable(monkeypatch):
    def refuse(*args, **kwargs):
        raise urllib.error.URLError("connection refused")

    monkeypatch.setattr(urllib.request, "urlopen", refuse)
    with pytest.raises(RuntimeError, match="Ollama not reachable"):
        OllamaAdvisor(model="test-model", url="http://127.0.0.1:11434")


def test_init_raises_when_model_missing(monkeypatch):
    monkeypatch.setattr(
        urllib.request, "urlopen",
        lambda *a, **k: _FakeResp({"models": [{"name": "other:latest"}]}))
    with pytest.raises(RuntimeError, match="not found"):
        OllamaAdvisor(model="test-model", url="http://127.0.0.1:11434")


def test_init_accepts_tagged_model_name(monkeypatch):
    calls = []

    def fake_urlopen(req, timeout=None):
        url = req if isinstance(req, str) else req.full_url
        calls.append(url)
        if url.endswith("/api/tags"):
            return _FakeResp({"models": [{"name": "llama3.2:3b"}]})
        return _FakeResp({})  # warm-up /api/generate

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    adv = OllamaAdvisor(model="llama3.2:3b", url="http://127.0.0.1:11434")
    assert adv.model == "llama3.2:3b"
    assert any(u.endswith("/api/generate") for u in calls)  # warm-up fired
