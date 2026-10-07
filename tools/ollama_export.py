#!/usr/bin/env python3
"""Package Ollama models from one machine's store so another can import them.

WHY
---
Two reasons, both hit on 2026-09-11:

  * Re-downloading is slow. Pulling qwen3-vl:4b and :4b-instruct again ran
    at 5 MB/s -- about 22 minutes -- while the same bytes move across a LAN
    in roughly one.
  * A pull can simply fail. The desktop's Ollama reported "file does not
    exist" for both qwen3-vl tags while happily pulling gemma3:4b, and the
    laptop had them working the whole time. Copying files sidesteps the
    registry entirely.

HOW OLLAMA STORES MODELS
------------------------
    <store>/manifests/registry.ollama.ai/library/<model>/<tag>   small JSON
    <store>/blobs/sha256-<digest>                                the weights

The manifest lists every layer by digest; the blobs are content-addressed
and SHARED between models, so two tags built on one base cost one copy of
it. This walks the manifests, collects only the blobs they reference,
deduplicates, and writes a zip whose paths are relative to the store -- so
importing is "extract into the store", with no Ollama involvement.

    python3 tools/ollama_export.py qwen3-vl:4b qwen3-vl:4b-instruct
    python3 tools/ollama_export.py --list

Import on the far machine (quit Ollama from the tray first):

    tar -xf ollama_models.zip -C %USERPROFILE%\\.ollama\\models

then start Ollama and `ollama list` shows them.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import zipfile


def default_store() -> str:
    env = os.environ.get("OLLAMA_MODELS")
    if env:
        return env
    return os.path.join(os.path.expanduser("~"), ".ollama", "models")


def manifest_path(store: str, ref: str) -> tuple[str, str]:
    """('qwen3-vl:4b') -> (path on disk, path relative to the store)."""
    name, _, tag = ref.partition(":")
    tag = tag or "latest"
    rel = os.path.join("manifests", "registry.ollama.ai", "library", name, tag)
    return os.path.join(store, rel), rel


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("models", nargs="*", help="e.g. qwen3-vl:4b qwen3.5:4b")
    ap.add_argument("--store", default=None,
                    help="model store (default $OLLAMA_MODELS or "
                         "~/.ollama/models)")
    ap.add_argument("--out", default="ollama_models.zip")
    ap.add_argument("--list", action="store_true",
                    help="list what this store holds and exit")
    args = ap.parse_args()

    store = args.store or default_store()
    if not os.path.isdir(store):
        raise SystemExit(f"no model store at {store}")

    if args.list:
        root = os.path.join(store, "manifests", "registry.ollama.ai", "library")
        for name in sorted(os.listdir(root)):
            d = os.path.join(root, name)
            if os.path.isdir(d):
                for tag in sorted(os.listdir(d)):
                    m = json.load(open(os.path.join(d, tag), encoding="utf-8"))
                    total = sum(l["size"] for l in m.get("layers", []))
                    print(f"  {name}:{tag:<24} {total / 1e9:6.2f} GB")
        return

    if not args.models:
        raise SystemExit("name at least one model, or pass --list")

    # digest -> size, deduplicated across every requested model
    blobs: dict[str, int] = {}
    manifests: list[tuple[str, str]] = []
    for ref in args.models:
        path, rel = manifest_path(store, ref)
        if not os.path.exists(path):
            raise SystemExit(f"{ref}: no manifest at {path}\n"
                             f"  run --list to see what this store has")
        man = json.load(open(path, encoding="utf-8"))
        manifests.append((path, rel))
        layers = list(man.get("layers", []))
        if man.get("config"):
            layers.append(man["config"])
        for layer in layers:
            blobs[layer["digest"]] = layer.get("size", 0)
        print(f"  {ref}: {len(layers)} layers")

    missing = []
    total = 0
    for digest in blobs:
        blob = os.path.join(store, "blobs", digest.replace(":", "-"))
        if not os.path.exists(blob):
            missing.append(digest)
        else:
            total += os.path.getsize(blob)
    if missing:
        raise SystemExit(f"{len(missing)} blob(s) referenced but not on disk; "
                         f"the store may be mid-pull or corrupt:\n  "
                         + "\n  ".join(missing[:5]))

    print(f"\n{len(blobs)} unique blob(s), {total / 1e9:.2f} GB "
          f"(shared layers counted once)")
    # Stored, not deflated: these are already-compressed weights, so
    # deflate costs minutes of CPU to save almost nothing.
    with zipfile.ZipFile(args.out, "w", zipfile.ZIP_STORED,
                         allowZip64=True) as z:
        for path, rel in manifests:
            z.write(path, rel.replace(os.sep, "/"))
        for i, digest in enumerate(sorted(blobs), 1):
            fn = digest.replace(":", "-")
            print(f"  [{i}/{len(blobs)}] {fn[:20]}... "
                  f"{os.path.getsize(os.path.join(store, 'blobs', fn))/1e9:.2f} GB")
            z.write(os.path.join(store, "blobs", fn), f"blobs/{fn}")
    print(f"\nwrote {args.out}  ({os.path.getsize(args.out) / 1e9:.2f} GB)")
    print("import with Ollama STOPPED on the far machine:")
    print(r"  tar -xf " + os.path.basename(args.out)
          + r" -C %USERPROFILE%\.ollama\models")


if __name__ == "__main__":
    main()
