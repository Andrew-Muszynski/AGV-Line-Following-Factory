#!/usr/bin/env python3
"""Build the JPEG-quality arms of the 2x2 from the existing RCBD frames.

THE DESIGN THIS FEEDS
---------------------
    Overlay ON/OFF  x  JPEG 75/95,  24 blocks,  2 frames per cell

tools/rcbd_samples already holds the overlay factor: 24 posed blocks, each
captured raw and with the overlay, two frames each = 96 samples. This adds
the quality factor WITHOUT recapturing anything, so the robot placement and
every ground-truth value are bit-identical across all four cells. Recapturing
would have re-posed the robots and turned a paired design into an unpaired
one.

    96 source frames  ->  192 samples  =  24 blocks x 2 overlay x 2 quality x 2 frames

WHY BOTH ARMS ARE RE-ENCODED
----------------------------
Measured 2026-09-21: all 96 source frames are quality 95 and there is no
lossless original anywhere on disk. The obvious build -- copy the originals
as the 95 arm, re-encode for the 75 arm -- makes the arms asymmetric: the 75
frames would carry one more encode generation than the 95 frames, and any
difference could be that rather than the quality setting. Both arms are
therefore re-encoded once from the same q95 parent. The 95 arm measures back
at ~95 and is visually indistinguishable from its parent.

    original q95 ──┬── re-encode q95 ──> arm "95"
                   └── re-encode q75 ──> arm "75"

KNOWN BIAS, AND ITS DIRECTION
-----------------------------
The parent is already q95, so the 75 arm is a double compression: q95 loss
baked in, then q75 loss on top. A production frame would go raw sensor -> q75
in one step, which is strictly cleaner. **This experiment therefore
OVERSTATES the harm of quality 75.** That is the safe direction for the
decision it informs -- if 75 shows no accuracy cost even under a pessimistic
encode, it is genuinely safe to use. If it does show a cost, the true cost is
smaller than measured and the experiment should be rebuilt from lossless
captures before the number is quoted as the penalty.

To remove the bias for future work, have the capture tool write frame.png
alongside frame.jpg; this script will prefer a lossless parent when it finds
one.

USAGE
-----
    python3 tools/make_jpeg_arms.py --dry-run
    python3 tools/make_jpeg_arms.py
    python3 tools/make_jpeg_arms.py --qualities 75 95 --out tools/rcbd_2x2
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SRC = "tools/rcbd_samples"
DEFAULT_OUT = "tools/rcbd_2x2"

#: Preferred in order. A lossless parent removes the double-compression bias
#: documented above, so it wins whenever one exists.
PARENT_NAMES = ("frame.png", "frame.bmp", "frame.jpg")


def load_source(sample_dir: str):
    """(image, parent_path, lossless?) for one sample, or None."""
    from PIL import Image
    for name in PARENT_NAMES:
        path = os.path.join(sample_dir, name)
        if os.path.isfile(path):
            return Image.open(path), path, not name.endswith(".jpg")
    return None


def encode(im, dest: str, quality: int) -> None:
    """One baseline JPEG at `quality`, matching the capture path's encoder.

    Chroma subsampling is copied from the parent rather than left to PIL's
    default: changing 4:2:0 to 4:4:4 between arms would add a second
    difference to a comparison that is supposed to isolate quality. Progressive
    and optimize stay off because camera_bridge_windows.py's cv2.imencode
    writes baseline, and the arms should look like frames the rig would
    actually produce.
    """
    from PIL import JpegImagePlugin
    try:
        sub = JpegImagePlugin.get_sampling(im)
    except Exception:
        sub = 2
    rgb = im.convert("RGB")
    rgb.save(dest, "JPEG", quality=quality, subsampling=sub,
             optimize=False, progressive=False)


def relabel(label: str | None, quality: int) -> str:
    """"B02-overlay" -> "B02-overlay-q75", keeping the arm tokens parseable.

    vlm_aggregate.Scenes reads the overlay arm and the block out of this
    string by TOKEN, not by suffix, so appending the quality does not hide
    the overlay arm.
    """
    base = label if label and label != "None" else "unlabelled"
    return f"{base}-q{quality}"


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=ROOT)
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--qualities", type=int, nargs="+", default=[75, 95])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing output pool")
    args = ap.parse_args()

    src_root = os.path.join(args.root, args.src)
    out_root = os.path.join(args.root, args.out)
    if not os.path.isdir(src_root):
        raise SystemExit(f"no source pool at {src_root}")
    if os.path.isdir(out_root) and not (args.dry_run or args.force):
        raise SystemExit(f"{out_root} exists; pass --force to rebuild it")

    samples = [d for d in sorted(os.listdir(src_root))
               if os.path.isfile(os.path.join(src_root, d, "truth.json"))]
    if not samples:
        raise SystemExit(f"no samples with truth.json under {src_root}")

    sys.path.insert(0, os.path.join(args.root, "tools"))
    from vlm_aggregate import jpeg_quality

    made = 0
    lossless_parents = 0
    blocks: dict[str, set] = {}
    for sid in samples:
        sdir = os.path.join(src_root, sid)
        truth = json.load(open(os.path.join(sdir, "truth.json"),
                               encoding="utf-8"))
        loaded = load_source(sdir)
        if loaded is None:
            print(f"  SKIP {sid}: no frame")
            continue
        im, parent, lossless = loaded
        if lossless:
            lossless_parents += 1
        label = truth.get("label")
        block = str(label).split("-")[0] if label else sid

        for q in args.qualities:
            new_sid = f"{sid}_q{q}"
            dest_dir = os.path.join(out_root, new_sid)
            blocks.setdefault(block, set()).add((truth.get("label"), q))
            if args.dry_run:
                made += 1
                continue
            os.makedirs(dest_dir, exist_ok=True)
            dest_frame = os.path.join(dest_dir, "frame.jpg")
            encode(im, dest_frame, q)

            # Ground truth is copied VERBATIM except for the fields that
            # identify the file. Every pose, distance and relation is
            # therefore identical across all four cells by construction --
            # that is the whole point of generating rather than recapturing.
            new_truth = dict(truth)
            new_truth["sample_id"] = new_sid
            new_truth["label"] = relabel(label, q)
            new_truth["frame_bytes"] = os.path.getsize(dest_frame)
            new_truth["jpeg_quality_requested"] = q
            new_truth["jpeg_quality_measured"] = jpeg_quality(dest_frame)
            new_truth["derived_from"] = {
                "sample_id": truth.get("sample_id", sid),
                "parent_file": os.path.relpath(parent, args.root).replace(os.sep, "/"),
                "parent_lossless": lossless,
                "parent_quality": None if lossless else jpeg_quality(parent),
                "note": ("re-encoded once from the parent; both arms get "
                         "exactly one generation so the contrast isolates "
                         "quality" + ("" if lossless else
                                      ". Parent is lossy, so the lower arm is "
                                      "a double compression and OVERSTATES "
                                      "the harm of low quality")),
            }
            with open(os.path.join(dest_dir, "truth.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(new_truth, fh, indent=2, sort_keys=True)
                fh.write("\n")
            made += 1
        im.close()

    cells = sum(len(v) for v in blocks.values())
    print(f"{'would build' if args.dry_run else 'built'} {made} samples "
          f"from {len(samples)} source frames")
    print(f"  {len(blocks)} blocks, {cells} block x overlay x quality cells")
    print(f"  qualities: {args.qualities}")
    if lossless_parents:
        print(f"  {lossless_parents} lossless parent(s) used -- no "
              f"double-compression bias for those")
    else:
        print("  every parent was already JPEG: the LOW quality arm is a "
              "double compression and OVERSTATES the harm of low quality "
              "(see this file's docstring)")
    if not args.dry_run:
        print(f"  -> {out_root}")
        print()
        print("Add the new pool to vlm_aggregate.SAMPLE_DIRS, then run the "
              "benchmark against it with --samples-dir " + args.out)


if __name__ == "__main__":
    main()
