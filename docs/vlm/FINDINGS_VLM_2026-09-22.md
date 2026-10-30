# VLM configuration experiment — findings, 2026-09-22

A full 2×2×2 on the overhead-camera question set: **pose overlay × JPEG
quality × blank reference image**, 32,192 question-instances, 8 cells,
4 models, 24 blocks, one sample pool with identical robot placement in every
cell.

Headline: **of the three factors tested, the one that was supposed to be a
held-constant nuisance parameter is the only one that moved anything.**

---

## 1. The design

```
24 blocks × {overlay on, off} × {JPEG 95, 75} × {blank ref image, none}
         × 2 frames × 4 models  =  32,192 question-instances
```

Every cell shows the same 24 posed robot placements. The JPEG arms were
generated offline from the same q95 parents (`tools/make_jpeg_arms.py`), so
no robot was re-posed between conditions and every ground-truth value is
bit-identical across cells. The block is the pairing unit for every effect
below; intervals are 95% t on the 24 block differences and p-values are
Wilcoxon signed-rank on the same 24 numbers.

Models: `gemma4:e4b-it-qat`, `gemma4:e2b-it-qat`, `minicpm-v4.6:1b`,
`qwen3.5:0.8b`. Chosen by screening within a single condition cell — see §7.

Source files: `tools/rcbd_2x2_results.json` (blank image on),
`tools/rcbd_2x2_noblank.json` (off). Both served by the NVIDIA
RTX 5060 Laptop GPU (8 GB VRAM), labelled `rtx5060-laptop-8gb`.

![Effect of Three Configuration Factors on VLM Accuracy](../../figures/fig_vlm_factors.png)

*Figure 1. Effect of Three Configuration Factors on VLM Accuracy.*

| factor | effect | 95% CI | p |
|---|---|---|---|
| blank reference image | **−4.53 pp** | [−5.39, −3.67] | 0.00002 |
| pose overlay | +0.54 pp | [−0.21, +1.30] | 0.13 |
| JPEG 95 vs 75 | +0.04 pp | [−0.45, +0.53] | 0.70 |

---

## 2. JPEG quality does not matter — halve the bytes

95 vs 75 moved accuracy by **+0.04 pp** (p=0.70) across 28,256 paired
instances. Nothing significant in any capability family. Per model the
spread is −0.2 to +0.4 pp, all non-significant.

This is the strong form of the result because the q75 arm is
**pessimistic**: the parents were already q95 and no lossless original
exists, so the low arm carries a double compression and overstates any harm.
It showed none anyway.

**Action:** drop the capture/serving path to quality 75. Median frame size
falls from 504 KB to 260 KB at no measurable accuracy cost.

---

## 3. The blank reference image is harmful, and only to the small models

Supplying an empty-testbed photo alongside the scene frame — intended as
free context — cost **−4.53 pp overall** (p=0.00002), and the harm is
entirely in the two smallest models.

![Configuration Factor Effects on VLM Accuracy by Model](../../figures/fig_vlm_by_model.png)

*Figure 2. Configuration Factor Effects on VLM Accuracy by Model.*

| model | effect | p |
|---|---|---|
| gemma4:e4b-it-qat | +0.60 pp | 0.13 |
| gemma4:e2b-it-qat | −0.27 pp | 0.65 |
| minicpm-v4.6:1b | −3.82 pp | 0.001 |
| qwen3.5:0.8b | **−14.62 pp** | 0.00002 |

Four families are significantly harmed after Holm correction:

![Effect of Reference Image on VLM Accuracy by Question Category](../../figures/fig_vlm_blank_families.png)

*Figure 3. Effect of Reference Image on VLM Accuracy by Question Category.*

The telling entries are **control probes (−6.3 pp)** and **yaw readback
(−7.5 pp)**. Both are answered from the measured-state *text*, not the
picture — the second image should be irrelevant to them. The small models
are spending capacity on an image the prompt explicitly told them to ignore.
One unambiguous failure: minicpm answered a bare `0` on 28 samples, which is
the robot count of the *reference* image rather than the scene.

It also costs latency on every model, worst where accuracy is worst:

![VLM Accuracy and p95 Latency With and Without Reference Image](../../figures/fig_vlm_accuracy_latency.png)

*Figure 4. VLM Accuracy and p95 Latency With and Without Reference Image.*

**Action:** do not supply the blank reference image. It helps nothing, harms
half the models, and costs p95 latency across the board.

**Process note:** this factor was held constant in the original 2×2 on the
grounds that it was a nuisance parameter. That was the wrong call — it is
several times larger than either factor the experiment set out to test, and
crossing it took one extra 2.7 h run.

---

## 4. The overlay is a counting aid, not a general improvement

Pooled effect +0.54 pp (p=0.13, not significant). But counting alone moves
**+7.6 pp** (Holm p=0.0013), which is 86% of the pooled move, and the per-model
split is on whether the model can already count:

| model | overlay effect | p |
|---|---|---|
| gemma4:e4b-it-qat | −1.3 pp | 0.043 |
| gemma4:e2b-it-qat | −0.9 pp | 0.13 |
| minicpm-v4.6:1b | +1.5 pp | 0.044 |
| qwen3.5:0.8b | +2.9 pp | 0.015 |

It *helps* the two models that miscount and *hurts* the one that does not.
For a model that counts reliably the overlay is clutter.

---

## 5. Model selection: gemma4:e4b-it-qat

| model | accuracy (all cells) | cheapest cell | p95 no-ref | p95 with-ref |
|---|---|---|---|---|
| gemma4:e4b-it-qat | **84.2%** | 84.8% | **0.40 s** | 0.47 s |
| gemma4:e2b-it-qat | 83.8% | 83.9% | 1.13 s | 1.55 s |
| minicpm-v4.6:1b | 73.1% | 75.2% | 1.97 s | 2.86 s |
| qwen3.5:0.8b | 63.1% | 69.3% | 1.64 s | 4.27 s |

e4b is unmoved by the blank image and by JPEG quality, is the fastest in the
set by a factor of three, and ties for the highest accuracy. Every other
model is sensitive to at least one condition.

**Recommended configuration: gemma4:e4b-it-qat, JPEG 75, no overlay, no
reference image** — the cheapest setting of all three factors is also its
best, at p95 0.40 s against a 5 s operational ceiling.

---

## 6. Every model has a systematic orientation bias

![VLM Orientation Accuracy by True Robot Facing](../../figures/fig_vlm_orientation.png)

*Figure 5. VLM Orientation Accuracy by True Robot Facing.*

An overall orientation score hides this completely. gemma4:e4b is **100% on
true west and 14% on true south** — and its most common wrong answer on
*every* other axis is also "west". That is a response bias, not a resolution
limit, and it is invisible in the 59–63% overall orientation figure.

The models do not share a bias: minicpm is worst on west (16%), qwen3.5:0.8b
scores **0/132 on true south**. Nothing here is close to usable for
orientation, and it is the clearest open capability gap.

---

## 7. Method notes and caveats

**The counting grader takes the first number.** `vlm_bench` scores a count
via `_numbers_in(...)[0]`. A model answering *"Box 1 is at (63.48, 76.78) …
Box 6 is at … so there are 6"* is scored as having said 1. The blank-image
arm made models narrate both images, so this rare weakness became the
dominant failure there and inflated the measured harm. 64% of wrong counting
answers (145/228) contain "6" in the text.

Every counting number here is therefore reported under three reading rules:

| rule | overall effect | counting effect |
|---|---|---|
| as-graded | −4.53 pp | −27.9 pp |
| last number | −3.72 pp | −10.9 pp |
| concluding sentence | −3.68 pp | −10.2 pp |

The blank-image effect survives at p<0.0001 under all three. **Quote −3.7 pp
overall and −10.9 pp on counting, not the as-graded figures.** `correct` in
the master table was never modified.

**Counting is ceiling-limited.** All 192 samples contain exactly 6 robots, so
answering "6" every time scores 100%. "Counting" here partly measures
propensity to answer 6. Fixing this needs scenes with a spread of robot
counts; `tools/vlm_samples` has `posed-2robots` … `posed-6robots` plus 20
empty-table frames, but they carry no overlay arm.

**The q75 arm is double-compressed** (§2) — bias is toward overstating harm,
which is the safe direction for the conclusion drawn.

**Latency is machine-bound.** Both runs were served by the NVIDIA RTX 5060
Laptop GPU (8 GB VRAM), so every second quoted here is a GPU figure on 8 GB of
VRAM. Measurements from other machines are not comparable and are excluded.
The `--machine` flag was given as `windows-i7-uhd` on the command line, which
was wrong; the correction is declared in `tools/vlm_runs.json` rather than
edited into the result files, so the raw record stays untouched.

---

## 8. Tooling built or fixed today

- `tools/vlm_findings.py` — computes every number in this document into
  `analysis/findings_2026-09-22.json`. Delegates effect estimation to
  `vlm_aggregate.contrast` after an earlier independent implementation
  disagreed by up to 1 pp (the overlay arms do not ask an identical question
  set — `build_questions` rotates them via `_spread`).
- `tools/make_vlm_figures.py` — the five figures above, all from that JSON.
- `vlm_aggregate.contrast` now returns `block_diffs` and `ci95`.
- Fixed: 222 ungraded rows (`ops:status`, open-ended) were being scored as
  **wrong**, depressing every model's live-suite accuracy.
- Fixed: the failure classifier had no live-suite branches — 401 rows were
  falling into a `wrong_other` catch-all.
- Fixed: `str(expected or "")` turned the integer **0** into "no truth
  value", which is the correct answer for every empty-table counting question.
- Added: `given` column (what the model actually committed to, read with the
  grader's own extractors), `failure_examples.csv`, `confusion.csv`,
  `given_agreement.csv`.

---

## 9. Reproduce

```bash
python3 tools/vlm_aggregate.py        # master table from every result file
python3 tools/vlm_findings.py         # the numbers
python3 tools/make_vlm_figures.py     # the figures
python3 tools/vlm_aggregate.py --contrast blank_image --model qwen3.5:0.8b
```

Raw results are never modified. Every row in `analysis/vlm_master.csv`
carries `source_file` + `row_index`, so any percentage above traces back to
one model's one response to one question about one frame.

---

## 10. Open items

1. **Counting ceiling** — build a quality/overlay pool with varying robot
   counts, or accept that counting is not measurable on this pool.
2. **Counting grader** — offer a non-destructive sensitivity column in the
   aggregator rather than leaving the correction in this document.
3. **Orientation** — no model is usable. Worth a dedicated experiment before
   any orientation-dependent capability is designed around a VLM.
4. **`vlm_capture.py` has no quality flag** — `/full.jpg` re-encodes at a
   hardcoded q95, so the 2026-09-10 `A*-q75` pilot compared q95 against q95
   and measured nothing. Offline re-encoding is currently the only way to
   vary quality.
