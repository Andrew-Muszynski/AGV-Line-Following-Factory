# Safe pupil-apriltags homography build

This directory reproducibly patches the native detector actually used by the
WSL localization process. It pins:

- `pupil-labs/apriltags` wrapper commit
  `f5334c6e007dc7256386e30e948d63fef5dbc264` (release
  `1.0.4.post11`)
- `pupil-labs/apriltags-source` submodule commit
  `1a0d17fb4031d70fca303d81c494fad7cfdcf0d8`

The patch rejects a singular/non-finite candidate before invalid elimination,
back-substitution, inversion, or decoding. It also makes `quad->H`/`Hinv`
failure ownership explicit and exposes a process-wide rejection counter. A
rejection occurs before tag decoding, so it cannot be attributed to a tag ID;
compare the counter with per-ID dropout data from `--log-quality`.

Build and install in Ubuntu 24.04 WSL:

```bash
cd /mnt/c/Users/muszy/VRP
bash third_party/pupil_apriltags_safe_homography/build_wheel.sh
python3 -m pip install --user --break-system-packages --force-reinstall \
  --no-deps third_party/pupil_apriltags_safe_homography/build/dist/\
pupil_apriltags-1.0.4.post11+agvsafe1-cp312-cp312-linux_x86_64.whl
```

`--no-deps` is intentional: the existing NumPy/OpenCV runtime is retained.
The wheel platform/Python tag changes on other hosts. The build script creates
a fresh source directory for each run and prints the wheel SHA-256.

Verify the process has loaded this build:

```bash
python3 -c "import importlib.metadata as m; from pupil_apriltags import Detector; d=Detector(); print(m.version('pupil-apriltags')); print(d.libc._name); print(hasattr(d.libc, 'apriltag_get_rejected_homography_count'))"
```

`apriltag_localize.py` prints the same version, binary path, and counter
availability on startup. It warns clearly if an unpatched binary is loaded.

Run the reproducible AddressSanitizer/LeakSanitizer check with:

```bash
bash third_party/pupil_apriltags_safe_homography/run_asan_test.sh
```

It makes a fresh pinned/patched source tree, builds the native library with
`-fsanitize=address`, and runs `backend_safety_test.c`. The test injects a
degenerate runtime `struct quad`, then runs 10,000 valid and 10,000 degenerate
constructions and verifies the exact rejection count and null matrix state.
