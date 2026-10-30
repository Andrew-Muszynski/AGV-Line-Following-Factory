#!/usr/bin/env python3
"""
test_apriltag_localize.py — Synthetic-camera tests for apriltag_localize.py.

Simulates a tilted overhead camera as a ground-truth world->image homography,
projects fake reference-tag and robot-tag detections through it, and checks
that fit_table_homography() + tag_world_pose() recover the robot poses.

Covers both possible detector corner-winding conventions and arbitrary
in-plane rotations of the reference tags, since the fitter claims to be
independent of both. Run directly (python test_apriltag_localize.py) or via
pytest.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from apriltag_localize import (
    REF_TAG_WORLD,
    fit_table_homography,
    tag_world_pose,
)

# Measured side lengths between the detector's tag corners: the edge where the
# white and black tag borders meet, not the outside of the printed paper.
REF_SIZE_IN = 3.835
ROBOT_SIZE_IN = 3.05

# These are synthetic-regression ceilings, not acceptance limits for the
# camera-based line-centering controller. Set that controller's limits from
# measured lateral and yaw error on the physical tape.
TWO_REF_MAX_NOISE_ERROR_IN = 4.0
FOUR_REF_MAX_NOISE_ERROR_IN = 1.0


def project_points_oracle(H: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Project 2-D points without using apriltag_localize.map_points().

    Keeping synthetic data generation independent from the production mapping
    helper prevents a shared mapping bug from cancelling itself in these tests.
    """
    H = np.asarray(H, dtype=np.float64)
    points = np.asarray(points, dtype=np.float64)
    if H.shape != (3, 3):
        raise ValueError(f"H must have shape (3, 3), got {H.shape}")
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"points must have shape (N, 2), got {points.shape}")
    if not np.all(np.isfinite(H)) or not np.all(np.isfinite(points)):
        raise ValueError("H and points must contain only finite values")

    projected = cv2.perspectiveTransform(
        points.reshape(-1, 1, 2),
        H,
    )
    return projected.reshape(-1, 2)


def quad_center(corners: np.ndarray) -> np.ndarray:
    """Intersection of the quad's diagonals = the perspective projection of
    the physical tag center (what the real apriltag library reports as
    det.center — NOT the arithmetic mean of the corners)."""
    corners = np.asarray(corners, dtype=float)
    if corners.shape != (4, 2):
        raise ValueError(f"corners must have shape (4, 2), got {corners.shape}")
    if not np.all(np.isfinite(corners)):
        raise ValueError("corners must contain only finite values")

    p0, p1, p2, p3 = (np.append(c, 1.0) for c in corners)
    c = np.cross(np.cross(p0, p2), np.cross(p1, p3))
    if abs(c[2]) < 1e-12:
        raise ValueError("tag diagonals do not have a finite intersection")
    return c[:2] / c[2]


class FakeDetection:
    def __init__(self, tag_id: int, corners: np.ndarray):
        self.tag_id = tag_id
        self.corners = np.asarray(corners, dtype=float)
        self.center = quad_center(self.corners)


def ground_truth_H() -> np.ndarray:
    """World (inches, y up) -> image (pixels, y down), overhead camera with a
    modest tilt. World bottom edge lands at the bottom of the frame (large y),
    so the map flips handedness like a real overhead view."""
    src = np.float32([[0, 0], [97, 0], [97, 97], [0, 97]])
    dst = np.float32([[210, 960], [1690, 930], [1580, 140], [320, 170]])
    return cv2.getPerspectiveTransform(src, dst).astype(float)


def tag_world_corners(center, size: float, theta: float, physical_ccw: bool) -> np.ndarray:
    """Corner positions of a face-up tag in world coords, in the order the
    detector would index them. physical_ccw picks which winding convention the
    simulated detector library uses."""
    h = size / 2.0
    template = np.array([(-h, -h), (h, -h), (h, h), (-h, h)], dtype=float)
    if not physical_ccw:
        template = template[::-1].copy()
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return np.asarray(center, dtype=float) + template @ R.T


def project_tag(H_wi: np.ndarray, tag_id: int, center, size: float, theta: float,
                physical_ccw: bool, noise_px: float = 0.0,
                rng: np.random.Generator | None = None) -> FakeDetection:
    if size <= 0.0:
        raise ValueError(f"tag size must be positive, got {size}")
    if noise_px < 0.0:
        raise ValueError(f"noise_px must be nonnegative, got {noise_px}")
    if noise_px > 0.0 and rng is None:
        raise ValueError("rng is required when noise_px is nonzero")

    img = project_points_oracle(
        H_wi,
        tag_world_corners(center, size, theta, physical_ccw),
    )
    if noise_px > 0.0:
        img = img + rng.normal(0.0, noise_px, img.shape)
    return FakeDetection(tag_id, img)


def fit_from_refs(H_wi, ref_ids, ref_thetas, physical_ccw, noise_px=0.0, rng=None):
    ref_ids = tuple(ref_ids)
    ref_thetas = tuple(ref_thetas)
    if len(ref_ids) != len(ref_thetas):
        raise ValueError(
            "ref_ids and ref_thetas must have the same length: "
            f"{len(ref_ids)} != {len(ref_thetas)}"
        )
    if len(set(ref_ids)) != len(ref_ids):
        raise ValueError(f"reference-tag IDs must be unique: {ref_ids}")

    unknown_ids = sorted(set(ref_ids) - set(REF_TAG_WORLD))
    if unknown_ids:
        raise ValueError(f"unknown reference-tag IDs: {unknown_ids}")

    ref_corners = {
        tid: project_tag(H_wi, tid, REF_TAG_WORLD[tid], REF_SIZE_IN, th,
                         physical_ccw, noise_px, rng).corners
        for tid, th in zip(ref_ids, ref_thetas, strict=True)
    }
    return fit_table_homography(ref_corners, REF_SIZE_IN)


def robot_error(H_fit, H_wi, world_xy, yaw, physical_ccw, noise_px=0.0, rng=None):
    det = project_tag(H_wi, 5, world_xy, ROBOT_SIZE_IN, yaw, physical_ccw, noise_px, rng)
    x, y, _ = tag_world_pose(H_fit, det)
    return math.hypot(x - world_xy[0], y - world_xy[1])


def test_exact_recovery_two_diagonal_refs():
    """Noise-free, refs 20+23 only: with 2 tags the fitter snaps tag rotations
    to 90-degree multiples, so recovery must be exact for square-placed tags
    in ANY of the four orientations, under both winding conventions."""
    H_wi = ground_truth_H()
    robot_spots = [(30.0, 70.0), (48.5, 48.5), (5.0, 92.0), (90.0, 8.0)]
    for physical_ccw in (True, False):
        for thetas in [(0.0, 0.0), (math.pi / 2, math.pi),
                       (-math.pi / 2, 3 * math.pi / 2)]:
            fit = fit_from_refs(H_wi, [20, 23], thetas, physical_ccw)
            assert fit is not None, f"fit failed (ccw={physical_ccw}, thetas={thetas})"
            H_fit, rms, ids, _ = fit
            assert ids == [20, 23]
            assert rms < 1e-8, f"rms={rms} (ccw={physical_ccw}, thetas={thetas})"
            for spot in robot_spots:
                err = robot_error(H_fit, H_wi, spot, 0.4, physical_ccw)
                assert err < 1e-6, (
                    f"robot err={err} at {spot} (ccw={physical_ccw}, thetas={thetas})")
    print("exact recovery, 2 diagonal refs (square placements): OK")


def test_two_refs_crooked_tags_bounded_error():
    """Refs taped a few degrees off square: the 2-tag snap assumption is then
    slightly wrong. Error must stay modest (and shows up in the reported rms)."""
    H_wi = ground_truth_H()
    fit = fit_from_refs(H_wi, [20, 23], (math.radians(3.0), math.radians(-2.0)), True)
    assert fit is not None
    H_fit, rms, _, _ = fit
    errs = {spot: robot_error(H_fit, H_wi, spot, 0.4, True)
            for spot in [(48.5, 48.5), (30.0, 70.0), (5.0, 92.0), (90.0, 8.0)]}
    print(f"  crooked refs (3/-2 deg): rms={rms:.3f} in; robot err by spot: "
          + ", ".join(f"{s}={e:.2f}" for s, e in errs.items()))
    # Error grows toward the empty 21/22 corners (small lever arm perpendicular
    # to the 20-23 diagonal); mid-table stays much tighter. Squarely placed
    # tags or the remaining two corner tags shrink this.
    assert max(errs.values()) < 4.0, f"crooked-tag error too large: {errs}"
    print("crooked 2-ref tags, bounded error: OK")


def test_exact_recovery_four_refs():
    H_wi = ground_truth_H()
    for physical_ccw in (True, False):
        fit = fit_from_refs(H_wi, [20, 21, 22, 23], [0.1, -0.4, 1.7, 3.0], physical_ccw)
        assert fit is not None
        H_fit, rms, ids, _ = fit
        assert ids == [20, 21, 22, 23]
        assert rms < 1e-8, f"rms={rms}"
        err = robot_error(H_fit, H_wi, (25.0, 60.0), 1.0, physical_ccw)
        assert err < 1e-6, f"robot err={err}"
    print("exact recovery, 4 refs: OK")


def angle_difference_deg(end: float, start: float) -> float:
    """Return the signed shortest angular displacement from start to end."""
    return (end - start + 180.0) % 360.0 - 180.0


def test_yaw_difference_is_exact_for_both_corner_windings():
    """Absolute yaw has a mounting-dependent offset, but the DIFFERENCE between
    two robot headings must be recovered exactly for either detector corner
    winding, including across the +/-180-degree wrap boundary."""
    H_wi = ground_truth_H()
    cases_deg = [
        (40.0, 35.0),
        (170.0, 25.0),
        (-170.0, -25.0),
    ]

    for physical_ccw in (True, False):
        fit = fit_from_refs(
            H_wi,
            [20, 23],
            (0.0, math.pi / 2),
            physical_ccw,
        )
        assert fit is not None
        H_fit, _, _, _ = fit

        for start_deg, delta_deg in cases_deg:
            d1 = project_tag(
                H_wi,
                5,
                (40.0, 40.0),
                ROBOT_SIZE_IN,
                math.radians(start_deg),
                physical_ccw,
            )
            d2 = project_tag(
                H_wi,
                6,
                (60.0, 40.0),
                ROBOT_SIZE_IN,
                math.radians(start_deg + delta_deg),
                physical_ccw,
            )
            _, _, yaw1 = tag_world_pose(H_fit, d1)
            _, _, yaw2 = tag_world_pose(H_fit, d2)
            measured = angle_difference_deg(yaw2, yaw1)
            assert abs(angle_difference_deg(measured, delta_deg)) < 1e-3, (
                f"delta yaw={measured}, expected={delta_deg}, "
                f"ccw={physical_ccw}"
            )
    print("yaw difference, both corner windings and angle wrapping: OK")


def test_noise_robustness():
    """0.3 px corner noise. With only the diagonal pair the fit is weakly
    constrained perpendicular to the 20-23 diagonal, so tolerances are loose
    near the empty corners; with all four refs they tighten a lot."""
    H_wi = ground_truth_H()
    rng = np.random.default_rng(42)
    trials = 200

    def run(ref_ids, spot):
        errs = []
        for _ in range(trials):
            fit = fit_from_refs(H_wi, ref_ids, [0.0] * len(ref_ids), True,
                                noise_px=0.3, rng=rng)
            assert fit is not None
            errs.append(robot_error(fit[0], H_wi, spot, 0.4, True,
                                    noise_px=0.3, rng=rng))
        return float(np.mean(errs)), float(np.max(errs))

    for spot in [(48.5, 48.5), (30.0, 70.0), (5.0, 92.0)]:
        mean2, max2 = run([20, 23], spot)
        mean4, max4 = run([20, 21, 22, 23], spot)
        print(f"  noise 0.3px at {spot}: 2 refs mean={mean2:.2f} max={max2:.2f} in | "
              f"4 refs mean={mean4:.2f} max={max4:.2f} in")
        assert max2 < TWO_REF_MAX_NOISE_ERROR_IN, (
            f"2-ref error too big at {spot}: {max2}"
        )
        assert max4 < FOUR_REF_MAX_NOISE_ERROR_IN, (
            f"4-ref error too big at {spot}: {max4}"
        )
    print("noise robustness: OK (see per-spot numbers above)")


if __name__ == "__main__":
    test_exact_recovery_two_diagonal_refs()
    test_two_refs_crooked_tags_bounded_error()
    test_exact_recovery_four_refs()
    test_yaw_difference_is_exact_for_both_corner_windings()
    test_noise_robustness()
    print("\nAll tests passed.")
