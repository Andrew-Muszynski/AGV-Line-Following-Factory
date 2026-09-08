import math
import sys
from pathlib import Path


FLEET_DIR = Path(__file__).resolve().parents[1] / "fleet"
if str(FLEET_DIR) not in sys.path:
    sys.path.insert(0, str(FLEET_DIR))

from localization_modes import (  # noqa: E402
    OdomAnchor,
    blended_correction,
    mode_requires_camera,
    mode_requires_odom,
    normalize_localization_mode,
    odom_to_world,
)


def assert_pose(actual, expected):
    for got, want in zip(actual, expected):
        assert math.isclose(got, want, abs_tol=1e-9)


def test_mode_aliases_and_requirements():
    assert normalize_localization_mode("encoder-only") == "encoder"
    assert normalize_localization_mode("fused") == "camera_assist"
    assert normalize_localization_mode("camera_only") == "camera"
    assert mode_requires_odom("encoder")
    assert not mode_requires_camera("encoder")
    assert mode_requires_odom("camera_assist")
    assert mode_requires_camera("camera_assist")
    assert not mode_requires_odom("camera")
    assert mode_requires_camera("camera")


def test_south_facing_odometry_forward_maps_to_negative_world_y():
    anchor = OdomAnchor(0, 0, 0, 20, 10.5, 0)
    assert_pose(odom_to_world(10, 0, 0, anchor), (20, 0.5, 0))


def test_east_facing_odometry_forward_maps_to_positive_world_x():
    anchor = OdomAnchor(0, 0, 0, 13.3, 2.9, 90)
    assert_pose(odom_to_world(10, 0, 30, anchor), (23.3, 2.9, 120))


def test_anchor_handles_nonzero_raw_origin_and_yaw():
    anchor = OdomAnchor(4, -3, 25, 40, 50, 180)
    assert_pose(odom_to_world(4, -3, 40, anchor), (40, 50, -165))


def test_blended_correction_is_incremental():
    assert blended_correction(2.0, 10.0, 0.25) == 4.5

