from __future__ import annotations

import json
import math
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from apriltag_localize import (
    AprilTagBackendDiagnostics,
    CircularYawFilter,
    LensUndistorter,
    REF_TAG_WORLD,
    TableCalibration,
    build_detector,
    corrected_robot_yaw,
    format_preview_pose_line,
    pose_csv_row,
    pose_json_fields,
    tag_world_pose,
    wrap_degrees,
)


def circular_error(actual: float, expected: float) -> float:
    return wrap_degrees(actual - expected)


def detection(corners, tag_id=1, center=None):
    corners = np.asarray(corners, dtype=np.float64)
    if center is None:
        center = np.mean(corners, axis=0)
    return SimpleNamespace(
        tag_id=tag_id, corners=corners,
        center=np.asarray(center, dtype=np.float64),
        decision_margin=75.0, hamming=0)


def world_tag(yaw_deg: float, center=(20.25, 30.75), size=3.0):
    h = size / 2.0
    template = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    theta = math.radians(yaw_deg)
    rotation = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)],
    ])
    corners = template @ rotation.T + np.asarray(center)
    return corners, np.asarray(center, dtype=np.float64)


@pytest.mark.parametrize("yaw", [12.345, -90.125, 179.75, -179.75])
@pytest.mark.parametrize("world_to_image", [
    np.eye(3, dtype=np.float64),
    np.array([[2.5, 0.0, 100.0], [0.0, 1.5, -20.0], [0.0, 0.0, 1.0]]),
    np.array([[1.7, 0.12, 80.0], [-0.08, 1.4, 40.0],
              [0.0012, -0.0007, 1.0]]),
])
def test_fractional_yaw_under_homographies(yaw, world_to_image):
    corners_world, center_world = world_tag(yaw)
    corners_image = cv2.perspectiveTransform(
        corners_world.reshape(-1, 1, 2), world_to_image).reshape(-1, 2)
    center_image = cv2.perspectiveTransform(
        center_world.reshape(-1, 1, 2), world_to_image).reshape(2)
    pose = tag_world_pose(
        np.linalg.inv(world_to_image),
        detection(corners_image, center=center_image))
    assert pose is not None
    assert abs(circular_error(pose[2], yaw)) < 1e-8


def test_four_corner_average_preserves_canonical_direction():
    angle_a, angle_b = map(math.radians, (10.0, 14.0))
    v_a = np.array([math.cos(angle_a), math.sin(angle_a)])
    v_b = np.array([math.cos(angle_b), math.sin(angle_b)])
    corners = np.array([[0.0, 0.0], v_a, [0.0, 1.0] + v_b, [0.0, 1.0]])
    pose = tag_world_pose(np.eye(3), detection(corners))
    assert pose is not None
    assert abs(circular_error(pose[2], 12.0)) < 1e-12
    assert abs(circular_error(pose[2], 10.0)) > 1.0  # not just corner 0 -> 1


@pytest.mark.parametrize("bad", [
    np.zeros((4, 2)),
    np.array([[0, 0], [1, 0], [np.nan, 1], [0, 1]]),
])
def test_invalid_world_geometry_is_rejected(bad):
    assert tag_world_pose(np.eye(3), detection(bad)) is None


def test_mount_offset_layer_is_separate_and_zero_by_default():
    assert corrected_robot_yaw(1, 12.345678) == pytest.approx(12.345678)


def reference_detection(tag_id: int):
    center = np.asarray(REF_TAG_WORLD[tag_id], dtype=np.float64)
    h = 3.835 / 2.0
    world = center + np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    # A simple, exact world->pixel transform.
    pixels = world * np.array([10.0, -8.0]) + np.array([400.0, 900.0])
    return detection(pixels, tag_id=tag_id)


def test_frozen_calibration_counts_each_reference_independently():
    calib = TableCalibration(3.835, freeze_after_n=3, freeze_min_refs=2)
    d20, d21 = reference_detection(20), reference_detection(21)
    calib.update([d20, d21])
    calib.update([d20])  # reference 21 drops out
    calib.update([d20, d21])
    assert calib.reference_counts == {20: 3, 21: 2}
    assert not calib.frozen
    calib.update([d21])
    assert calib.frozen
    assert calib.reference_counts == {20: 3, 21: 3}
    assert np.allclose(calib.corners[21], d21.corners)
    calib.reset()
    assert calib.reference_counts == {}
    assert not calib.frozen


def test_freeze_min_refs_and_continuous_mode():
    frozen = TableCalibration(3.835, freeze_after_n=1, freeze_min_refs=4)
    frozen.update([reference_detection(i) for i in (20, 21, 22)])
    assert not frozen.frozen
    frozen.update([reference_detection(23)])
    assert frozen.frozen
    assert frozen.used_ids == [20, 21, 22, 23]

    continuous = TableCalibration(3.835, freeze_after_n=0)
    continuous.update([reference_detection(20), reference_detection(21)])
    assert continuous.H is not None
    assert not continuous.frozen


def test_precision_serialization_and_wrap_safe_filter():
    fields = pose_json_fields(
        1.0, 2.0, 12.345678, 12.345678, 3.0, 4.0, 1, 7, 9, 1.0)
    assert fields["yaw_deg"] == 12.345678
    assert isinstance(fields["yaw_deg"], float)
    assert fields["tag_yaw_raw_deg"] == 12.345678
    assert pose_csv_row(1.0, 1, 2.0, 3.0, 12.345678, 4.0, 5.0)[4] == "12.345678"
    yaw = 12.345678
    assert "yaw= +12.346" in format_preview_pose_line(1, 2.0, 3.0, yaw)
    assert yaw == 12.345678  # formatting cannot quantize the control value

    yaw_filter = CircularYawFilter(0.5)
    yaw_filter.update(1, 179.0)
    value = yaw_filter.update(1, -179.0)
    assert abs(abs(value) - 180.0) < 1e-9


def test_json_lens_calibration_and_dimension_validation(tmp_path):
    path = tmp_path / "camera.json"
    path.write_text(json.dumps({
        "model": "fisheye",
        "camera_matrix": [[1000, 0, 960], [0, 1000, 540], [0, 0, 1]],
        "distortion_coefficients": [0, 0, 0, 0],
        "image_width": 1920,
        "image_height": 1080,
    }))
    undistorter = LensUndistorter.from_file(path)
    assert undistorter.model == "fisheye"
    frame = np.zeros((540, 960, 3), dtype=np.uint8)
    assert undistorter.apply(frame).shape == frame.shape
    with pytest.raises(ValueError, match="dimensions changed"):
        undistorter.apply(np.zeros((1080, 1920, 3), dtype=np.uint8))


def test_patched_backend_rejects_degenerate_without_warning(capfd):
    detector = build_detector("tag36h11", 1.5, 1, True)
    backend = AprilTagBackendDiagnostics(detector)
    if not backend.available or backend.validate_correspondences(
            np.zeros((4, 4))) is None:
        pytest.skip("repository-patched native backend is not loaded")
    backend.reset()
    valid = np.array([
        [-1, -1, 10, 10], [1, -1, 20, 10],
        [1, 1, 20, 20], [-1, 1, 10, 20],
    ], dtype=np.float64)
    assert backend.validate_correspondences(valid)
    assert not backend.validate_correspondences(np.zeros((4, 4)))
    assert backend.rejected_count() == 1
    assert "Matrix is singular" not in capfd.readouterr().err
