import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "oculus_reader" / "scripts"))

from teleop_math import (  # noqa: E402
    AnchoredTeleopConfig,
    AnchoredTeleopMapper,
    map_delta_pos,
    map_delta_quat,
    quat_to_axis_angle,
)


def quat_from_axis(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return Rotation.from_rotvec(axis * angle).as_quat()


def test_runtime_standard_position_mapping_matches_jetson_scripts():
    raw = np.array([0.1, 0.2, 0.3])

    np.testing.assert_allclose(map_delta_pos("right", raw), [0.3, 0.2, 0.1])
    np.testing.assert_allclose(map_delta_pos("left", raw), [0.3, -0.2, -0.1])


def test_runtime_standard_orientation_axis_mapping_matches_jetson_scripts():
    q_vr = quat_from_axis([1.0, 0.0, 0.0], 0.4)

    right_axis, right_angle = quat_to_axis_angle(map_delta_quat("right", q_vr))
    left_axis, left_angle = quat_to_axis_angle(map_delta_quat("left", q_vr))

    np.testing.assert_allclose(right_axis, [0.0, 0.0, -1.0], atol=1e-6)
    np.testing.assert_allclose(left_axis, [0.0, 0.0, 1.0], atol=1e-6)
    assert math.isclose(right_angle, 0.4, abs_tol=1e-6)
    assert math.isclose(left_angle, 0.4, abs_tol=1e-6)


def test_repeated_anchor_reset_keeps_right_arm_direction_stable():
    cfg = AnchoredTeleopConfig(
        hand_name="right",
        output_gain=1.0,
        max_step_pos=1.0,
        alpha_pos=1.0,
    )
    mapper = AnchoredTeleopMapper(cfg)

    mapper.reset_anchor(
        handle_pos=[1.0, 2.0, 3.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
        robot_pos=[0.4, -0.2, 0.3],
        robot_quat=[0.0, 0.0, 0.0, 1.0],
    )
    first = mapper.update(
        handle_pos=[1.1, 2.0, 3.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
    )

    mapper.reset_anchor(
        handle_pos=[-2.0, 5.0, 7.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
        robot_pos=[0.6, -0.1, 0.2],
        robot_quat=[0.0, 0.0, 0.0, 1.0],
    )
    second = mapper.update(
        handle_pos=[-1.9, 5.0, 7.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
    )

    np.testing.assert_allclose(first.position - np.array([0.4, -0.2, 0.3]), [0.0, 0.0, 0.1])
    np.testing.assert_allclose(second.position - np.array([0.6, -0.1, 0.2]), [0.0, 0.0, 0.1])


def test_step_limit_and_deadband_stabilize_small_motion():
    cfg = AnchoredTeleopConfig(
        hand_name="right",
        output_gain=1.0,
        deadband_pos=0.005,
        max_step_pos=0.02,
        alpha_pos=1.0,
    )
    mapper = AnchoredTeleopMapper(cfg)
    mapper.reset_anchor(
        handle_pos=[0.0, 0.0, 0.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
        robot_pos=[0.0, 0.0, 0.0],
        robot_quat=[0.0, 0.0, 0.0, 1.0],
    )

    tiny = mapper.update(
        handle_pos=[0.004, 0.0, 0.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
    )
    large = mapper.update(
        handle_pos=[0.20, 0.0, 0.0],
        handle_quat=[0.0, 0.0, 0.0, 1.0],
    )

    np.testing.assert_allclose(tiny.position, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(large.position, [0.0, 0.0, 0.02])
