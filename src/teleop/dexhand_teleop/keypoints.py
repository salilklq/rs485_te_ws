"""Canonicalise 21 hand keypoints into the MANO frame for dex-retargeting.

This mirrors dex-retargeting's example/vector_retargeting/single_hand_detector.py
(estimate_frame_from_hand_points + OPERATOR2MANO) so our MANUS-sourced keypoints
go through the exact same, validated transform. Because the wrist frame is
re-estimated from the hand geometry every frame, the absolute orientation of the
incoming MANUS frame does not matter.
"""
import numpy as np

# From dex_retargeting.constants
OPERATOR2MANO_RIGHT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
OPERATOR2MANO_LEFT = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=float)


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    """3x3 orientation of the wrist in MANO convention, from points [0,5,9]
    (wrist, index-MCP, middle-MCP). Verbatim port from dex-retargeting."""
    assert keypoint_3d_array.shape == (21, 3)
    points = keypoint_3d_array[[0, 5, 9], :]

    x_vector = points[0] - points[2]
    points = points - np.mean(points, axis=0, keepdims=True)
    u, s, v = np.linalg.svd(points)
    normal = v[2, :]

    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / (np.linalg.norm(x) + 1e-9)
    z = np.cross(x, normal)
    if np.sum(z * (points[1] - points[2])) < 0:
        normal *= -1
        z *= -1
    frame = np.stack([x, normal, z], axis=1)
    return frame


def to_mano_frame(points: np.ndarray, is_right: bool, position_scale: float = 1.0) -> np.ndarray:
    """(21,3) raw keypoints -> (21,3) in the canonical MANO frame, wrist at origin.

    Args:
        points: 21x3 keypoints (any consistent frame), e.g. meters from MANUS.
        is_right: hand type, selects OPERATOR2MANO.
        position_scale: multiply positions (use to convert source units to meters).
    """
    kp = np.asarray(points, dtype=float).reshape(21, 3) * position_scale
    kp = kp - kp[0:1, :]                       # wrist -> origin
    wrist_rot = estimate_frame_from_hand_points(kp)
    operator2mano = OPERATOR2MANO_RIGHT if is_right else OPERATOR2MANO_LEFT
    return kp @ wrist_rot @ operator2mano
