"""Geometry and pose math for synthetic CubeSat vision experiments."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class CubeSatModel:
    """Approximate 3U CubeSat target with two deployable solar panels.

    Coordinates are expressed in the target body frame in meters.
    The model is intentionally simple so it can be rendered with OpenGL first and
    replaced by a higher-fidelity CAD/Unreal asset later without changing labels.
    """

    body_x: float = 0.10
    body_y: float = 0.10
    body_z: float = 0.34
    panel_span: float = 0.468
    panel_height: float = 0.22

    def keypoints_3d(self) -> np.ndarray:
        """Return 11 stable 3D keypoints matching the SPEED-UE-Cube spirit."""
        hx, hy, hz = self.body_x / 2, self.body_y / 2, self.body_z / 2
        half_span = self.panel_span / 2
        panel_z_top = self.panel_height / 2
        panel_z_bottom = -self.panel_height / 2

        return np.array(
            [
                [-half_span, -hy, panel_z_bottom],
                [-half_span, -hy, panel_z_top],
                [half_span, -hy, panel_z_top],
                [half_span, -hy, panel_z_bottom],
                [-hx, hy, hz],
                [-hx, -hy, hz],
                [hx, -hy, hz],
                [hx, hy, hz],
                [-hx, 0.0, -hz - 0.09],
                [hx, 0.0, -hz - 0.09],
                [0.0, hy + 0.09, hz],
            ],
            dtype=np.float64,
        )

    def as_dict(self) -> Dict[str, float]:
        return {
            "body_x": self.body_x,
            "body_y": self.body_y,
            "body_z": self.body_z,
            "panel_span": self.panel_span,
            "panel_height": self.panel_height,
        }


def random_quaternion(rng: np.random.Generator) -> np.ndarray:
    """Sample a random unit quaternion in scalar-first convention [qw, qx, qy, qz]."""
    quat_xyzw = Rotation.random(random_state=rng).as_quat()
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64)


def quaternion_to_rvec(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert scalar-first quaternion to OpenCV rotation vector."""
    q = np.asarray(q_wxyz, dtype=np.float64)
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    return rot.as_rotvec().reshape(3, 1)


def rvec_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    """Convert OpenCV rotation vector to scalar-first quaternion."""
    q_xyzw = Rotation.from_rotvec(np.asarray(rvec, dtype=np.float64).reshape(3)).as_quat()
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=np.float64)


def project_points(points_3d: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    """Project target-frame 3D points into image pixels."""
    import cv2

    image_points, _ = cv2.projectPoints(points_3d, rvec, tvec, camera_matrix, None)
    return image_points.reshape(-1, 2)


def pose_errors(pred_t: np.ndarray, true_t: np.ndarray, pred_q: np.ndarray, true_q: np.ndarray) -> Tuple[float, float]:
    """Return translation error [m] and rotation error [deg]."""
    pred_t = np.asarray(pred_t, dtype=np.float64).reshape(3)
    true_t = np.asarray(true_t, dtype=np.float64).reshape(3)
    pred_q = np.asarray(pred_q, dtype=np.float64).reshape(4)
    true_q = np.asarray(true_q, dtype=np.float64).reshape(4)

    translation_error_m = float(np.linalg.norm(pred_t - true_t))
    dot = abs(float(np.dot(pred_q / np.linalg.norm(pred_q), true_q / np.linalg.norm(true_q))))
    dot = float(np.clip(dot, -1.0, 1.0))
    rotation_error_deg = float(np.rad2deg(2.0 * np.arccos(dot)))
    return translation_error_m, rotation_error_deg
