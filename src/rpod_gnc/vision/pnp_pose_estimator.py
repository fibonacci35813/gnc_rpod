"""PnP-based 6-DOF pose estimation baseline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np

from .geometry import pose_errors, rvec_to_quaternion


@dataclass(frozen=True)
class PoseEstimate:
    translation_m: np.ndarray
    quaternion_wxyz: np.ndarray
    reprojection_error_px: float


def estimate_pose_pnp(
    keypoints_3d_m: np.ndarray,
    keypoints_2d_px: np.ndarray,
    camera_matrix: np.ndarray,
) -> PoseEstimate:
    """Estimate target pose from 2D-3D correspondences using EPnP + iterative refinement."""
    object_points = np.asarray(keypoints_3d_m, dtype=np.float64)
    image_points = np.asarray(keypoints_2d_px, dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        None,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok:
        raise RuntimeError("cv2.solvePnP failed to estimate a pose")

    rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, camera_matrix, None, rvec, tvec)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, None)
    reprojection_error = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)))

    return PoseEstimate(
        translation_m=tvec.reshape(3),
        quaternion_wxyz=rvec_to_quaternion(rvec),
        reprojection_error_px=reprojection_error,
    )


def evaluate_pose_estimate(
    estimate: PoseEstimate,
    true_translation_m: np.ndarray,
    true_quaternion_wxyz: np.ndarray,
) -> Tuple[float, float]:
    return pose_errors(
        estimate.translation_m,
        true_translation_m,
        estimate.quaternion_wxyz,
        true_quaternion_wxyz,
    )
