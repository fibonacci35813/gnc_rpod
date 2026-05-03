"""Evaluate synthetic pose labels with the PnP baseline."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from .camera import PinholeCamera
from .pnp_pose_estimator import estimate_pose_pnp, evaluate_pose_estimate


def evaluate_metadata(metadata_path: str | Path, pixel_noise_std: float = 0.0, seed: int = 11) -> Dict[str, Any]:
    metadata_path = Path(metadata_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    cam_info = metadata["camera"]
    camera = PinholeCamera(
        width=int(cam_info["width"]),
        height=int(cam_info["height"]),
        horizontal_fov_deg=float(cam_info["horizontal_fov_deg"]),
    )
    rng = np.random.default_rng(seed)

    rows: List[Dict[str, float]] = []
    for sample in metadata["samples"]:
        keypoints_2d = np.asarray(sample["keypoints_2d_px"], dtype=np.float64)
        if pixel_noise_std > 0:
            keypoints_2d = keypoints_2d + rng.normal(0.0, pixel_noise_std, size=keypoints_2d.shape)

        estimate = estimate_pose_pnp(
            np.asarray(sample["keypoints_3d_m"], dtype=np.float64),
            keypoints_2d,
            camera.matrix,
        )
        et, er = evaluate_pose_estimate(
            estimate,
            np.asarray(sample["translation_m"], dtype=np.float64),
            np.asarray(sample["quaternion_wxyz"], dtype=np.float64),
        )
        rows.append(
            {
                "translation_error_m": et,
                "rotation_error_deg": er,
                "reprojection_error_px": estimate.reprojection_error_px,
            }
        )

    translation_errors = np.array([r["translation_error_m"] for r in rows])
    rotation_errors = np.array([r["rotation_error_deg"] for r in rows])
    reprojection_errors = np.array([r["reprojection_error_px"] for r in rows])

    return {
        "num_samples": len(rows),
        "pixel_noise_std": pixel_noise_std,
        "translation_error_m": {
            "mean": float(np.mean(translation_errors)),
            "std": float(np.std(translation_errors)),
            "max": float(np.max(translation_errors)),
        },
        "rotation_error_deg": {
            "mean": float(np.mean(rotation_errors)),
            "std": float(np.std(rotation_errors)),
            "max": float(np.max(rotation_errors)),
        },
        "reprojection_error_px": {
            "mean": float(np.mean(reprojection_errors)),
            "std": float(np.std(reprojection_errors)),
            "max": float(np.max(reprojection_errors)),
        },
    }
