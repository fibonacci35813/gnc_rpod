"""OpenGL/PnP relative pose sensor bridge for RPOD simulations."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rpod_gnc.sensors.noisy_relative_pose import RelativePoseMeasurement
from rpod_gnc.vision.camera import PinholeCamera
from rpod_gnc.vision.geometry import CubeSatModel, project_points, quaternion_to_rvec
from rpod_gnc.vision.opengl_renderer import OpenGLCubeSatRenderer


@dataclass(frozen=True)
class OpenGLPoseSensorConfig:
    position_noise_std_m: float = 0.0
    keypoint_noise_std_px: float = 1.0
    dropout_probability: float = 0.0
    seed: int = 7
    save_debug_images: bool = False
    debug_image_dir: str = "outputs/opengl_sensor_debug"


class OpenGLPoseSensor:
    """Estimate relative position using synthetic rendering plus EPnP."""

    def __init__(self, config: OpenGLPoseSensorConfig) -> None:
        if config.position_noise_std_m < 0.0:
            raise ValueError("position_noise_std_m must be non-negative.")
        if config.keypoint_noise_std_px < 0.0:
            raise ValueError("keypoint_noise_std_px must be non-negative.")
        if not 0.0 <= config.dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be between 0.0 and 1.0.")

        self._config = config
        self._rng = np.random.default_rng(config.seed)
        self._camera = PinholeCamera()
        self._model = CubeSatModel()
        self._renderer = OpenGLCubeSatRenderer(camera=self._camera, model=self._model)
        self._points_3d_m = self._model.keypoints_3d()
        self._identity_quaternion_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._identity_rvec = quaternion_to_rvec(self._identity_quaternion_wxyz)
        self._measurement_index = 0
        self._debug_image_dir = Path(config.debug_image_dir)

    def measure(
        self,
        time_s: float,
        true_position_m: np.ndarray,
        true_velocity_mps: np.ndarray | None = None,
    ) -> RelativePoseMeasurement:
        """Return an OpenGL/PnP-based relative pose measurement."""

        true_position_m = np.asarray(true_position_m, dtype=np.float64).reshape(3)
        true_velocity_array = None
        if true_velocity_mps is not None:
            true_velocity_array = np.asarray(true_velocity_mps, dtype=np.float64).reshape(3)

        try:
            from rpod_gnc.vision.pnp_pose_estimator import estimate_pose_pnp

            tvec_camera_m = true_position_m.reshape(3, 1)
            keypoints_px = project_points(
                self._points_3d_m,
                self._identity_rvec,
                tvec_camera_m,
                self._camera.matrix,
            )
            noisy_keypoints_px = keypoints_px.copy()
            if self._config.keypoint_noise_std_px > 0.0:
                noisy_keypoints_px = noisy_keypoints_px + self._rng.normal(
                    0.0,
                    self._config.keypoint_noise_std_px,
                    size=noisy_keypoints_px.shape,
                )

            render_result = self._renderer.render(
                self._identity_quaternion_wxyz,
                tvec_camera_m,
                noisy_keypoints_px,
            )
            if self._config.save_debug_images:
                filename = f"measurement_{self._measurement_index:06d}_{time_s:010.3f}.png"
                self._renderer.save_image(self._debug_image_dir / filename, render_result.rgb)

            pose_estimate = estimate_pose_pnp(
                self._points_3d_m,
                noisy_keypoints_px,
                self._camera.matrix,
            )
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "OpenGL pose sensor requires optional dependencies: cv2, pyrender, and trimesh."
            ) from exc

        measured_position_m = pose_estimate.translation_m.copy()
        if self._config.position_noise_std_m > 0.0:
            measured_position_m = measured_position_m + self._rng.normal(
                0.0,
                self._config.position_noise_std_m,
                size=3,
            )

        valid = bool(self._rng.random() >= self._config.dropout_probability)
        self._measurement_index += 1

        return RelativePoseMeasurement(
            time_s=float(time_s),
            position_m=measured_position_m,
            velocity_mps=true_velocity_array,
            valid=valid,
            position_std_m=self._config.position_noise_std_m,
        )
