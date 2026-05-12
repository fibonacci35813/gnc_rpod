"""Simple PD translational controller for RPOD simulations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PDControllerConfig:
    kp: float = 0.002
    kd: float = 0.08
    max_accel_mps2: float = 0.02


class PDController:
    """Compute a saturated translational acceleration command."""

    def __init__(self, config: PDControllerConfig) -> None:
        if config.max_accel_mps2 < 0.0:
            raise ValueError("max_accel_mps2 must be non-negative.")

        self._config = config

    def compute_acceleration(
        self,
        estimated_position_m: np.ndarray,
        estimated_velocity_mps: np.ndarray,
        desired_position_m: np.ndarray,
        desired_velocity_mps: np.ndarray,
    ) -> np.ndarray:
        """Return the PD acceleration command with norm saturation."""

        estimated_position_m = np.asarray(estimated_position_m, dtype=float).reshape(3)
        estimated_velocity_mps = np.asarray(estimated_velocity_mps, dtype=float).reshape(3)
        desired_position_m = np.asarray(desired_position_m, dtype=float).reshape(3)
        desired_velocity_mps = np.asarray(desired_velocity_mps, dtype=float).reshape(3)

        error_pos = desired_position_m - estimated_position_m
        error_vel = desired_velocity_mps - estimated_velocity_mps
        accel_cmd = self._config.kp * error_pos + self._config.kd * error_vel

        accel_norm = float(np.linalg.norm(accel_cmd))
        if accel_norm > self._config.max_accel_mps2 and accel_norm > 0.0:
            accel_cmd = accel_cmd * (self._config.max_accel_mps2 / accel_norm)

        return accel_cmd
