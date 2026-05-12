"""Ideal acceleration actuator for RPOD simulations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class IdealAccelActuatorConfig:
    max_accel_mps2: float = 0.02


@dataclass(frozen=True)
class ActuatorOutput:
    applied_accel_mps2: np.ndarray
    delta_v_step_mps: float


class IdealAccelActuator:
    """Apply an acceleration command with ideal norm saturation."""

    def __init__(self, config: IdealAccelActuatorConfig) -> None:
        if config.max_accel_mps2 < 0.0:
            raise ValueError("max_accel_mps2 must be non-negative.")

        self._config = config

    def apply(self, accel_cmd_mps2: np.ndarray, dt_s: float) -> ActuatorOutput:
        """Return the saturated acceleration and delta-v for the step."""

        if dt_s < 0.0:
            raise ValueError("dt_s must be non-negative.")

        applied_accel_mps2 = np.asarray(accel_cmd_mps2, dtype=float).reshape(3)
        accel_norm = float(np.linalg.norm(applied_accel_mps2))
        if accel_norm > self._config.max_accel_mps2 and accel_norm > 0.0:
            applied_accel_mps2 = applied_accel_mps2 * (
                self._config.max_accel_mps2 / accel_norm
            )
            accel_norm = self._config.max_accel_mps2

        return ActuatorOutput(
            applied_accel_mps2=applied_accel_mps2,
            delta_v_step_mps=accel_norm * float(dt_s),
        )
