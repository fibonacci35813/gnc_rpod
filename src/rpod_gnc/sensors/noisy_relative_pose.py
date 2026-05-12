"""Simple noisy relative-pose sensor model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RelativePoseMeasurement:
    time_s: float
    position_m: np.ndarray
    velocity_mps: np.ndarray | None
    valid: bool
    position_std_m: float


@dataclass(frozen=True)
class NoisyRelativePoseSensorConfig:
    position_noise_std_m: float = 0.05
    dropout_probability: float = 0.0
    seed: int = 7


class NoisyRelativePoseSensor:
    """Position-only relative pose sensor with Gaussian noise and dropouts."""

    def __init__(self, config: NoisyRelativePoseSensorConfig) -> None:
        if not 0.0 <= config.dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be between 0.0 and 1.0.")
        if config.position_noise_std_m < 0.0:
            raise ValueError("position_noise_std_m must be non-negative.")

        self._config = config
        self._rng = np.random.default_rng(config.seed)

    def measure(
        self,
        time_s: float,
        true_position_m: np.ndarray,
        true_velocity_mps: np.ndarray | None = None,
    ) -> RelativePoseMeasurement:
        """Return a noisy relative-pose measurement."""

        true_position_m = np.asarray(true_position_m, dtype=float).reshape(3)
        velocity_mps = None
        if true_velocity_mps is not None:
            velocity_mps = np.asarray(true_velocity_mps, dtype=float).reshape(3)

        position_noise_m = self._rng.normal(
            loc=0.0,
            scale=self._config.position_noise_std_m,
            size=3,
        )
        measured_position_m = true_position_m + position_noise_m
        valid = bool(self._rng.random() >= self._config.dropout_probability)

        return RelativePoseMeasurement(
            time_s=float(time_s),
            position_m=measured_position_m,
            velocity_mps=velocity_mps,
            valid=valid,
            position_std_m=self._config.position_noise_std_m,
        )
