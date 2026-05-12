"""Simple hold-point guidance for RPOD simulations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HoldPointGuidanceConfig:
    hold_position_lvlh_m: np.ndarray


class HoldPointGuidance:
    """Return a fixed LVLH hold-point reference."""

    def __init__(self, config: HoldPointGuidanceConfig) -> None:
        self._hold_position_lvlh_m = np.asarray(
            config.hold_position_lvlh_m,
            dtype=float,
        ).reshape(3)

    def get_reference(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        """Return desired LVLH position and velocity at the given time."""

        del time_s
        desired_position_lvlh_m = self._hold_position_lvlh_m.copy()
        desired_velocity_lvlh_mps = np.zeros(3, dtype=float)
        return desired_position_lvlh_m, desired_velocity_lvlh_mps
