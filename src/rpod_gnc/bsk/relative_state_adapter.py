"""Relative state utilities for Basilisk backed RPOD SIL prototypes.
Basilisk gives spacecraft states in an inertial frame. 
For RPOD guidance/control we need relative states in a local-vertical-local-horizontal (LVLH) frame.

r_rel = r_chaser - r_target
v_rel = v_chaser - v_target

"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

@dataclass(frozen=True)
class RelativeState:
    """Relative state of chaser with respect to target in LVLH frame."""

    position_m: np.ndarray
    velocity_mps: np.ndarray

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.position_m))
    
    @property
    def closing_speed_mps(self) -> float:
        if self.range_m < 1e-9:
            return 0.0
        radial_unit = self.position_m / self.range_m
        return float(np.dot(self.velocity_mps, radial_unit))
    
def compute_relative_state(
        target_position_m: np.ndarray,
        target_velocity_mps: np.ndarray,
        chaser_position_m: np.ndarray,
        chaser_velocity_mps: np.ndarray,
) -> RelativeState:
    
    target_position_m = np.asarray(target_position_m, dtype=float).reshape(3)
    target_velocity_mps = np.asarray(target_velocity_mps, dtype=float).reshape(3)
    chaser_position_m = np.asarray(chaser_position_m, dtype=float).reshape(3)
    chaser_velocity_mps = np.asarray(chaser_velocity_mps, dtype=float).reshape(3)

    return RelativeState(
        position_m=chaser_position_m - target_position_m,
        velocity_mps=chaser_velocity_mps - target_velocity_mps,
    )