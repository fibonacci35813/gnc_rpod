#!/usr/bin/env python3

"""Sanity-check the ECI/LVLH frame utilities."""

from __future__ import annotations

import numpy as np

from rpod_gnc.bsk.frames import (
    eci_to_lvlh_frame,
    lvlh_force_to_eci,
    relative_state_eci_to_lvlh,
)


def main() -> None:
    target_r = np.array([7000e3, 0.0, 0.0])
    target_v = np.array([0.0, 7500.0, 0.0])
    chaser_r = target_r + np.array([0.0, -10.0, 0.0])
    chaser_v = target_v

    rotation_eci_to_lvlh = eci_to_lvlh_frame(target_r, target_v)
    relative_position_lvlh_m, relative_velocity_lvlh_mps, rotation_from_state = (
        relative_state_eci_to_lvlh(
            target_r,
            target_v,
            chaser_r,
            chaser_v,
        )
    )

    if not np.allclose(rotation_eci_to_lvlh, rotation_from_state):
        raise RuntimeError("Rotation matrix mismatch between frame utilities.")

    force_lvlh_n = np.array([1.0, -0.5, 0.25])
    force_eci_n = lvlh_force_to_eci(force_lvlh_n, rotation_eci_to_lvlh)

    print("Frame check: OK")
    print(f"Relative position LVLH (m): {relative_position_lvlh_m}")
    print(f"Relative velocity LVLH (m/s): {relative_velocity_lvlh_mps}")
    print(f"LVLH force (N): {force_lvlh_n}")
    print(f"ECI force (N): {force_eci_n}")


if __name__ == "__main__":
    main()
