#!/usr/bin/env python3

"""Run a simple hold-point guidance check."""

from __future__ import annotations

import numpy as np

from rpod_gnc.guidance.hold_point import HoldPointGuidance, HoldPointGuidanceConfig


def main() -> None:
    guidance = HoldPointGuidance(
        HoldPointGuidanceConfig(
            hold_position_lvlh_m=np.array([0.0, -0.5, 0.0]),
        )
    )
    desired_position_m, desired_velocity_mps = guidance.get_reference(time_s=0.0)

    print("Guidance check: OK")
    print(f"Desired position LVLH (m): {desired_position_m}")
    print(f"Desired velocity LVLH (m/s): {desired_velocity_mps}")


if __name__ == "__main__":
    main()
