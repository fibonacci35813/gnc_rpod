#!/usr/bin/env python3

"""Run a simple PD controller check."""

from __future__ import annotations

import numpy as np

from rpod_gnc.control.pd_controller import PDController, PDControllerConfig


def main() -> None:
    controller = PDController(PDControllerConfig())
    estimated_position_m = np.array([0.0, -10.0, 0.0])
    estimated_velocity_mps = np.array([0.0, 0.0, 0.0])
    desired_position_m = np.array([0.0, -0.5, 0.0])
    desired_velocity_mps = np.array([0.0, 0.0, 0.0])

    accel_cmd_mps2 = controller.compute_acceleration(
        estimated_position_m=estimated_position_m,
        estimated_velocity_mps=estimated_velocity_mps,
        desired_position_m=desired_position_m,
        desired_velocity_mps=desired_velocity_mps,
    )

    print("PD controller check: OK")
    print(f"Acceleration command (m/s^2): {accel_cmd_mps2}")


if __name__ == "__main__":
    main()
