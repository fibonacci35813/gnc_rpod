#!/usr/bin/env python3

"""Run a simple ideal actuator check."""

from __future__ import annotations

import numpy as np

from rpod_gnc.actuators.ideal_accel import IdealAccelActuator, IdealAccelActuatorConfig


def main() -> None:
    actuator = IdealAccelActuator(IdealAccelActuatorConfig())
    accel_cmd_mps2 = np.array([0.0, 0.03, 0.0])
    output = actuator.apply(accel_cmd_mps2=accel_cmd_mps2, dt_s=1.0)

    print("Actuator check: OK")
    print(f"Command accel (m/s^2): {accel_cmd_mps2}")
    print(f"Applied accel (m/s^2): {output.applied_accel_mps2}")
    print(f"Delta-v step (m/s): {output.delta_v_step_mps}")


if __name__ == "__main__":
    main()
