#!/usr/bin/env python3

"""Run an open-loop end-to-end stack interface check."""

from __future__ import annotations

import numpy as np

from rpod_gnc.actuators.ideal_accel import IdealAccelActuator, IdealAccelActuatorConfig
from rpod_gnc.control.pd_controller import PDController, PDControllerConfig
from rpod_gnc.guidance.hold_point import HoldPointGuidance, HoldPointGuidanceConfig
from rpod_gnc.sensors.noisy_relative_pose import (
    NoisyRelativePoseSensor,
    NoisyRelativePoseSensorConfig,
)


def main() -> None:
    try:
        from rpod_gnc.bsk.two_spacecraft_runner import run_two_spacecraft_demo
    except ImportError as exc:
        raise SystemExit(
            "Basilisk runner import failed. Install or activate Basilisk to run this stack check."
        ) from exc

    result = run_two_spacecraft_demo(duration_s=60.0, dt_s=1.0)
    final_lvlh_state = result.relative_states_lvlh[-1]
    final_time_s = float(result.times_s[-1])

    sensor = NoisyRelativePoseSensor(
        NoisyRelativePoseSensorConfig(
            position_noise_std_m=0.01,
            dropout_probability=0.0,
            seed=7,
        )
    )
    measurement = sensor.measure(
        time_s=final_time_s,
        true_position_m=final_lvlh_state.position_m,
        true_velocity_mps=final_lvlh_state.velocity_mps,
    )

    guidance = HoldPointGuidance(
        HoldPointGuidanceConfig(
            hold_position_lvlh_m=np.array([0.0, -0.5, 0.0]),
        )
    )
    desired_position_m, desired_velocity_mps = guidance.get_reference(time_s=final_time_s)

    if measurement.velocity_mps is None:
        raise RuntimeError("Measurement velocity is unavailable for PD control.")

    controller = PDController(PDControllerConfig())
    acceleration_command_mps2 = controller.compute_acceleration(
        estimated_position_m=measurement.position_m,
        estimated_velocity_mps=measurement.velocity_mps,
        desired_position_m=desired_position_m,
        desired_velocity_mps=desired_velocity_mps,
    )

    actuator = IdealAccelActuator(IdealAccelActuatorConfig())
    actuator_output = actuator.apply(
        accel_cmd_mps2=acceleration_command_mps2,
        dt_s=1.0,
    )

    print("Stack open-loop check: OK")
    print(f"True final LVLH position (m): {final_lvlh_state.position_m}")
    print(f"Measured position (m): {measurement.position_m}")
    print(f"Desired position (m): {desired_position_m}")
    print(f"Acceleration command (m/s^2): {acceleration_command_mps2}")
    print(f"Applied acceleration (m/s^2): {actuator_output.applied_accel_mps2}")
    print(f"Delta-v step (m/s): {actuator_output.delta_v_step_mps}")


if __name__ == "__main__":
    main()
