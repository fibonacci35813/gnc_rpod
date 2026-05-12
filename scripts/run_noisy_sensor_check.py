#!/usr/bin/env python3

"""Run a simple check of the noisy relative-pose sensor."""

from __future__ import annotations

import numpy as np

from rpod_gnc.sensors.noisy_relative_pose import (
    NoisyRelativePoseSensor,
    NoisyRelativePoseSensorConfig,
)


def main() -> None:
    sensor = NoisyRelativePoseSensor(
        NoisyRelativePoseSensorConfig(
            position_noise_std_m=0.05,
            dropout_probability=0.0,
        )
    )
    true_position_m = np.array([0.0, -10.0, 0.0])
    measurement = sensor.measure(time_s=0.0, true_position_m=true_position_m)

    print("Noisy sensor check: OK")
    print(f"True position (m): {true_position_m}")
    print(f"Measured position (m): {measurement.position_m}")


if __name__ == "__main__":
    main()
