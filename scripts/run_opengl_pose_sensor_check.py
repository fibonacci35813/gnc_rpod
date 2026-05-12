#!/usr/bin/env python3

"""Run a simple OpenGL/PnP pose sensor check."""

from __future__ import annotations

import numpy as np

from rpod_gnc.sensors.opengl_pose_sensor import OpenGLPoseSensor, OpenGLPoseSensorConfig


def main() -> None:
    sensor = OpenGLPoseSensor(OpenGLPoseSensorConfig())
    true_position_m = np.array([0.0, 0.0, 5.0])
    try:
        measurement = sensor.measure(time_s=0.0, true_position_m=true_position_m)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    position_error_m = float(np.linalg.norm(measurement.position_m - true_position_m))

    print("OpenGL pose sensor check: OK")
    print(f"True position (m): {true_position_m}")
    print(f"Estimated position (m): {measurement.position_m}")
    print(f"Position error (m): {position_error_m}")


if __name__ == "__main__":
    main()
