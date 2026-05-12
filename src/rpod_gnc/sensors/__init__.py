"""Sensor models for RPOD simulations."""

from rpod_gnc.sensors.noisy_relative_pose import (
    NoisyRelativePoseSensor,
    NoisyRelativePoseSensorConfig,
    RelativePoseMeasurement,
)

__all__ = [
    "NoisyRelativePoseSensor",
    "NoisyRelativePoseSensorConfig",
    "RelativePoseMeasurement",
]

try:
    from rpod_gnc.sensors.opengl_pose_sensor import OpenGLPoseSensor, OpenGLPoseSensorConfig

    __all__.extend(
        [
            "OpenGLPoseSensor",
            "OpenGLPoseSensorConfig",
        ]
    )
except ModuleNotFoundError:
    pass
