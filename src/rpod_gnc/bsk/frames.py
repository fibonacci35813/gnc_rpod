"""Frame utilities for Basilisk-backed RPOD simulations.

Basilisk provides spacecraft states in ECI/inertial coordinates.
RPOD guidance and control typically operate in the target-centered LVLH frame.

LVLH convention used here:
  x = radial outward from Earth to target
  y = along-track
  z = orbit-normal
"""

from __future__ import annotations

import numpy as np


def _as_vector(vector: np.ndarray) -> np.ndarray:
    """Return a float 3-vector."""

    return np.asarray(vector, dtype=float).reshape(3)


def _normalize(vector: np.ndarray, name: str) -> np.ndarray:
    """Return a unit vector and fail on degenerate inputs."""

    norm = float(np.linalg.norm(vector))
    if norm <= 0.0:
        raise ValueError(f"{name} must be non-zero.")
    return vector / norm


def eci_to_lvlh_frame(
    target_position_eci_m: np.ndarray,
    target_velocity_eci_mps: np.ndarray,
) -> np.ndarray:
    """Return the 3x3 rotation matrix from ECI to the target LVLH frame.

    The returned matrix `R` maps ECI vectors into LVLH coordinates as:
    `vector_lvlh = R @ vector_eci`.
    """

    target_position_eci_m = _as_vector(target_position_eci_m)
    target_velocity_eci_mps = _as_vector(target_velocity_eci_mps)

    radial_hat_eci = _normalize(target_position_eci_m, "target_position_eci_m")
    orbit_normal_eci = np.cross(target_position_eci_m, target_velocity_eci_mps)
    orbit_normal_hat_eci = _normalize(orbit_normal_eci, "target angular momentum")
    along_track_hat_eci = _normalize(
        np.cross(orbit_normal_hat_eci, radial_hat_eci),
        "target along-track direction",
    )

    return np.vstack(
        (
            radial_hat_eci,
            along_track_hat_eci,
            orbit_normal_hat_eci,
        )
    )


def relative_state_eci_to_lvlh(
    target_position_eci_m: np.ndarray,
    target_velocity_eci_mps: np.ndarray,
    chaser_position_eci_m: np.ndarray,
    chaser_velocity_eci_mps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the chaser state relative to the target in LVLH coordinates."""

    target_position_eci_m = _as_vector(target_position_eci_m)
    target_velocity_eci_mps = _as_vector(target_velocity_eci_mps)
    chaser_position_eci_m = _as_vector(chaser_position_eci_m)
    chaser_velocity_eci_mps = _as_vector(chaser_velocity_eci_mps)

    rotation_eci_to_lvlh = eci_to_lvlh_frame(
        target_position_eci_m,
        target_velocity_eci_mps,
    )

    relative_position_eci_m = chaser_position_eci_m - target_position_eci_m
    relative_velocity_eci_mps = chaser_velocity_eci_mps - target_velocity_eci_mps

    radius_squared_m2 = float(np.dot(target_position_eci_m, target_position_eci_m))
    omega_eci_radps = np.cross(target_position_eci_m, target_velocity_eci_mps) / radius_squared_m2

    relative_position_lvlh_m = rotation_eci_to_lvlh @ relative_position_eci_m
    relative_velocity_lvlh_mps = rotation_eci_to_lvlh @ (
        relative_velocity_eci_mps - np.cross(omega_eci_radps, relative_position_eci_m)
    )

    return (
        relative_position_lvlh_m,
        relative_velocity_lvlh_mps,
        rotation_eci_to_lvlh,
    )


def lvlh_force_to_eci(
    force_lvlh_n: np.ndarray,
    rotation_eci_to_lvlh: np.ndarray,
) -> np.ndarray:
    """Return a force vector expressed in ECI coordinates."""

    force_lvlh_n = _as_vector(force_lvlh_n)
    rotation_eci_to_lvlh = np.asarray(rotation_eci_to_lvlh, dtype=float).reshape(3, 3)

    return rotation_eci_to_lvlh.T @ force_lvlh_n
