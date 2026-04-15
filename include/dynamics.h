/**
 * @file    dynamics.h
 * @brief   Clohessy-Wiltshire (CW) relative dynamics propagator.
 *
 * Implements the linearised Hill-Clohessy-Wiltshire equations of motion
 * for spacecraft relative motion in LVLH coordinates.
 *
 *   ẍ − 2n·ẏ − 3n²·x = fx / m
 *   ÿ + 2n·ẋ         = fy / m
 *   z̈ + n²·z         = fz / m
 *
 * All computations in double precision.  No dynamic allocation.
 */

#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "gnc_types.h"

/**
 * @brief  Compute mean motion n (rad/s) for a circular orbit.
 * @param  semi_major_m  Semi-major axis in metres (must be > 0).
 * @param  n_out         Output mean motion (rad/s).
 * @return GNC_OK on success, ERR_BAD_PARAM if semi_major_m <= 0.
 */
GncStatus dyn_mean_motion(double semi_major_m, double *n_out);

/**
 * @brief  Propagate relative state one time step using CW equations.
 *
 *   Uses the analytical CW state-transition matrix Φ(Δt) for precision.
 *   Applied force is treated as constant over [t, t+dt].
 *
 * @param  pos_in    Initial relative position (m), LVLH.
 * @param  vel_in    Initial relative velocity (m/s), LVLH.
 * @param  force_N   Applied force vector (N), LVLH.
 * @param  n         Mean motion (rad/s), must be > 0.
 * @param  dt_s      Time step (s), must be > 0.
 * @param  mass_kg   Chaser mass (kg), must be > 0.
 * @param  pos_out   Propagated position (m).
 * @param  vel_out   Propagated velocity (m/s).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus dyn_propagate(
    const Vec3 *pos_in,
    const Vec3 *vel_in,
    const Vec3 *force_N,
    double      n,
    double      dt_s,
    double      mass_kg,
    Vec3       *pos_out,
    Vec3       *vel_out);

/**
 * @brief  Build the 6×6 CW discrete state-transition matrix Φ(Δt).
 *
 * @param  n     Mean motion (rad/s), must be > 0.
 * @param  dt_s  Time step (s), must be > 0.
 * @param  phi   Output 6×6 matrix.
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus dyn_build_phi(double n, double dt_s, Mat6x6 *phi);

/**
 * @brief  Compute the scalar range (Euclidean norm) from pos.
 * @param  pos    Position vector.
 * @param  range  Output range (m).
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus dyn_range(const Vec3 *pos, double *range);

/* -----------------------------------------------------------------------
 * Phase 7: high-fidelity environment perturbation model
 *
 * True dynamics use J2 + drag; onboard GNC stays with plain CW.
 * Perturbations are DIFFERENTIAL (chaser minus target reference).
 * ----------------------------------------------------------------------- */

/** J2 oblateness coefficient (dimensionless). */
#define DYN_J2       1.08263e-3
/** Earth mean equatorial radius (m). */
#define DYN_RE_M     6.371e6
/** Sea-level atmospheric density (kg/m^3). */
#define DYN_RHO0     1.225
/** Density scale height (m) — valid troposphere, tiny at orbit altitude. */
#define DYN_H_SCALE  8500.0

/**
 * @brief  J2 oblateness perturbation acceleration at r_eci.
 *
 * Uses the standard zonal harmonic formula in ECI Cartesian coordinates:
 *   a_x = coeff * x * (1 - 5*(z/r)^2)
 *   a_y = coeff * y * (1 - 5*(z/r)^2)
 *   a_z = coeff * z * (3 - 5*(z/r)^2)
 *   coeff = -1.5 * J2 * mu * Re^2 / r^5
 *
 * @param  r_eci     ECI position vector (m); magnitude must exceed Re.
 * @param  a_j2_out  Output J2 acceleration (m/s^2).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus dyn_j2_accel(const Vec3 *r_eci, Vec3 *a_j2_out);

/**
 * @brief  Atmospheric drag acceleration at r_eci with velocity v_eci.
 *
 * Exponential atmosphere model: rho = rho0 * exp(-alt / H_scale).
 * Drag: a_drag = -0.5 * rho * Cd * (area/mass) * |v| * v
 *
 * @param  r_eci       ECI position (m).
 * @param  v_eci       ECI velocity (m/s); must be non-zero.
 * @param  env         Environment model (Cd, area_m2).
 * @param  mass_kg     Chaser mass (kg).
 * @param  a_drag_out  Output drag acceleration (m/s^2).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus dyn_drag_accel(
    const Vec3     *r_eci,
    const Vec3     *v_eci,
    const EnvModel *env,
    double          mass_kg,
    Vec3           *a_drag_out);

/**
 * @brief  Propagate relative state one step using CW + perturbations.
 *
 * CW base step is run first.  Differential perturbations (J2 and/or drag)
 * are then applied as constant-acceleration corrections over dt.
 * The nav filter always uses plain dyn_propagate (CW only).
 *
 * @param  pos_in   Initial relative position (m), LVLH.
 * @param  vel_in   Initial relative velocity (m/s), LVLH.
 * @param  force_N  Applied force vector (N), LVLH.
 * @param  env      Environment flags (use_j2, use_drag, Cd, area_m2).
 * @param  n        Mean motion (rad/s).
 * @param  dt_s     Time step (s).
 * @param  mass_kg  Chaser mass (kg).
 * @param  pos_out  Propagated position (m).
 * @param  vel_out  Propagated velocity (m/s).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus dyn_propagate_perturbed(
    const Vec3     *pos_in,
    const Vec3     *vel_in,
    const Vec3     *force_N,
    const EnvModel *env,
    double          n,
    double          dt_s,
    double          mass_kg,
    Vec3           *pos_out,
    Vec3           *vel_out);

#endif /* DYNAMICS_H */
