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

#endif /* DYNAMICS_H */
