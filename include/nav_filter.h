/**
 * @file    nav_filter.h
 * @brief   6-state linear Kalman filter for relative navigation.
 *
 * State vector: X = [x, y, z, vx, vy, vz]ᵀ (relative LVLH, metres/m·s⁻¹)
 *
 * Propagation:  X_{k+1} = Φ·X_k + Γ·u_k   (CW dynamics)
 * Measurement:  z_k     = H·X_k + v_k      (LIDAR range + bearing)
 *
 * H selects the position states:  H = [I₃ | 0₃]
 */

#ifndef NAV_FILTER_H
#define NAV_FILTER_H

#include "gnc_types.h"

/**
 * @brief  Initialise navigation filter with prior state estimate.
 *
 * @param  nav        State to initialise.
 * @param  pos0       Initial position estimate (m).
 * @param  vel0       Initial velocity estimate (m/s).
 * @param  pos_sigma  1-σ position uncertainty (m), must be > 0.
 * @param  vel_sigma  1-σ velocity uncertainty (m/s), must be > 0.
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus nav_init(
    NavState   *nav,
    const Vec3 *pos0,
    const Vec3 *vel0,
    double      pos_sigma,
    double      vel_sigma);

/**
 * @brief  Kalman propagation step (time update).
 *
 *   X⁻ = Φ·X + Γ·u
 *   P⁻ = Φ·P·Φᵀ + Q
 *
 * @param  nav       Filter state (updated in place).
 * @param  force_N   Control force applied over last Δt (N).
 * @param  n         Orbit mean motion (rad/s).
 * @param  dt_s      Time step (s).
 * @param  mass_kg   Chaser mass (kg).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus nav_propagate(
    NavState   *nav,
    const Vec3 *force_N,
    double      n,
    double      dt_s,
    double      mass_kg);

/**
 * @brief  Kalman measurement update step.
 *
 *   y  = z − H·X⁻
 *   S  = H·P⁻·Hᵀ + R
 *   K  = P⁻·Hᵀ·S⁻¹
 *   X  = X⁻ + K·y
 *   P  = (I − K·H)·P⁻
 *
 * @param  nav      Filter state (updated in place).
 * @param  meas_pos Measured position from sensor (m).
 * @param  meas_sigma Sensor 1-σ noise per axis (m), must be > 0.
 * @return GNC_OK, ERR_NULL_PTR, ERR_BAD_PARAM, or ERR_SINGULAR.
 */
GncStatus nav_update(
    NavState   *nav,
    const Vec3 *meas_pos,
    double      meas_sigma);

/* -----------------------------------------------------------------------
 * Phase 8 — EKF with RAE measurements + IMU-driven propagation
 * ----------------------------------------------------------------------- */

/**
 * @brief  EKF measurement update with range-azimuth-elevation (RAE).
 *
 * Nonlinear measurement model:
 *   h(x) = [sqrt(x²+y²+z²), atan2(y,x), asin(z/r)]
 * Linearised Jacobian H = dh/dx (3×6) computed at current estimate.
 *
 * @param  nav        Filter state (updated in place).
 * @param  range_m    Measured range (m), must be > 0.
 * @param  az_rad     Measured azimuth (rad).
 * @param  el_rad     Measured elevation (rad).
 * @param  sigma_r    Range noise 1-σ (m), must be > 0.
 * @param  sigma_ang  Angle noise 1-σ (rad), must be > 0.
 * @return GNC_OK, ERR_NULL_PTR, ERR_BAD_PARAM, or ERR_SINGULAR.
 */
GncStatus nav_update_ekf(
    NavState *nav,
    double    range_m,
    double    az_rad,
    double    el_rad,
    double    sigma_r,
    double    sigma_ang);

/**
 * @brief  Propagate covariance only using CW STM (no mean state change).
 *
 * Used in the two-rate loop: mean state propagated by IMU at high rate,
 * covariance updated once per outer (LIDAR) step.
 *
 * @param  nav    Filter state (covariance updated in place).
 * @param  n      Orbit mean motion (rad/s).
 * @param  dt_s   Time step for CW STM (s).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus nav_propagate_cov_only(NavState *nav, double n, double dt_s);

/**
 * @brief  Propagate mean state using IMU measurement at inner-loop rate.
 *
 * Euler integration augmented with CW gravity-gradient coupling terms:
 *   a_eff = a_imu + [2n*vy + 3n²*x, -2n*vx, -n²*z]
 * This accounts for the tidal accelerations that the accelerometer
 * cannot sense (they are part of the free-fall reference trajectory).
 *
 * Does NOT update covariance (call nav_propagate_cov_only once per
 * outer 1-Hz step after running 10 inner IMU steps).
 *
 * @param  nav        Filter state (mean state updated in place).
 * @param  meas_accel IMU-measured specific force in LVLH frame (m/s^2).
 * @param  n          Orbit mean motion (rad/s).
 * @param  dt_s       Inner-loop time step (s), typically 0.1 s.
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus nav_propagate_imu(
    NavState   *nav,
    const Vec3 *meas_accel,
    double      n,
    double      dt_s);

#endif /* NAV_FILTER_H */
