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

#endif /* NAV_FILTER_H */
