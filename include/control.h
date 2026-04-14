/**
 * @file    control.h
 * @brief   6-DOF PD controller for autonomous docking.
 *
 * Computes a force command in LVLH that drives relative position and
 * velocity errors to zero.  Applied through ±x, ±y, ±z thruster pairs.
 *
 * Control law (per axis):
 *   F_i = Kp_i · (r_ref_i − r_i) + Kd_i · (v_ref_i − v_i)
 *
 * Output is saturated to [−F_MAX, +F_MAX] per axis.
 * A minimum-impulse-bit (MIB) dead-band suppresses thruster chatter.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "gnc_types.h"

/**
 * @brief  PD gain set — one per translational axis.
 *         Radial (x), along-track (y), cross-track (z).
 */
typedef struct {
    double kp[3];    /* proportional gains (N/m)         */
    double kd[3];    /* derivative (damping) gains (N·s/m) */
} PdGains;

/**
 * @brief  Initialise PD gains to tuned defaults for the docking scenario.
 *
 * @param  gains   Output gain struct.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus ctrl_init_gains(PdGains *gains);

/**
 * @brief  Compute force command from position/velocity errors.
 *
 *   Applies saturation and MIB dead-band.
 *   Also accumulates ΔV and propellant usage.
 *
 * @param  gains     PD gain set.
 * @param  pos_err   Position error: r_ref − r_est  (m).
 * @param  vel_err   Velocity error: v_ref − v_est  (m/s).
 * @param  mass_kg   Current chaser mass (kg), must be > 0.
 * @param  dt_s      Control step (s), must be > 0.
 * @param  mib_Ns    Minimum impulse bit threshold (N·s), must be >= 0.
 *                   Pass GNC_MIN_IMPULSE_BIT normally; 0.005 in terminal.
 * @param  cmd       Output control command.
 * @param  fuel      Fuel accounting state (updated in place).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus ctrl_compute(
    const PdGains *gains,
    const Vec3    *pos_err,
    const Vec3    *vel_err,
    double         mass_kg,
    double         dt_s,
    double         mib_Ns,
    ControlCmd    *cmd,
    FuelState     *fuel);

/**
 * @brief  Compute Euclidean norm of a Vec3.
 * @param  v    Input vector.
 * @param  norm Output norm.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus ctrl_vec3_norm(const Vec3 *v, double *norm);

/**
 * @brief  Subtract two Vec3: result = a - b.
 * @param  a, b     Input vectors.
 * @param  result   Output vector.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus ctrl_vec3_sub(const Vec3 *a, const Vec3 *b, Vec3 *result);

/**
 * @brief  Override gains with high-bandwidth terminal values (range < 5 m).
 *
 *   Called once when the chaser enters the terminal zone.
 *   Increases proportional gain 4× and derivative gain 3× for tight
 *   final approach.  Should only be called in the terminal phase.
 *
 * @param  gains  Gains struct to overwrite.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus ctrl_apply_terminal_gains(PdGains *gains);

#endif /* CONTROL_H */
