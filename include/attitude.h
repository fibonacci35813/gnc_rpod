/**
 * @file    attitude.h
 * @brief   Quaternion attitude kinematics and PD torque control.
 *
 * Coordinate conventions:
 *   q = [w, x, y, z]  (scalar-first)
 *   Identity quaternion: q = [1, 0, 0, 0]
 *   Rotation: body-to-inertial (passive convention)
 *
 * Pointing law:
 *   Phase 0-2 : maintain initial attitude (coast — no torque)
 *   Phase 3   : rotate body -y axis to point at target (align docking port)
 *               Tolerance: pointing error < ATT_DOCK_TOL_RAD before docking
 *
 * NASA Power of 10 compliant.
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "gnc_types.h"

/* Attitude tolerances */
#define ATT_DOCK_TOL_RAD   0.01745  /* 1 degree in radians               */
#define ATT_MIB_NMS        0.001    /* minimum torque impulse (N·m·s)    */
#define ATT_KP             0.5      /* proportional gain (N·m/rad)        */
#define ATT_KD             2.0      /* derivative gain   (N·m·s/rad)      */

/**
 * @brief  Initialise attitude state to identity quaternion, zero rates.
 * @param  att  Output attitude state.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus att_init(AttState *att);

/**
 * @brief  Propagate quaternion kinematics one step using RK4.
 *
 *   q_dot = 0.5 * q ⊗ [0, omega]
 *
 * @param  att    Attitude state (updated in place).
 * @param  dt_s   Time step (s).
 * @return GNC_OK or error code.
 */
GncStatus att_kinematics(AttState *att, double dt_s);

/**
 * @brief  Compute attitude error quaternion: q_err = q_cmd^-1 ⊗ q.
 *
 *   The vector part of q_err approximates the small-angle rotation error.
 *
 * @param  q_cmd    Commanded quaternion [w,x,y,z].
 * @param  q_curr   Current quaternion   [w,x,y,z].
 * @param  q_err    Output error quaternion [w,x,y,z].
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus att_error(const double q_cmd[4], const double q_curr[4],
                    double q_err[4]);

/**
 * @brief  PD torque control law.
 *
 *   torque = -Kp * q_err_vec - Kd * omega_err
 *
 *   where q_err_vec = [q_err[1], q_err[2], q_err[3]]
 *         omega_err = omega - omega_cmd  (omega_cmd = 0 for hold)
 *
 * @param  q_err_vec   Vector part of attitude error quaternion [3].
 * @param  omega_err   Angular velocity error (rad/s) [3].
 * @param  mib_Nms     Minimum impulse bit threshold (N·m·s).
 * @param  dt_s        Time step (s).
 * @param  cmd         Output torque command.
 * @return GNC_OK or error code.
 */
GncStatus att_pd_control(
    const double q_err_vec[3],
    const double omega_err[3],
    double       mib_Nms,
    double       dt_s,
    AttCmd      *cmd);

/**
 * @brief  Build docking-port pointing command quaternion.
 *
 *   Computes q_cmd such that the body -y axis points toward the target
 *   (i.e. docking port aligned with approach corridor).
 *   When already aligned (pos_lvlh is near zero), returns identity.
 *
 * @param  pos_lvlh   Relative position vector in LVLH (m) [3].
 * @param  q_cmd      Output command quaternion [w,x,y,z].
 * @return GNC_OK or error code.
 */
GncStatus att_docking_cmd(const double pos_lvlh[3], double q_cmd[4]);

/**
 * @brief  Check whether pointing error is within docking tolerance.
 *
 * @param  q_err     Error quaternion [w,x,y,z].
 * @param  aligned   Output: 1 if aligned, 0 otherwise.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus att_check_aligned(const double q_err[4], uint8_t *aligned);

#endif /* ATTITUDE_H */
