/**
 * @file    src/attitude.c
 * @brief   Quaternion attitude kinematics and PD torque control.
 *          NASA Power of 10 compliant.
 *
 * Quaternion convention: q = [w, x, y, z] (scalar first).
 * q_dot = 0.5 * q ⊗ [0, omega_x, omega_y, omega_z]
 *
 * Product rule (Hamilton):
 *   (p ⊗ q)_w = pw*qw - px*qx - py*qy - pz*qz
 *   (p ⊗ q)_x = pw*qx + px*qw + py*qz - pz*qy
 *   (p ⊗ q)_y = pw*qy - px*qz + py*qw + pz*qx
 *   (p ⊗ q)_z = pw*qz + px*qy - py*qx + pz*qw
 */

#include <math.h>
#include "attitude.h"
#include "gnc_assert.h"

/* Maximum torque per axis (N·m) */
#define ATT_MAX_TORQUE  10.0

/* -----------------------------------------------------------------------
 * Internal: normalise a quaternion in-place.
 * ----------------------------------------------------------------------- */
static GncStatus quat_normalise(double q[4])
{
    GNC_ASSERT(q != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double mag2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    GNC_ASSERT(mag2 > 1.0e-12, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double inv = 1.0 / sqrt(mag2);
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Internal: quaternion product r = p ⊗ q (Hamilton convention)
 * ----------------------------------------------------------------------- */
static GncStatus quat_mul(const double p[4], const double q[4], double r[4])
{
    GNC_ASSERT(p != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(q != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(r != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    r[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    r[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    r[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    r[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Internal: one RK4 sub-step for quaternion kinematics
 *   q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
 * ----------------------------------------------------------------------- */
static GncStatus quat_deriv(const double q[4], const double omega[3],
                             double qdot[4])
{
    GNC_ASSERT(q     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(omega != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(qdot  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* omega_quat = [0, wx, wy, wz] */
    double om[4];
    om[0] = 0.0;
    om[1] = omega[0];
    om[2] = omega[1];
    om[3] = omega[2];

    double tmp[4];
    GncStatus rc = quat_mul(q, om, tmp);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    qdot[0] = 0.5 * tmp[0];
    qdot[1] = 0.5 * tmp[1];
    qdot[2] = 0.5 * tmp[2];
    qdot[3] = 0.5 * tmp[3];
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

GncStatus att_init(AttState *att)
{
    GNC_ASSERT(att != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    att->q[0] = 1.0;    /* identity: w=1 */
    att->q[1] = 0.0;
    att->q[2] = 0.0;
    att->q[3] = 0.0;
    att->omega[0] = 0.0;
    att->omega[1] = 0.0;
    att->omega[2] = 0.0;

    GNC_ASSERT(att->q[0] > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

GncStatus att_kinematics(AttState *att, double dt_s)
{
    GNC_ASSERT(att  != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(dt_s >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* RK4 integration of q_dot = 0.5 * q ⊗ [0, omega] */
    double k1[4];
    double k2[4];
    double k3[4];
    double k4[4];
    double q_tmp[4];
    GncStatus rc;

    /* k1 = f(q) */
    rc = quat_deriv(att->q, att->omega, k1);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* k2 = f(q + 0.5*dt*k1) */
    q_tmp[0] = att->q[0] + 0.5*dt_s*k1[0];
    q_tmp[1] = att->q[1] + 0.5*dt_s*k1[1];
    q_tmp[2] = att->q[2] + 0.5*dt_s*k1[2];
    q_tmp[3] = att->q[3] + 0.5*dt_s*k1[3];
    rc = quat_deriv(q_tmp, att->omega, k2);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* k3 = f(q + 0.5*dt*k2) */
    q_tmp[0] = att->q[0] + 0.5*dt_s*k2[0];
    q_tmp[1] = att->q[1] + 0.5*dt_s*k2[1];
    q_tmp[2] = att->q[2] + 0.5*dt_s*k2[2];
    q_tmp[3] = att->q[3] + 0.5*dt_s*k2[3];
    rc = quat_deriv(q_tmp, att->omega, k3);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* k4 = f(q + dt*k3) */
    q_tmp[0] = att->q[0] + dt_s*k3[0];
    q_tmp[1] = att->q[1] + dt_s*k3[1];
    q_tmp[2] = att->q[2] + dt_s*k3[2];
    q_tmp[3] = att->q[3] + dt_s*k3[3];
    rc = quat_deriv(q_tmp, att->omega, k4);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* q_new = q + dt/6*(k1 + 2*k2 + 2*k3 + k4) */
    double inv6 = dt_s / 6.0;
    att->q[0] += inv6*(k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]);
    att->q[1] += inv6*(k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]);
    att->q[2] += inv6*(k1[2] + 2.0*k2[2] + 2.0*k3[2] + k4[2]);
    att->q[3] += inv6*(k1[3] + 2.0*k2[3] + 2.0*k3[3] + k4[3]);

    rc = quat_normalise(att->q);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    return GNC_OK;
}

GncStatus att_error(const double q_cmd[4], const double q_curr[4],
                    double q_err[4])
{
    GNC_ASSERT(q_cmd  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(q_curr != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(q_err  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* q_cmd_inv = [qw, -qx, -qy, -qz] (conjugate = inverse for unit q) */
    double q_cmd_inv[4];
    q_cmd_inv[0] =  q_cmd[0];
    q_cmd_inv[1] = -q_cmd[1];
    q_cmd_inv[2] = -q_cmd[2];
    q_cmd_inv[3] = -q_cmd[3];

    /* q_err = q_cmd^-1 ⊗ q_curr */
    GncStatus rc = quat_mul(q_cmd_inv, q_curr, q_err);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Enforce short-arc: if w < 0, negate entire quaternion */
    if (q_err[0] < 0.0) {
        q_err[0] = -q_err[0];
        q_err[1] = -q_err[1];
        q_err[2] = -q_err[2];
        q_err[3] = -q_err[3];
    }
    return GNC_OK;
}

GncStatus att_pd_control(
    const double q_err_vec[3],
    const double omega_err[3],
    double       mib_Nms,
    double       dt_s,
    AttCmd      *cmd)
{
    GNC_ASSERT(q_err_vec != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(omega_err != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(cmd       != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(dt_s      >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(mib_Nms   >= 0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t ax = 0U;
    for (ax = 0U; ax < 3U; ax++) {
        double tau = -ATT_KP * q_err_vec[ax] - ATT_KD * omega_err[ax];

        /* Saturate */
        if (tau >  ATT_MAX_TORQUE) { tau =  ATT_MAX_TORQUE; }
        if (tau < -ATT_MAX_TORQUE) { tau = -ATT_MAX_TORQUE; }

        /* MIB dead-band */
        if (fabs(tau) * dt_s < mib_Nms) { tau = 0.0; }

        cmd->torque[ax] = tau;
    }
    return GNC_OK;
}

GncStatus att_docking_cmd(const double pos_lvlh[3], double q_cmd[4])
{
    GNC_ASSERT(pos_lvlh != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(q_cmd    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* Compute unit vector pointing from chaser to target (LVLH origin) */
    double dx = -pos_lvlh[0];
    double dy = -pos_lvlh[1];
    double dz = -pos_lvlh[2];
    double dist = sqrt(dx*dx + dy*dy + dz*dz);

    /* If already at origin or very close, return identity */
    if (dist < 1.0e-3) {
        q_cmd[0] = 1.0;
        q_cmd[1] = 0.0;
        q_cmd[2] = 0.0;
        q_cmd[3] = 0.0;
        return GNC_OK;
    }

    double ux = dx / dist;
    double uy = dy / dist;
    double uz = dz / dist;

    /* Align body -y axis (docking port) with target direction.
     * Body -y = [0,-1,0] in body frame.
     * Rotation axis = (-y) × target = [0,-1,0] × [ux,uy,uz]
     *               = [-1*uz - 0, 0*ux - 0*uz, 0*uy - (-1)*ux]
     *               = [-uz, 0, ux]
     * Rotation angle θ: cos(θ) = (-y)·target = -uy
     * q = [cos(θ/2), axis*sin(θ/2)]
     */
    double cos_theta = -uy;
    /* Clamp to [-1,1] to guard acos domain */
    if (cos_theta >  1.0) { cos_theta =  1.0; }
    if (cos_theta < -1.0) { cos_theta = -1.0; }
    double theta    = acos(cos_theta);
    double axis_x   = -uz;
    double axis_z   =  ux;
    double axis_mag = sqrt(axis_x*axis_x + axis_z*axis_z);

    if (axis_mag < 1.0e-9) {
        /* Already aligned or anti-aligned */
        if (cos_theta >= 0.0) {
            /* aligned */
            q_cmd[0] = 1.0;
            q_cmd[1] = 0.0;
            q_cmd[2] = 0.0;
            q_cmd[3] = 0.0;
        } else {
            /* 180-degree flip around x-axis */
            q_cmd[0] = 0.0;
            q_cmd[1] = 1.0;
            q_cmd[2] = 0.0;
            q_cmd[3] = 0.0;
        }
        return GNC_OK;
    }

    double inv_ax  = 1.0 / axis_mag;
    double half_th = 0.5 * theta;
    double sht     = sin(half_th);

    q_cmd[0] = cos(half_th);
    q_cmd[1] = axis_x * inv_ax * sht;
    q_cmd[2] = 0.0;
    q_cmd[3] = axis_z * inv_ax * sht;

    GNC_ASSERT(q_cmd[0] >= -1.0001, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

GncStatus att_check_aligned(const double q_err[4], uint8_t *aligned)
{
    GNC_ASSERT(q_err   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(aligned != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* Pointing error ≈ 2 * acos(|q_err_w|) */
    double qw = q_err[0];
    if (qw >  1.0) { qw =  1.0; }
    if (qw < -1.0) { qw = -1.0; }
    double angle = 2.0 * acos(fabs(qw));

    *aligned = (uint8_t)(angle < ATT_DOCK_TOL_RAD);
    return GNC_OK;
}
