/**
 * @file    control.c
 * @brief   6-DOF PD controller for docking — NASA Power of 10 compliant.
 *
 * Computes three-axis force commands from position and velocity errors.
 * Applies saturation and minimum-impulse-bit (MIB) dead-band.
 */

#include <math.h>
#include "control.h"
#include "gnc_assert.h"
#include "params.h"

/* -----------------------------------------------------------------------
 * Default tuned gains — normal approach phases
 * Higher Kd needed to brake vehicle at 0.3 m/s within short distances.
 * Kp chosen for gentle position spring; Kd for ~1.0 damping ratio.
 *   ζ = Kd / (2*sqrt(Kp*m)):
 *     x,z: 30 / (2*sqrt(0.30*500)) = 30/24.5 = 1.22  ✓
 *     y:   25 / (2*sqrt(0.20*500)) = 25/20.0 = 1.25  ✓
 * ----------------------------------------------------------------------- */
#define CTRL_KP_RADIAL       0.30     /* N/m */
#define CTRL_KP_ALONG        0.20     /* N/m */
#define CTRL_KP_CROSS        0.30     /* N/m */
#define CTRL_KD_RADIAL      30.00     /* N·s/m  (ζ=1.22) */
#define CTRL_KD_ALONG       25.00     /* N·s/m  (ζ=1.25) */
#define CTRL_KD_CROSS       30.00     /* N·s/m  (ζ=1.22) */

/* Specific impulse for propellant accounting (s) */
#define CTRL_ISP_S         220.0
#define CTRL_G0_MPS2         9.80665

/*@ requires \valid(gains);
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM;
  @ ensures \result == GNC_OK ==>
  @         gains->kp[0] > 0.0 && gains->kp[1] > 0.0 && gains->kp[2] > 0.0 &&
  @         gains->kd[0] > 0.0 && gains->kd[1] > 0.0 && gains->kd[2] > 0.0;
  @ assigns *gains;
@*/
GncStatus ctrl_init_gains(PdGains *gains, const GncParams *p)
{
    GNC_ASSERT(gains != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    if (p != NULL) {
        gains->kp[0] = p->kp[0];
        gains->kp[1] = p->kp[1];
        gains->kp[2] = p->kp[2];
        gains->kd[0] = p->kd[0];
        gains->kd[1] = p->kd[1];
        gains->kd[2] = p->kd[2];
    } else {
        gains->kp[0] = CTRL_KP_RADIAL;
        gains->kp[1] = CTRL_KP_ALONG;
        gains->kp[2] = CTRL_KP_CROSS;
        gains->kd[0] = CTRL_KD_RADIAL;
        gains->kd[1] = CTRL_KD_ALONG;
        gains->kd[2] = CTRL_KD_CROSS;
    }

    /* Sanity: all gains must be positive */
    GNC_ASSERT(gains->kp[0] > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(gains->kd[0] > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

/*@ requires \valid_read(v);
  @ requires \valid(norm);
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM;
  @ ensures \result == GNC_OK ==> *norm >= 0.0;
  @ assigns *norm;
@*/
GncStatus ctrl_vec3_norm(const Vec3 *v, double *norm)
{
    GNC_ASSERT(v    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(norm != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double sq = v->v[0]*v->v[0] + v->v[1]*v->v[1] + v->v[2]*v->v[2];
    GNC_ASSERT(sq >= 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    *norm = sqrt(sq);
    return GNC_OK;
}

/*@ requires \valid_read(a) && \valid_read(b);
  @ requires \valid(result);
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR;
  @ assigns *result;
@*/
GncStatus ctrl_vec3_sub(const Vec3 *a, const Vec3 *b, Vec3 *result)
{
    GNC_ASSERT(a      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(b      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(result != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    result->v[0] = a->v[0] - b->v[0];
    result->v[1] = a->v[1] - b->v[1];
    result->v[2] = a->v[2] - b->v[2];
    return GNC_OK;
}

GncStatus ctrl_apply_terminal_gains(PdGains *gains, const GncParams *p)
{
    GNC_ASSERT(gains != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    if (p != NULL) {
        gains->kp[0] = p->kp_terminal[0];
        gains->kp[1] = p->kp_terminal[1];
        gains->kp[2] = p->kp_terminal[2];
        gains->kd[0] = p->kd_terminal[0];
        gains->kd[1] = p->kd_terminal[1];
        gains->kd[2] = p->kd_terminal[2];
    } else {
        /* Hardcoded overdamped defaults: ζ=1.33, ωn=0.0316 rad/s
         * Kp=0.50 N/m, Kd=42 N·s/m → T_settle≈95 s */
        gains->kp[0] = 0.50;
        gains->kp[1] = 0.50;
        gains->kp[2] = 0.50;
        gains->kd[0] = 42.0;
        gains->kd[1] = 42.0;
        gains->kd[2] = 42.0;
    }

    GNC_ASSERT(gains->kp[0] > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(gains->kd[0] > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

/*@ requires \valid_read(gains) && \valid_read(pos_err) && \valid_read(vel_err);
  @ requires \valid(cmd) && \valid(fuel);
  @ requires mass_kg > 0.0 && dt_s > 0.0 && mib_Ns >= 0.0;
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM || \result == ERR_BOUNDS;
  @ assigns *cmd, *fuel;
@*/
GncStatus ctrl_compute(
    const PdGains *gains,
    const Vec3    *pos_err,
    const Vec3    *vel_err,
    double         mass_kg,
    double         dt_s,
    double         mib_Ns,
    ControlCmd    *cmd,
    FuelState     *fuel)
{
    GNC_ASSERT(gains   != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(pos_err != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel_err != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(cmd     != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(fuel    != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(mass_kg >  0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s    >  0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(mib_Ns  >= 0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* PD law per axis: F = Kp·ep + Kd·ev */
    double f[3];
    uint32_t ax = 0U;
    for (ax = 0U; ax < 3U; ax++) {
        f[ax] = gains->kp[ax] * pos_err->v[ax]
              + gains->kd[ax] * vel_err->v[ax];
    }

    /* Saturate to maximum thrust per axis */
    for (ax = 0U; ax < 3U; ax++) {
        if (f[ax] >  GNC_MAX_THRUST) { f[ax] =  GNC_MAX_THRUST; }
        if (f[ax] < -GNC_MAX_THRUST) { f[ax] = -GNC_MAX_THRUST; }
    }

    /* Minimum impulse bit dead-band: if impulse too small, zero it */
    uint32_t fired = 0U;
    for (ax = 0U; ax < 3U; ax++) {
        double impulse = fabs(f[ax]) * dt_s;
        if (impulse < mib_Ns) {
            f[ax] = 0.0;
        } else {
            fired++;
        }
    }

    cmd->force_N.v[0] = f[0];
    cmd->force_N.v[1] = f[1];
    cmd->force_N.v[2] = f[2];

    /* ΔV = |F|·dt / m */
    double F_mag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    cmd->delta_v_mps  = (F_mag * dt_s) / mass_kg;
    cmd->prop_step_kg = (F_mag * dt_s) / (CTRL_ISP_S * CTRL_G0_MPS2);

    fuel->total_dv_mps += cmd->delta_v_mps;
    fuel->prop_kg      += cmd->prop_step_kg;
    fuel->fire_count   += fired;

    /* Sanity: per-step propellant must be non-negative and sub-total-mass */
    GNC_ASSERT(cmd->prop_step_kg >= 0.0,     ERR_BOUNDS, return ERR_BOUNDS);
    GNC_ASSERT(cmd->prop_step_kg < mass_kg,  ERR_BOUNDS, return ERR_BOUNDS);
    return GNC_OK;
}
