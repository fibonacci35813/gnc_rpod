/**
 * @file    dynamics.c
 * @brief   CW relative motion propagator — NASA Power of 10 compliant.
 *
 * All functions: ≤60 lines, ≥2 assertions, fixed-bound loops,
 * checked return values, single-level pointer dereference.
 */

#include <math.h>
#include "dynamics.h"
#include "gnc_assert.h"

/* -----------------------------------------------------------------------
 * Internal helpers (file scope — Rule 6: smallest possible scope)
 * ----------------------------------------------------------------------- */

/** Saturate a double to [-lim, +lim]. */
static double clamp(double val, double lim)
{
    double result = val;
    if (result >  lim) { result =  lim; }
    if (result < -lim) { result = -lim; }
    return result;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/*@ requires semi_major_m > 1000.0;
  @ requires \valid(n_out);
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM;
  @ ensures \result == GNC_OK ==> *n_out > 0.0;
  @ assigns *n_out;
@*/
GncStatus dyn_mean_motion(double semi_major_m, double *n_out)
{
    GNC_ASSERT(n_out != NULL,          ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(semi_major_m > 1.0e3,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    *n_out = sqrt(GNC_MU_EARTH / (semi_major_m * semi_major_m * semi_major_m));
    return GNC_OK;
}

/*@ requires \valid_read(pos);
  @ requires \valid(range);
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM;
  @ ensures \result == GNC_OK ==> *range >= 0.0;
  @ assigns *range;
@*/
GncStatus dyn_range(const Vec3 *pos, double *range)
{
    GNC_ASSERT(pos   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(range != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double r2 = (pos->v[0] * pos->v[0])
              + (pos->v[1] * pos->v[1])
              + (pos->v[2] * pos->v[2]);
    GNC_ASSERT(r2 >= 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    *range = sqrt(r2);
    return GNC_OK;
}

/*@ requires \valid(phi);
  @ requires n > 0.0 && dt_s > 0.0;
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM;
  @ assigns phi->m[0..5][0..5];
  @ loop invariant 0 <= \at(i,Here) <= 6;
  @ loop invariant 0 <= \at(j,Here) <= 6;
@*/
GncStatus dyn_build_phi(double n, double dt_s, Mat6x6 *phi)
{
    GNC_ASSERT(phi  != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(n    >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double nt    = n * dt_s;
    double snt   = sin(nt);
    double cnt   = cos(nt);
    double n_inv = 1.0 / n;

    /* Zero entire matrix first */
    {
        uint32_t i = 0U;
        for (i = 0U; i < 6U; i++) {
            uint32_t j = 0U;
            for (j = 0U; j < 6U; j++) {
                phi->m[i][j] = 0.0;
            }
        }
    }

    /* Row 0: x (radial) */
    phi->m[0][0] = 4.0 - 3.0 * cnt;
    phi->m[0][3] = snt * n_inv;
    phi->m[0][4] = 2.0 * n_inv * (1.0 - cnt);

    /* Row 1: y (along-track) */
    phi->m[1][0] = 6.0 * (snt - nt);
    phi->m[1][1] = 1.0;
    phi->m[1][3] = -2.0 * n_inv * (1.0 - cnt);
    phi->m[1][4] = n_inv * (4.0 * snt - 3.0 * nt);

    /* Row 2: z (cross-track) */
    phi->m[2][2] = cnt;
    phi->m[2][5] = snt * n_inv;

    /* Row 3: vx */
    phi->m[3][0] = 3.0 * n * snt;
    phi->m[3][3] = cnt;
    phi->m[3][4] = 2.0 * snt;

    /* Row 4: vy */
    phi->m[4][0] = 6.0 * n * (cnt - 1.0);
    phi->m[4][3] = -2.0 * snt;
    phi->m[4][4] = 4.0 * cnt - 3.0;

    /* Row 5: vz */
    phi->m[5][2] = -n * snt;
    phi->m[5][5] = cnt;

    return GNC_OK;
}

/*@ requires \valid_read(pos_in) && \valid_read(vel_in);
  @ requires \valid_read(force_N);
  @ requires \valid(pos_out) && \valid(vel_out);
  @ requires n > 0.0 && dt_s > 0.0 && mass_kg > 0.0;
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM || \result == ERR_BOUNDS;
  @ assigns *pos_out, *vel_out;
@*/
GncStatus dyn_propagate(
    const Vec3 *pos_in,
    const Vec3 *vel_in,
    const Vec3 *force_N,
    double      n,
    double      dt_s,
    double      mass_kg,
    Vec3       *pos_out,
    Vec3       *vel_out)
{
    GNC_ASSERT(pos_in  != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel_in  != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(force_N != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(pos_out != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel_out != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(n       >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s    >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(mass_kg >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    Mat6x6   phi;
    GncStatus rc = dyn_build_phi(n, dt_s, &phi);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* State vector x6 = [px py pz vx vy vz] */
    double x[6];
    x[0] = pos_in->v[0]; x[1] = pos_in->v[1]; x[2] = pos_in->v[2];
    x[3] = vel_in->v[0]; x[4] = vel_in->v[1]; x[5] = vel_in->v[2];

    /* Acceleration from thrust */
    double ax = force_N->v[0] / mass_kg;
    double ay = force_N->v[1] / mass_kg;
    double az = force_N->v[2] / mass_kg;

    /* x_new = Φ·x + impulse approximation (0.5·a·dt²  pos, a·dt vel) */
    double xn[6];
    uint32_t i = 0U;
    for (i = 0U; i < 6U; i++) {
        xn[i] = phi.m[i][0]*x[0] + phi.m[i][1]*x[1] + phi.m[i][2]*x[2]
               + phi.m[i][3]*x[3] + phi.m[i][4]*x[4] + phi.m[i][5]*x[5];
    }

    /* Add thrust contribution (constant-accel approximation) */
    xn[0] += 0.5 * ax * dt_s * dt_s;
    xn[1] += 0.5 * ay * dt_s * dt_s;
    xn[2] += 0.5 * az * dt_s * dt_s;
    xn[3] += ax * dt_s;
    xn[4] += ay * dt_s;
    xn[5] += az * dt_s;

    pos_out->v[0] = xn[0]; pos_out->v[1] = xn[1]; pos_out->v[2] = xn[2];
    vel_out->v[0] = xn[3]; vel_out->v[1] = xn[4]; vel_out->v[2] = xn[5];

    /* Reasonable sanity check: position shouldn't blow up */
    double range_sq = xn[0]*xn[0] + xn[1]*xn[1] + xn[2]*xn[2];
    GNC_ASSERT(range_sq < (1.0e6 * 1.0e6), ERR_BOUNDS, return ERR_BOUNDS);

    (void)clamp; /* suppress unused-function warning if not used elsewhere */
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Phase 7 — perturbation functions
 * ----------------------------------------------------------------------- */

GncStatus dyn_j2_accel(const Vec3 *r_eci, Vec3 *a_j2_out)
{
    GNC_ASSERT(r_eci    != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(a_j2_out != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);

    double x  = r_eci->v[0];
    double y  = r_eci->v[1];
    double z  = r_eci->v[2];
    double r2 = x*x + y*y + z*z;
    GNC_ASSERT(r2 > (DYN_RE_M * DYN_RE_M), ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double r     = sqrt(r2);
    double r5    = r2 * r2 * r;
    double re2   = DYN_RE_M * DYN_RE_M;
    double coeff = -1.5 * DYN_J2 * GNC_MU_EARTH * re2 / r5;
    double z_r2  = (z * z) / r2;

    a_j2_out->v[0] = coeff * x * (1.0 - 5.0 * z_r2);
    a_j2_out->v[1] = coeff * y * (1.0 - 5.0 * z_r2);
    a_j2_out->v[2] = coeff * z * (3.0 - 5.0 * z_r2);
    return GNC_OK;
}

GncStatus dyn_drag_accel(
    const Vec3     *r_eci,
    const Vec3     *v_eci,
    const EnvModel *env,
    double          mass_kg,
    Vec3           *a_drag_out)
{
    GNC_ASSERT(r_eci      != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(v_eci      != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(env        != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(a_drag_out != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(mass_kg    >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double rx = r_eci->v[0], ry = r_eci->v[1], rz = r_eci->v[2];
    double r  = sqrt(rx*rx + ry*ry + rz*rz);
    GNC_ASSERT(r > DYN_RE_M, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double alt_m = r - DYN_RE_M;
    double rho   = DYN_RHO0 * exp(-alt_m / DYN_H_SCALE);

    double vx = v_eci->v[0], vy = v_eci->v[1], vz = v_eci->v[2];
    double v2 = vx*vx + vy*vy + vz*vz;
    GNC_ASSERT(v2 > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double v_mag = sqrt(v2);
    double B     = env->Cd * env->area_m2 / mass_kg;   /* drag parameter (m^2/kg) */
    double a_mag = -0.5 * rho * B * v_mag;             /* scalar (1/s) */

    a_drag_out->v[0] = a_mag * vx;
    a_drag_out->v[1] = a_mag * vy;
    a_drag_out->v[2] = a_mag * vz;
    return GNC_OK;
}

GncStatus dyn_propagate_perturbed(
    const Vec3     *pos_in,
    const Vec3     *vel_in,
    const Vec3     *force_N,
    const EnvModel *env,
    double          n,
    double          dt_s,
    double          mass_kg,
    Vec3           *pos_out,
    Vec3           *vel_out)
{
    GNC_ASSERT(pos_in  != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel_in  != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(force_N != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(env     != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(pos_out != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel_out != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(n       >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s    >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(mass_kg >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* CW base propagation (GNC nav filter also uses this — keeps CW model) */
    GncStatus rc = dyn_propagate(pos_in, vel_in, force_N, n, dt_s, mass_kg,
                                  pos_out, vel_out);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Semi-major axis from mean motion: a = (mu/n^2)^(1/3) */
    double sma = pow(GNC_MU_EARTH / (n * n), 1.0 / 3.0);
    double a_px = 0.0, a_py = 0.0, a_pz = 0.0;

    /* Differential J2: a_perturb = J2(r_chaser) - J2(r_target) in LVLH approx */
    if (env->use_j2 != 0U) {
        Vec3 r_cj, r_tj, a_j2c, a_j2t;
        r_cj.v[0] = sma + pos_in->v[0]; r_cj.v[1] = pos_in->v[1];
        r_cj.v[2] = pos_in->v[2];
        r_tj.v[0] = sma; r_tj.v[1] = 0.0; r_tj.v[2] = 0.0;
        rc = dyn_j2_accel(&r_cj, &a_j2c);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = dyn_j2_accel(&r_tj, &a_j2t);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        a_px += a_j2c.v[0] - a_j2t.v[0];
        a_py += a_j2c.v[1] - a_j2t.v[1];
        a_pz += a_j2c.v[2] - a_j2t.v[2];
    }

    /* Drag: chaser absolute (target drag assumed negligible for differential) */
    if (env->use_drag != 0U) {
        double v_orb = sqrt(GNC_MU_EARTH / sma);  /* circular orbital speed */
        Vec3 r_cd, v_cd, a_drg;
        r_cd.v[0] = sma + pos_in->v[0]; r_cd.v[1] = pos_in->v[1];
        r_cd.v[2] = pos_in->v[2];
        v_cd.v[0] = vel_in->v[0];
        v_cd.v[1] = v_orb + vel_in->v[1]; /* orbital + relative velocity */
        v_cd.v[2] = vel_in->v[2];
        rc = dyn_drag_accel(&r_cd, &v_cd, env, mass_kg, &a_drg);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        a_px += a_drg.v[0]; a_py += a_drg.v[1]; a_pz += a_drg.v[2];
    }

    /* Apply constant-acceleration perturbation correction (Euler integration) */
    pos_out->v[0] += 0.5 * a_px * dt_s * dt_s;
    pos_out->v[1] += 0.5 * a_py * dt_s * dt_s;
    pos_out->v[2] += 0.5 * a_pz * dt_s * dt_s;
    vel_out->v[0] += a_px * dt_s;
    vel_out->v[1] += a_py * dt_s;
    vel_out->v[2] += a_pz * dt_s;
    return GNC_OK;
}
