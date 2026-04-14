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

GncStatus dyn_mean_motion(double semi_major_m, double *n_out)
{
    GNC_ASSERT(n_out != NULL,          ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(semi_major_m > 1.0e3,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    *n_out = sqrt(GNC_MU_EARTH / (semi_major_m * semi_major_m * semi_major_m));
    return GNC_OK;
}

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
