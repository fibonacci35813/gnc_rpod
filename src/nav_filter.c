/**
 * @file    nav_filter.c
 * @brief   6-state linear Kalman filter — NASA Power of 10 compliant.
 *
 * State: X = [x, y, z, vx, vy, vz]ᵀ
 * Measurement model: H = [I3 | 03] (position-only update)
 */

#include <math.h>
#include "nav_filter.h"
#include "dynamics.h"
#include "gnc_assert.h"

/* -----------------------------------------------------------------------
 * Process noise (tuning parameters — small for near-circular orbit)
 * ----------------------------------------------------------------------- */
#define NAV_Q_POS   1.0e-4    /* (m)^2 per step   */
#define NAV_Q_VEL   1.0e-6    /* (m/s)^2 per step */

/* -----------------------------------------------------------------------
 * Internal: multiply 6×6 matrices  C = A * B  (fixed 6-step loops)
 * ----------------------------------------------------------------------- */
static GncStatus mat6_mul(const Mat6x6 *a, const Mat6x6 *b, Mat6x6 *c)
{
    GNC_ASSERT(a != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(b != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(c != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    uint32_t i = 0U;
    for (i = 0U; i < 6U; i++) {
        uint32_t j = 0U;
        for (j = 0U; j < 6U; j++) {
            uint32_t k = 0U;
            c->m[i][j] = 0.0;
            for (k = 0U; k < 6U; k++) {
                c->m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
    return GNC_OK;
}

/* Internal: transpose 6×6 matrix */
static GncStatus mat6_transpose(const Mat6x6 *a, Mat6x6 *at)
{
    GNC_ASSERT(a  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(at != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    uint32_t i = 0U;
    for (i = 0U; i < 6U; i++) {
        uint32_t j = 0U;
        for (j = 0U; j < 6U; j++) {
            at->m[j][i] = a->m[i][j];
        }
    }
    return GNC_OK;
}

/* Internal: invert a 3×3 matrix (used for innovation covariance S) */
static GncStatus mat3_inv(const Mat3x3 *m, Mat3x3 *inv)
{
    GNC_ASSERT(m   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(inv != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double det =   m->m[0][0]*(m->m[1][1]*m->m[2][2] - m->m[1][2]*m->m[2][1])
                 - m->m[0][1]*(m->m[1][0]*m->m[2][2] - m->m[1][2]*m->m[2][0])
                 + m->m[0][2]*(m->m[1][0]*m->m[2][1] - m->m[1][1]*m->m[2][0]);

    GNC_ASSERT(fabs(det) > 1.0e-15, ERR_SINGULAR, return ERR_SINGULAR);

    double inv_det = 1.0 / det;
    inv->m[0][0] = inv_det*(m->m[1][1]*m->m[2][2] - m->m[1][2]*m->m[2][1]);
    inv->m[0][1] = inv_det*(m->m[0][2]*m->m[2][1] - m->m[0][1]*m->m[2][2]);
    inv->m[0][2] = inv_det*(m->m[0][1]*m->m[1][2] - m->m[0][2]*m->m[1][1]);
    inv->m[1][0] = inv_det*(m->m[1][2]*m->m[2][0] - m->m[1][0]*m->m[2][2]);
    inv->m[1][1] = inv_det*(m->m[0][0]*m->m[2][2] - m->m[0][2]*m->m[2][0]);
    inv->m[1][2] = inv_det*(m->m[0][2]*m->m[1][0] - m->m[0][0]*m->m[1][2]);
    inv->m[2][0] = inv_det*(m->m[1][0]*m->m[2][1] - m->m[1][1]*m->m[2][0]);
    inv->m[2][1] = inv_det*(m->m[0][1]*m->m[2][0] - m->m[0][0]*m->m[2][1]);
    inv->m[2][2] = inv_det*(m->m[0][0]*m->m[1][1] - m->m[0][1]*m->m[1][0]);
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

GncStatus nav_init(
    NavState   *nav,
    const Vec3 *pos0,
    const Vec3 *vel0,
    double      pos_sigma,
    double      vel_sigma)
{
    GNC_ASSERT(nav       != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(pos0      != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel0      != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(pos_sigma >  0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(vel_sigma >  0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    nav->pos    = *pos0;
    nav->vel    = *vel0;
    nav->time_s = 0.0;
    nav->valid  = 1U;

    /* Diagonal initial covariance */
    double p_pos = pos_sigma * pos_sigma;
    double p_vel = vel_sigma * vel_sigma;
    {
        uint32_t i = 0U;
        for (i = 0U; i < 6U; i++) {
            uint32_t j = 0U;
            for (j = 0U; j < 6U; j++) {
                nav->cov.m[i][j] = 0.0;
            }
        }
    }
    nav->cov.m[0][0] = p_pos;
    nav->cov.m[1][1] = p_pos;
    nav->cov.m[2][2] = p_pos;
    nav->cov.m[3][3] = p_vel;
    nav->cov.m[4][4] = p_vel;
    nav->cov.m[5][5] = p_vel;
    return GNC_OK;
}

GncStatus nav_propagate(
    NavState   *nav,
    const Vec3 *force_N,
    double      n,
    double      dt_s,
    double      mass_kg)
{
    GNC_ASSERT(nav     != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(force_N != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(nav->valid == 1U, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s    >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* Propagate mean state through CW dynamics */
    Vec3      new_pos;
    Vec3      new_vel;
    GncStatus rc = dyn_propagate(&nav->pos, &nav->vel, force_N,
                                 n, dt_s, mass_kg, &new_pos, &new_vel);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    nav->pos     = new_pos;
    nav->vel     = new_vel;
    nav->time_s += dt_s;

    /* Covariance propagation: P = Φ·P·Φᵀ + Q */
    Mat6x6    phi;
    rc = dyn_build_phi(n, dt_s, &phi);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    Mat6x6 phi_t;
    rc = mat6_transpose(&phi, &phi_t);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    Mat6x6 tmp;
    rc = mat6_mul(&phi, &nav->cov, &tmp);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    rc = mat6_mul(&tmp, &phi_t, &nav->cov);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Add process noise (diagonal Q) */
    nav->cov.m[0][0] += NAV_Q_POS;
    nav->cov.m[1][1] += NAV_Q_POS;
    nav->cov.m[2][2] += NAV_Q_POS;
    nav->cov.m[3][3] += NAV_Q_VEL;
    nav->cov.m[4][4] += NAV_Q_VEL;
    nav->cov.m[5][5] += NAV_Q_VEL;
    return GNC_OK;
}

GncStatus nav_update(
    NavState   *nav,
    const Vec3 *meas_pos,
    double      meas_sigma)
{
    GNC_ASSERT(nav      != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(meas_pos != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(meas_sigma > 0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(nav->valid == 1U,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double r2 = meas_sigma * meas_sigma;

    /* Innovation y = z - H·x (H selects positions) */
    double y[3];
    y[0] = meas_pos->v[0] - nav->pos.v[0];
    y[1] = meas_pos->v[1] - nav->pos.v[1];
    y[2] = meas_pos->v[2] - nav->pos.v[2];

    /* S = H·P·Hᵀ + R  (upper-left 3×3 of P plus R·I3) */
    Mat3x3 S;
    S.m[0][0] = nav->cov.m[0][0] + r2;
    S.m[0][1] = nav->cov.m[0][1];
    S.m[0][2] = nav->cov.m[0][2];
    S.m[1][0] = nav->cov.m[1][0];
    S.m[1][1] = nav->cov.m[1][1] + r2;
    S.m[1][2] = nav->cov.m[1][2];
    S.m[2][0] = nav->cov.m[2][0];
    S.m[2][1] = nav->cov.m[2][1];
    S.m[2][2] = nav->cov.m[2][2] + r2;

    Mat3x3 S_inv;
    GncStatus rc = mat3_inv(&S, &S_inv);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* K = P·Hᵀ·S⁻¹  — K is 6×3 (P[:, 0:3] * S⁻¹, since H = [I3|0]) */
    double K[6][3];
    {
        uint32_t i = 0U;
        for (i = 0U; i < 6U; i++) {
            uint32_t j = 0U;
            for (j = 0U; j < 3U; j++) {
                uint32_t k = 0U;
                K[i][j] = 0.0;
                for (k = 0U; k < 3U; k++) {
                    K[i][j] += nav->cov.m[i][k] * S_inv.m[k][j];
                }
            }
        }
    }

    /* State update: X = X + K·y */
    {
        double dx[6];
        uint32_t i = 0U;
        for (i = 0U; i < 6U; i++) {
            dx[i] = K[i][0]*y[0] + K[i][1]*y[1] + K[i][2]*y[2];
        }
        nav->pos.v[0] += dx[0];
        nav->pos.v[1] += dx[1];
        nav->pos.v[2] += dx[2];
        nav->vel.v[0] += dx[3];
        nav->vel.v[1] += dx[4];
        nav->vel.v[2] += dx[5];
    }

    /* Covariance update: P = P - K·H·P  (direct form, H = [I3|0])
     * HP = H·P = P[0:3, :]  (top 3 rows of P)
     * P  = P - K·HP
     */
    {
        double HP[3][6];
        uint32_t i = 0U;
        for (i = 0U; i < 3U; i++) {
            uint32_t j = 0U;
            for (j = 0U; j < 6U; j++) {
                HP[i][j] = nav->cov.m[i][j];
            }
        }
        for (i = 0U; i < 6U; i++) {
            uint32_t j = 0U;
            for (j = 0U; j < 6U; j++) {
                double khp = K[i][0]*HP[0][j]
                           + K[i][1]*HP[1][j]
                           + K[i][2]*HP[2][j];
                nav->cov.m[i][j] -= khp;
            }
        }
    }
    return GNC_OK;
}
