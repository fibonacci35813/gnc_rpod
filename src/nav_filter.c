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
#define NAV_Q_VEL   1.0e-5    /* (m/s)^2 per step — IMU noise over 10 substeps */

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

    /* Threshold accounts for min R determinant: det(R_min) ~ 4e-18
     * when sigma_range=0.002 m, sigma_ang=0.001 rad. Use 1e-25 to admit
     * all valid S = H*P*H^T + R while still catching true singularities. */
    GNC_ASSERT(fabs(det) > 1.0e-25, ERR_SINGULAR, return ERR_SINGULAR);

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

/*@ requires \valid(nav) && \valid_read(meas_pos);
  @ requires meas_sigma > 0.0;
  @ requires nav->valid == 1;
  @ ensures \result == GNC_OK || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM || \result == ERR_SINGULAR;
  @ assigns nav->pos, nav->vel, nav->cov;
@*/
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

/* -----------------------------------------------------------------------
 * Phase 8 — EKF with RAE + IMU-driven propagation
 * ----------------------------------------------------------------------- */

/* RAE Jacobian H (3×6): rows = [d_range, d_az, d_el]/dx.  Vel cols = 0. */
static GncStatus nav_rae_jacobian(const NavState *nav, double H[3][6])
{
    GNC_ASSERT(nav != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(H   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double x = nav->pos.v[0], y = nav->pos.v[1], z = nav->pos.v[2];
    double r2 = x*x + y*y + z*z;
    GNC_ASSERT(r2 > 1.0e-6, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double r = sqrt(r2), rxy2 = x*x + y*y, rxy = sqrt(rxy2);
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++) {
        H[i][0]=H[i][1]=H[i][2]=H[i][3]=H[i][4]=H[i][5]=0.0;
    }
    H[0][0] = x/r;  H[0][1] = y/r;  H[0][2] = z/r;
    if (rxy2 > 1.0e-6) { H[1][0] = -y/rxy2;  H[1][1] = x/rxy2; }
    if (rxy  > 1.0e-6) {
        H[2][0] = -(x*z)/(r2*rxy);  H[2][1] = -(y*z)/(r2*rxy);
        H[2][2] = rxy/r2;
    }
    return GNC_OK;
}

/* S = H*P*H' + R (3×3).  r_diag[3] = diagonal of R. */
static GncStatus nav_hph_r(double H[3][6], const Mat6x6 *P,
                             const double r_diag[3], Mat3x3 *S)
{
    GNC_ASSERT(H != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(P != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(r_diag != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(S != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    uint32_t i = 0U;
    for (i = 0U; i < GNC_MEAS_DIM; i++) {
        uint32_t j = 0U;
        for (j = 0U; j < GNC_MEAS_DIM; j++) {
            uint32_t k = 0U;
            S->m[i][j] = (i == j) ? r_diag[i] : 0.0;
            for (k = 0U; k < GNC_STATE_DIM; k++) {
                uint32_t l = 0U;
                for (l = 0U; l < GNC_STATE_DIM; l++) {
                    S->m[i][j] += H[i][k] * P->m[k][l] * H[j][l];
                }
            }
        }
    }
    return GNC_OK;
}

/* K = P*H'*S_inv (6×3): compute PH=P*H' then K=PH*S_inv. */
static GncStatus nav_kalman_gain(const Mat6x6 *P, double H[3][6],
                                   const Mat3x3 *S_inv, double K[6][3])
{
    GNC_ASSERT(P     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(H     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(S_inv != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(K     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    double PH[6][3];
    uint32_t i = 0U;
    for (i = 0U; i < GNC_STATE_DIM; i++) {
        uint32_t k = 0U;
        for (k = 0U; k < GNC_MEAS_DIM; k++) {
            uint32_t l = 0U;
            PH[i][k] = 0.0;
            for (l = 0U; l < GNC_STATE_DIM; l++) {
                PH[i][k] += P->m[i][l] * H[k][l];
            }
        }
    }
    for (i = 0U; i < GNC_STATE_DIM; i++) {
        uint32_t j = 0U;
        for (j = 0U; j < GNC_MEAS_DIM; j++) {
            uint32_t k = 0U;
            K[i][j] = 0.0;
            for (k = 0U; k < GNC_MEAS_DIM; k++) {
                K[i][j] += PH[i][k] * S_inv->m[k][j];
            }
        }
    }
    return GNC_OK;
}

/* P = (I - K*H)*P = P - K*(H*P).  HP=H*P (3×6), then P -= K*HP. */
static GncStatus nav_cov_update(double K[6][3], double H[3][6], NavState *nav)
{
    GNC_ASSERT(K   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(H   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(nav != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    double HP[3][6];
    uint32_t i = 0U;
    for (i = 0U; i < GNC_MEAS_DIM; i++) {
        uint32_t j = 0U;
        for (j = 0U; j < GNC_STATE_DIM; j++) {
            /* HP = H*P: only pos cols of H are non-zero (cols 3-5 of H = 0) */
            HP[i][j] = H[i][0]*nav->cov.m[0][j]
                     + H[i][1]*nav->cov.m[1][j]
                     + H[i][2]*nav->cov.m[2][j];
        }
    }
    for (i = 0U; i < GNC_STATE_DIM; i++) {
        uint32_t j = 0U;
        for (j = 0U; j < GNC_STATE_DIM; j++) {
            nav->cov.m[i][j] -= K[i][0]*HP[0][j]
                              +  K[i][1]*HP[1][j]
                              +  K[i][2]*HP[2][j];
        }
    }
    return GNC_OK;
}

GncStatus nav_update_ekf(
    NavState *nav,
    double    range_m,
    double    az_rad,
    double    el_rad,
    double    sigma_r,
    double    sigma_ang)
{
    GNC_ASSERT(nav       != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(sigma_r   >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(sigma_ang >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(nav->valid == 1U,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double x = nav->pos.v[0], y = nav->pos.v[1], z = nav->pos.v[2];
    double r2 = x*x + y*y + z*z;
    GNC_ASSERT(r2 > 1.0e-6, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    double r  = sqrt(r2);
    double zr = z / r;
    if (zr >  1.0) { zr =  1.0; }
    if (zr < -1.0) { zr = -1.0; }

    double dz[3];
    dz[0] = range_m - r;
    dz[1] = az_rad  - atan2(y, x);
    dz[2] = el_rad  - asin(zr);

    double H[3][6];
    GncStatus rc = nav_rae_jacobian(nav, H);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    double r_diag[3];
    r_diag[0] = sigma_r * sigma_r;
    r_diag[1] = sigma_ang * sigma_ang;
    r_diag[2] = sigma_ang * sigma_ang;

    Mat3x3 S;
    rc = nav_hph_r(H, &nav->cov, r_diag, &S);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    Mat3x3 S_inv;
    rc = mat3_inv(&S, &S_inv);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    double K[6][3];
    rc = nav_kalman_gain(&nav->cov, H, &S_inv, K);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* State update X += K*dz */
    {
        uint32_t i = 0U;
        for (i = 0U; i < GNC_STATE_DIM; i++) {
            double dx = K[i][0]*dz[0] + K[i][1]*dz[1] + K[i][2]*dz[2];
            if (i < 3U) { nav->pos.v[i] += dx; }
            else        { nav->vel.v[i - 3U] += dx; }
        }
    }

    rc = nav_cov_update(K, H, nav);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    return GNC_OK;
}

GncStatus nav_propagate_cov_only(NavState *nav, double n, double dt_s)
{
    GNC_ASSERT(nav       != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(nav->valid == 1U,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(n         >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s      >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    Mat6x6    phi;
    GncStatus rc = dyn_build_phi(n, dt_s, &phi);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    Mat6x6 phi_t;
    rc = mat6_transpose(&phi, &phi_t);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    Mat6x6 tmp;
    rc = mat6_mul(&phi, &nav->cov, &tmp);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    rc = mat6_mul(&tmp, &phi_t, &nav->cov);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    nav->cov.m[0][0] += NAV_Q_POS;  nav->cov.m[1][1] += NAV_Q_POS;
    nav->cov.m[2][2] += NAV_Q_POS;  nav->cov.m[3][3] += NAV_Q_VEL;
    nav->cov.m[4][4] += NAV_Q_VEL;  nav->cov.m[5][5] += NAV_Q_VEL;
    return GNC_OK;
}

GncStatus nav_propagate_imu(
    NavState   *nav,
    const Vec3 *meas_accel,
    double      n,
    double      dt_s)
{
    GNC_ASSERT(nav        != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(meas_accel != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(nav->valid == 1U,   ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(n          >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(dt_s       >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* CW gravity-gradient coupling (tidal terms not sensed by accelerometer) */
    double px = nav->pos.v[0], pz = nav->pos.v[2];
    double vx = nav->vel.v[0], vy = nav->vel.v[1];
    double ax = meas_accel->v[0] + 2.0*n*vy + 3.0*n*n*px;
    double ay = meas_accel->v[1] - 2.0*n*vx;
    double az = meas_accel->v[2] - n*n*pz;

    /* Semi-implicit Euler: velocity first, then position */
    nav->vel.v[0] += ax * dt_s;
    nav->vel.v[1] += ay * dt_s;
    nav->vel.v[2] += az * dt_s;
    nav->pos.v[0] += nav->vel.v[0] * dt_s;
    nav->pos.v[1] += nav->vel.v[1] * dt_s;
    nav->pos.v[2] += nav->vel.v[2] * dt_s;
    nav->time_s   += dt_s;
    return GNC_OK;
}
