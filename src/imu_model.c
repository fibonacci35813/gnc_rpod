/**
 * @file    imu_model.c
 * @brief   IMU sensor model — NASA Power of 10 compliant.
 *
 * Implements a simple strapdown IMU with:
 *   - White Gaussian noise (Box-Muller via deterministic LCG)
 *   - Constant bias + slow random walk
 *
 * All functions: ≤60 lines, ≥2 assertions, fixed-bound loops,
 * checked return values, single-level pointer dereference.
 */

#include <math.h>
#include "imu_model.h"
#include "gnc_assert.h"

/* -----------------------------------------------------------------------
 * Local LCG PRNG state (file-scope; separate from sim/main.c state)
 * ----------------------------------------------------------------------- */
#define IMU_LCG_SEED  0xA3B1C2D4U

static uint32_t s_imu_lcg = IMU_LCG_SEED;

/* Advance LCG and return a standard-normal sample via Box-Muller.
 * Uses two consecutive uniform samples; avoids log(0) with small floor. */
static double imu_normal(void)
{
    GNC_ASSERT(s_imu_lcg != 0U, ERR_BAD_PARAM, return 0.0);

    s_imu_lcg = s_imu_lcg * 1664525U + 1013904223U;
    double u1 = (double)(s_imu_lcg) / 4294967296.0 + 1.0e-9;
    s_imu_lcg = s_imu_lcg * 1664525U + 1013904223U;
    double u2 = (double)(s_imu_lcg) / 4294967296.0;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

GncStatus imu_init(ImuState *imu, double accel_sigma, double gyro_sigma)
{
    GNC_ASSERT(imu         != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(accel_sigma >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(gyro_sigma  >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    imu->accel_sigma      = accel_sigma;
    imu->gyro_sigma       = gyro_sigma;
    imu->bias_instability = 1.0e-5;   /* small random walk per step */

    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++) {
        imu->accel_bias[i] = 0.0;
        imu->gyro_bias[i]  = 0.0;
    }
    return GNC_OK;
}

GncStatus imu_measure(
    ImuState       *imu,
    const Vec3     *true_accel,
    const Vec3     *true_omega,
    Vec3           *meas_accel,
    Vec3           *meas_omega)
{
    GNC_ASSERT(imu        != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(true_accel != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(true_omega != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(meas_accel != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(meas_omega != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(imu->accel_sigma > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++) {
        /* Accelerometer: true + bias + white noise */
        meas_accel->v[i] = true_accel->v[i]
                         + imu->accel_bias[i]
                         + imu->accel_sigma * imu_normal();

        /* Gyroscope: true + bias + white noise */
        meas_omega->v[i] = true_omega->v[i]
                         + imu->gyro_bias[i]
                         + imu->gyro_sigma  * imu_normal();

        /* Bias random walk (small instability per step) */
        imu->accel_bias[i] += imu->bias_instability * imu_normal();
        imu->gyro_bias[i]  += imu->bias_instability * imu_normal();
    }
    return GNC_OK;
}
