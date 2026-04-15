/**
 * @file    imu_model.h
 * @brief   IMU sensor model: accelerometer + gyro with bias and white noise.
 *
 * Models a strapdown IMU consisting of:
 *   - 3-axis accelerometer: measures specific force (m/s^2) with bias + noise
 *   - 3-axis gyroscope   : measures angular rate (rad/s) with bias + noise
 *
 * Bias terms follow a simple random walk (bias_instability per step).
 * White noise is generated with a deterministic LCG PRNG seeded internally.
 *
 * All units are SI.  Output used for high-rate nav propagation (inner loop).
 */

#ifndef IMU_MODEL_H
#define IMU_MODEL_H

#include "gnc_types.h"

/* -----------------------------------------------------------------------
 * IMU state — carries bias estimates and noise parameters
 * ----------------------------------------------------------------------- */
typedef struct {
    double accel_bias[3];    /* accelerometer bias (m/s^2) per axis    */
    double gyro_bias[3];     /* gyroscope bias (rad/s) per axis         */
    double accel_sigma;      /* accelerometer white-noise 1-σ (m/s^2)  */
    double gyro_sigma;       /* gyroscope white-noise 1-σ (rad/s)      */
    double bias_instability; /* bias random-walk std per step (common)  */
} ImuState;

/**
 * @brief  Initialise IMU state with zero biases and given noise parameters.
 *
 * @param  imu          IMU state to initialise.
 * @param  accel_sigma  Accelerometer white-noise 1-σ (m/s^2), must be > 0.
 * @param  gyro_sigma   Gyroscope white-noise 1-σ (rad/s), must be > 0.
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus imu_init(ImuState *imu, double accel_sigma, double gyro_sigma);

/**
 * @brief  Simulate one IMU measurement given the true body-frame values.
 *
 * Adds white Gaussian noise and current bias to true signals.
 * Updates bias with a small random walk (bias_instability).
 *
 * @param  imu          IMU state (bias updated in place).
 * @param  true_accel   True specific force in body frame (m/s^2).
 * @param  true_omega   True angular rate in body frame (rad/s).
 * @param  meas_accel   Measured (noisy) acceleration output (m/s^2).
 * @param  meas_omega   Measured (noisy) angular rate output (rad/s).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BAD_PARAM.
 */
GncStatus imu_measure(
    ImuState       *imu,
    const Vec3     *true_accel,
    const Vec3     *true_omega,
    Vec3           *meas_accel,
    Vec3           *meas_omega);

#endif /* IMU_MODEL_H */
