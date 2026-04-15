/**
 * @file    rw_model.h
 * @brief   3-axis reaction wheel model with momentum saturation and desaturation.
 *
 * Three reaction wheels aligned with body x, y, z axes.
 * Each wheel stores angular momentum h_i (N·m·s).
 * Desaturation fires thrusters when any wheel reaches saturation.
 *
 * NASA Power of 10 compliant.
 */

#ifndef RW_MODEL_H
#define RW_MODEL_H

#include "gnc_types.h"

/* Reaction wheel momentum limits */
#define RW_MAX_MOMENTUM_NMS   0.1    /* saturation at ±0.1 N·m·s per wheel */
#define RW_DESAT_THRESH_NMS   0.08   /* desaturation trigger threshold      */
#define RW_DESAT_RATE         0.5    /* desaturation dump rate (fraction/step)*/

/**
 * @brief  Reaction wheel state — one per axis.
 */
typedef struct {
    double h[GNC_MAX_RW];       /* stored angular momentum (N·m·s) per wheel */
    uint8_t saturated[GNC_MAX_RW]; /* 1 if wheel is near saturation           */
    uint32_t desat_count;          /* number of desaturation events           */
} RwState;

/**
 * @brief  Initialise reaction wheels to zero momentum.
 * @param  rw   Output wheel state.
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus rw_init(RwState *rw);

/**
 * @brief  Apply a torque command to the reaction wheels.
 *
 *   h_i += torque_i * dt_s  (equal and opposite to satellite body)
 *   Clamps to ±RW_MAX_MOMENTUM_NMS.
 *
 * @param  rw        Wheel state (updated in place).
 * @param  torque    Commanded torque vector (N·m) [3].
 * @param  dt_s      Time step (s).
 * @return GNC_OK or error code.
 */
GncStatus rw_apply_torque(RwState *rw, const double torque[3], double dt_s);

/**
 * @brief  Check saturation and perform momentum desaturation if needed.
 *
 *   If |h_i| >= RW_DESAT_THRESH_NMS, dumps momentum by applying
 *   a small attitude torque (implemented as direct momentum reduction).
 *   In practice desaturation thrusters fire; here we just bleed momentum.
 *
 * @param  rw        Wheel state (updated in place).
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus rw_desat(RwState *rw);

/**
 * @brief  Return effective torque actually applied to spacecraft body.
 *
 *   tau_body_i = -h_dot_i  (reaction to wheel acceleration)
 *
 * @param  rw         Wheel state.
 * @param  torque_cmd Commanded torque (N·m) [3].
 * @param  torque_out Actual torque applied to body (N·m) [3].
 * @return GNC_OK or ERR_NULL_PTR.
 */
GncStatus rw_effective_torque(
    const RwState *rw,
    const double   torque_cmd[3],
    double         torque_out[3]);

#endif /* RW_MODEL_H */
