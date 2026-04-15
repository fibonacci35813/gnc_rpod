/**
 * @file    src/rw_model.c
 * @brief   Reaction wheel model — momentum storage, saturation, desaturation.
 *          NASA Power of 10 compliant.
 *
 * Three orthogonal wheels (aligned with body x, y, z).
 * Momentum is updated each step: h_i += torque_i * dt.
 * Desaturation event: bleed h_i back toward zero at RW_DESAT_RATE per step.
 */

#include <math.h>
#include "rw_model.h"
#include "gnc_assert.h"

GncStatus rw_init(RwState *rw)
{
    GNC_ASSERT(rw != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    uint32_t i = 0U;
    for (i = 0U; i < GNC_MAX_RW; i++) {
        rw->h[i]         = 0.0;
        rw->saturated[i] = 0U;
    }
    rw->desat_count = 0U;

    GNC_ASSERT(GNC_MAX_RW == 3U, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

GncStatus rw_apply_torque(RwState *rw, const double torque[3], double dt_s)
{
    GNC_ASSERT(rw     != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(torque != NULL, ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(dt_s   >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t i = 0U;
    for (i = 0U; i < GNC_MAX_RW; i++) {
        rw->h[i] += torque[i] * dt_s;

        /* Clamp to saturation limit */
        if (rw->h[i] >  RW_MAX_MOMENTUM_NMS) { rw->h[i] =  RW_MAX_MOMENTUM_NMS; }
        if (rw->h[i] < -RW_MAX_MOMENTUM_NMS) { rw->h[i] = -RW_MAX_MOMENTUM_NMS; }

        /* Flag saturation */
        rw->saturated[i] = (uint8_t)(fabs(rw->h[i]) >= RW_MAX_MOMENTUM_NMS);
    }
    return GNC_OK;
}

GncStatus rw_desat(RwState *rw)
{
    GNC_ASSERT(rw != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    uint8_t  any_desat = 0U;
    uint32_t i         = 0U;
    for (i = 0U; i < GNC_MAX_RW; i++) {
        if (fabs(rw->h[i]) >= RW_DESAT_THRESH_NMS) {
            /* Bleed momentum toward zero */
            double bleed = RW_DESAT_RATE * rw->h[i];
            rw->h[i] -= bleed;
            any_desat  = 1U;
        }
    }

    if (any_desat != 0U) {
        rw->desat_count++;
    }

    GNC_ASSERT(rw->desat_count < 1000000U, ERR_BOUNDS, return ERR_BOUNDS);
    return GNC_OK;
}

GncStatus rw_effective_torque(
    const RwState *rw,
    const double   torque_cmd[3],
    double         torque_out[3])
{
    GNC_ASSERT(rw          != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(torque_cmd  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(torque_out  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* When a wheel is NOT saturated, full command passes through.
     * When saturated, the wheel cannot accelerate further in that direction,
     * so clamp effective torque to zero for that axis.
     */
    uint32_t i = 0U;
    for (i = 0U; i < GNC_MAX_RW; i++) {
        if ((rw->saturated[i] != 0U) &&
            (torque_cmd[i] * rw->h[i] > 0.0)) {
            /* Trying to push momentum further into saturation — zero it */
            torque_out[i] = 0.0;
        } else {
            torque_out[i] = torque_cmd[i];
        }
    }
    return GNC_OK;
}
