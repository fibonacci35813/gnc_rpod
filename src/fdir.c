/**
 * @file    src/fdir.c
 * @brief   Fault Detection, Isolation and Recovery — NASA Power of 10.
 *
 * Fault priority (highest → lowest):
 *   FAULT_THR_STUCK_OPEN  (latched, immediately aborts)
 *   FAULT_NAV_DIVERGE     (not latched, retreat)
 *   FAULT_SENSOR_DROPOUT  (not latched, hold then resume)
 *   FAULT_THR_STUCK_CLOSED(not latched, hold then resume)
 *   FAULT_ATT_UNSTABLE    (not latched, hold then resume)
 */

#include <math.h>
#include <stdint.h>
#include "gnc_types.h"
#include "gnc_assert.h"
#include "fdir.h"
#include "params.h"

/* Number of axes in Vec3 — used for bounded loops (Rule 2) */
#define FDIR_NUM_AXES  3U

/* -----------------------------------------------------------------------
 * Internal helper: conditionally set a fault.
 *   - Latched faults cannot be overridden.
 *   - stuck_open cannot be downgraded to any other fault.
 * ----------------------------------------------------------------------- */
static void fdir_set_fault(FaultState *fs, FaultCode code, uint8_t latched)
{
    GNC_ASSERT(fs   != NULL,                 ERR_NULL_PTR,  return);
    GNC_ASSERT(code <= FAULT_ATT_UNSTABLE,   ERR_BAD_PARAM, return);
    if (fs->fault_latched != 0U) { return; }
    if ((fs->active_fault == FAULT_THR_STUCK_OPEN) &&
        (code != FAULT_THR_STUCK_OPEN)) { return; }
    fs->active_fault  = code;
    fs->fault_latched = latched;
}

/* ----------------------------------------------------------------------- */
GncStatus fdir_init(FaultState *fs, const GncParams *p)
{
    GNC_ASSERT(fs   != NULL,            ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(sizeof(*fs) > 0U,        ERR_BAD_PARAM, return ERR_BAD_PARAM);

    fs->active_fault  = FAULT_NONE;
    fs->mode          = MODE_NOMINAL;
    fs->fault_step    = 0U;
    fs->dropout_count = 0U;
    fs->hold_steps    = 0U;
    fs->valid_count   = 0U;
    fs->fault_latched = 0U;
    fs->recovery_done = 0U;

    /* Runtime-configurable thresholds — params or hardcoded defines */
    if (p != NULL) {
        fs->hold_timeout_steps = (p->fdir_hold_timeout_s > 0.0)
            ? (uint32_t)(p->fdir_hold_timeout_s / GNC_DT_SEC)
            : FDIR_HOLD_TIMEOUT;
        fs->dropout_limit = p->fdir_dropout_limit;
    } else {
        fs->hold_timeout_steps = FDIR_HOLD_TIMEOUT;
        fs->dropout_limit      = FDIR_DROPOUT_LIMIT;
    }
    return GNC_OK;
}

/* ----------------------------------------------------------------------- */
GncStatus fdir_check_thruster(
    const Vec3 *force_cmd,
    const Vec3 *force_actual,
    FaultState *fs)
{
    GNC_ASSERT(force_cmd    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(force_actual != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(fs           != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    if (fs->fault_latched != 0U) { return GNC_OK; }

    /* stuck_open takes priority — scan all axes first */
    for (uint32_t i = 0U; i < FDIR_NUM_AXES; i++) {
        double fc = fabs(force_cmd->v[i]);
        double fa = fabs(force_actual->v[i]);
        if ((fa > FDIR_THR_OPEN_THRESHOLD) && (fc < FDIR_THR_FORCE_DEAD_ZONE)) {
            fdir_set_fault(fs, FAULT_THR_STUCK_OPEN, 1U);
            return GNC_OK;   /* stuck_open latched — stop scanning */
        }
    }

    /* stuck_closed — only if no stuck_open found */
    for (uint32_t i = 0U; i < FDIR_NUM_AXES; i++) {
        double fc = fabs(force_cmd->v[i]);
        double fa = fabs(force_actual->v[i]);
        if ((fc > FDIR_THR_FORCE_DEAD_ZONE) && (fa < FDIR_THR_FORCE_DEAD_ZONE)) {
            fdir_set_fault(fs, FAULT_THR_STUCK_CLOSED, 0U);
            return GNC_OK;
        }
    }
    return GNC_OK;
}

/* ----------------------------------------------------------------------- */
GncStatus fdir_check_sensor(uint8_t meas_valid, FaultState *fs)
{
    GNC_ASSERT(fs != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT((meas_valid == 0U) || (meas_valid == 1U),
               ERR_BAD_PARAM, return ERR_BAD_PARAM);

    if (fs->fault_latched != 0U) { return GNC_OK; }

    if (meas_valid == 0U) {
        fs->dropout_count++;
        fs->valid_count = 0U;
        if (fs->dropout_count >= fs->dropout_limit) {
            fdir_set_fault(fs, FAULT_SENSOR_DROPOUT, 0U);
        }
    } else {
        /* Valid measurement: reset dropout counter */
        fs->dropout_count = 0U;
        /* Clear dropout fault after FDIR_DROPOUT_CLEAR consecutive valid steps */
        if (fs->active_fault == FAULT_SENSOR_DROPOUT) {
            fs->valid_count++;
            if (fs->valid_count >= FDIR_DROPOUT_CLEAR) {
                fs->active_fault = FAULT_NONE;
                fs->valid_count  = 0U;
            }
        }
    }
    return GNC_OK;
}

/* ----------------------------------------------------------------------- */
GncStatus fdir_check_nav(const NavState *nav, FaultState *fs)
{
    GNC_ASSERT(nav != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(fs  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    if (fs->fault_latched != 0U) { return GNC_OK; }

    /* Covariance trace: sum of main diagonal of 6×6 matrix */
    double trace = 0.0;
    for (uint32_t i = 0U; i < GNC_STATE_DIM; i++) {
        trace += nav->cov.m[i][i];
    }

    double rx    = nav->pos.v[0];
    double ry    = nav->pos.v[1];
    double rz    = nav->pos.v[2];
    double range = sqrt(rx*rx + ry*ry + rz*rz);

    if ((trace > FDIR_COV_TRACE_MAX) || (range > FDIR_RANGE_MAX_M)) {
        fdir_set_fault(fs, FAULT_NAV_DIVERGE, 0U);
    }
    return GNC_OK;
}

/* ----------------------------------------------------------------------- */
GncStatus fdir_check_attitude(const AttState *att, FaultState *fs)
{
    GNC_ASSERT(att != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(fs  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    if (fs->fault_latched != 0U) { return GNC_OK; }

    double ox        = att->omega[0];
    double oy        = att->omega[1];
    double oz        = att->omega[2];
    double omega_mag = sqrt(ox*ox + oy*oy + oz*oz);

    if (omega_mag > FDIR_OMEGA_MAX_RPS) {
        fdir_set_fault(fs, FAULT_ATT_UNSTABLE, 0U);
    }
    return GNC_OK;
}

/* ----------------------------------------------------------------------- */
MissionMode fdir_get_mode(const FaultState *fs)
{
    GNC_ASSERT(fs != NULL, ERR_NULL_PTR, return MODE_FAILED);
    GNC_ASSERT(fs->active_fault <= FAULT_ATT_UNSTABLE,
               ERR_BAD_PARAM, return MODE_FAILED);

    if (fs->active_fault == FAULT_NONE)           { return MODE_NOMINAL;       }
    if (fs->active_fault == FAULT_THR_STUCK_OPEN) { return MODE_ABORT_SAFE;    }
    if (fs->active_fault == FAULT_NAV_DIVERGE)    { return MODE_ABORT_RETREAT; }
    /* FAULT_THR_STUCK_CLOSED, FAULT_SENSOR_DROPOUT, FAULT_ATT_UNSTABLE */
    return MODE_HOLD;
}

/* ----------------------------------------------------------------------- */
GncStatus fdir_update(FaultState *fs, uint32_t step)
{
    GNC_ASSERT(fs   != NULL,             ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(step <= GNC_MAX_SIM_STEPS, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* Latch the step at which the first fault was detected */
    if ((fs->active_fault != FAULT_NONE) && (fs->fault_step == 0U)) {
        fs->fault_step = step;
    }

    /* Compute current mode and store it */
    MissionMode mode = fdir_get_mode(fs);
    fs->mode = mode;

    /* Advance hold timer only for non-latched faults in HOLD mode */
    if ((mode == MODE_HOLD) && (fs->fault_latched == 0U)) {
        fs->hold_steps++;
        if (fs->hold_steps >= fs->hold_timeout_steps) {
            fs->recovery_done = 1U;
            fs->active_fault  = FAULT_NONE;
            fs->hold_steps    = 0U;
            fs->mode          = MODE_NOMINAL;
        }
    }
    return GNC_OK;
}
