/**
 * @file    src/mission_mgr.c
 * @brief   Mission Manager — abort hierarchy and guidance override.
 *          NASA Power of 10 compliant.
 *
 * Implements the four-level abort hierarchy:
 *   NOMINAL → HOLD → ABORT_RETREAT → ABORT_SAFE
 *
 * De-escalation: HOLD → NOMINAL when fault clears and stable for
 * MGR_DEESCALATE_S seconds. ABORT_RETREAT and ABORT_SAFE are sticky.
 */

#include "mission_mgr.h"
#include "gnc_assert.h"

/* -----------------------------------------------------------------------
 * Static helpers (private, no recursion)
 * ----------------------------------------------------------------------- */

/* Seconds elapsed since the current mode was entered. */
static double mgr_elapsed_s(const MissionManager *mgr,
                             uint32_t step, double dt_s)
{
    return (double)(step - mgr->mode_entry_step) * dt_s;
}

/* Handle transitions out of MODE_NOMINAL. */
static void mgr_step_nominal(MissionManager *mgr,
                              const FaultState *fs, uint32_t step)
{
    if (fs->active_fault != FAULT_NONE) {
        mgr->prev_mode         = MODE_NOMINAL;
        mgr->current_mode      = MODE_HOLD;
        mgr->mode_entry_step   = step;
        mgr->prev_waypoint_idx = 0U;   /* always retreat to Phase 0 */
        mgr->stable_elapsed_s  = 0.0;
    }
}

/* Handle transitions out of MODE_HOLD. */
static void mgr_step_hold(MissionManager *mgr, const FaultState *fs,
                           uint32_t step, double dt_s)
{
    if (fs->active_fault != FAULT_NONE) {
        /* Fault (re-)active: reset stability timer, check escalation. */
        mgr->stable_elapsed_s = 0.0;
        double el = mgr_elapsed_s(mgr, step, dt_s);
        uint8_t timed_out = (el > MGR_HOLD_TIMEOUT_S) ? 1U : 0U;
        if ((fs->fault_latched != 0U) || (timed_out != 0U)) {
            mgr->prev_mode       = MODE_HOLD;
            mgr->current_mode    = MODE_ABORT_RETREAT;
            mgr->mode_entry_step = step;
        }
    } else {
        /* No active fault: accumulate stability time, check de-escalation. */
        mgr->stable_elapsed_s += dt_s;
        if (mgr->stable_elapsed_s > MGR_DEESCALATE_S) {
            mgr->prev_mode        = MODE_HOLD;
            mgr->current_mode     = MODE_NOMINAL;
            mgr->mode_entry_step  = step;
            mgr->stable_elapsed_s = 0.0;
        }
    }
}

/* Handle transitions out of MODE_ABORT_RETREAT. */
static void mgr_step_retreat(MissionManager *mgr,
                              uint32_t step, double dt_s)
{
    double el = mgr_elapsed_s(mgr, step, dt_s);
    if (el > MGR_RETREAT_TIMEOUT_S) {
        mgr->prev_mode       = MODE_ABORT_RETREAT;
        mgr->current_mode    = MODE_ABORT_SAFE;
        mgr->mode_entry_step = step;
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

GncStatus mgr_init(MissionManager *mgr)
{
    GNC_ASSERT(mgr != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    mgr->current_mode      = MODE_NOMINAL;
    mgr->prev_mode         = MODE_NOMINAL;
    mgr->mode_entry_step   = 0U;
    mgr->prev_waypoint_idx = 0U;
    mgr->safe_hold_reached = 0U;
    mgr->stable_elapsed_s  = 0.0;

    GNC_ASSERT(mgr->current_mode == MODE_NOMINAL, ERR_BAD_PARAM,
               return ERR_BAD_PARAM);
    return GNC_OK;
}

GncStatus mgr_update(MissionManager *mgr, const FaultState *fs,
                     GuidancePlan *plan, uint32_t step, double dt_s)
{
    GNC_ASSERT(mgr  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(fs   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(plan != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(dt_s > 0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    (void)plan;   /* reserved: waypoint selection on de-escalation */

    switch (mgr->current_mode) {
    case MODE_NOMINAL:
        mgr_step_nominal(mgr, fs, step);
        break;
    case MODE_HOLD:
        mgr_step_hold(mgr, fs, step, dt_s);
        break;
    case MODE_ABORT_RETREAT:
        mgr_step_retreat(mgr, step, dt_s);
        break;
    case MODE_ABORT_SAFE:
        /* sticky — manual reset required */
        break;
    case MODE_DOCKED:
        /* fall-through: no mission manager action after docking */
        break;
    case MODE_FAILED:
        /* fall-through: no mission manager action */
        break;
    default:
        /* unreachable — GNC_ASSERT guards on entry */
        break;
    }

    return GNC_OK;
}

GncStatus mgr_get_guidance_override(const MissionManager *mgr,
                                    const GuidancePlan   *plan,
                                    const NavState       *nav,
                                    Vec3                 *pos_override,
                                    uint8_t              *override_active)
{
    GNC_ASSERT(mgr            != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(plan           != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(nav            != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(pos_override   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(override_active != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    static const Vec3 s_zero     = {{0.0, 0.0, 0.0}};
    static const Vec3 s_safe_pos = {{0.0, MGR_SAFE_HOLD_Y_M, 0.0}};

    switch (mgr->current_mode) {
    case MODE_HOLD:
        *pos_override    = nav->pos;    /* hold at current estimated position */
        *override_active = 1U;
        break;

    case MODE_ABORT_RETREAT: {
        uint32_t idx = mgr->prev_waypoint_idx;
        GNC_ASSERT(idx < plan->count, ERR_BOUNDS, return ERR_BOUNDS);
        *pos_override    = plan->table[idx].pos_ref;
        *override_active = 1U;
        break;
    }

    case MODE_ABORT_SAFE:
        *pos_override    = s_safe_pos;
        *override_active = 1U;
        break;

    case MODE_NOMINAL:
    case MODE_DOCKED:
    case MODE_FAILED:
    default:
        *pos_override    = s_zero;
        *override_active = 0U;
        break;
    }

    return GNC_OK;
}
