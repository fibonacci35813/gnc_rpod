/**
 * @file    include/mission_mgr.h
 * @brief   Mission Manager — abort hierarchy and guidance override.
 *          NASA Power of 10 compliant.
 *
 * The Mission Manager reads FDIR fault state and drives mission-mode
 * escalation / de-escalation.  It is intentionally separated from FDIR
 * so that FDIR can detect faults while the mission manager decides the
 * operational response.
 *
 * Mode hierarchy:
 *   NOMINAL → HOLD → ABORT_RETREAT → ABORT_SAFE
 *
 * De-escalation: only HOLD de-escalates (to NOMINAL on fault clear).
 * ABORT_RETREAT and ABORT_SAFE are sticky — manual reset required.
 */

#ifndef MISSION_MGR_H
#define MISSION_MGR_H

#include <stdint.h>
#include "gnc_types.h"
#include "nav_filter.h"
#include "guidance.h"
#include "fdir.h"

/* -----------------------------------------------------------------------
 * Timing constants
 * ----------------------------------------------------------------------- */
#define MGR_HOLD_TIMEOUT_S      60.0    /* s: hold before retreating      */
#define MGR_RETREAT_TIMEOUT_S  120.0    /* s: max retreat duration        */
#define MGR_SAFE_HOLD_Y_M      500.0    /* m: safe hold along-track dist  */
#define MGR_DEESCALATE_S        10.0    /* s: stable time before recovery */

/* -----------------------------------------------------------------------
 * Mission manager state
 * ----------------------------------------------------------------------- */
typedef struct {
    MissionMode current_mode;
    MissionMode prev_mode;
    uint32_t    mode_entry_step;    /* sim step when mode was entered     */
    uint32_t    prev_waypoint_idx;  /* retreat-to waypoint table index    */
    uint8_t     safe_hold_reached;  /* 1 = vehicle at safe distance       */
    double      stable_elapsed_s;   /* s since fault cleared in HOLD      */
} MissionManager;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief Initialise mission manager to NOMINAL with all counters zero.
 */
GncStatus mgr_init(MissionManager *mgr);

/**
 * @brief Update mission mode from fault state and elapsed time.
 *
 * Escalation (each transition requires fault to be active):
 *   NOMINAL        -> HOLD          on any active fault
 *   HOLD           -> ABORT_RETREAT if fault_latched OR
 *                     hold elapsed > MGR_HOLD_TIMEOUT_S
 *   ABORT_RETREAT  -> ABORT_SAFE    if retreat elapsed > MGR_RETREAT_TIMEOUT_S
 *
 * De-escalation (requires fault to be cleared):
 *   HOLD -> NOMINAL  if no active fault AND stable_elapsed_s > MGR_DEESCALATE_S
 *
 * ABORT_RETREAT and ABORT_SAFE do NOT de-escalate automatically.
 */
GncStatus mgr_update(MissionManager *mgr, const FaultState *fs,
                     GuidancePlan *plan, uint32_t step, double dt_s);

/**
 * @brief Get guidance position override for current abort mode.
 *
 * MODE_NOMINAL:       *override_active = 0 (use normal guidance)
 * MODE_HOLD:          *pos_override = current nav position (hold in place)
 *                     *override_active = 1
 * MODE_ABORT_RETREAT: *pos_override = plan->table[prev_waypoint_idx].pos_ref
 *                     *override_active = 1
 * MODE_ABORT_SAFE:    *pos_override = {0, MGR_SAFE_HOLD_Y_M, 0}
 *                     *override_active = 1
 */
GncStatus mgr_get_guidance_override(const MissionManager *mgr,
                                    const GuidancePlan   *plan,
                                    const NavState       *nav,
                                    Vec3                 *pos_override,
                                    uint8_t              *override_active);

#endif /* MISSION_MGR_H */
