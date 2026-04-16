/**
 * @file    guidance.h
 * @brief   Multi-phase approach guidance for autonomous docking — v0.3
 *
 * Two guidance sub-modes per phase:
 *
 *   V-BAR APPROACH (range-to-waypoint > corridor_m):
 *     pos_ref = nav->pos   (zero position error — no spring fight)
 *     vel_ref = -v_close * unit(err_to_wp)
 *     v_close = min(v_phase_max, GUID_K_V * range_to_wp)
 *
 *   PD HOLD (range-to-waypoint <= corridor_m):
 *     pos_ref = waypoint.pos_ref
 *     vel_ref = {0, 0, 0}
 *
 * Waypoint advance: position within corridor AND speed < vel_thresh_mps
 *
 * Phases:
 *   0 — Far-field:    WP @ y=+200m, corr=5m, v_thresh=0.15 m/s
 *   1 — Mid-range:    WP @ y=+50m,  corr=3m, v_thresh=0.10 m/s
 *   2 — Close:        WP @ y=+10m,  corr=1m, v_thresh=0.04 m/s
 *   3 — Terminal:     WP @ origin,  corr=0.05m (docking port)
 */

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "gnc_types.h"

/* Phase velocity caps — default values (runtime-configurable via GncParams) */
#define GUID_V_FAR_MAX     1.00   /* phase 0: long-range, up to 1.0 m/s   */
#define GUID_V_MID_MAX     0.50   /* phase 1: mid-range, up to 0.5 m/s    */
#define GUID_V_CLOSE_MAX   0.08   /* phase 2: close approach               */
#define GUID_V_TERM_MAX    0.04   /* phase 3: terminal (K_V profile brakes)*/

/* V-bar gain default.  Stored in GuidancePlan.K_V at runtime. */
#define GUID_K_V           0.010

/* Number of guidance phases (bounds the v_phase_max loop in guid_init_plan) */
#define GUID_NUM_PHASES    4U

/* Forward declaration — full definition in sim/params.h */
#ifndef GNC_PARAMS_FWDECL
#define GNC_PARAMS_FWDECL
typedef struct GncParams GncParams;
#endif

/**
 * @brief  Initialise the 4-phase guidance plan.
 *
 * @param  plan  GuidancePlan to initialise.
 * @param  p     GNC params (NULL → hardcoded defaults).
 * @return GNC_OK, ERR_NULL_PTR, or ERR_BOUNDS.
 */
GncStatus guid_init_plan(GuidancePlan *plan, const GncParams *p);

GncStatus guid_compute_ref(
    GuidancePlan    *plan,
    const NavState  *nav,
    double           dt_s,
    Vec3            *pos_ref,
    Vec3            *vel_ref);

GncStatus guid_corridor_check(
    const GuidancePlan *plan,
    const NavState     *nav,
    uint8_t            *in_corr);

GncStatus guid_active_phase(const GuidancePlan *plan, uint32_t *phase);

#endif /* GUIDANCE_H */
