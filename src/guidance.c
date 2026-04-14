/**
 * @file    guidance.c
 * @brief   Two-mode approach guidance — v0.3, NASA Power of 10 compliant.
 *
 * V-BAR MODE  (far from waypoint):
 *   pos_ref = current nav position  (eliminates spring-fight with vel control)
 *   vel_ref = -v_close * unit(err)
 *   v_close = min(v_phase_max, GUID_K_V * range_to_wp)
 *
 * PD-HOLD MODE (inside waypoint corridor):
 *   pos_ref = waypoint target
 *   vel_ref = {0, 0, 0}
 *
 * Waypoint advance requires BOTH position within corridor AND speed below
 * vel_thresh_mps, preventing blow-through on high-speed arrival.
 */

#include <math.h>
#include "guidance.h"
#include "gnc_assert.h"

/* Velocity threshold per waypoint to gate advancement */
#define GUID_VT_FAR     0.30   /* phase 0 → 1: must slow below 0.30 m/s */
#define GUID_VT_MID     0.15   /* phase 1 → 2: must slow below 0.15 m/s */
#define GUID_VT_CLOSE   0.04   /* phase 2 → 3: must slow below 0.04 m/s */

/* Minimum creep velocity so controller always has a target */
#define GUID_V_CREEP    0.001  /* (m/s) — gives 1.2cm braking dist at terminal */

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static double sq3(const Vec3 *v)
{
    return v->v[0]*v->v[0] + v->v[1]*v->v[1] + v->v[2]*v->v[2];
}

static double approach_vel(double range_m, double v_max)
{
    double v = GUID_K_V * range_m;
    if (v > v_max)    { v = v_max;    }
    if (v < GUID_V_CREEP) { v = GUID_V_CREEP; }
    return v;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

GncStatus guid_init_plan(GuidancePlan *plan)
{
    GNC_ASSERT(plan != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    uint32_t i = 0U;
    for (i = 0U; i < GNC_MAX_WAYPOINTS; i++) {
        plan->table[i].pos_ref.v[0]  = 0.0;
        plan->table[i].pos_ref.v[1]  = 0.0;
        plan->table[i].pos_ref.v[2]  = 0.0;
        plan->table[i].vel_ref.v[0]  = 0.0;
        plan->table[i].vel_ref.v[1]  = 0.0;
        plan->table[i].vel_ref.v[2]  = 0.0;
        plan->table[i].corridor_m    = 0.0;
        plan->table[i].hold_time_s   = 0.0;
    }

    /* Phase 0 — far-field hold @ y = +200 m */
    plan->table[0].pos_ref.v[1] = 200.0;
    plan->table[0].corridor_m   = 5.0;
    plan->table[0].hold_time_s  = 20.0;

    /* Phase 1 — mid-range @ y = +50 m */
    plan->table[1].pos_ref.v[1] = 50.0;
    plan->table[1].corridor_m   = 3.0;
    plan->table[1].hold_time_s  = 15.0;

    /* Phase 2 — close approach @ y = +10 m */
    plan->table[2].pos_ref.v[1] = 10.0;
    plan->table[2].corridor_m   = 1.0;
    plan->table[2].hold_time_s  = 10.0;

    /* Phase 3 — terminal ingress @ docking port (origin)
     * K_V profile gives natural decel:
     *   range=4m → v=0.040 m/s   range=1m → v=0.010 m/s
     *   range=0.2m → v=0.002 m/s  range≤0.1m → v=CREEP=0.001 m/s
     * Entry speed into 5cm corridor ≈ 0.001 m/s → braking dist ≈ 12mm ✓
     */
    plan->table[3].pos_ref.v[1] = 0.0;
    plan->table[3].corridor_m   = GNC_DOCK_POS_TOL;   /* 5 cm */
    plan->table[3].hold_time_s  = 0.0;

    plan->count            = 4U;
    plan->active           = 0U;
    plan->phase_elapsed_s  = 0.0;

    GNC_ASSERT(plan->count <= GNC_MAX_WAYPOINTS, ERR_BOUNDS, return ERR_BOUNDS);
    return GNC_OK;
}

/* Per-phase velocity threshold table — indexed by plan->active */
static double phase_vel_thresh(uint32_t phase)
{
    double tbl[4];
    tbl[0] = GUID_VT_FAR;
    tbl[1] = GUID_VT_MID;
    tbl[2] = GUID_VT_CLOSE;
    tbl[3] = GNC_DOCK_VEL_TOL;
    if (phase < 4U) { return tbl[phase]; }
    return GUID_VT_FAR;
}

/* Per-phase approach velocity cap table */
static double phase_v_max(uint32_t phase)
{
    double tbl[4];
    tbl[0] = GUID_V_FAR_MAX;
    tbl[1] = GUID_V_MID_MAX;
    tbl[2] = GUID_V_CLOSE_MAX;
    tbl[3] = GUID_V_TERM_MAX;  /* 0.04 m/s; K_V profile gives natural decel */
    if (phase < 4U) { return tbl[phase]; }
    return GUID_V_CREEP;
}

GncStatus guid_compute_ref(
    GuidancePlan    *plan,
    const NavState  *nav,
    double           dt_s,
    Vec3            *pos_ref,
    Vec3            *vel_ref)
{
    GNC_ASSERT(plan    != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(nav     != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(pos_ref != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(vel_ref != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
    GNC_ASSERT(dt_s    >  0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);
    GNC_ASSERT(plan->active < plan->count, ERR_BOUNDS, return ERR_BOUNDS);

    plan->phase_elapsed_s += dt_s;

    const Waypoint *wp = &plan->table[plan->active];

    /* Error vector from current position to waypoint target */
    Vec3 err;
    err.v[0] = nav->pos.v[0] - wp->pos_ref.v[0];
    err.v[1] = nav->pos.v[1] - wp->pos_ref.v[1];
    err.v[2] = nav->pos.v[2] - wp->pos_ref.v[2];
    double err_sq = sq3(&err);

    /* Current vehicle speed */
    double spd_sq = sq3(&nav->vel);
    double speed  = sqrt(spd_sq);

    /* Velocity threshold for this phase */
    double v_thresh = phase_vel_thresh(plan->active);

    /* Waypoint advance: inside corridor AND slow enough */
    uint8_t pos_ok  = (uint8_t)(err_sq   <= (wp->corridor_m * wp->corridor_m));
    uint8_t vel_ok  = (uint8_t)(speed    <   v_thresh);
    uint8_t time_ok = (uint8_t)(plan->phase_elapsed_s >= wp->hold_time_s);
    uint8_t at_wp   = (uint8_t)(pos_ok && vel_ok && time_ok);

    if ((at_wp != 0U) && (plan->active < (plan->count - 1U))) {
        plan->active++;
        plan->phase_elapsed_s = 0.0;
        wp = &plan->table[plan->active];
    }

    /* ---------------------------------------------------------------
     * Recompute error to (possibly advanced) waypoint, then choose mode
     * --------------------------------------------------------------- */
    err.v[0] = nav->pos.v[0] - wp->pos_ref.v[0];
    err.v[1] = nav->pos.v[1] - wp->pos_ref.v[1];
    err.v[2] = nav->pos.v[2] - wp->pos_ref.v[2];
    double range_to_wp = sqrt(sq3(&err));

    if (range_to_wp > wp->corridor_m) {
        /* V-BAR approach: no position spring, pure velocity command */
        *pos_ref = nav->pos;    /* pos_err = 0 → no spring fight         */

        double v_close = approach_vel(range_to_wp, phase_v_max(plan->active));
        if (range_to_wp > 1.0e-6) {
            vel_ref->v[0] = -v_close * err.v[0] / range_to_wp;
            vel_ref->v[1] = -v_close * err.v[1] / range_to_wp;
            vel_ref->v[2] = -v_close * err.v[2] / range_to_wp;
        } else {
            vel_ref->v[0] = 0.0;
            vel_ref->v[1] = 0.0;
            vel_ref->v[2] = 0.0;
        }
    } else {
        /* PD-HOLD mode: drive to exact waypoint position with zero velocity */
        *pos_ref       = wp->pos_ref;
        vel_ref->v[0]  = 0.0;
        vel_ref->v[1]  = 0.0;
        vel_ref->v[2]  = 0.0;
    }

    GNC_ASSERT(plan->active < plan->count, ERR_BOUNDS, return ERR_BOUNDS);
    return GNC_OK;
}

GncStatus guid_corridor_check(
    const GuidancePlan *plan,
    const NavState     *nav,
    uint8_t            *in_corr)
{
    GNC_ASSERT(plan    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(nav     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(in_corr != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(plan->active < plan->count, ERR_BOUNDS, return ERR_BOUNDS);

    const Waypoint *wp = &plan->table[plan->active];
    double dx  = nav->pos.v[0] - wp->pos_ref.v[0];
    double dz  = nav->pos.v[2] - wp->pos_ref.v[2];
    double lat = dx*dx + dz*dz;

    *in_corr = (uint8_t)(lat < (wp->corridor_m * wp->corridor_m));
    return GNC_OK;
}

GncStatus guid_active_phase(const GuidancePlan *plan, uint32_t *phase)
{
    GNC_ASSERT(plan  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(phase != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(plan->active < plan->count, ERR_BOUNDS, return ERR_BOUNDS);

    *phase = plan->active;
    return GNC_OK;
}
