/**
 * @file    sim/mc_fdir.c
 * @brief   FDIR fault-injection Monte Carlo — NASA Power of 10 compliant.
 *
 * Injects one of three fault modes at MCFDIR_INJECT_STEP and verifies
 * detection / recovery / docking gates.
 *
 * Usage:
 *   ./mc_fdir_sim stuck_open  [N]   → P(abort ≤ 10 steps) == 1.00
 *   ./mc_fdir_sim dropout     [N]   → P(dock)             >= 0.90
 *   ./mc_fdir_sim stuck_closed [N]  → P(dock)             >= 0.80
 *
 * Compile via:  make mc-fdir FAULT=stuck_open N=100
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "gnc_types.h"
#include "gnc_assert.h"
#include "dynamics.h"
#include "nav_filter.h"
#include "guidance.h"
#include "control.h"
#include "fdir.h"
#include "mission_mgr.h"

/* -----------------------------------------------------------------------
 * Constants
 * ----------------------------------------------------------------------- */
#define MCFDIR_MAX_RUNS        2000U   /* hard bound on N                   */
#define MCFDIR_MAX_STEPS       12000U  /* steps per run (slightly more than MC) */
#define MCFDIR_DOCK_DWELL      30U     /* steps required for dock confirm   */
#define MCFDIR_TERM_RANGE_M    5.0     /* terminal phase arming threshold   */
#define MCFDIR_TERM_MIB_NS     0.005   /* terminal MIB                      */
#define MCFDIR_NAV_P0_POS      5.0
#define MCFDIR_NAV_P0_VEL      0.5
#define MCFDIR_POS0_Y          200.0

/* Fault injection parameters */
#define MCFDIR_INJECT_STEP     500U    /* simulation step at which fault starts */
#define MCFDIR_DROPOUT_STEPS   3U      /* steps of consecutive sensor dropout   */
#define MCFDIR_STUCK_FORCE     0.5     /* N: injected uncmd'd force (> 0.1 threshold;
                                        *    small so controller correction stays < MIB
                                        *    after z has converged by step 500)         */
#define MCFDIR_ABORT_WINDOW    10U     /* steps: stuck_open must be detected within */
#define MCFDIR_STUCK_CL_STEPS  50U     /* steps of zero actual-force (stuck_closed) */

/* Sensor model */
#define MCFDIR_SENSOR_K        0.003
#define MCFDIR_SENSOR_FLOOR    0.002

/* Dispersion (same as nominal MC) */
#define MCFDIR_SIGMA_POS       3.0
#define MCFDIR_SIGMA_VEL       0.05
#define MCFDIR_THR_HALF        0.05

/* -----------------------------------------------------------------------
 * Fault type enumeration
 * ----------------------------------------------------------------------- */
typedef enum {
    MCFDIR_FAULT_STUCK_OPEN   = 0,
    MCFDIR_FAULT_DROPOUT      = 1,
    MCFDIR_FAULT_STUCK_CLOSED = 2,
    MCFDIR_FAULT_RETREAT      = 3,  /* Phase 9: abort-retreat verification  */
    MCFDIR_FAULT_UNKNOWN      = 4
} McFdirFaultType;

/* -----------------------------------------------------------------------
 * Per-run result
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t run;
    uint32_t steps;
    uint8_t  fault_detected;       /* 1 = FDIR flagged the fault            */
    uint32_t detect_step;          /* sim step when FDIR first flagged it   */
    uint8_t  docked;
    /* Phase 9 retreat verification (populated only for MCFDIR_FAULT_RETREAT) */
    uint32_t mode_hold_step;       /* step when MODE_HOLD first reached      */
    uint32_t mode_retreat_step;    /* step when MODE_ABORT_RETREAT reached   */
    double   y_at_retreat_plus_80; /* true_pos.y 80 steps after retreat      */
} McFdirResult;

/* -----------------------------------------------------------------------
 * LCG PRNG
 * ----------------------------------------------------------------------- */
static uint32_t s_rng = 0U;

static void rng_seed(uint32_t seed)  { s_rng = seed; }

static double rng_uniform(void)
{
    s_rng = s_rng * 1664525U + 1013904223U;
    return (double)(s_rng) / 4294967296.0;
}

static double rng_normal(void)
{
    double u1 = rng_uniform() + 1.0e-9;
    double u2 = rng_uniform();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* -----------------------------------------------------------------------
 * Sensor simulation
 * ----------------------------------------------------------------------- */
static GncStatus mcf_sensor(const Vec3 *true_pos, Vec3 *meas, double *sigma_out)
{
    GNC_ASSERT(true_pos  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(meas      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(sigma_out != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double range = 0.0;
    GncStatus rc = dyn_range(true_pos, &range);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    double sigma = MCFDIR_SENSOR_K * range;
    if (sigma < MCFDIR_SENSOR_FLOOR) { sigma = MCFDIR_SENSOR_FLOOR; }

    meas->v[0] = true_pos->v[0] + sigma * rng_normal();
    meas->v[1] = true_pos->v[1] + sigma * rng_normal();
    meas->v[2] = true_pos->v[2] + sigma * rng_normal();

    *sigma_out = sigma;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Compute actual (post-fault) force applied to dynamics.
 * Also returns meas_valid for the FDIR sensor check.
 * ----------------------------------------------------------------------- */
static void mcf_inject_fault(
    McFdirFaultType  fault_type,
    uint32_t         step,
    const Vec3      *force_cmd,
    Vec3            *actual_force,
    uint8_t         *meas_valid)
{
    /* Default: actual == commanded, sensor valid */
    *actual_force = *force_cmd;
    *meas_valid   = 1U;

    if (step < MCFDIR_INJECT_STEP) { return; }

    if (fault_type == MCFDIR_FAULT_STUCK_OPEN) {
        /* Inject uncmd'd force on cross-track (z) axis.
         * After 500 steps of active control z-error has converged;
         * the small stuck force means the required correction stays
         * below MIB, so cmd.v[2] stays near zero → instant detection. */
        actual_force->v[2] = MCFDIR_STUCK_FORCE;
    } else if (fault_type == MCFDIR_FAULT_DROPOUT) {
        if (step < (MCFDIR_INJECT_STEP + MCFDIR_DROPOUT_STEPS)) {
            *meas_valid = 0U;
        }
    } else if (fault_type == MCFDIR_FAULT_STUCK_CLOSED) {
        if (step < (MCFDIR_INJECT_STEP + MCFDIR_STUCK_CL_STEPS)) {
            actual_force->v[0] = 0.0;
            actual_force->v[1] = 0.0;
            actual_force->v[2] = 0.0;
        }
    } else {
        /* MCFDIR_FAULT_UNKNOWN: no injection */
    }
}

/* -----------------------------------------------------------------------
 * Run one GNC simulation with fault injection.
 * ----------------------------------------------------------------------- */
static GncStatus run_one_fdir(
    const Vec3     *pos0,
    const Vec3     *vel0,
    double          mass0,
    const double    thr_scale[3],
    McFdirFaultType fault_type,
    McFdirResult   *result,
    uint32_t        run_idx)
{
    GNC_ASSERT(pos0      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(vel0      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(thr_scale != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(result    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(mass0     >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double sma   = GNC_EARTH_RADIUS + GNC_ISS_ALTITUDE;
    double n_rad = 0.0;
    GncStatus rc = dyn_mean_motion(sma, &n_rad);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    Vec3 true_pos = *pos0;
    Vec3 true_vel = *vel0;
    double mass   = mass0;

    NavState nav;
    rc = nav_init(&nav, &true_pos, &true_vel, MCFDIR_NAV_P0_POS, MCFDIR_NAV_P0_VEL);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    GuidancePlan plan;
    rc = guid_init_plan(&plan);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    PdGains gains;
    rc = ctrl_init_gains(&gains);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    FaultState fs;
    rc = fdir_init(&fs);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    FuelState fuel;
    fuel.total_dv_mps = 0.0;
    fuel.prop_kg      = 0.0;
    fuel.fire_count   = 0U;

    double   mib_Ns     = GNC_MIN_IMPULSE_BIT;
    uint8_t  term_armed = 0U;
    uint32_t dock_dwell = 0U;
    uint8_t  docked     = 0U;
    uint8_t  prev_meas_valid = 1U;

    uint32_t step = 0U;
    for (step = 0U; (step < MCFDIR_MAX_STEPS) && (docked == 0U); step++) {

        double range = 0.0;
        rc = dyn_range(&true_pos, &range);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Arm terminal gains */
        {
            uint32_t cur_phase = 0U;
            rc = guid_active_phase(&plan, &cur_phase);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
            if ((term_armed == 0U) && (cur_phase >= (plan.count - 1U))) {
                rc = ctrl_apply_terminal_gains(&gains);
                GNC_ASSERT(rc == GNC_OK, rc, return rc);
                mib_Ns = MCFDIR_TERM_MIB_NS;
                term_armed = 1U;
            }
        }

        /* Docking check */
        {
            double spd = 0.0;
            rc = ctrl_vec3_norm(&true_vel, &spd);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
            uint8_t in_tol = (uint8_t)((range < GNC_DOCK_POS_TOL) &&
                                        (spd   < GNC_DOCK_VEL_TOL));
            if (in_tol != 0U) { dock_dwell++; } else { dock_dwell = 0U; }
            if (dock_dwell >= MCFDIR_DOCK_DWELL) { docked = 1U; break; }
        }

        /* Guidance, control */
        Vec3 pos_ref;
        Vec3 vel_ref;
        rc = guid_compute_ref(&plan, &nav, GNC_DT_SEC, &pos_ref, &vel_ref);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        Vec3 pos_err;
        Vec3 vel_err;
        rc = ctrl_vec3_sub(&pos_ref, &nav.pos, &pos_err);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = ctrl_vec3_sub(&vel_ref, &nav.vel, &vel_err);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        ControlCmd cmd;
        rc = ctrl_compute(&gains, &pos_err, &vel_err,
                          mass, GNC_DT_SEC, mib_Ns, &cmd, &fuel);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Compute fault-injected actual force and sensor validity */
        Vec3    actual_force;
        uint8_t meas_valid_now;
        mcf_inject_fault(fault_type, step, &cmd.force_N,
                         &actual_force, &meas_valid_now);

        /* Apply thruster scale dispersion to actual force */
        actual_force.v[0] *= thr_scale[0];
        actual_force.v[1] *= thr_scale[1];
        actual_force.v[2] *= thr_scale[2];

        /* FDIR checks using actual force and previous step's sensor validity */
        rc = fdir_check_thruster(&cmd.force_N, &actual_force, &fs);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = fdir_check_sensor(prev_meas_valid, &fs);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = fdir_check_nav(&nav, &fs);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = fdir_update(&fs, step);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Propagate true dynamics with actual (faulted) force */
        Vec3 new_pos;
        Vec3 new_vel;
        rc = dyn_propagate(&true_pos, &true_vel, &actual_force,
                           n_rad, GNC_DT_SEC, mass, &new_pos, &new_vel);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        true_pos = new_pos;
        true_vel = new_vel;

        /* Sensor: skip nav_update on dropout steps */
        Vec3   meas;
        double meas_sigma = 0.0;
        if (meas_valid_now != 0U) {
            rc = mcf_sensor(&true_pos, &meas, &meas_sigma);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
        }

        /* Nav propagation uses commanded (not actual) force — GNC assumption */
        rc = nav_propagate(&nav, &cmd.force_N, n_rad, GNC_DT_SEC, mass);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        if (meas_valid_now != 0U) {
            rc = nav_update(&nav, &meas, meas_sigma);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
        }

        mass -= cmd.prop_step_kg;
        if (mass < 10.0) { mass = 10.0; }

        /* Update sensor validity for next step's FDIR check */
        prev_meas_valid = meas_valid_now;

        /* For stuck_open: stop early once abort is declared (save time) */
        if ((fault_type == MCFDIR_FAULT_STUCK_OPEN) &&
            (fs.active_fault == FAULT_THR_STUCK_OPEN)) {
            break;
        }
    }

    result->run            = run_idx;
    result->steps          = step;
    result->fault_detected = (fs.active_fault != FAULT_NONE) ||
                             (fs.fault_step   != 0U) ? 1U : 0U;
    result->detect_step    = fs.fault_step;
    result->docked         = docked;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Phase 9 Retreat verification — types and helpers
 * ----------------------------------------------------------------------- */
#define MCFDIR_RETREAT_RECORD_DELAY  80U   /* steps after retreat to sample y */
#define MCFDIR_RETREAT_MAX_STEPS   2000U   /* max steps for retreat test       */

/* All GNC state for one retreat-verification run (static, no malloc). */
typedef struct {
    Vec3           true_pos;
    Vec3           true_vel;
    double         mass;
    double         n_rad;
    NavState       nav;
    GuidancePlan   plan;
    PdGains        gains;
    FaultState     fs;
    MissionManager mgr;
    FuelState      fuel;
    double         mib_Ns;
    uint8_t        term_armed;
    uint8_t        prev_meas_valid;
    uint32_t       mode_hold_step;
    uint32_t       mode_retreat_step;
    uint8_t        hold_found;
    uint8_t        retreat_found;
} RetreatRun;

static GncStatus retreat_init(RetreatRun *r,
    const Vec3 *pos0, const Vec3 *vel0, double mass0)
{
    GNC_ASSERT(r    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(pos0 != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(vel0 != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(mass0 > 0.0, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double sma = GNC_EARTH_RADIUS + GNC_ISS_ALTITUDE;
    GncStatus rc = dyn_mean_motion(sma, &r->n_rad);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    r->true_pos = *pos0;
    r->true_vel = *vel0;
    r->mass     = mass0;

    rc = nav_init(&r->nav, pos0, vel0, MCFDIR_NAV_P0_POS, MCFDIR_NAV_P0_VEL);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = guid_init_plan(&r->plan);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = ctrl_init_gains(&r->gains);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = fdir_init(&r->fs);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = mgr_init(&r->mgr);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    r->fuel.total_dv_mps = 0.0;
    r->fuel.prop_kg      = 0.0;
    r->fuel.fire_count   = 0U;
    r->mib_Ns            = GNC_MIN_IMPULSE_BIT;
    r->term_armed        = 0U;
    r->prev_meas_valid   = 1U;
    r->mode_hold_step    = 0U;
    r->mode_retreat_step = 0U;
    r->hold_found        = 0U;
    r->retreat_found     = 0U;
    return GNC_OK;
}

/* Step 1: guidance (with mission-manager override) + control. */
static GncStatus retreat_guidance_ctrl(RetreatRun *r, ControlCmd *cmd_out)
{
    GNC_ASSERT(r       != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(cmd_out != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* Arm terminal gains on entry to last phase */
    {
        uint32_t cur_phase = 0U;
        GncStatus rc = guid_active_phase(&r->plan, &cur_phase);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        if ((r->term_armed == 0U) && (cur_phase >= (r->plan.count - 1U))) {
            rc = ctrl_apply_terminal_gains(&r->gains);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
            r->mib_Ns     = MCFDIR_TERM_MIB_NS;
            r->term_armed = 1U;
        }
    }

    /* Mission-manager guidance override */
    Vec3    pos_ref, vel_ref;
    {
        Vec3    pos_ov;
        uint8_t ov_active = 0U;
        GncStatus rc = mgr_get_guidance_override(&r->mgr, &r->plan, &r->nav,
                                                  &pos_ov, &ov_active);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        if (ov_active != 0U) {
            pos_ref        = pos_ov;
            vel_ref.v[0]   = 0.0;
            vel_ref.v[1]   = 0.0;
            vel_ref.v[2]   = 0.0;
        } else {
            rc = guid_compute_ref(&r->plan, &r->nav, GNC_DT_SEC,
                                  &pos_ref, &vel_ref);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
        }
    }

    /* PD control */
    Vec3 pos_err, vel_err;
    GncStatus rc = ctrl_vec3_sub(&pos_ref, &r->nav.pos, &pos_err);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = ctrl_vec3_sub(&vel_ref, &r->nav.vel, &vel_err);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = ctrl_compute(&r->gains, &pos_err, &vel_err,
                      r->mass, GNC_DT_SEC, r->mib_Ns, cmd_out, &r->fuel);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    return GNC_OK;
}

/* Step 2: fault-inject → FDIR → mgr_update → dynamics → nav. */
static GncStatus retreat_fdir_dyn(RetreatRun *r, uint32_t step,
                                   const double ts[3], const ControlCmd *cmd)
{
    GNC_ASSERT(r   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(ts  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(cmd != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    /* Stuck-open injection on z-axis at step >= MCFDIR_INJECT_STEP */
    Vec3    actual_force = cmd->force_N;
    uint8_t meas_valid   = 1U;
    if (step >= MCFDIR_INJECT_STEP) {
        actual_force.v[2] = MCFDIR_STUCK_FORCE;
    }
    actual_force.v[0] *= ts[0];
    actual_force.v[1] *= ts[1];
    actual_force.v[2] *= ts[2];

    /* FDIR checks + update */
    GncStatus rc = fdir_check_thruster(&cmd->force_N, &actual_force, &r->fs);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = fdir_check_sensor(r->prev_meas_valid, &r->fs);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = fdir_check_nav(&r->nav, &r->fs);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = fdir_update(&r->fs, step);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Mission manager mode update */
    rc = mgr_update(&r->mgr, &r->fs, &r->plan, step, GNC_DT_SEC);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Track first occurrence of HOLD and ABORT_RETREAT */
    if ((r->hold_found == 0U) && (r->mgr.current_mode == MODE_HOLD)) {
        r->mode_hold_step = step;
        r->hold_found     = 1U;
    }
    if ((r->retreat_found == 0U) &&
        (r->mgr.current_mode == MODE_ABORT_RETREAT)) {
        r->mode_retreat_step = step;
        r->retreat_found     = 1U;
    }

    /* True dynamics propagation with faulted force */
    Vec3 new_pos, new_vel;
    rc = dyn_propagate(&r->true_pos, &r->true_vel, &actual_force,
                       r->n_rad, GNC_DT_SEC, r->mass, &new_pos, &new_vel);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    r->true_pos = new_pos;
    r->true_vel = new_vel;

    /* Navigation propagation and update */
    Vec3   meas;
    double sigma = 0.0;
    if (mcf_sensor(&r->true_pos, &meas, &sigma) == GNC_OK) { meas_valid = 1U; }

    rc = nav_propagate(&r->nav, &cmd->force_N, r->n_rad, GNC_DT_SEC, r->mass);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    if (meas_valid != 0U) {
        rc = nav_update(&r->nav, &meas, sigma);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
    }

    r->mass -= cmd->prop_step_kg;
    if (r->mass < 10.0) { r->mass = 10.0; }
    r->prev_meas_valid = meas_valid;
    return GNC_OK;
}

static GncStatus run_one_retreat(
    const Vec3 *pos0, const Vec3 *vel0, double mass0,
    const double ts[3], McFdirResult *result, uint32_t run_idx)
{
    GNC_ASSERT(pos0   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(vel0   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(ts     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(result != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    static RetreatRun s_r;   /* P10 Rule 3: static, not heap-allocated */
    GncStatus rc = retreat_init(&s_r, pos0, vel0, mass0);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    uint32_t step = 0U;
    for (step = 0U; step < MCFDIR_RETREAT_MAX_STEPS; step++) {
        ControlCmd cmd;
        rc = retreat_guidance_ctrl(&s_r, &cmd);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        rc = retreat_fdir_dyn(&s_r, step, ts, &cmd);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Stop once 80 steps beyond the retreat step have elapsed */
        if ((s_r.retreat_found != 0U) &&
            (step >= (s_r.mode_retreat_step + MCFDIR_RETREAT_RECORD_DELAY))) {
            break;
        }
    }

    result->run                  = run_idx;
    result->steps                = step;
    result->fault_detected       = (s_r.fs.fault_step != 0U) ? 1U : 0U;
    result->detect_step          = s_r.fs.fault_step;
    result->docked               = 0U;
    result->mode_hold_step       = s_r.mode_hold_step;
    result->mode_retreat_step    = s_r.mode_retreat_step;
    result->y_at_retreat_plus_80 = s_r.true_pos.v[1];
    return GNC_OK;
}

static GncStatus print_retreat_summary(
    const McFdirResult *results, uint32_t n_runs)
{
    GNC_ASSERT(results != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(n_runs  >  0U,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t n_hold_ok    = 0U;
    uint32_t n_retreat_ok = 0U;
    uint32_t n_y_ok       = 0U;

    for (uint32_t i = 0U; i < n_runs; i++) {
        if (results[i].mode_hold_step >= MCFDIR_INJECT_STEP) {
            uint32_t hl = results[i].mode_hold_step - MCFDIR_INJECT_STEP;
            if (hl <= 10U) { n_hold_ok++; }
        }
        if (results[i].mode_retreat_step >= MCFDIR_INJECT_STEP) {
            uint32_t rl = results[i].mode_retreat_step - MCFDIR_INJECT_STEP;
            if (rl <= 70U) { n_retreat_ok++; }
        }
        if (results[i].y_at_retreat_plus_80 > 50.0) { n_y_ok++; }
    }

    double p_hold    = (double)n_hold_ok    / (double)n_runs;
    double p_retreat = (double)n_retreat_ok / (double)n_runs;
    double p_y       = (double)n_y_ok       / (double)n_runs;

    (void)printf("\n===== RETREAT VERIFICATION SUMMARY (%u runs) =====\n",
                 n_runs);
    (void)printf("Hold within 10 steps    : %u/%u  P=%.3f  gate=1.00\n",
                 n_hold_ok,    n_runs, p_hold);
    (void)printf("Retreat within 70 steps : %u/%u  P=%.3f  gate=1.00\n",
                 n_retreat_ok, n_runs, p_retreat);
    (void)printf("y > 50 m at retreat+80  : %u/%u  P=%.3f  gate=1.00\n",
                 n_y_ok,       n_runs, p_y);

    uint8_t pass = ((p_hold >= 1.0) && (p_retreat >= 1.0) && (p_y >= 1.0))
                   ? 1U : 0U;
    if (pass != 0U) {
        (void)printf("PASS: all retreat gates met\n");
    } else {
        (void)printf("FAIL: one or more gates not met\n");
    }
    (void)printf("==================================================\n\n");
    return (pass != 0U) ? GNC_OK : ERR_BAD_PARAM;
}

/* -----------------------------------------------------------------------
 * Parse fault type string from command-line argument
 * ----------------------------------------------------------------------- */
static McFdirFaultType parse_fault(const char *s)
{
    GNC_ASSERT(s != NULL, ERR_NULL_PTR, return MCFDIR_FAULT_UNKNOWN);
    GNC_ASSERT(strlen(s) > 0U, ERR_BAD_PARAM, return MCFDIR_FAULT_UNKNOWN);

    if (strcmp(s, "stuck_open")   == 0) { return MCFDIR_FAULT_STUCK_OPEN;   }
    if (strcmp(s, "dropout")      == 0) { return MCFDIR_FAULT_DROPOUT;      }
    if (strcmp(s, "stuck_closed") == 0) { return MCFDIR_FAULT_STUCK_CLOSED; }
    if (strcmp(s, "retreat")      == 0) { return MCFDIR_FAULT_RETREAT;      }
    return MCFDIR_FAULT_UNKNOWN;
}

/* -----------------------------------------------------------------------
 * Print summary and check pass/fail gate
 * ----------------------------------------------------------------------- */
static GncStatus print_fdir_summary(
    const McFdirResult *results,
    uint32_t            n_runs,
    McFdirFaultType     fault_type)
{
    GNC_ASSERT(results != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(n_runs  >  0U,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t n_docked       = 0U;
    uint32_t n_fast_abort   = 0U;  /* abort within MCFDIR_ABORT_WINDOW steps */
    uint32_t n_detected     = 0U;

    for (uint32_t i = 0U; i < n_runs; i++) {
        if (results[i].docked != 0U) { n_docked++; }
        if (results[i].fault_detected != 0U) {
            n_detected++;
            uint32_t latency = results[i].detect_step - MCFDIR_INJECT_STEP;
            if (latency <= MCFDIR_ABORT_WINDOW) { n_fast_abort++; }
        }
    }

    double p_dock       = (double)n_docked     / (double)n_runs;
    double p_detected   = (double)n_detected   / (double)n_runs;
    double p_fast_abort = (double)n_fast_abort  / (double)n_runs;

    (void)printf("\n===== FDIR MC SUMMARY (%u runs) =====\n", n_runs);
    (void)printf("Fault injected at step : %u\n", MCFDIR_INJECT_STEP);
    (void)printf("Fault detected         : %u / %u  (P=%.3f)\n",
                 n_detected, n_runs, p_detected);

    if (fault_type == MCFDIR_FAULT_STUCK_OPEN) {
        (void)printf("Abort within %u steps  : %u / %u  (P=%.3f)\n",
                     MCFDIR_ABORT_WINDOW, n_fast_abort, n_runs, p_fast_abort);
        (void)printf("Gate: P(abort <= %u steps) == 1.00\n", MCFDIR_ABORT_WINDOW);
        if (p_fast_abort < 1.0) {
            (void)printf("FAIL: P=%.3f < 1.00\n", p_fast_abort);
            return ERR_BAD_PARAM;
        }
        (void)printf("PASS: P=%.3f >= 1.00\n", p_fast_abort);
    } else if (fault_type == MCFDIR_FAULT_DROPOUT) {
        (void)printf("Docked                 : %u / %u  (P=%.3f)\n",
                     n_docked, n_runs, p_dock);
        (void)printf("Gate: P(dock) >= 0.90\n");
        if (p_dock < 0.90) {
            (void)printf("FAIL: P=%.3f < 0.90\n", p_dock);
            return ERR_BAD_PARAM;
        }
        (void)printf("PASS: P=%.3f >= 0.90\n", p_dock);
    } else {
        (void)printf("Docked                 : %u / %u  (P=%.3f)\n",
                     n_docked, n_runs, p_dock);
        (void)printf("Gate: P(dock) >= 0.80\n");
        if (p_dock < 0.80) {
            (void)printf("FAIL: P=%.3f < 0.80\n", p_dock);
            return ERR_BAD_PARAM;
        }
        (void)printf("PASS: P=%.3f >= 0.80\n", p_dock);
    }
    (void)printf("=====================================\n\n");
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    GNC_ASSERT(argc >= 2, ERR_BAD_PARAM,
        { (void)fprintf(stderr,
            "Usage: mc_fdir_sim <stuck_open|dropout|stuck_closed|retreat> [N]\n");
          return 1; });
    GNC_ASSERT(argv != NULL, ERR_NULL_PTR, return (int)ERR_NULL_PTR);

    McFdirFaultType fault_type = parse_fault(argv[1]);
    GNC_ASSERT(fault_type != MCFDIR_FAULT_UNKNOWN, ERR_BAD_PARAM,
        { (void)fprintf(stderr,
            "Unknown fault type: %s\n", argv[1]); return 1; });

    uint32_t n_runs = 100U;
    if (argc >= 3) {
        int n_arg = atoi(argv[2]);
        if ((n_arg > 0) && ((uint32_t)n_arg <= MCFDIR_MAX_RUNS)) {
            n_runs = (uint32_t)n_arg;
        }
    }

    (void)printf("[MC-FDIR] fault=%s  N=%u  inject_step=%u\n",
                 argv[1], n_runs, MCFDIR_INJECT_STEP);

    static McFdirResult s_results[MCFDIR_MAX_RUNS];

    uint32_t run = 0U;
    for (run = 0U; run < n_runs; run++) {
        rng_seed(12345U + run * 6364136223846793005U % 4294967296U);

        Vec3 pos0;
        pos0.v[0] = 0.0             + MCFDIR_SIGMA_POS * rng_normal();
        pos0.v[1] = MCFDIR_POS0_Y   + MCFDIR_SIGMA_POS * rng_normal();
        pos0.v[2] = 0.0             + MCFDIR_SIGMA_POS * rng_normal();

        Vec3 vel0;
        vel0.v[0] = MCFDIR_SIGMA_VEL * rng_normal();
        vel0.v[1] = MCFDIR_SIGMA_VEL * rng_normal();
        vel0.v[2] = MCFDIR_SIGMA_VEL * rng_normal();

        double thr_scale[3];
        thr_scale[0] = 1.0 + MCFDIR_THR_HALF * (2.0 * rng_uniform() - 1.0);
        thr_scale[1] = 1.0 + MCFDIR_THR_HALF * (2.0 * rng_uniform() - 1.0);
        thr_scale[2] = 1.0 + MCFDIR_THR_HALF * (2.0 * rng_uniform() - 1.0);

        GncStatus rc;
        if (fault_type == MCFDIR_FAULT_RETREAT) {
            rc = run_one_retreat(&pos0, &vel0, GNC_CHASER_MASS,
                                 thr_scale, &s_results[run], run);
        } else {
            rc = run_one_fdir(&pos0, &vel0, GNC_CHASER_MASS,
                              thr_scale, fault_type, &s_results[run], run);
        }
        if (rc != GNC_OK) {
            s_results[run].run    = run;
            s_results[run].docked = 0U;
            (void)fprintf(stderr, "[MC-FDIR] Run %u error %d\n", run, (int)rc);
        }

        if (((run + 1U) % 5U) == 0U) {
            (void)printf("[MC-FDIR] Completed %u / %u\n", run + 1U, n_runs);
        }
    }

    GncStatus rc;
    if (fault_type == MCFDIR_FAULT_RETREAT) {
        rc = print_retreat_summary(s_results, n_runs);
    } else {
        rc = print_fdir_summary(s_results, n_runs, fault_type);
    }
    return (rc == GNC_OK) ? 0 : 1;
}
