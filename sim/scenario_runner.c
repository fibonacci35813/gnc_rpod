/**
 * @file    sim/scenario_runner.c
 * @brief   Batch scenario Monte Carlo runner — Phase 2, NASA P10 compliant.
 *
 * Runs all 8 standardised scenarios (or a named subset) and writes per-
 * scenario CSV files to sim/results/.  Prints a PASS/FAIL matrix.
 *
 * Usage:
 *   ./sim/scenario_runner --params=<file>
 *   ./sim/scenario_runner --params=<file> --scenario=<name>
 *   ./sim/scenario_runner --params=<file> --n=<seeds>
 *   ./sim/scenario_runner --params=<file> --json       (stdout JSON)
 *
 * For scenario "stuck_open": PASS when abort_rate == 1.00 (dock_rate
 * is expected 0.0 and is not used as the gate).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "gnc_types.h"
#include "gnc_assert.h"
#include "dynamics.h"
#include "nav_filter.h"
#include "guidance.h"
#include "control.h"
#include "fdir.h"
#include "mission_mgr.h"
#include "params.h"
#include "scenarios.h"

/* -----------------------------------------------------------------------
 * Runner constants (Rule 2)
 * ----------------------------------------------------------------------- */
#define SR_MAX_RUNS       2000U   /* hard upper bound on seeds per scenario  */
#define SR_MAX_STEPS      GNC_MAX_SIM_STEPS  /* must match fdir_update bound */
#define SR_DOCK_DWELL       30U   /* consecutive steps for dock confirmation */
#define SR_TERM_RANGE_M     5.0   /* arm terminal gains when range < this    */
#define SR_SIGMA_POS        3.0   /* position dispersion 1-sigma (m)         */
#define SR_SIGMA_VEL        0.05  /* velocity dispersion 1-sigma (m/s)       */
#define SR_THR_HALF         0.05  /* thruster scale dispersion ±5 %          */
#define SR_SENSOR_K         0.003 /* range-proportional noise gain           */
#define SR_SENSOR_FLOOR     0.002 /* noise floor (m)                         */
#define SR_STUCK_FORCE      0.5   /* N: injected stuck-open force magnitude  */
#define SR_STUCK_CL_STEPS   50U   /* steps of zero thrust for stuck_closed   */
#define SR_DROPOUT_STEPS     3U   /* consecutive sensor-off steps             */
#define SR_RESULTS_DIR     "sim/results"
#define SR_PATH_MAX        128U   /* max chars in output file path           */

/* -----------------------------------------------------------------------
 * Per-run result
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t run;
    uint32_t steps;
    uint8_t  docked;
    uint8_t  aborted;         /* stuck_open: abort declared              */
    double   final_pos_m;
    double   total_dv_mps;
    double   prop_kg;
} SrResult;

/* -----------------------------------------------------------------------
 * Aggregated per-scenario metrics
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t n_runs;
    uint32_t n_docked;
    uint32_t n_aborted;
    double   mean_dv_mps;
    double   mean_pos_m;
    double   mean_steps;
} SrMetrics;

/* -----------------------------------------------------------------------
 * LCG PRNG — deterministic, bounded (Rule 2)
 * ----------------------------------------------------------------------- */
static uint32_t sr_rng = 0U;

static void sr_seed(uint32_t seed) { sr_rng = seed; }

static double sr_uniform(void)
{
    sr_rng = sr_rng * 1664525U + 1013904223U;
    return (double)(sr_rng) / 4294967296.0;
}

static double sr_normal(void)
{
    double u1 = sr_uniform() + 1.0e-9;
    double u2 = sr_uniform();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* -----------------------------------------------------------------------
 * Sensor simulation
 * ----------------------------------------------------------------------- */
static GncStatus sr_sensor(const Vec3 *tp, Vec3 *meas, double *sig)
{
    GNC_ASSERT(tp   != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(meas != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(sig  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double range = 0.0;
    GncStatus rc = dyn_range(tp, &range);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    double s = SR_SENSOR_K * range;
    if (s < SR_SENSOR_FLOOR) { s = SR_SENSOR_FLOOR; }

    meas->v[0] = tp->v[0] + s * sr_normal();
    meas->v[1] = tp->v[1] + s * sr_normal();
    meas->v[2] = tp->v[2] + s * sr_normal();
    *sig = s;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Apply fault injection: modify actual_force and meas_valid in place.
 * ----------------------------------------------------------------------- */
static void sr_inject(
    FaultInjType    type,
    uint32_t        inject_step,
    uint32_t        step,
    const Vec3     *cmd,
    Vec3           *actual,
    uint8_t        *meas_valid)
{
    *actual    = *cmd;
    *meas_valid = 1U;

    if (step < inject_step) { return; }

    if (type == FAULT_INJ_STUCK_OPEN) {
        actual->v[2] = SR_STUCK_FORCE;  /* uncmd'd force on z-axis */

    } else if (type == FAULT_INJ_DROPOUT) {
        if (step < (inject_step + SR_DROPOUT_STEPS)) {
            *meas_valid = 0U;
        }

    } else if (type == FAULT_INJ_STUCK_CLOSED) {
        if (step < (inject_step + SR_STUCK_CL_STEPS)) {
            actual->v[0] = 0.0;
            actual->v[1] = 0.0;
            actual->v[2] = 0.0;
        }

    } else {
        /* FAULT_INJ_NONE: no modification */
    }
}

/* -----------------------------------------------------------------------
 * Docking check helper — returns 1 if dock dwell counter reached.
 * ----------------------------------------------------------------------- */
static uint8_t sr_check_dock(
    const Vec3 *tp,
    const Vec3 *tv,
    uint32_t   *dwell)
{
    double range = 0.0;
    (void)dyn_range(tp, &range);
    double spd = sqrt(tv->v[0]*tv->v[0] + tv->v[1]*tv->v[1] + tv->v[2]*tv->v[2]);

    uint8_t in_tol = (uint8_t)((range < GNC_DOCK_POS_TOL) && (spd < GNC_DOCK_VEL_TOL));
    if (in_tol != 0U) { (*dwell)++; } else { *dwell = 0U; }
    return (uint8_t)(*dwell >= SR_DOCK_DWELL);
}

/* -----------------------------------------------------------------------
 * Run one GNC simulation for a scenario seed.
 * Initialises GNC from params (or NULL→defaults), disperses ICs.
 * ----------------------------------------------------------------------- */
static GncStatus sr_run_one(
    const Scenario  *sc,
    const GncParams *p,
    uint32_t         seed,
    uint32_t         run_idx,
    SrResult        *out)
{
    GNC_ASSERT(sc  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(out != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    sr_seed(seed);

    /* Dispersed initial conditions */
    Vec3 true_pos;
    Vec3 true_vel;
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++) {
        true_pos.v[i] = sc->pos0.v[i] + SR_SIGMA_POS * sr_normal();
        true_vel.v[i] = sc->vel0.v[i] + SR_SIGMA_VEL * sr_normal();
    }
    double mass = sc->prop_init_kg + 495.0;  /* prop + dry mass */

    /* Thruster scale dispersion */
    double thr[3];
    for (i = 0U; i < 3U; i++) {
        thr[i] = 1.0 + (2.0 * sr_uniform() - 1.0) * SR_THR_HALF;
    }

    /* Orbit mean motion */
    double sma   = GNC_EARTH_RADIUS + GNC_ISS_ALTITUDE;
    double n_rad = 0.0;
    GncStatus rc = dyn_mean_motion(sma, &n_rad);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* GNC initialisation */
    NavState      nav;
    GuidancePlan  plan;
    PdGains       gains;
    FaultState    fs;
    MissionManager mgr;

    rc = nav_init(&nav, &true_pos, &true_vel, 5.0, 0.5);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = guid_init_plan(&plan, p);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = ctrl_init_gains(&gains, p);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = fdir_init(&fs, p);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);
    rc = mgr_init(&mgr);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    FuelState fuel;
    fuel.total_dv_mps = 0.0;
    fuel.prop_kg      = 0.0;
    fuel.fire_count   = 0U;

    double   mib_Ns     = (p != NULL) ? p->mib_normal_ns   : GNC_MIN_IMPULSE_BIT;
    double   mib_term   = (p != NULL) ? p->mib_terminal_ns : 0.005;
    uint8_t  term_armed = 0U;
    uint32_t dock_dwell = 0U;
    uint8_t  docked     = 0U;
    uint8_t  aborted    = 0U;
    uint8_t  prev_valid = 1U;
    uint32_t step       = 0U;

    for (step = 0U; (step < SR_MAX_STEPS) && (docked == 0U) && (aborted == 0U); step++) {

        /* Arm terminal gains */
        {
            uint32_t cur_ph = 0U;
            rc = guid_active_phase(&plan, &cur_ph);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
            if ((term_armed == 0U) && (cur_ph >= (plan.count - 1U))) {
                rc = ctrl_apply_terminal_gains(&gains, p);
                GNC_ASSERT(rc == GNC_OK, rc, return rc);
                mib_Ns     = mib_term;
                term_armed = 1U;
            }
        }

        /* Docking check */
        if (sr_check_dock(&true_pos, &true_vel, &dock_dwell) != 0U) {
            docked = 1U;
            break;
        }

        /* Guidance reference — with abort override if needed */
        Vec3 pos_ref;
        Vec3 vel_ref;
        Vec3 abort_override;
        uint8_t override_active = 0U;
        rc = mgr_get_guidance_override(&mgr, &plan, &nav,
                                       &abort_override, &override_active);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        if (override_active != 0U) {
            pos_ref = abort_override;
            vel_ref.v[0] = 0.0; vel_ref.v[1] = 0.0; vel_ref.v[2] = 0.0;
        } else {
            rc = guid_compute_ref(&plan, &nav, GNC_DT_SEC, &pos_ref, &vel_ref);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
        }

        /* Control */
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

        /* Fault injection — produces faulted force (no scale yet).
         * FDIR compares cmd vs faulted force (pre-scale) so that
         * normal thrust uncertainty (±5 %) does not trigger false
         * stuck_closed detections. */
        Vec3    faulted_force;   /* fault-injected, pre-scale (for FDIR)   */
        Vec3    actual_force;    /* faulted + scale (for true dynamics)     */
        uint8_t meas_valid_now;
        sr_inject(sc->fault.type, sc->fault.inject_step,
                  step, &cmd.force_N, &faulted_force, &meas_valid_now);

        /* Scale dispersion applied ONLY to dynamics, not FDIR check */
        actual_force.v[0] = faulted_force.v[0] * thr[0];
        actual_force.v[1] = faulted_force.v[1] * thr[1];
        actual_force.v[2] = faulted_force.v[2] * thr[2];

        /* FDIR uses cmd vs faulted_force (pre-scale) */
        rc = fdir_check_thruster(&cmd.force_N, &faulted_force, &fs);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = fdir_check_sensor(prev_valid, &fs);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = fdir_check_nav(&nav, &fs);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = fdir_update(&fs, step);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Mission manager update */
        rc = mgr_update(&mgr, &fs, &plan, step, GNC_DT_SEC);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Abort detection for stuck_open */
        if ((sc->fault.type == FAULT_INJ_STUCK_OPEN) &&
            (mgr.current_mode == MODE_ABORT_SAFE)) {
            aborted = 1U;
            break;
        }

        /* Propagate true dynamics (with env perturbations) */
        Vec3 new_pos;
        Vec3 new_vel;
        rc = dyn_propagate_perturbed(&true_pos, &true_vel, &actual_force,
                                     &sc->env, n_rad, GNC_DT_SEC, mass,
                                     &new_pos, &new_vel);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        true_pos = new_pos;
        true_vel = new_vel;

        /* Nav */
        Vec3   meas;
        double meas_sigma = 0.0;
        if (meas_valid_now != 0U) {
            rc = sr_sensor(&true_pos, &meas, &meas_sigma);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
        }
        rc = nav_propagate(&nav, &cmd.force_N, n_rad, GNC_DT_SEC, mass);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        if (meas_valid_now != 0U) {
            rc = nav_update(&nav, &meas, meas_sigma);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);
        }

        mass -= cmd.prop_step_kg;
        if (mass < 10.0) { mass = 10.0; }
        prev_valid = meas_valid_now;
    }

    out->run          = run_idx;
    out->steps        = step;
    out->docked       = docked;
    out->aborted      = aborted;
    out->final_pos_m  = sqrt(true_pos.v[0]*true_pos.v[0] +
                             true_pos.v[1]*true_pos.v[1] +
                             true_pos.v[2]*true_pos.v[2]);
    out->total_dv_mps = fuel.total_dv_mps;
    out->prop_kg      = fuel.prop_kg;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Aggregate results into metrics.
 * ----------------------------------------------------------------------- */
static void sr_aggregate(const SrResult *results, uint32_t n,
                         SrMetrics *m)
{
    m->n_runs    = n;
    m->n_docked  = 0U;
    m->n_aborted = 0U;
    m->mean_dv_mps = 0.0;
    m->mean_pos_m  = 0.0;
    m->mean_steps  = 0.0;

    uint32_t k = 0U;
    for (k = 0U; k < n; k++) {
        if (results[k].docked  != 0U) { m->n_docked++;  }
        if (results[k].aborted != 0U) { m->n_aborted++; }
        m->mean_dv_mps += results[k].total_dv_mps;
        m->mean_pos_m  += results[k].final_pos_m;
        m->mean_steps  += (double)results[k].steps;
    }

    if (n > 0U) {
        double nd = (double)n;
        m->mean_dv_mps /= nd;
        m->mean_pos_m  /= nd;
        m->mean_steps  /= nd;
    }
}

/* -----------------------------------------------------------------------
 * Write CSV for one scenario: sim/results/<name>_mc.csv
 * ----------------------------------------------------------------------- */
static int sr_write_csv(const char *name, const SrResult *results,
                        uint32_t n)
{
    char path[SR_PATH_MAX];
    int rv = snprintf(path, SR_PATH_MAX,
                      "%s/%s_mc.csv", SR_RESULTS_DIR, name);
    if ((rv < 0) || ((uint32_t)rv >= SR_PATH_MAX)) { return -1; }

    FILE *fp = fopen(path, "w");
    if (fp == NULL) { return -1; }

    int r = 0;
    r |= (fprintf(fp, "run,steps,docked,aborted,final_pos_m,"
                      "total_dv_mps,prop_kg\n") < 0) ? 1 : 0;

    uint32_t k = 0U;
    for (k = 0U; (k < n) && (r == 0); k++) {
        r |= (fprintf(fp, "%u,%u,%u,%u,%.6f,%.6f,%.6f\n",
            results[k].run, results[k].steps,
            results[k].docked, results[k].aborted,
            results[k].final_pos_m,
            results[k].total_dv_mps,
            results[k].prop_kg) < 0) ? 1 : 0;
    }

    (void)fclose(fp);
    return (r != 0) ? -1 : 0;
}

/* -----------------------------------------------------------------------
 * Determine PASS/FAIL for a scenario.
 * ----------------------------------------------------------------------- */
static uint8_t sr_gate_pass(const Scenario *sc, const SrMetrics *m)
{
    if (sc->fault.type == FAULT_INJ_STUCK_OPEN) {
        /* Gate: abort_rate == 1.00 */
        return (uint8_t)(m->n_aborted == m->n_runs);
    }
    double dock_rate = (m->n_runs > 0U)
        ? ((double)m->n_docked / (double)m->n_runs) : 0.0;
    return (uint8_t)(dock_rate >= sc->dock_rate_gate);
}

/* -----------------------------------------------------------------------
 * Print results summary line.
 * ----------------------------------------------------------------------- */
static void sr_print_result(const Scenario *sc, const SrMetrics *m,
                            uint8_t pass)
{
    double dock_rate = (m->n_runs > 0U)
        ? ((double)m->n_docked  / (double)m->n_runs) : 0.0;
    double abort_rate = (m->n_runs > 0U)
        ? ((double)m->n_aborted / (double)m->n_runs) : 0.0;

    (void)printf("  %-18s  dock=%.3f  abort=%.3f  dv=%.3f  pos=%.4f"
                 "  steps=%.0f  [%s]\n",
                 sc->name,
                 dock_rate,
                 abort_rate,
                 m->mean_dv_mps,
                 m->mean_pos_m,
                 m->mean_steps,
                 pass ? "PASS" : "FAIL");
}

/* -----------------------------------------------------------------------
 * Print JSON line for one scenario (--json mode).
 * ----------------------------------------------------------------------- */
/* Emit one JSON object per line (no array wrapper, no trailing commas).
 * Python parser reads line-by-line and picks up the matching scenario. */
static void sr_print_json(const Scenario *sc, const SrMetrics *m,
                          uint8_t pass, uint8_t last)
{
    double dock_rate = (m->n_runs > 0U)
        ? ((double)m->n_docked  / (double)m->n_runs) : 0.0;
    double abort_rate = (m->n_runs > 0U)
        ? ((double)m->n_aborted / (double)m->n_runs) : 0.0;

    /* Suppress unused-param warning: last is kept for API compatibility */
    (void)last;

    (void)printf("{\"%s\":{\"dock_rate\":%.4f,\"abort_rate\":%.4f,"
                 "\"mean_dv_mps\":%.4f,\"mean_final_pos_m\":%.4f,"
                 "\"mean_steps\":%.1f,\"pass\":%s}}\n",
                 sc->name,
                 dock_rate, abort_rate,
                 m->mean_dv_mps, m->mean_pos_m, m->mean_steps,
                 pass ? "true" : "false");
}

/* -----------------------------------------------------------------------
 * Run one scenario: allocate stack results, run seeds, aggregate, write.
 * Returns 1 if gate passes, 0 if fails.
 * ----------------------------------------------------------------------- */
static uint8_t sr_run_scenario(const Scenario *sc,
                               const GncParams *p,
                               uint32_t n_override,
                               uint8_t json_mode)
{
    uint32_t n = (n_override > 0U) ? n_override : sc->n_runs;
    if (n > SR_MAX_RUNS) { n = SR_MAX_RUNS; }

    /* Stack-allocated results array (bounded by SR_MAX_RUNS = 2000) */
    SrResult results[SR_MAX_RUNS];
    uint32_t k = 0U;

    for (k = 0U; k < n; k++) {
        uint32_t seed = sc->seed_start + k;
        SrResult res;
        GncStatus rc = sr_run_one(sc, p, seed, k, &res);
        if (rc != GNC_OK) {
            res.docked       = 0U;
            res.aborted      = 0U;
            res.steps        = 0U;
            res.final_pos_m  = 9999.0;
            res.total_dv_mps = 0.0;
            res.prop_kg      = 0.0;
        }
        results[k] = res;
    }

    SrMetrics m;
    sr_aggregate(results, n, &m);

    /* Write CSV */
    (void)sr_write_csv(sc->name, results, n);

    uint8_t pass = sr_gate_pass(sc, &m);

    if (json_mode != 0U) {
        /* caller decides comma; just always print for now */
        sr_print_json(sc, &m, pass, 0U);
    } else {
        sr_print_result(sc, &m, pass);
    }

    return pass;
}

/* -----------------------------------------------------------------------
 * Command-line argument parsing helpers
 * ----------------------------------------------------------------------- */
#define SR_ARG_MAX 256U

static void parse_str_arg(const char *arg, const char *prefix,
                          char *out, uint32_t out_len)
{
    size_t plen = strlen(prefix);
    if (strncmp(arg, prefix, plen) == 0) {
        strncpy(out, arg + plen, (size_t)(out_len - 1U));
        out[out_len - 1U] = '\0';
    }
}

static void parse_uint_arg(const char *arg, const char *prefix,
                           uint32_t *out)
{
    size_t plen = strlen(prefix);
    if (strncmp(arg, prefix, plen) == 0) {
        int v = atoi(arg + plen);
        if (v > 0) { *out = (uint32_t)v; }
    }
}

/* -----------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    char     params_path[SR_ARG_MAX];
    char     scenario_name[SCENARIO_NAME_LEN];
    uint32_t n_override = 0U;
    uint8_t  json_mode  = 0U;

    params_path[0]    = '\0';
    scenario_name[0]  = '\0';

    int i = 1;
    for (i = 1; i < argc; i++) {
        parse_str_arg(argv[i], "--params=",   params_path,   SR_ARG_MAX);
        parse_str_arg(argv[i], "--scenario=", scenario_name, SCENARIO_NAME_LEN);
        parse_uint_arg(argv[i], "--n=", &n_override);
        if (strncmp(argv[i], "--json", 6) == 0) { json_mode = 1U; }
    }

    /* Load params (defaults if no --params) */
    GncParams params;
    params_set_defaults(&params);
    if (params_path[0] != '\0') {
        (void)params_read_json(&params, params_path);
    }

    /* Create results directory (best-effort; ignore error) */
    {
        int sysrc = system("mkdir -p " SR_RESULTS_DIR);
        (void)sysrc;
    }

    uint32_t n_pass  = 0U;
    uint32_t n_total = 0U;

    if (json_mode == 0U) {
        (void)printf("[SCENARIO_RUNNER] Running scenarios...\n");
    }

    uint32_t k = 0U;
    for (k = 0U; k < SCENARIO_COUNT; k++) {
        const Scenario *sc = &SCENARIO_TABLE[k];

        /* Filter by --scenario= if provided */
        if (scenario_name[0] != '\0') {
            if (strncmp(sc->name, scenario_name, SCENARIO_NAME_LEN) != 0) {
                continue;
            }
        }

        uint8_t pass = sr_run_scenario(sc, &params, n_override, json_mode);
        n_total++;
        if (pass != 0U) { n_pass++; }
    }

    if (json_mode == 0U) {
        (void)printf("[SCENARIO_RUNNER] Results: %u/%u PASS\n",
                     n_pass, n_total);
        (void)printf("[SCENARIO_RUNNER] CSVs in %s/\n", SR_RESULTS_DIR);
        if (n_pass < n_total) {
            return 1;
        }
    }

    return 0;
}
