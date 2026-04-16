/**
 * @file    sim/monte_carlo.c
 * @brief   Monte Carlo harness — NASA Power of 10 compliant.
 *
 * Disperses initial conditions and thruster scale factors using a
 * deterministic LCG, runs N independent GNC simulations, and writes
 * results to sim/mc_results.csv.
 *
 * Dispersions (Gaussian, Box-Muller from LCG):
 *   pos0  += N(0, 3.0) m  per axis
 *   vel0  += N(0, 0.05) m/s per axis
 *   mass0  = 500 + N(0, 0.1*500/100) kg  — small variation
 *   thr_scale per axis = 1.0 + U(-0.05, 0.05)
 *
 * Usage:
 *   ./mc_sim [N]   (default N=100)
 *
 * Compile via:
 *   make mc N=100
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

/* -----------------------------------------------------------------------
 * Monte Carlo configuration constants
 * ----------------------------------------------------------------------- */
#define MC_MAX_RUNS        2000U    /* hard upper bound for N runs           */
#define MC_MAX_STEPS       10000U   /* steps per run (matches main sim)      */
#define MC_DOCK_DWELL      30U      /* dwell steps required for dock confirm */
#define MC_TERM_RANGE_M    5.0      /* terminal phase arming threshold (m)   */

#define MC_SIGMA_POS       3.0      /* position dispersion 1-sigma (m)       */
#define MC_SIGMA_VEL       0.05     /* velocity dispersion 1-sigma (m/s)     */
#define MC_THR_HALF_RANGE  0.05     /* thruster scale dispersion ±5 %        */

/* Sensor model (same as main sim) */
#define MC_SENSOR_K        0.003
#define MC_SENSOR_FLOOR    0.002

/* Nominal initial conditions (must match main sim) */
#define MC_POS0_Y          200.0
#define MC_TERM_MIB_NS     0.005
#define MC_NAV_P0_POS      5.0
#define MC_NAV_P0_VEL      0.5

/* -----------------------------------------------------------------------
 * Result record — one per run
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t run;
    uint32_t steps;
    double   final_pos_err_m;   /* ‖r‖ at end (m)              */
    double   final_vel_err_mps; /* ‖v‖ at end (m/s)            */
    double   total_dv_mps;
    double   prop_kg;
    uint8_t  docked;            /* 1 = success                 */
} McResult;

/* -----------------------------------------------------------------------
 * LCG PRNG — deterministic, bounded (Rule 2)
 * ----------------------------------------------------------------------- */
static uint32_t s_rng = 0U;

static void rng_seed(uint32_t seed)
{
    s_rng = seed;
}

/** Advance LCG once; return uniform double in [0,1). */
static double rng_uniform(void)
{
    s_rng = s_rng * 1664525U + 1013904223U;
    return (double)(s_rng) / 4294967296.0;
}

/** Box-Muller normal variate using two LCG uniform draws. */
static double rng_normal(void)
{
    double u1 = rng_uniform() + 1.0e-9;   /* avoid log(0) */
    double u2 = rng_uniform();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* -----------------------------------------------------------------------
 * Sensor simulation (range-proportional noise)
 * ----------------------------------------------------------------------- */
static GncStatus mc_sensor(const Vec3 *true_pos, Vec3 *meas, double *sigma_out)
{
    GNC_ASSERT(true_pos  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(meas      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(sigma_out != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double range = 0.0;
    GncStatus rc = dyn_range(true_pos, &range);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    double sigma = MC_SENSOR_K * range;
    if (sigma < MC_SENSOR_FLOOR) { sigma = MC_SENSOR_FLOOR; }

    meas->v[0] = true_pos->v[0] + sigma * rng_normal();
    meas->v[1] = true_pos->v[1] + sigma * rng_normal();
    meas->v[2] = true_pos->v[2] + sigma * rng_normal();

    *sigma_out = sigma;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Run a single GNC simulation with dispersed initial conditions.
 * thr_scale[3] multiplies command force per axis (simulates thruster error).
 * ----------------------------------------------------------------------- */
static GncStatus run_one(
    const Vec3 *pos0,
    const Vec3 *vel0,
    double      mass0,
    const double thr_scale[3],
    McResult   *result,
    uint32_t    run_idx)
{
    GNC_ASSERT(pos0      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(vel0      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(thr_scale != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(result    != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(mass0     >  0.0,  ERR_BAD_PARAM, return ERR_BAD_PARAM);

    /* Orbit setup */
    double sma   = GNC_EARTH_RADIUS + GNC_ISS_ALTITUDE;
    double n_rad = 0.0;
    GncStatus rc = dyn_mean_motion(sma, &n_rad);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* True initial state */
    Vec3 true_pos = *pos0;
    Vec3 true_vel = *vel0;
    double mass   = mass0;

    /* Nav filter */
    NavState nav;
    rc = nav_init(&nav, &true_pos, &true_vel, MC_NAV_P0_POS, MC_NAV_P0_VEL);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Guidance */
    GuidancePlan plan;
    rc = guid_init_plan(&plan, NULL);   /* NULL → hardcoded defaults */
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Gains */
    PdGains gains;
    rc = ctrl_init_gains(&gains, NULL); /* NULL → hardcoded defaults */
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    /* Fuel */
    FuelState fuel;
    fuel.total_dv_mps = 0.0;
    fuel.prop_kg      = 0.0;
    fuel.fire_count   = 0U;

    double   mib_Ns     = GNC_MIN_IMPULSE_BIT;
    uint8_t  term_armed = 0U;
    uint32_t dock_dwell = 0U;
    uint8_t  docked     = 0U;
    uint32_t step       = 0U;

    for (step = 0U; (step < MC_MAX_STEPS) && (docked == 0U); step++) {

        /* Range on true state */
        double range = 0.0;
        rc = dyn_range(&true_pos, &range);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Arm terminal gains */
        {
            uint32_t cur_phase = 0U;
            rc = guid_active_phase(&plan, &cur_phase);
            GNC_ASSERT(rc == GNC_OK, rc, return rc);

            if ((term_armed == 0U) && (cur_phase >= (plan.count - 1U))) {
                rc = ctrl_apply_terminal_gains(&gains, NULL); /* NULL → defaults */
                GNC_ASSERT(rc == GNC_OK, rc, return rc);
                mib_Ns     = MC_TERM_MIB_NS;
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
            if (in_tol != 0U) {
                dock_dwell++;
            } else {
                dock_dwell = 0U;
            }

            if (dock_dwell >= MC_DOCK_DWELL) {
                docked = 1U;
                break;
            }
        }

        /* Guidance */
        Vec3 pos_ref;
        Vec3 vel_ref;
        rc = guid_compute_ref(&plan, &nav, GNC_DT_SEC, &pos_ref, &vel_ref);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Control errors */
        Vec3 pos_err;
        Vec3 vel_err;
        rc = ctrl_vec3_sub(&pos_ref, &nav.pos, &pos_err);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = ctrl_vec3_sub(&vel_ref, &nav.vel, &vel_err);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Control command */
        ControlCmd cmd;
        rc = ctrl_compute(&gains, &pos_err, &vel_err,
                          mass, GNC_DT_SEC, mib_Ns, &cmd, &fuel);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Apply thruster scale factor (dispersion) */
        Vec3 effective_force;
        effective_force.v[0] = cmd.force_N.v[0] * thr_scale[0];
        effective_force.v[1] = cmd.force_N.v[1] * thr_scale[1];
        effective_force.v[2] = cmd.force_N.v[2] * thr_scale[2];

        /* Propagate true dynamics */
        Vec3 new_pos;
        Vec3 new_vel;
        rc = dyn_propagate(&true_pos, &true_vel, &effective_force,
                           n_rad, GNC_DT_SEC, mass, &new_pos, &new_vel);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        true_pos = new_pos;
        true_vel = new_vel;

        /* Sensor */
        Vec3   meas;
        double meas_sigma = 0.0;
        rc = mc_sensor(&true_pos, &meas, &meas_sigma);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Nav update */
        rc = nav_propagate(&nav, &cmd.force_N, n_rad, GNC_DT_SEC, mass);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);
        rc = nav_update(&nav, &meas, meas_sigma);
        GNC_ASSERT(rc == GNC_OK, rc, return rc);

        /* Mass burn-down */
        mass -= cmd.prop_step_kg;
        if (mass < 10.0) { mass = 10.0; }
    }

    /* Compute final position and velocity errors */
    double pos_err_norm = sqrt(true_pos.v[0]*true_pos.v[0] +
                               true_pos.v[1]*true_pos.v[1] +
                               true_pos.v[2]*true_pos.v[2]);
    double vel_err_norm = sqrt(true_vel.v[0]*true_vel.v[0] +
                               true_vel.v[1]*true_vel.v[1] +
                               true_vel.v[2]*true_vel.v[2]);

    result->run              = run_idx;
    result->steps            = step;
    result->final_pos_err_m  = pos_err_norm;
    result->final_vel_err_mps= vel_err_norm;
    result->total_dv_mps     = fuel.total_dv_mps;
    result->prop_kg          = fuel.prop_kg;
    result->docked           = docked;

    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Write CSV results
 * ----------------------------------------------------------------------- */
static GncStatus write_mc_results(
    FILE            *fp,
    const McResult  *results,
    uint32_t         n_runs)
{
    GNC_ASSERT(fp      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(results != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(n_runs  >  0U,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    int ret = fprintf(fp,
        "run,steps,final_pos_err_m,final_vel_err_mps,"
        "total_dv_mps,prop_kg,docked\n");
    GNC_ASSERT(ret > 0, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t i = 0U;
    for (i = 0U; i < n_runs; i++) {
        ret = fprintf(fp, "%u,%u,%.6f,%.6f,%.6f,%.6f,%u\n",
            results[i].run,
            results[i].steps,
            results[i].final_pos_err_m,
            results[i].final_vel_err_mps,
            results[i].total_dv_mps,
            results[i].prop_kg,
            (unsigned)results[i].docked);
        GNC_ASSERT(ret > 0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    }
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Print summary statistics
 * ----------------------------------------------------------------------- */
static GncStatus print_summary(const McResult *results, uint32_t n_runs)
{
    GNC_ASSERT(results != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(n_runs  >  0U,   ERR_BAD_PARAM, return ERR_BAD_PARAM);

    uint32_t n_docked = 0U;
    double   sum_dv   = 0.0;
    double   sum_pos  = 0.0;
    double   min_dv   = results[0].total_dv_mps;
    double   max_dv   = min_dv;

    uint32_t i = 0U;
    for (i = 0U; i < n_runs; i++) {
        if (results[i].docked != 0U) { n_docked++; }
        sum_dv  += results[i].total_dv_mps;
        sum_pos += results[i].final_pos_err_m;
        if (results[i].total_dv_mps < min_dv) { min_dv = results[i].total_dv_mps; }
        if (results[i].total_dv_mps > max_dv) { max_dv = results[i].total_dv_mps; }
    }

    double mean_dv    = sum_dv  / (double)n_runs;
    double mean_pos   = sum_pos / (double)n_runs;
    double p_dock     = (double)n_docked / (double)n_runs;

    (void)printf("\n===== MONTE CARLO SUMMARY (%u runs) =====\n", n_runs);
    (void)printf("Docked           : %u / %u  (P=%.3f)\n", n_docked, n_runs, p_dock);
    (void)printf("Mean total DeltaV: %.4f m/s\n", mean_dv);
    (void)printf("Min  total DeltaV: %.4f m/s\n", min_dv);
    (void)printf("Max  total DeltaV: %.4f m/s\n", max_dv);
    (void)printf("Mean final pos   : %.4f m\n",   mean_pos);
    (void)printf("==========================================\n\n");

    if (p_dock < 0.95) {
        (void)printf("FAIL: Dock probability %.3f < 0.95 threshold\n", p_dock);
        GNC_ASSERT(p_dock >= 0.95, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    } else {
        (void)printf("PASS: Dock probability %.3f >= 0.95\n", p_dock);
    }
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    /* Parse optional run count argument */
    uint32_t n_runs = 100U;
    if (argc >= 2) {
        int n_arg = atoi(argv[1]);
        if ((n_arg > 0) && ((uint32_t)n_arg <= MC_MAX_RUNS)) {
            n_runs = (uint32_t)n_arg;
        } else {
            (void)fprintf(stderr,
                "mc_sim: N=%s invalid; using default N=%u (max %u)\n",
                argv[1], n_runs, MC_MAX_RUNS);
        }
    }

    (void)printf("[MC] Starting Monte Carlo: N=%u runs\n", n_runs);

    /* Static result pool */
    static McResult s_results[MC_MAX_RUNS];

    /* Nominal initial state */
    double pos0_y = MC_POS0_Y;

    uint32_t n_ok    = 0U;
    uint32_t run     = 0U;

    for (run = 0U; run < n_runs; run++) {

        /* Seed RNG per run: run * prime offset for decorrelation */
        rng_seed(12345U + run * 6364136223846793005U % 4294967296U);

        /* Disperse initial position (N(0, MC_SIGMA_POS) per axis) */
        Vec3 pos0;
        pos0.v[0] = 0.0     + MC_SIGMA_POS * rng_normal();
        pos0.v[1] = pos0_y  + MC_SIGMA_POS * rng_normal();
        pos0.v[2] = 0.0     + MC_SIGMA_POS * rng_normal();

        /* Disperse initial velocity */
        Vec3 vel0;
        vel0.v[0] = MC_SIGMA_VEL * rng_normal();
        vel0.v[1] = MC_SIGMA_VEL * rng_normal();
        vel0.v[2] = MC_SIGMA_VEL * rng_normal();

        /* Disperse mass */
        double mass0 = GNC_CHASER_MASS;

        /* Thruster scale factor: U(1-half, 1+half) per axis */
        double thr_scale[3];
        thr_scale[0] = 1.0 + MC_THR_HALF_RANGE * (2.0 * rng_uniform() - 1.0);
        thr_scale[1] = 1.0 + MC_THR_HALF_RANGE * (2.0 * rng_uniform() - 1.0);
        thr_scale[2] = 1.0 + MC_THR_HALF_RANGE * (2.0 * rng_uniform() - 1.0);

        GncStatus rc = run_one(&pos0, &vel0, mass0, thr_scale,
                               &s_results[run], run);
        if (rc == GNC_OK) {
            n_ok++;
        } else {
            /* Mark failed run */
            s_results[run].run    = run;
            s_results[run].docked = 0U;
            (void)fprintf(stderr, "[MC] Run %u returned error %d\n",
                          run, (int)rc);
        }

        /* Progress heartbeat every 10 runs */
        if (((run + 1U) % 10U) == 0U) {
            (void)printf("[MC] Completed %u / %u runs\n", run + 1U, n_runs);
        }
    }

    /* Write CSV */
    FILE *fp = fopen("sim/mc_results.csv", "w");
    GNC_ASSERT(fp != NULL, ERR_NULL_PTR, return (int)ERR_NULL_PTR);

    GncStatus rc = write_mc_results(fp, s_results, n_runs);
    (void)fclose(fp);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    (void)printf("[MC] Results written to sim/mc_results.csv\n");

    /* Summary + pass/fail gate */
    rc = print_summary(s_results, n_runs);
    if (rc != GNC_OK) {
        return 1;
    }
    (void)n_ok;
    return 0;
}
