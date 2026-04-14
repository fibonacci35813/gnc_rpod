/**
 * @file    sim/main.c
 * @brief   Autonomous docking simulation harness — NASA Power of 10.
 *
 * Runs a closed-loop GNC simulation:
 *   1. Propagate true dynamics (CW + thrust)
 *   2. Simulate LIDAR sensor noise
 *   3. Run Kalman nav filter
 *   4. Compute guidance reference
 *   5. Compute control command
 *   6. Log telemetry to telem.csv
 *   7. Check docking condition
 *
 * Compile:
 *   gcc -std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion \
 *       -Wdouble-promotion -I include/ \
 *       src/dynamics.c src/nav_filter.c src/guidance.c src/control.c \
 *       sim/main.c -lm -o sim/dock_sim
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "gnc_types.h"
#include "gnc_assert.h"
#include "dynamics.h"
#include "nav_filter.h"
#include "guidance.h"
#include "control.h"

/* -----------------------------------------------------------------------
 * Simulation parameters
 * ----------------------------------------------------------------------- */
#define SIM_SEED              42U     /* LCG seed for noise                */
#define NAV_P0_POS             5.0    /* initial position uncertainty (m)  */
#define NAV_P0_VEL             0.5    /* initial velocity uncertainty (m/s)*/
#define SIM_TERMINAL_RANGE_M   5.0    /* range at which terminal gains arm */
#define SIM_TERMINAL_MIB_NS    0.005  /* terminal MIB: 5 milli-N·s        */
#define SIM_DOCK_DWELL_STEPS  30U     /* must be within tol for 30 steps   */

/* -----------------------------------------------------------------------
 * Minimal LCG PRNG (no stdlib rand — deterministic, bounded)
 * ----------------------------------------------------------------------- */
static uint32_t s_lcg_state = SIM_SEED;

static double lcg_normal(void)
{
    /* Box-Muller using two uniform LCG samples */
    s_lcg_state = s_lcg_state * 1664525U + 1013904223U;
    double u1 = (double)(s_lcg_state) / 4294967296.0 + 1.0e-9;
    s_lcg_state = s_lcg_state * 1664525U + 1013904223U;
    double u2 = (double)(s_lcg_state) / 4294967296.0;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307 * u2);
}

/* -----------------------------------------------------------------------
 * Sensor simulation: range-proportional LIDAR noise model.
 *   sigma(r) = max(SIGMA_FLOOR, SIGMA_K * r)
 *   SIGMA_K = 0.003:  at 200 m => 0.60 m noise; at 1 m => 0.003 m noise
 * ----------------------------------------------------------------------- */
#define SENSOR_SIGMA_K      0.003
#define SENSOR_SIGMA_FLOOR  0.002

static GncStatus sim_sensor(const Vec3 *true_pos, Vec3 *meas,
                             double *sigma_out)
{
    GNC_ASSERT(true_pos  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(meas      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(sigma_out != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double range = 0.0;
    GncStatus rc = dyn_range(true_pos, &range);
    GNC_ASSERT(rc == GNC_OK, rc, return rc);

    double sigma = SENSOR_SIGMA_K * range;
    if (sigma < SENSOR_SIGMA_FLOOR) { sigma = SENSOR_SIGMA_FLOOR; }

    meas->v[0] = true_pos->v[0] + sigma * lcg_normal();
    meas->v[1] = true_pos->v[1] + sigma * lcg_normal();
    meas->v[2] = true_pos->v[2] + sigma * lcg_normal();

    *sigma_out = sigma;
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Write CSV header
 * ----------------------------------------------------------------------- */
static GncStatus write_header(FILE *fp)
{
    GNC_ASSERT(fp != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    int ret = fprintf(fp,
        "step,time_s,"
        "true_x,true_y,true_z,"
        "true_vx,true_vy,true_vz,"
        "est_x,est_y,est_z,"
        "cmd_fx,cmd_fy,cmd_fz,"
        "range_m,dv_step\n");
    GNC_ASSERT(ret > 0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Write one telemetry record
 * ----------------------------------------------------------------------- */
static GncStatus write_record(FILE *fp, const TelemetryRecord *rec)
{
    GNC_ASSERT(fp  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(rec != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    int ret = fprintf(fp,
        "%u,%.3f,"
        "%.4f,%.4f,%.4f,"
        "%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.6f\n",
        rec->step, rec->time_s,
        rec->true_pos.v[0], rec->true_pos.v[1], rec->true_pos.v[2],
        rec->true_vel.v[0], rec->true_vel.v[1], rec->true_vel.v[2],
        rec->est_pos.v[0],  rec->est_pos.v[1],  rec->est_pos.v[2],
        rec->cmd_force.v[0],rec->cmd_force.v[1],rec->cmd_force.v[2],
        rec->range_m, rec->delta_v);
    GNC_ASSERT(ret > 0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Print final docking report to stdout
 * ----------------------------------------------------------------------- */
static GncStatus print_report(
    uint32_t steps, double time_s,
    const Vec3 *pos, const Vec3 *vel,
    const FuelState *fuel, uint8_t docked)
{
    GNC_ASSERT(pos  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(vel  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(fuel != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double pos_err = sqrt(pos->v[0]*pos->v[0]+pos->v[1]*pos->v[1]+pos->v[2]*pos->v[2]);
    double vel_err = sqrt(vel->v[0]*vel->v[0]+vel->v[1]*vel->v[1]+vel->v[2]*vel->v[2]);

    (void)printf("\n===== DOCKING SIMULATION REPORT =====\n");
    (void)printf("Steps run        : %u\n", steps);
    (void)printf("Mission time     : %.1f s\n", time_s);
    (void)printf("Docked           : %s\n", docked ? "YES" : "NO");
    (void)printf("Final pos error  : %.4f m  (tol %.3f m)\n",
                 pos_err, GNC_DOCK_POS_TOL);
    (void)printf("Final vel error  : %.4f m/s (tol %.3f m/s)\n",
                 vel_err, GNC_DOCK_VEL_TOL);
    (void)printf("Total DeltaV     : %.4f m/s\n",  fuel->total_dv_mps);
    (void)printf("Propellant used  : %.4f kg\n",   fuel->prop_kg);
    (void)printf("Thruster fires   : %u\n",         fuel->fire_count);
    (void)printf("======================================\n\n");

    GNC_ASSERT(pos_err < 1.0e4, ERR_BOUNDS, return ERR_BOUNDS);
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * main — simulation entry point
 * ----------------------------------------------------------------------- */
int main(void)
{
    /* --- Orbit setup -------------------------------------------------- */
    double sma   = GNC_EARTH_RADIUS + GNC_ISS_ALTITUDE;   /* semi-major axis */
    double n_rad = 0.0;
    GncStatus rc = dyn_mean_motion(sma, &n_rad);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- Initial true state ------------------------------------------- */
    Vec3 true_pos = {{0.0, 200.0, 0.0}};   /* 200 m behind in along-track */
    Vec3 true_vel = {{0.0,   0.0, 0.0}};   /* initially at rest           */

    /* --- Navigation filter -------------------------------------------- */
    NavState nav;
    rc = nav_init(&nav, &true_pos, &true_vel, NAV_P0_POS, NAV_P0_VEL);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- Guidance plan ------------------------------------------------- */
    GuidancePlan plan;
    rc = guid_init_plan(&plan);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- PD gains ------------------------------------------------------- */
    PdGains gains;
    rc = ctrl_init_gains(&gains);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- Fuel state ----------------------------------------------------- */
    FuelState fuel        = {0.0, 0.0, 0U};
    double    mass        = GNC_CHASER_MASS;
    uint8_t   term_armed  = 0U;
    double    mib_Ns      = GNC_MIN_IMPULSE_BIT;
    uint32_t  dock_dwell  = 0U;   /* steps chaser has been inside tolerance */

    /* --- Telemetry file ------------------------------------------------- */
    FILE *fp = fopen("sim/telem.csv", "w");
    GNC_ASSERT(fp != NULL, ERR_NULL_PTR, return (int)ERR_NULL_PTR);

    rc = write_header(fp);
    GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

    /* --- Main simulation loop (Rule 2: fixed bound GNC_MAX_SIM_STEPS) - */
    uint8_t  docked = 0U;
    uint32_t step   = 0U;

    for (step = 0U; step < GNC_MAX_SIM_STEPS; step++) {

        /* 1. Compute range on TRUE state */
        double range = 0.0;
        rc = dyn_range(&true_pos, &range);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 2. Arm terminal high-gain controller on entry into final ingress phase */
        {
            uint32_t cur_phase = 0U;
            rc = guid_active_phase(&plan, &cur_phase);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            if ((term_armed == 0U) && (cur_phase >= (plan.count - 1U))) {
                rc = ctrl_apply_terminal_gains(&gains);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });
                mib_Ns     = SIM_TERMINAL_MIB_NS;
                term_armed = 1U;
                (void)printf(
                    "[%5u] *** Final-ingress gains armed  range=%.3f m ***\n",
                    step, range);
            }
        }

        /* 3. TRUE-state docking check with mandatory dwell (not nav estimate).
         *    Both position AND velocity must be within tolerance for at least
         *    SIM_DOCK_DWELL_STEPS consecutive steps before declaring docked.
         *    Also call guid_corridor_check and ctrl_vec3_norm to exercise API.
         */
        {
            double spd = 0.0;
            rc = ctrl_vec3_norm(&true_vel, &spd);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            uint8_t in_tol = (uint8_t)((range < GNC_DOCK_POS_TOL) &&
                                        (spd   < GNC_DOCK_VEL_TOL));

            uint8_t in_corr = 0U;
            rc = guid_corridor_check(&plan, &nav, &in_corr);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            if (in_tol != 0U) {
                dock_dwell++;
            } else {
                dock_dwell = 0U;  /* reset on any violation */
            }

            if (dock_dwell >= SIM_DOCK_DWELL_STEPS) {
                (void)printf(
                    "[%5u] DOCKED — dwell=%u steps  corridor=%u  "
                    "range=%.4f m  spd=%.4f m/s\n",
                    step, dock_dwell, in_corr, range, spd);
                docked = 1U;
                break;
            }
        }

        /* 4. Guidance reference */
        Vec3 pos_ref;
        Vec3 vel_ref;
        rc = guid_compute_ref(&plan, &nav, GNC_DT_SEC, &pos_ref, &vel_ref);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 5. Control errors: reference minus estimated state */
        Vec3 pos_err;
        Vec3 vel_err;
        rc = ctrl_vec3_sub(&pos_ref, &nav.pos, &pos_err);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        rc = ctrl_vec3_sub(&vel_ref, &nav.vel, &vel_err);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 6. Control command */
        ControlCmd cmd;
        rc = ctrl_compute(&gains, &pos_err, &vel_err,
                          mass, GNC_DT_SEC, mib_Ns, &cmd, &fuel);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 7. Propagate true dynamics */
        Vec3 new_pos;
        Vec3 new_vel;
        rc = dyn_propagate(&true_pos, &true_vel, &cmd.force_N,
                           n_rad, GNC_DT_SEC, mass,
                           &new_pos, &new_vel);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        true_pos = new_pos;
        true_vel = new_vel;

        /* 8. Range-dependent sensor measurement */
        Vec3   meas;
        double meas_sigma = 0.0;
        rc = sim_sensor(&true_pos, &meas, &meas_sigma);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 9. Nav filter update */
        rc = nav_propagate(&nav, &cmd.force_N, n_rad, GNC_DT_SEC, mass);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        rc = nav_update(&nav, &meas, meas_sigma);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 10. Update propellant mass using per-step delta from ControlCmd */
        mass -= cmd.prop_step_kg;
        if (mass < 10.0) { mass = 10.0; }   /* dry mass floor */

        /* 11. Telemetry */
        TelemetryRecord rec;
        rec.step      = step;
        rec.time_s    = nav.time_s;
        rec.true_pos  = true_pos;
        rec.true_vel  = true_vel;
        rec.est_pos   = nav.pos;
        rec.est_vel   = nav.vel;
        rec.cmd_force = cmd.force_N;
        rec.range_m   = range;
        rec.delta_v   = cmd.delta_v_mps;

        rc = write_record(fp, &rec);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 12. Console heartbeat every 100 steps */
        if ((step % 100U) == 0U) {
            (void)printf("[%5u] t=%.0f s  range=%.3f m  phase=%u  dv=%.3f m/s\n",
                step, nav.time_s, range, plan.active, fuel.total_dv_mps);
        }
    }

    (void)fclose(fp);

    rc = print_report(step, nav.time_s, &true_pos, &true_vel, &fuel, docked);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    if (docked != 1U) {
        (void)printf("WARNING: sim ended without confirmed docking.\n");
        return 1;
    }
    return 0;
}
