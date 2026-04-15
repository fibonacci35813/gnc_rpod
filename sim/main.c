/**
 * @file    sim/main.c
 * @brief   Autonomous docking simulation harness — NASA Power of 10.
 *
 * Runs a closed-loop GNC simulation:
 *   1. Propagate true dynamics (CW + thrust)
 *   2. Simulate LIDAR sensor noise
 *   3. Run Kalman nav filter
 *   4. Compute guidance reference
 *   5. Compute translational control command
 *   6. Compute attitude control (Phase 3: align docking port)
 *   7. Log telemetry to telem.csv
 *   8. Check docking condition (pos + vel + attitude)
 *
 * Compile:
 *   gcc -std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion \
 *       -Wdouble-promotion -I include/ \
 *       src/dynamics.c src/nav_filter.c src/guidance.c src/control.c \
 *       src/attitude.c src/rw_model.c sim/main.c -lm -o sim/dock_sim
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
#include "attitude.h"
#include "rw_model.h"
#include "fdir.h"
#include "imu_model.h"
#include "mission_mgr.h"

/* -----------------------------------------------------------------------
 * Simulation parameters
 * ----------------------------------------------------------------------- */
#define SIM_SEED              42U     /* LCG seed for noise                */
#define NAV_P0_POS             5.0    /* initial position uncertainty (m)  */
#define NAV_P0_VEL             0.5    /* initial velocity uncertainty (m/s)*/
#define SIM_TERMINAL_RANGE_M   5.0    /* range at which terminal gains arm */
#define SIM_TERMINAL_MIB_NS    0.005  /* terminal MIB: 5 milli-N·s        */
#define SIM_DOCK_DWELL_STEPS  30U     /* must be within tol for 30 steps   */
#define SIM_IMU_DT_S           0.1    /* IMU inner-loop step (s) — 10 Hz   */
#define SIM_IMU_SUBSTEPS      10U     /* substeps per 1-Hz outer step      */
#define SIM_SIGMA_RANGE        0.003  /* LIDAR range noise scale (sigma=k*r)*/
#define SIM_SIGMA_ANG_RAD      0.001  /* LIDAR angle noise 1-σ (rad) 1 mrad*/
#define SIM_IMU_ACCEL_SIGMA    0.01   /* IMU accelerometer noise (m/s^2)   */
#define SIM_IMU_GYRO_SIGMA     0.001  /* IMU gyroscope noise (rad/s)       */

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
 * Phase 8 Sensor simulation: RAE LIDAR noise model.
 *   sigma_range = max(SIGMA_FLOOR, SIM_SIGMA_RANGE * range)
 *   sigma_angle = SIM_SIGMA_ANG_RAD (fixed, typical docking LIDAR)
 * ----------------------------------------------------------------------- */
#define SENSOR_SIGMA_FLOOR  0.002

static GncStatus sim_sensor_rae(
    const Vec3 *true_pos,
    double     *range_m,
    double     *az_rad,
    double     *el_rad,
    double     *sigma_r_out,
    double     *sigma_ang_out)
{
    GNC_ASSERT(true_pos     != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(range_m      != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(az_rad       != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(el_rad       != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(sigma_r_out  != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(sigma_ang_out!= NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    double x = true_pos->v[0], y = true_pos->v[1], z = true_pos->v[2];
    double r2 = x*x + y*y + z*z;
    GNC_ASSERT(r2 > 1.0e-9, ERR_BAD_PARAM, return ERR_BAD_PARAM);

    double r      = sqrt(r2);
    double sig_r  = SIM_SIGMA_RANGE * r;
    if (sig_r < SENSOR_SIGMA_FLOOR) { sig_r = SENSOR_SIGMA_FLOOR; }

    double zr = z / r;
    if (zr >  1.0) { zr =  1.0; }
    if (zr < -1.0) { zr = -1.0; }

    *range_m      = r           + sig_r                * lcg_normal();
    *az_rad       = atan2(y, x) + SIM_SIGMA_ANG_RAD    * lcg_normal();
    *el_rad       = asin(zr)    + SIM_SIGMA_ANG_RAD    * lcg_normal();
    *sigma_r_out  = sig_r;
    *sigma_ang_out = SIM_SIGMA_ANG_RAD;
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
        "range_m,dv_step,"
        "att_qw,att_qx,att_qy,att_qz,"
        "att_err_deg,att_aligned,"
        "mission_mode,fault_code,"
        "imu_ax,imu_ay,imu_az\n");
    GNC_ASSERT(ret > 0, ERR_BAD_PARAM, return ERR_BAD_PARAM);
    return GNC_OK;
}

/* -----------------------------------------------------------------------
 * Write one telemetry record
 * ----------------------------------------------------------------------- */
static GncStatus write_record(FILE *fp, const TelemetryRecord *rec,
                               const AttState *att, double att_err_deg,
                               uint8_t att_aligned,
                               const FaultState *fs,
                               MissionMode mgr_mode,
                               const Vec3 *imu_accel)
{
    GNC_ASSERT(fp        != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(rec       != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(att       != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(fs        != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);
    GNC_ASSERT(imu_accel != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

    int ret = fprintf(fp,
        "%u,%.3f,"
        "%.4f,%.4f,%.4f,"
        "%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.4f,%u,"
        "%d,%d,"
        "%.6f,%.6f,%.6f\n",
        rec->step, rec->time_s,
        rec->true_pos.v[0], rec->true_pos.v[1], rec->true_pos.v[2],
        rec->true_vel.v[0], rec->true_vel.v[1], rec->true_vel.v[2],
        rec->est_pos.v[0],  rec->est_pos.v[1],  rec->est_pos.v[2],
        rec->cmd_force.v[0],rec->cmd_force.v[1],rec->cmd_force.v[2],
        rec->range_m, rec->delta_v,
        att->q[0], att->q[1], att->q[2], att->q[3],
        att_err_deg, (unsigned)att_aligned,
        (int)mgr_mode, (int)fs->active_fault,
        imu_accel->v[0], imu_accel->v[1], imu_accel->v[2]);
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

    /* --- Attitude state and reaction wheels ----------------------------- */
    AttState att;
    rc = att_init(&att);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    RwState rw;
    rc = rw_init(&rw);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- Phase 7 environment model (true dynamics only; GNC stays CW) ----- */
    EnvModel env;
    env.use_j2  = 1U;    /* enable differential J2 perturbation    */
    env.use_drag = 1U;   /* enable atmospheric drag (exponential)  */
    env.Cd      = 2.2;   /* drag coefficient (dimensionless)       */
    env.area_m2 = 2.0;   /* cross-sectional area (m^2)             */

    /* --- Phase 8 IMU model ------------------------------------------------ */
    ImuState imu;
    rc = imu_init(&imu, SIM_IMU_ACCEL_SIGMA, SIM_IMU_GYRO_SIGMA);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- FDIR fault state ----------------------------------------------- */
    FaultState fault_state;
    rc = fdir_init(&fault_state);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- Phase 9 mission manager ---------------------------------------- */
    MissionManager mgr;
    rc = mgr_init(&mgr);
    GNC_ASSERT(rc == GNC_OK, rc, return (int)rc);

    /* --- Fuel state ----------------------------------------------------- */
    FuelState fuel        = {0.0, 0.0, 0U};
    double    mass        = GNC_CHASER_MASS;
    uint8_t   term_armed  = 0U;
    double    mib_Ns      = GNC_MIN_IMPULSE_BIT;
    uint32_t  dock_dwell  = 0U;   /* steps chaser has been inside tolerance */
    /* meas_valid is a per-step local in the LIDAR block (step 0) */
    Vec3      last_imu_accel = {{0.0, 0.0, 0.0}};  /* last IMU sample (telem) */

    /* CW-velocity correction: store EKF state + last command force so each
     * outer step can replace the IMU-integrated nav.vel with a clean CW
     * prediction.  IMU noise accumulated over 10 substeps would otherwise
     * be amplified by Kd=42 and prevent terminal docking.
     * Initialised to true IC so step-0 CW prediction is physically correct. */
    Vec3 ekf_pos_prev    = true_pos;
    Vec3 ekf_vel_prev    = true_vel;
    Vec3 cmd_force_prev  = {{0.0, 0.0, 0.0}};

    /* --- Telemetry file ------------------------------------------------- */
    FILE *fp = fopen("sim/telem.csv", "w");
    GNC_ASSERT(fp != NULL, ERR_NULL_PTR, return (int)ERR_NULL_PTR);

    rc = write_header(fp);
    GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

    /* --- Main simulation loop (Rule 2: fixed bound GNC_MAX_SIM_STEPS) - */
    uint8_t  docked = 0U;
    uint32_t step   = 0U;

    for (step = 0U; step < GNC_MAX_SIM_STEPS; step++) {

        /* 0. LIDAR RAE sensor + EKF measurement update — runs BEFORE guidance
         *    so nav (pos + vel) is corrected before reference tracking begins.
         *    Predict-correct cycle: cov_only propagation closes previous step;
         *    EKF correction opens this step. */
        {
            double range_s = 0.0, az_s = 0.0, el_s = 0.0;
            double sig_r = 0.0, sig_a = 0.0;
            uint8_t meas_valid = 0U;
            rc = sim_sensor_rae(&true_pos, &range_s, &az_s, &el_s,
                                &sig_r, &sig_a);
            if (rc == GNC_OK) {
                meas_valid = 1U;
                rc = nav_update_ekf(&nav, range_s, az_s, el_s, sig_r, sig_a);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });
            }
            rc = fdir_check_sensor(meas_valid, &fault_state);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        }

        /* 0b. CW velocity correction: replace IMU-integrated nav.vel with a
         *     dynamics-model prediction (no noise).  The EKF corrects nav.pos
         *     accurately; velocity is predicted from the previous EKF state and
         *     the commanded force — exact CW + thrust, no IMU measurement noise.
         *     This prevents Kd amplifying 0.003 m/s IMU noise into 0.1 N
         *     spurious forces that would otherwise prevent terminal docking. */
        {
            Vec3 cw_p, cw_v;
            rc = dyn_propagate(&ekf_pos_prev, &ekf_vel_prev, &cmd_force_prev,
                               n_rad, GNC_DT_SEC, mass, &cw_p, &cw_v);
            if (rc == GNC_OK) { nav.vel = cw_v; }
            ekf_pos_prev = nav.pos;   /* post-EKF position for next step */
            ekf_vel_prev = nav.vel;   /* CW-corrected velocity for next step */
        }

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

        /* 3. Attitude control:
         *    Phase 0-2: coast (no torque, maintain identity attitude)
         *    Phase 3:   align docking port toward target
         *
         * Integration order: SYMPLECTIC EULER — q updated FIRST with current
         * omega, then omega updated with the computed torque.  This avoids the
         * instability of the reversed order at dt=1 s (|eigenvalue|>1 otherwise).
         */
        double att_err_deg = 0.0;
        uint8_t att_aligned = 1U;   /* trivially aligned in coast phases */
        {
            double eff_torque[3];
            eff_torque[0] = 0.0;
            eff_torque[1] = 0.0;
            eff_torque[2] = 0.0;

            uint32_t cur_phase = 0U;
            rc = guid_active_phase(&plan, &cur_phase);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            if (cur_phase >= (plan.count - 1U)) {
                /* Phase 3: compute pointing command */
                double pos_arr[3];
                pos_arr[0] = true_pos.v[0];
                pos_arr[1] = true_pos.v[1];
                pos_arr[2] = true_pos.v[2];
                double q_cmd[4];
                rc = att_docking_cmd(pos_arr, q_cmd);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Attitude error from current q (BEFORE kinematics update) */
                double q_err[4];
                rc = att_error(q_cmd, att.q, q_err);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Check alignment */
                rc = att_check_aligned(q_err, &att_aligned);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Pointing error in degrees (for telemetry) */
                double qw_c = q_err[0];
                if (qw_c >  1.0) { qw_c =  1.0; }
                if (qw_c < -1.0) { qw_c = -1.0; }
                att_err_deg = (2.0 * acos(fabs(qw_c))) * (180.0 / 3.141592653589793);

                /* PD torque (from current q error and current omega) */
                double q_err_vec[3];
                q_err_vec[0] = q_err[1];
                q_err_vec[1] = q_err[2];
                q_err_vec[2] = q_err[3];
                double omega_err[3];
                omega_err[0] = att.omega[0];
                omega_err[1] = att.omega[1];
                omega_err[2] = att.omega[2];

                AttCmd att_cmd;
                rc = att_pd_control(q_err_vec, omega_err,
                                    ATT_MIB_NMS, GNC_DT_SEC, &att_cmd);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Apply torque to reaction wheels */
                rc = rw_apply_torque(&rw, att_cmd.torque, GNC_DT_SEC);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Desaturation check */
                rc = rw_desat(&rw);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Effective torque on body */
                rc = rw_effective_torque(&rw, att_cmd.torque, eff_torque);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });
            }

            /* SYMPLECTIC EULER STEP 1: propagate q with CURRENT omega */
            rc = att_kinematics(&att, GNC_DT_SEC);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            /* SYMPLECTIC EULER STEP 2: update omega with computed torque */
            att.omega[0] += eff_torque[0] * GNC_DT_SEC;
            att.omega[1] += eff_torque[1] * GNC_DT_SEC;
            att.omega[2] += eff_torque[2] * GNC_DT_SEC;
        }

        /* 4. TRUE-state docking check with mandatory dwell (not nav estimate).
         *    pos + vel within tolerance AND attitude aligned (Phase 3 only).
         */
        {
            double spd = 0.0;
            rc = ctrl_vec3_norm(&true_vel, &spd);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            uint8_t in_tol = (uint8_t)((range < GNC_DOCK_POS_TOL) &&
                                        (spd   < GNC_DOCK_VEL_TOL) &&
                                        (att_aligned != 0U));

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
                    "range=%.4f m  spd=%.4f m/s  att_err=%.3f deg\n",
                    step, dock_dwell, in_corr, range, spd, att_err_deg);
                docked = 1U;
                break;
            }
        }

        /* 5. Phase 9 guidance override from mission manager */
        Vec3    pos_override;
        Vec3    vel_override = {{0.0, 0.0, 0.0}};
        uint8_t override_active = 0U;
        rc = mgr_get_guidance_override(&mgr, &plan, &nav,
                                       &pos_override, &override_active);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 5b. Guidance reference (normal or abort-overridden) */
        Vec3 pos_ref;
        Vec3 vel_ref;
        if (override_active != 0U) {
            pos_ref = pos_override;
            vel_ref = vel_override;
        } else {
            rc = guid_compute_ref(&plan, &nav, GNC_DT_SEC, &pos_ref, &vel_ref);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        }

        /* 6. Control errors: reference minus estimated state */
        Vec3 pos_err_vec;
        Vec3 vel_err_vec;
        rc = ctrl_vec3_sub(&pos_ref, &nav.pos, &pos_err_vec);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        rc = ctrl_vec3_sub(&vel_ref, &nav.vel, &vel_err_vec);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 7. Control command */
        ControlCmd cmd;
        rc = ctrl_compute(&gains, &pos_err_vec, &vel_err_vec,
                          mass, GNC_DT_SEC, mib_Ns, &cmd, &fuel);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        cmd_force_prev = cmd.force_N;   /* stored for next step's CW vel prediction */

        /* 7b. FDIR health checks — nominal run: actual == commanded */
        {
            Vec3 actual_force = cmd.force_N;  /* no fault injected in main sim */
            rc = fdir_check_thruster(&cmd.force_N, &actual_force, &fault_state);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
            /* fdir_check_sensor called in step 0 (LIDAR block) this step */
            rc = fdir_check_nav(&nav, &fault_state);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
            rc = fdir_check_attitude(&att, &fault_state);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
            rc = fdir_update(&fault_state, step);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            /* Phase 9: mission manager escalation driven by FDIR fault state */
            rc = mgr_update(&mgr, &fault_state, &plan, step, GNC_DT_SEC);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

            MissionMode fdir_mode = fdir_get_mode(&fault_state);
            if ((fdir_mode == MODE_ABORT_RETREAT) ||
                (fdir_mode == MODE_ABORT_SAFE)) {
                (void)printf("[%5u] FDIR ABORT: mode=%d fault=%d\n",
                    step, (int)fdir_mode, (int)fault_state.active_fault);
            }
        }

        /* 8. Inner two-rate loop: 10 substeps at SIM_IMU_DT_S = 0.1 s.
         *     True dynamics (J2+drag): dyn_propagate_perturbed at 10 Hz.
         *     Nav mean state: nav_propagate_imu (IMU-driven) at 10 Hz.
         *     Covariance: nav_propagate_cov_only once after inner loop.
         */
        {
            Vec3 true_accel;
            uint32_t sub = 0U;
            for (sub = 0U; sub < SIM_IMU_SUBSTEPS; sub++) {
                /* True control acceleration for IMU input */
                true_accel.v[0] = cmd.force_N.v[0] / mass;
                true_accel.v[1] = cmd.force_N.v[1] / mass;
                true_accel.v[2] = cmd.force_N.v[2] / mass;

                /* IMU measurement: control accel + noise + bias */
                Vec3 imu_omega = {{0.0, 0.0, 0.0}};  /* gyro not used for nav */
                rc = imu_measure(&imu, &true_accel, &imu_omega,
                                 &last_imu_accel, &imu_omega);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* Nav mean state: IMU-driven propagation (includes CW coupling) */
                rc = nav_propagate_imu(&nav, &last_imu_accel,
                                       n_rad, SIM_IMU_DT_S);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });

                /* True dynamics: CW + J2 + drag at 10 Hz */
                Vec3 sub_pos, sub_vel;
                rc = dyn_propagate_perturbed(&true_pos, &true_vel, &cmd.force_N,
                                             &env, n_rad, SIM_IMU_DT_S, mass,
                                             &sub_pos, &sub_vel);
                GNC_ASSERT(rc == GNC_OK, rc,
                           { (void)fclose(fp); return (int)rc; });
                true_pos = sub_pos;
                true_vel = sub_vel;
            }

            /* Covariance propagation: once per outer 1-Hz step using CW STM */
            rc = nav_propagate_cov_only(&nav, n_rad, GNC_DT_SEC);
            GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });
        }

        /* 11. Update propellant mass using per-step delta from ControlCmd */
        mass -= cmd.prop_step_kg;
        if (mass < 10.0) { mass = 10.0; }   /* dry mass floor */

        /* 12. Telemetry */
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

        rc = write_record(fp, &rec, &att, att_err_deg, att_aligned,
                          &fault_state, mgr.current_mode, &last_imu_accel);
        GNC_ASSERT(rc == GNC_OK, rc, { (void)fclose(fp); return (int)rc; });

        /* 13. Console heartbeat every 100 steps */
        if ((step % 100U) == 0U) {
            (void)printf("[%5u] t=%.0f s  range=%.3f m  phase=%u  dv=%.3f m/s"
                         "  att_err=%.2f deg\n",
                step, nav.time_s, range, plan.active, fuel.total_dv_mps,
                att_err_deg);
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
