/**
 * @file    include/fdir.h
 * @brief   Fault Detection, Isolation and Recovery (FDIR) — NASA P10 compliant.
 *
 * Monitors five fault modes:
 *   FAULT_THR_STUCK_CLOSED — commanded force not produced (dead thruster)
 *   FAULT_THR_STUCK_OPEN   — uncmdnd force produced (open valve)
 *   FAULT_SENSOR_DROPOUT   — LIDAR returns no measurement for > LIMIT steps
 *   FAULT_NAV_DIVERGE      — Kalman covariance trace or range out of bounds
 *   FAULT_ATT_UNSTABLE     — angular rate exceeds limit
 *
 * Mission mode transitions:
 *   FAULT_NONE             → MODE_NOMINAL
 *   FAULT_THR_STUCK_OPEN   → MODE_ABORT_SAFE    (latched, immediately abort)
 *   FAULT_NAV_DIVERGE      → MODE_ABORT_RETREAT (not latched, retreat 1 waypoint)
 *   others                 → MODE_HOLD          (not latched, resume after timeout)
 */

#ifndef FDIR_H
#define FDIR_H

#include <stdint.h>
#include "gnc_types.h"
#include "nav_filter.h"
#include "attitude.h"

/* -----------------------------------------------------------------------
 * Fault and mission-mode enumerations (per spec)
 * ----------------------------------------------------------------------- */
typedef enum {
    FAULT_NONE             = 0,
    FAULT_THR_STUCK_CLOSED = 1,  /* thruster fires no impulse when commanded */
    FAULT_THR_STUCK_OPEN   = 2,  /* thruster fires continuously uncmd'd      */
    FAULT_SENSOR_DROPOUT   = 3,  /* LIDAR returns no measurement             */
    FAULT_NAV_DIVERGE      = 4,  /* covariance trace or range out of bounds  */
    FAULT_ATT_UNSTABLE     = 5   /* |omega| exceeds limit                    */
} FaultCode;

typedef enum {
    MODE_NOMINAL       = 0,
    MODE_HOLD          = 1,  /* stop advancing waypoints, hold position  */
    MODE_ABORT_RETREAT = 2,  /* fly back to previous waypoint            */
    MODE_ABORT_SAFE    = 3,  /* manoeuvre to 500 m hold point            */
    MODE_DOCKED        = 4,
    MODE_FAILED        = 5
} MissionMode;

/* -----------------------------------------------------------------------
 * Fault state (per spec + recovery helpers)
 * ----------------------------------------------------------------------- */
typedef struct {
    FaultCode   active_fault;
    MissionMode mode;
    uint32_t    fault_step;       /* sim step when fault was detected          */
    uint32_t    dropout_count;    /* consecutive steps without measurement     */
    uint32_t    hold_steps;       /* steps spent in current hold mode          */
    uint32_t    valid_count;      /* consecutive valid meas (dropout recovery) */
    uint8_t     fault_latched;    /* 1 = fault cannot self-clear               */
    uint8_t     recovery_done;    /* 1 = hold period done, continue degraded   */
} FaultState;

/* -----------------------------------------------------------------------
 * Detection thresholds (per spec)
 * ----------------------------------------------------------------------- */
#define FDIR_THR_FORCE_DEAD_ZONE   0.05     /* N: force below → stuck-closed  */
#define FDIR_THR_OPEN_THRESHOLD    0.1      /* N: uncmd'd above → stuck-open  */
#define FDIR_DROPOUT_LIMIT         3U       /* steps: consecutive no-meas     */
#define FDIR_DROPOUT_CLEAR         3U       /* steps: consecutive valid → OK  */
#define FDIR_COV_TRACE_MAX         1.0e4    /* m^2: covariance trace limit    */
#define FDIR_RANGE_MAX_M           500.0    /* m: impossible range threshold  */
#define FDIR_OMEGA_MAX_RPS         0.1      /* rad/s: attitude rate limit     */
#define FDIR_ATT_ERR_MAX_DEG       10.0     /* deg: attitude error limit      */
#define FDIR_HOLD_TIMEOUT          50U      /* steps in HOLD before resuming  */

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise fault state to NOMINAL, all counters zero.
 */
GncStatus fdir_init(FaultState *fs);

/**
 * @brief  Check thruster health: compare commanded vs actual force per axis.
 *
 *   stuck_closed: |cmd| > DEAD_ZONE and |actual| < DEAD_ZONE
 *   stuck_open  : |actual| > OPEN_THRESHOLD and |cmd| < DEAD_ZONE
 *   stuck_open takes priority (more dangerous).
 */
GncStatus fdir_check_thruster(
    const Vec3 *force_cmd,
    const Vec3 *force_actual,
    FaultState *fs);

/**
 * @brief  Check sensor health: flag dropout after FDIR_DROPOUT_LIMIT steps.
 *
 *   Dropout clears after FDIR_DROPOUT_CLEAR consecutive valid measurements
 *   (fault_latched = 0 for dropout).
 */
GncStatus fdir_check_sensor(
    uint8_t    meas_valid,
    FaultState *fs);

/**
 * @brief  Check navigation health: covariance trace and estimated range.
 */
GncStatus fdir_check_nav(
    const NavState *nav,
    FaultState     *fs);

/**
 * @brief  Check attitude health: angular velocity magnitude.
 */
GncStatus fdir_check_attitude(
    const AttState *att,
    FaultState     *fs);

/**
 * @brief  Determine current mission mode from fault state.
 *
 *   Returns the override mode implied by the active fault.
 *   Does NOT modify the fault state.
 */
MissionMode fdir_get_mode(const FaultState *fs);

/**
 * @brief  Advance hold-mode timer; resume NOMINAL when timeout expires.
 *
 *   Must be called once per simulation step.
 *   For non-latched faults in MODE_HOLD: after FDIR_HOLD_TIMEOUT steps
 *   sets recovery_done = 1, clears active_fault, returns to MODE_NOMINAL.
 *
 * @param  fs    Fault state (updated in place).
 * @param  step  Current simulation step (stored in fault_step on first fault).
 */
GncStatus fdir_update(FaultState *fs, uint32_t step);

#endif /* FDIR_H */
