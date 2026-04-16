/**
 * @file    sim/scenarios.h
 * @brief   Standardised 8-scenario test library — Phase 2.
 *
 * Each scenario specifies initial conditions, environment model, fault
 * injection mode, and the pass/fail gate for dock rate (or abort rate
 * for stuck_open).
 */

#ifndef SCENARIOS_H
#define SCENARIOS_H

#include "../include/gnc_types.h"

#define MAX_SCENARIOS    8U
#define SCENARIO_NAME_LEN 64U

/* -----------------------------------------------------------------------
 * Fault injection descriptor
 * ----------------------------------------------------------------------- */
typedef enum {
    FAULT_INJ_NONE         = 0,  /* no fault injected                       */
    FAULT_INJ_STUCK_OPEN   = 1,  /* thruster valve stuck open               */
    FAULT_INJ_STUCK_CLOSED = 2,  /* thruster stuck closed (dead thruster)   */
    FAULT_INJ_DROPOUT      = 3   /* sensor dropout for N consecutive steps  */
} FaultInjType;

typedef struct {
    FaultInjType type;           /* fault mode                              */
    uint32_t     inject_step;    /* simulation step at which fault begins   */
} FaultInjection;

/* -----------------------------------------------------------------------
 * Scenario definition
 * ----------------------------------------------------------------------- */
typedef struct {
    char           name[SCENARIO_NAME_LEN];
    Vec3           pos0;             /* initial relative position (m)       */
    Vec3           vel0;             /* initial relative velocity (m/s)     */
    double         prop_init_kg;     /* propellant mass at start (kg)       */
    EnvModel       env;              /* true-dynamics environment model     */
    FaultInjection fault;            /* fault injection spec                */
    uint32_t       seed_start;       /* first LCG seed for MC runs          */
    uint32_t       n_runs;           /* number of Monte Carlo seeds to run  */
    double         dock_rate_gate;   /* minimum required dock rate (0–1)    */
} Scenario;

extern const Scenario SCENARIO_TABLE[MAX_SCENARIOS];
extern const uint32_t SCENARIO_COUNT;

#endif /* SCENARIOS_H */
