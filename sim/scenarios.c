/**
 * @file    sim/scenarios.c
 * @brief   8-scenario standardised test table — Phase 2.
 *
 * Each entry matches the specification in CLAUDE.md §PHASE 2.
 * Seed start values are spread to avoid LCG correlations.
 */

#include "scenarios.h"

/* -----------------------------------------------------------------------
 * Helper macros for EnvModel literals (J2 + nominal drag, or high-drag).
 * ----------------------------------------------------------------------- */
#define ENV_J2_DRAG_NOM  { 1U, 1U, 2.2, 2.0 }
#define ENV_J2_DRAG_HI   { 1U, 1U, 3.5, 5.0 }
#define ENV_J2_DRAG_MED  { 1U, 1U, 3.0, 4.0 }

/* -----------------------------------------------------------------------
 * No-fault and fault descriptors
 * ----------------------------------------------------------------------- */
#define FAULT_NONE_DESC         { FAULT_INJ_NONE,         0U    }
#define FAULT_DROPOUT_500       { FAULT_INJ_DROPOUT,      500U  }
#define FAULT_STUCK_CLOSED_300  { FAULT_INJ_STUCK_CLOSED, 300U  }
#define FAULT_STUCK_OPEN_500    { FAULT_INJ_STUCK_OPEN,   500U  }

/* -----------------------------------------------------------------------
 * Scenario table — 8 entries, indices 0–7.
 *
 * n_runs = 20: fast enough for the Phase-2 gate; scenario_runner
 * accepts --n= to override this at runtime.
 *
 * dock_rate_gate = 0.0 for stuck_open: the gate there is abort_rate = 1.00.
 * ----------------------------------------------------------------------- */
const Scenario SCENARIO_TABLE[MAX_SCENARIOS] = {

    /* 0 — nominal -------------------------------------------------------- */
    {
        "nominal",
        { {  0.0, 200.0,  0.0 } },  /* pos0 */
        { {  0.0,   0.0,  0.0 } },  /* vel0 */
        5.0,                         /* prop_init_kg */
        ENV_J2_DRAG_NOM,
        FAULT_NONE_DESC,
        1000U,                       /* seed_start */
        20U,                         /* n_runs */
        0.95                         /* dock_rate_gate */
    },

    /* 1 — off_axis ------------------------------------------------------- */
    {
        "off_axis",
        { { 15.0, 200.0, 10.0 } },
        { {  0.0,   0.0,  0.0 } },
        5.0,
        ENV_J2_DRAG_NOM,
        FAULT_NONE_DESC,
        2000U,
        20U,
        0.90
    },

    /* 2 — high_vel ------------------------------------------------------- */
    {
        "high_vel",
        { {  0.0, 200.0,  0.0 } },
        { {  0.0,  -0.5,  0.0 } },
        5.0,
        ENV_J2_DRAG_NOM,
        FAULT_NONE_DESC,
        3000U,
        20U,
        0.90
    },

    /* 3 — sensor_dropout (DROPOUT at step 500) --------------------------- */
    {
        "sensor_dropout",
        { {  0.0, 200.0,  0.0 } },
        { {  0.0,   0.0,  0.0 } },
        5.0,
        ENV_J2_DRAG_NOM,
        FAULT_DROPOUT_500,
        4000U,
        20U,
        0.90
    },

    /* 4 — stuck_closed (STUCK_CLOSED at step 300) ------------------------ */
    {
        "stuck_closed",
        { {  0.0, 200.0,  0.0 } },
        { {  0.0,   0.0,  0.0 } },
        5.0,
        ENV_J2_DRAG_NOM,
        FAULT_STUCK_CLOSED_300,
        5000U,
        20U,
        0.80
    },

    /* 5 — stuck_open (STUCK_OPEN at step 500; gate = abort_rate = 1.00) - */
    {
        "stuck_open",
        { {  0.0, 200.0,  0.0 } },
        { {  0.0,   0.0,  0.0 } },
        5.0,
        ENV_J2_DRAG_NOM,
        FAULT_STUCK_OPEN_500,
        6000U,
        20U,
        0.0   /* dock_rate_gate ignored; abort_rate must equal 1.00 */
    },

    /* 6 — high_drag ------------------------------------------------------ */
    {
        "high_drag",
        { {  0.0, 200.0,  0.0 } },
        { {  0.0,   0.0,  0.0 } },
        5.0,
        ENV_J2_DRAG_HI,
        FAULT_NONE_DESC,
        7000U,
        20U,
        0.85
    },

    /* 7 — combined_stress ------------------------------------------------ */
    {
        "combined_stress",
        { { 10.0, 200.0,  8.0 } },
        { {  0.1,  -0.2,  0.05 } },
        4.5,
        ENV_J2_DRAG_MED,
        FAULT_NONE_DESC,
        8000U,
        20U,
        0.80
    }
};

const uint32_t SCENARIO_COUNT = MAX_SCENARIOS;
