/**
 * @file    sim/params.h
 * @brief   Runtime-configurable GNC parameter set — Phase 1.
 *
 * GncParams bundles all tunable gains and thresholds so the Bayesian
 * optimiser can evaluate different combinations without recompiling.
 * When any init function receives p == NULL, it falls back to the
 * same compile-time defaults that were hardcoded before Phase 1.
 */

#ifndef PARAMS_H
#define PARAMS_H

#include "../include/gnc_types.h"

/* -----------------------------------------------------------------------
 * Tunable GNC parameter set
 *
 * The typedef is separated from the struct so that headers which only need
 * a forward declaration can emit "typedef struct GncParams GncParams;"
 * protected by GNC_PARAMS_FWDECL, while this header provides the full
 * struct body.  Only the first declaration of the typedef is emitted.
 * ----------------------------------------------------------------------- */
#ifndef GNC_PARAMS_FWDECL
#define GNC_PARAMS_FWDECL
typedef struct GncParams GncParams;
#endif

struct GncParams {
    double   kp[3];           /* proportional gains N/m   [x, y, z]      */
    double   kd[3];           /* derivative gains N.s/m   [x, y, z]      */
    double   kp_terminal[3];  /* terminal Kp N/m          [x, y, z]      */
    double   kd_terminal[3];  /* terminal Kd N.s/m        [x, y, z]      */
    double   K_V;             /* range-proportional velocity gain         */
    double   v_phase_max[4];  /* max approach speed m/s  [ph0..ph3]      */
    double   fdir_hold_timeout_s; /* FDIR HOLD timeout (seconds)          */
    uint32_t fdir_dropout_limit;  /* consecutive dropout steps → fault    */
    double   mib_normal_ns;   /* normal MIB threshold (N·s)              */
    double   mib_terminal_ns; /* terminal MIB threshold (N·s)            */
};

/**
 * @brief  Fill *p with the hardcoded baseline defaults.
 *         Identical to what ctrl_init_gains / guid_init_plan used before.
 */
void params_set_defaults(GncParams *p);

/**
 * @brief  Read key-value pairs from a JSON file into *p.
 *         Unknown keys are silently skipped.
 *         Missing keys keep their default value (params_set_defaults called first).
 * @return 0 on success, -1 if file cannot be opened.
 */
int  params_read_json(GncParams *p, const char *path);

/**
 * @brief  Write *p to a JSON file at path.
 * @return 0 on success, -1 on I/O error.
 */
int  params_write_json(const GncParams *p, const char *path);

#endif /* PARAMS_H */
