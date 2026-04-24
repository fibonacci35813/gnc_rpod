/**
 * @file    sim/params.c
 * @brief   Runtime GNC parameter I/O — Phase 1.
 *
 * JSON parser: reads key-value pairs line by line into a 256-char stack
 * buffer.  No malloc.  Unknown keys are silently ignored.
 * Format: one "key": value pair per line (trailing comma optional).
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "params.h"

/* -----------------------------------------------------------------------
 * Internal constants (Rule 2)
 * ----------------------------------------------------------------------- */
#define PARAMS_LINE_MAX   256U  /* max characters per JSON line            */
#define PARAMS_KEY_MAX     64U  /* max key string length (inc. NUL)        */
#define PARAMS_MAX_LINES  256U  /* hard upper bound on lines read          */

/* -----------------------------------------------------------------------
 * Default values — exactly matching the previous hardcoded constants.
 * ----------------------------------------------------------------------- */
void params_set_defaults(GncParams *p)
{
    if (p == NULL) { return; }

    /* Normal approach Kp / Kd (matches CTRL_KP_* / CTRL_KD_* in control.c) */
    p->kp[0] = 0.30;  p->kp[1] = 0.20;  p->kp[2] = 0.30;
    p->kd[0] = 30.0;  p->kd[1] = 25.0;  p->kd[2] = 30.0;

    /* Terminal gains (matches ctrl_apply_terminal_gains in control.c) */
    p->kp_terminal[0] = 0.50;  p->kp_terminal[1] = 0.50;  p->kp_terminal[2] = 0.50;
    p->kd_terminal[0] = 42.0;  p->kd_terminal[1] = 42.0;  p->kd_terminal[2] = 42.0;

    /* Guidance velocity profile (matches GUID_K_V / GUID_V_*_MAX) */
    p->K_V           = 0.010;
    p->v_phase_max[0] = 1.00;
    p->v_phase_max[1] = 0.50;
    p->v_phase_max[2] = 0.08;
    p->v_phase_max[3] = 0.04;

    /* FDIR thresholds (matches FDIR_HOLD_TIMEOUT*dt / FDIR_DROPOUT_LIMIT) */
    p->fdir_hold_timeout_s = 60.0;
    p->fdir_dropout_limit  = 3U;

    /* MIB thresholds (matches GNC_MIN_IMPULSE_BIT / SIM_TERMINAL_MIB_NS) */
    p->mib_normal_ns   = 0.100;
    p->mib_terminal_ns = 0.005;
}

/* -----------------------------------------------------------------------
 * Internal: assign a parsed (key, val) pair to the matching field in *p.
 * All values arrive as double; uint32_t fields are cast.
 * ----------------------------------------------------------------------- */
static void apply_param(GncParams *p, const char *key, double val)
{
    if ((p == NULL) || (key == NULL)) { return; }

    if (strncmp(key, "kp_x",            PARAMS_KEY_MAX) == 0) { p->kp[0] = val; return; }
    if (strncmp(key, "kp_y",            PARAMS_KEY_MAX) == 0) { p->kp[1] = val; return; }
    if (strncmp(key, "kp_z",            PARAMS_KEY_MAX) == 0) { p->kp[2] = val; return; }
    if (strncmp(key, "kd_x",            PARAMS_KEY_MAX) == 0) { p->kd[0] = val; return; }
    if (strncmp(key, "kd_y",            PARAMS_KEY_MAX) == 0) { p->kd[1] = val; return; }
    if (strncmp(key, "kd_z",            PARAMS_KEY_MAX) == 0) { p->kd[2] = val; return; }
    if (strncmp(key, "kp_terminal_x",   PARAMS_KEY_MAX) == 0) { p->kp_terminal[0] = val; return; }
    if (strncmp(key, "kp_terminal_y",   PARAMS_KEY_MAX) == 0) { p->kp_terminal[1] = val; return; }
    if (strncmp(key, "kp_terminal_z",   PARAMS_KEY_MAX) == 0) { p->kp_terminal[2] = val; return; }
    if (strncmp(key, "kd_terminal_x",   PARAMS_KEY_MAX) == 0) { p->kd_terminal[0] = val; return; }
    if (strncmp(key, "kd_terminal_y",   PARAMS_KEY_MAX) == 0) { p->kd_terminal[1] = val; return; }
    if (strncmp(key, "kd_terminal_z",   PARAMS_KEY_MAX) == 0) { p->kd_terminal[2] = val; return; }
    if (strncmp(key, "K_V",             PARAMS_KEY_MAX) == 0) { p->K_V = val; return; }
    if (strncmp(key, "v_phase_0_max",   PARAMS_KEY_MAX) == 0) { p->v_phase_max[0] = val; return; }
    if (strncmp(key, "v_phase_1_max",   PARAMS_KEY_MAX) == 0) { p->v_phase_max[1] = val; return; }
    if (strncmp(key, "v_phase_2_max",   PARAMS_KEY_MAX) == 0) { p->v_phase_max[2] = val; return; }
    if (strncmp(key, "v_phase_3_max",   PARAMS_KEY_MAX) == 0) { p->v_phase_max[3] = val; return; }
    if (strncmp(key, "fdir_hold_timeout_s", PARAMS_KEY_MAX) == 0) {
        p->fdir_hold_timeout_s = val; return;
    }
    if (strncmp(key, "fdir_dropout_limit", PARAMS_KEY_MAX) == 0) {
        p->fdir_dropout_limit = (uint32_t)val; return;
    }
    if (strncmp(key, "mib_normal_ns",   PARAMS_KEY_MAX) == 0) { p->mib_normal_ns = val; return; }
    if (strncmp(key, "mib_terminal_ns", PARAMS_KEY_MAX) == 0) { p->mib_terminal_ns = val; return; }
}

/* -----------------------------------------------------------------------
 * params_read_json — line-by-line sscanf parser, no malloc.
 *   Each line is parsed with: " "%63[^"]" : %lf"
 *   which matches: optional-space, '"', key-chars, '"', ':', double.
 *   Trailing commas in values are naturally handled by %lf stopping at ','.
 * ----------------------------------------------------------------------- */
int params_read_json(GncParams *p, const char *path)
{
    if ((p == NULL) || (path == NULL)) { return -1; }

    params_set_defaults(p);   /* fill with defaults before overriding */

    FILE *fp = fopen(path, "r");
    if (fp == NULL) { return -1; }

    char     buf[PARAMS_LINE_MAX];
    char     key[PARAMS_KEY_MAX];
    double   val;          /* written by sscanf before use; no init needed */
    uint32_t n    = 0U;

    while ((n < PARAMS_MAX_LINES) &&
           (fgets(buf, (int)PARAMS_LINE_MAX, fp) != NULL)) {
        key[0] = '\0';
        val    = 0.0;
        /* sscanf format: skip whitespace, match '"key": value' */
        if (sscanf(buf, " \"%63[^\"]\" : %lf", key, &val) == 2) {
            apply_param(p, key, val);
        }
        n++;
    }

    (void)fclose(fp);
    return 0;
}

/* -----------------------------------------------------------------------
 * params_write_json helpers — split to keep each function ≤ 60 lines.
 * ----------------------------------------------------------------------- */
static int write_gains(FILE *fp, const GncParams *p)
{
    int r = 0;
    r |= (fprintf(fp,
        "  \"kp_x\": %.6f, \"kp_y\": %.6f, \"kp_z\": %.6f,\n",
        p->kp[0], p->kp[1], p->kp[2]) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"kd_x\": %.6f, \"kd_y\": %.6f, \"kd_z\": %.6f,\n",
        p->kd[0], p->kd[1], p->kd[2]) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"kp_terminal_x\": %.6f, \"kp_terminal_y\": %.6f,"
        " \"kp_terminal_z\": %.6f,\n",
        p->kp_terminal[0], p->kp_terminal[1], p->kp_terminal[2]) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"kd_terminal_x\": %.6f, \"kd_terminal_y\": %.6f,"
        " \"kd_terminal_z\": %.6f,\n",
        p->kd_terminal[0], p->kd_terminal[1], p->kd_terminal[2]) < 0) ? 1 : 0;
    return r;
}

static int write_guidance_fdir(FILE *fp, const GncParams *p)
{
    int r = 0;
    r |= (fprintf(fp, "  \"K_V\": %.6f,\n", p->K_V) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"v_phase_0_max\": %.6f, \"v_phase_1_max\": %.6f,\n",
        p->v_phase_max[0], p->v_phase_max[1]) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"v_phase_2_max\": %.6f, \"v_phase_3_max\": %.6f,\n",
        p->v_phase_max[2], p->v_phase_max[3]) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"fdir_hold_timeout_s\": %.6f, \"fdir_dropout_limit\": %u,\n",
        p->fdir_hold_timeout_s, p->fdir_dropout_limit) < 0) ? 1 : 0;
    r |= (fprintf(fp,
        "  \"mib_normal_ns\": %.6f, \"mib_terminal_ns\": %.6f\n",
        p->mib_normal_ns, p->mib_terminal_ns) < 0) ? 1 : 0;
    return r;
}

/* cppcheck-suppress unusedFunction -- public API called by optimiser scripts */
int params_write_json(const GncParams *p, const char *path)
{
    if ((p == NULL) || (path == NULL)) { return -1; }

    FILE *fp = fopen(path, "w");
    if (fp == NULL) { return -1; }

    int r = 0;
    r |= (fprintf(fp, "{\n") < 0) ? 1 : 0;
    r |= write_gains(fp, p);
    r |= write_guidance_fdir(fp, p);
    r |= (fprintf(fp, "}\n") < 0) ? 1 : 0;

    (void)fclose(fp);
    return (r != 0) ? -1 : 0;
}
