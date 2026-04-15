/**
 * @file    bsk/gnc_bridge.c
 * @brief   Shared-library bridge implementation.
 *
 * Static pool of GNC contexts (no malloc after init — Rule 3).
 * Wraps our P10 GNC core for consumption by the Basilisk Python scenario.
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "gnc_bridge.h"

/* Include our P10 GNC headers */
#include "../include/gnc_types.h"
#include "../include/gnc_assert.h"
#include "../include/dynamics.h"
#include "../include/nav_filter.h"
#include "../include/guidance.h"
#include "../include/control.h"

/* -----------------------------------------------------------------------
 * Static context pool — supports up to 4 concurrent GNC instances
 * (e.g., Monte Carlo running N chaser vehicles simultaneously)
 * ----------------------------------------------------------------------- */
#define BRIDGE_MAX_CTX  4U
#define BRIDGE_DWELL    30U     /* docking dwell steps required            */

struct GncContext {
    NavState      nav;
    GuidancePlan  plan;
    PdGains       gains;
    FuelState     fuel;
    double        mass_kg;
    double        n_rad;        /* orbit mean motion (rad/s)              */
    double        mib_Ns;       /* active MIB threshold (N·s)             */
    uint32_t      dwell;        /* consecutive steps inside docking tol   */
    uint8_t       term_armed;   /* high-gain terminal mode active         */
    uint8_t       in_use;       /* slot occupied flag                     */
    Vec3          last_force;   /* commanded force from previous step (N) */
};

static struct GncContext s_pool[BRIDGE_MAX_CTX];  /* zero-init at startup */

/* -----------------------------------------------------------------------
 * Internal: copy double[3] to Vec3
 * ----------------------------------------------------------------------- */
static Vec3 arr_to_vec3(const double a[3])
{
    Vec3 v;
    v.v[0] = a[0];
    v.v[1] = a[1];
    v.v[2] = a[2];
    return v;
}

/* Internal: copy Vec3 to double[3] */
static void vec3_to_arr(const Vec3 *v, double a[3])
{
    a[0] = v->v[0];
    a[1] = v->v[1];
    a[2] = v->v[2];
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

GncContext *gnc_bridge_init(
    const double pos0_lvlh[3],
    const double vel0_lvlh[3],
    double mass_kg,
    double sma_m)
{
    if ((pos0_lvlh == NULL) || (vel0_lvlh == NULL)) { return NULL; }
    if ((mass_kg <= 0.0) || (sma_m <= 1.0e5))       { return NULL; }

    /* Find a free slot in the static pool */
    struct GncContext *ctx = NULL;
    uint32_t i = 0U;
    for (i = 0U; i < BRIDGE_MAX_CTX; i++) {
        if (s_pool[i].in_use == 0U) {
            ctx = &s_pool[i];
            break;
        }
    }
    if (ctx == NULL) {
        (void)fprintf(stderr, "[gnc_bridge] Pool exhausted\n");
        return NULL;
    }

    /* Zero the slot */
    (void)memset(ctx, 0, sizeof(struct GncContext));
    ctx->in_use  = 1U;
    ctx->mass_kg = mass_kg;
    ctx->mib_Ns  = GNC_MIN_IMPULSE_BIT;

    /* Mean motion */
    GncStatus rc = dyn_mean_motion(sma_m, &ctx->n_rad);
    if (rc != GNC_OK) { ctx->in_use = 0U; return NULL; }

    /* Nav filter */
    Vec3 p0 = arr_to_vec3(pos0_lvlh);
    Vec3 v0 = arr_to_vec3(vel0_lvlh);
    rc = nav_init(&ctx->nav, &p0, &v0, 5.0, 0.5);
    if (rc != GNC_OK) { ctx->in_use = 0U; return NULL; }

    /* Guidance plan */
    rc = guid_init_plan(&ctx->plan);
    if (rc != GNC_OK) { ctx->in_use = 0U; return NULL; }

    /* PD gains */
    rc = ctrl_init_gains(&ctx->gains);
    if (rc != GNC_OK) { ctx->in_use = 0U; return NULL; }

    return ctx;
}

int gnc_bridge_step(
    GncContext  *ctx,
    const double meas_lvlh[3],
    double       meas_sigma,
    double       dt_s,
    double       force_lvlh[3],
    double      *range_m,
    int         *docked)
{
    if ((ctx == NULL) || (meas_lvlh == NULL)) { return -1; }
    if ((force_lvlh == NULL) || (range_m == NULL) || (docked == NULL)) { return -1; }
    if (dt_s <= 0.0) { return -1; }

    *docked = 0;

    /* Nav filter: propagate using last commanded force (zero on first step) */
    GncStatus rc = nav_propagate(&ctx->nav, &ctx->last_force,
                                 ctx->n_rad, dt_s, ctx->mass_kg);
    if (rc != GNC_OK) { return (int)rc; }

    /* Measurement update */
    Vec3 meas = arr_to_vec3(meas_lvlh);
    double eff_sigma = (meas_sigma > 1.0e-6) ? meas_sigma : 1.0e-3;
    rc = nav_update(&ctx->nav, &meas, eff_sigma);
    if (rc != GNC_OK) { return (int)rc; }

    /* Range from nav estimate */
    double est_range = 0.0;
    rc = dyn_range(&ctx->nav.pos, &est_range);
    if (rc != GNC_OK) { return (int)rc; }
    *range_m = est_range;

    /* Arm terminal gains when entering final phase */
    {
        uint32_t phase = 0U;
        rc = guid_active_phase(&ctx->plan, &phase);
        if (rc != GNC_OK) { return (int)rc; }

        if ((ctx->term_armed == 0U) && (phase >= (ctx->plan.count - 1U))) {
            rc = ctrl_apply_terminal_gains(&ctx->gains);
            if (rc != GNC_OK) { return (int)rc; }
            ctx->mib_Ns    = 0.005;
            ctx->term_armed = 1U;
        }
    }

    /* Guidance reference */
    Vec3 pos_ref;
    Vec3 vel_ref;
    rc = guid_compute_ref(&ctx->plan, &ctx->nav,
                          dt_s, &pos_ref, &vel_ref);
    if (rc != GNC_OK) { return (int)rc; }

    /* Control errors */
    Vec3 pos_err;
    Vec3 vel_err;
    rc = ctrl_vec3_sub(&pos_ref, &ctx->nav.pos, &pos_err);
    if (rc != GNC_OK) { return (int)rc; }
    rc = ctrl_vec3_sub(&vel_ref, &ctx->nav.vel, &vel_err);
    if (rc != GNC_OK) { return (int)rc; }

    /* Control command */
    ControlCmd cmd;
    rc = ctrl_compute(&ctx->gains, &pos_err, &vel_err,
                      ctx->mass_kg, dt_s, ctx->mib_Ns, &cmd, &ctx->fuel);
    if (rc != GNC_OK) { return (int)rc; }

    /* Mass burn-down */
    ctx->mass_kg -= cmd.prop_step_kg;
    if (ctx->mass_kg < 10.0) { ctx->mass_kg = 10.0; }

    /* Store force for next-step nav propagation */
    ctx->last_force = cmd.force_N;

    /* Output force */
    vec3_to_arr(&cmd.force_N, force_lvlh);

    /* Docking dwell check on nav estimate */
    double spd = 0.0;
    rc = ctrl_vec3_norm(&ctx->nav.vel, &spd);
    if (rc != GNC_OK) { return (int)rc; }

    uint8_t in_tol = (uint8_t)((est_range < GNC_DOCK_POS_TOL) &&
                                (spd       < GNC_DOCK_VEL_TOL));
    if (in_tol != 0U) {
        ctx->dwell++;
    } else {
        ctx->dwell = 0U;
    }

    if (ctx->dwell >= BRIDGE_DWELL) {
        *docked = 1;
    }

    return 0;
}

int gnc_bridge_get_state(
    const GncContext *ctx,
    double pos_out[3],
    double vel_out[3],
    double *dv_total,
    double *prop_kg)
{
    if (ctx == NULL)      { return -1; }
    if (pos_out == NULL)  { return -1; }
    if (vel_out == NULL)  { return -1; }
    if (dv_total == NULL) { return -1; }
    if (prop_kg == NULL)  { return -1; }

    vec3_to_arr(&ctx->nav.pos, pos_out);
    vec3_to_arr(&ctx->nav.vel, vel_out);
    *dv_total = ctx->fuel.total_dv_mps;
    *prop_kg  = ctx->fuel.prop_kg;
    return 0;
}

int gnc_bridge_get_phase(const GncContext *ctx)
{
    if (ctx == NULL) { return -1; }
    return (int)ctx->plan.active;
}

void gnc_bridge_free(GncContext *ctx)
{
    if (ctx == NULL) { return; }
    ctx->in_use = 0U;
}
