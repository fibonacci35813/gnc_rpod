/**
 * @file    bsk/gnc_bridge.h
 * @brief   Shared-library API bridging our P10-compliant GNC core to Basilisk.
 *
 * This header is the ONLY interface Basilisk Python glue code touches.
 * All types use primitive arrays (double[], int) so ctypes can bind
 * directly without any struct layout gymnastics.
 *
 * Coordinate convention (LVLH, target-centred):
 *   pos[0] = radial (x)    pos[1] = along-track (y)    pos[2] = cross-track (z)
 *
 * All quantities SI: metres, m/s, newtons, seconds, kilograms.
 *
 * NASA Power of 10 compliance: this file is a thin adapter.
 * The actual algorithms live in src/ and remain fully P10-compliant.
 * This bridge layer is simulation infrastructure, not flight code.
 */

#ifndef GNC_BRIDGE_H
#define GNC_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Opaque GNC context — allocated once at scenario init, never after.
 * Caller holds a pointer; bridge owns the memory (static pool, no malloc).
 * ----------------------------------------------------------------------- */
typedef struct GncContext GncContext;

/**
 * @brief  Allocate and initialise a GNC context from a static pool.
 *
 * @param  pos0_lvlh   Initial relative position [3] (m, LVLH)
 * @param  vel0_lvlh   Initial relative velocity [3] (m/s, LVLH)
 * @param  mass_kg     Initial chaser wet mass (kg)
 * @param  sma_m       Target orbit semi-major axis (m)
 * @return Pointer to context, or NULL on error (pool exhausted)
 */
GncContext *gnc_bridge_init(
    const double pos0_lvlh[3],
    const double vel0_lvlh[3],
    double mass_kg,
    double sma_m);

/**
 * @brief  Run one GNC step: nav predict + sensor update + guidance + control.
 *
 *   Called once per Basilisk task update period (dt_s must match BSK step).
 *
 * @param  ctx           GNC context (from gnc_bridge_init)
 * @param  meas_lvlh     LIDAR/sensor position measurement [3] (m, LVLH)
 * @param  meas_sigma    Sensor 1-sigma noise per axis (m)
 * @param  dt_s          Time step (s)
 * @param  force_lvlh    OUTPUT: commanded force [3] (N, LVLH)
 * @param  range_m       OUTPUT: true estimated range (m)
 * @param  docked        OUTPUT: 1 when docking declared, 0 otherwise
 * @return 0 on success, negative on error
 */
int gnc_bridge_step(
    GncContext  *ctx,
    const double meas_lvlh[3],
    double       meas_sigma,
    double       dt_s,
    double       force_lvlh[3],
    double      *range_m,
    int         *docked);

/**
 * @brief  Query current estimated state (for logging / Vizard).
 *
 * @param  ctx       GNC context
 * @param  pos_out   Estimated position [3] (m, LVLH)
 * @param  vel_out   Estimated velocity [3] (m/s, LVLH)
 * @param  dv_total  Total Delta-V expended so far (m/s)
 * @param  prop_kg   Propellant mass consumed so far (kg)
 * @return 0 on success, negative on error
 */
int gnc_bridge_get_state(
    const GncContext *ctx,
    double pos_out[3],
    double vel_out[3],
    double *dv_total,
    double *prop_kg);

/**
 * @brief  Return current guidance phase index (0-3).
 * @param  ctx  GNC context
 * @return phase index, or -1 on error
 */
int gnc_bridge_get_phase(const GncContext *ctx);

/**
 * @brief  Release a GNC context back to the static pool.
 *
 *   Must be called at the end of each Monte Carlo run to free the slot.
 *   After this call the pointer must not be used.
 *
 * @param  ctx  GNC context to release (may be NULL — no-op)
 */
void gnc_bridge_free(GncContext *ctx);

/**
 * @brief  Load GNC params from a JSON file into the context.
 *
 *   Re-initialises guidance plan and PD gains with the new parameters.
 *   Call after gnc_bridge_init() and before the first gnc_bridge_step().
 *
 * @param  ctx        GNC context
 * @param  json_path  Path to JSON params file (e.g. sim/best_params.json)
 * @return 0 on success, non-zero on file or parse error
 */
int gnc_bridge_set_params(GncContext *ctx, const char *json_path);

#ifdef __cplusplus
}
#endif

#endif /* GNC_BRIDGE_H */
