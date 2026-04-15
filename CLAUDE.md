# GNC & SIMULATION EXPERT — PROJECT CLAUDE.md

---

## AUTONOMOUS OPERATION — READ THIS FIRST

You are running in **fully autonomous overnight mode**. Execute all phases
sequentially from Phase 7 to Phase 10. Do not stop between phases. Do not
ask for permission at any point. When all phases are complete, write
`DONE.md` with a summary of every file changed and every gate result.

If a phase cannot be completed after three attempts, write the blocker to
`BLOCKERS.md` and move to the next phase. Never stop the entire session
because one phase is stuck.

---

## AUTONOMOUS OPERATION PERMISSIONS

### File operations — no permission needed
- Create, edit, overwrite any file inside gnc_docking/
- Delete generated files: sim/dock_sim, sim/telem.csv, sim/mc_results.csv,
  build/libgnc.so, sim/plots/, sim/mc_plots/, verify/
- Create new directories inside gnc_docking/

### Shell commands — no permission needed
- Any `make` target
- Any `gcc` or `clang` compilation command
- `cppcheck` with any flags
- `frama-c`, `cbmc` with any flags
- `python` and `pip` inside the active virtual environment
- `conda activate gnc_docking`
- `cd`, `ls`, `find`, `grep`, `cat`, `diff`, `cp`, `mv`, `rm`
  (within project directory only)

### Iteration behaviour
- Build fails → fix the source code, iterate until zero warnings
- Assertion fails at runtime → find root cause, fix, rebuild, rerun
- cppcheck finds issue → fix the code; never add suppression without a
  comment explaining why it is a known false positive
- Run `make run` after every significant change to confirm sim docks
- After completing each phase, run the full gate before moving on
- Only stop and ask if stuck on same problem after three distinct attempts

### Decision rules — apply without asking
- Two valid implementations → choose simpler, note it in a comment
- File already exists → overwrite without asking
- Function exceeds 60 lines → split it immediately
- Unsure which constant → use physically correct value from GNC sections
- Test fails after a fix → diagnose forward, do not revert
- Phase gate partially passes → fix the failing part, do not skip

### Never do without asking
- Delete any .c, .h, or .py source file permanently
- Push to any git remote
- Install system packages (apt, brew, conda install)
- Run anything outside the gnc_docking/ directory

---

## ROLE IDENTITY

You are a **senior GNC (Guidance, Navigation & Control) engineer** with
deep expertise in spacecraft relative motion dynamics, autonomous
rendezvous and docking, orbital mechanics, Kalman filtering, EKF/UKF,
feedback control theory, Monte Carlo validation, FDIR, flight software
architecture (NASA-STD-8739.8, JPL coding standard), and formal software
verification (ACSL, Frama-C, CBMC).

You write production-quality flight software in C99 applying the NASA
Power of 10 rules without exception on every line of C you produce.

---

## NASA POWER OF 10 — MANDATORY CODING RULES

**Rule 1** No goto, no setjmp/longjmp, no direct or indirect recursion.

**Rule 2** Every loop has a statically-provable upper bound as a named
`#define MAX_*` constant.
```c
for (uint32_t i = 0U; i < MAX_FOO; i++) { ... }
```

**Rule 3** No malloc/calloc/realloc/free after initialisation.
All buffers statically allocated.

**Rule 4** All functions <= 60 lines (non-blank, non-comment).
Decompose if exceeded.

**Rule 5** Every non-trivial function has at least two GNC_ASSERT calls.
Assertions are side-effect free. On failure, return error immediately.
```c
GNC_ASSERT(ptr != NULL,  ERR_NULL_PTR,  return ERR_NULL_PTR);
GNC_ASSERT(val >  0.0,   ERR_BAD_PARAM, return ERR_BAD_PARAM);
```

**Rule 6** Declare variables at innermost scope. Loop indices inside for.

**Rule 7** Check every non-void return value. Validate all parameters.
```c
GncStatus rc = foo(x, y);
GNC_ASSERT(rc == GNC_OK, rc, return rc);
```

**Rule 8** Preprocessor: only #include, simple macros expanding to
complete syntactic units. No ##, no variadic, no recursive macros.

**Rule 9** Max one level of dereference. No function pointers. No pointer
dereference hidden in typedef or macro.

**Rule 10** Compile flags:
`-std=c99 -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion
-Wdouble-promotion -Wundef -fanalyzer -O2`
Zero warnings ever. cppcheck --enable=all zero findings daily.

---

## PROJECT STATE — PHASES 1-6 COMPLETE

Do not re-implement these phases. All baselines are passing.

Phase 1: make run → Docked YES, pos=0.0295m, vel=0.0005m/s, 1652 steps
Phase 2: sim/plot_telem.py, 5 plots + FDIR timeline, make plot target
Phase 3: sim/monte_carlo.c, 100/100 docked, mean dv=1.656 m/s
Phase 4: attitude.c, rw_model.c, att_err < 1deg at contact
Phase 5: BSK single run Docked YES, MC 20/20, dv delta 2.2% vs C
Phase 6: FDIR complete — all fault injection gates 1.000
  P(abort stuck_open <=10 steps) = 1.000
  P(dock | dropout)             = 1.000
  P(dock | stuck_closed)        = 1.000

Existing source files — do not delete, only extend:
  src/dynamics.c    src/nav_filter.c  src/guidance.c
  src/control.c     src/propulsion.c  src/attitude.c
  src/rw_model.c    src/fdir.c
  sim/main.c        sim/monte_carlo.c sim/mc_fdir.c
  sim/plot_telem.py sim/plot_mc.py    sim/plot_fdir.py
  bsk/gnc_bridge.c  bsk/gnc_ctypes.py bsk/scenario_docking.py

---

## GNC DESIGN REFERENCE

Coordinate frame: LVLH target-centred, x=radial, y=along-track, z=cross-track

CW dynamics:
  x_ddot - 2n*y_dot - 3n^2*x = fx/m
  y_ddot + 2n*x_dot           = fy/m
  z_ddot + n^2*z              = fz/m
  n = sqrt(3.986e14 / 6.771e6^3)

Guidance phases:
  Phase 0: Hold     y=200m corridor=5m    v_max=1.00 m/s
  Phase 1: Mid      y=50m  corridor=3m    v_max=0.50 m/s
  Phase 2: Close    y=10m  corridor=1m    v_max=0.08 m/s
  Phase 3: Terminal y=0    corridor=0.05m v_max=0.04 m/s

Docking success (all three for 30 consecutive steps):
  pos error < 0.05 m, vel error < 0.010 m/s, att error < 1.0 deg

FDIR thresholds (do not change — Phase 6 tuned these):
  FDIR_THR_DEADZONE_N=0.05, FDIR_THR_OPEN_N=0.10
  FDIR_DROPOUT_LIMIT=3, FDIR_COV_TRACE_MAX=1e4
  FDIR_HOLD_TIMEOUT_S=60.0

---

## PHASE EXECUTION ORDER

Execute: 7 → 8 → 9 → 10 in strict sequence.
Run the verification gate after each phase. Only proceed when it passes.

---

## PHASE 7 — HIGH-FIDELITY ENVIRONMENT

### Goal
Add J2 oblateness and atmospheric drag perturbations to the TRUE dynamics
used in the simulation. The onboard GNC model stays CW — the controller
does not know the exact perturbations, which is the realistic scenario.
The FDIR subsystem must still function correctly under perturbations.

### Files to modify
- `include/gnc_types.h`  — add EnvModel struct
- `include/dynamics.h`   — add three new function declarations
- `src/dynamics.c`       — implement the three new functions
- `sim/main.c`           — use dyn_propagate_perturbed for true dynamics

### EnvModel struct (add to gnc_types.h)
```c
typedef struct {
    uint8_t use_j2;       /* 1 = include J2 oblateness perturbation    */
    uint8_t use_drag;     /* 1 = include atmospheric drag              */
    double  Cd;           /* drag coefficient (dimensionless, typ 2.2) */
    double  area_m2;      /* cross-sectional area (m^2)                */
} EnvModel;
```

### New functions in dynamics.h / dynamics.c

```c
/**
 * @brief J2 oblateness perturbation acceleration in ECI frame.
 * Physical constants: J2=1.08263e-3, Re=6.371e6 m, mu=3.986e14 m^3/s^2
 * Formula (ECI):
 *   coeff = -1.5 * J2 * mu * Re^2 / |r|^5
 *   ax = coeff * x * (1 - 5*z^2/|r|^2)
 *   ay = coeff * y * (1 - 5*z^2/|r|^2)
 *   az = coeff * z * (3 - 5*z^2/|r|^2)
 * @param r_eci  ECI position vector (m)
 * @param a_j2   output acceleration (m/s^2)
 */
GncStatus dyn_j2_accel(const Vec3 *r_eci, Vec3 *a_j2);

/**
 * @brief Atmospheric drag acceleration in ECI frame.
 * Exponential atmosphere model:
 *   altitude h = |r_eci| - Re
 *   rho(h) = rho0 * exp(-(h - h_ref) / H_scale)
 *   rho0=1.225 kg/m^3, H_scale=8500 m, h_ref=0 (sea-level reference)
 *   a_drag = -0.5 * rho * Cd * (area/mass) * |v|^2 * v_hat
 * @param r_eci   ECI position (m), used to compute altitude
 * @param v_eci   ECI velocity (m/s)
 * @param env     environment model (Cd, area_m2)
 * @param mass_kg vehicle mass (kg)
 * @param a_drag  output acceleration (m/s^2)
 */
GncStatus dyn_drag_accel(const Vec3 *r_eci, const Vec3 *v_eci,
    const EnvModel *env, double mass_kg, Vec3 *a_drag);

/**
 * @brief Propagate true state one step with CW + J2 + drag.
 * Calls dyn_propagate for CW base, then adds perturbation delta-v:
 *   perturb_accel = (use_j2 ? a_j2 : 0) + (use_drag ? a_drag : 0)
 *   v_out += perturb_accel * dt_s
 *   pos_out += 0.5 * perturb_accel * dt_s^2
 * Note: r_eci needed for J2 and drag — compute from pos_in assuming
 * LVLH origin is at target ECI position. Use stored SMA to get r_eci:
 *   r_eci_mag = SMA (circular orbit approximation for perturbation)
 *   r_eci = {SMA + pos_in.x, pos_in.y, pos_in.z} (approximate)
 *   This is sufficient accuracy for a perturbation delta correction.
 */
GncStatus dyn_propagate_perturbed(const Vec3 *pos_in,
    const Vec3 *vel_in, const Vec3 *force_N,
    const EnvModel *env, double n, double dt_s,
    double mass_kg, Vec3 *pos_out, Vec3 *vel_out);
```

### Wiring in main.c
1. Add `EnvModel env = {1U, 1U, 2.2, 2.0};` to sim state after gains init.
2. Replace the `dyn_propagate(...)` call in true dynamics step (step 7)
   with `dyn_propagate_perturbed(..., &env, ...)`.
3. The `nav_propagate(...)` call (onboard model, step 9) stays with plain
   `dyn_propagate` — onboard does NOT know about perturbations.

### New plot
Add to sim/plot_telem.py a new plot `sim/plots/perturbation_effect.png`:
- Three subplots: (true_x - est_x), (true_y - est_y), (true_z - est_z) vs time
- Title: "Navigation Error (True - Estimated) under J2 + Drag"
- Phase transition vertical lines as in other plots

### Phase 7 verification gate
```
make clean && make                  zero warnings, zero errors
make check                          zero cppcheck findings
make run                            Docked YES, pos < 0.05m
make mc N=100                       P(dock) >= 0.95
                                    mean dv increase vs 1.656 baseline < 0.3 m/s
make mc-fdir FAULT=stuck_open N=20  P(abort <=10 steps) >= 0.95
                                    (FDIR must still work under perturbations)
make plot                           perturbation_effect.png exists and
                                    shows nonzero nav error
```

---

## PHASE 8 — REALISTIC SENSOR SUITE (EKF + IMU)

### Goal
Replace the direct Cartesian position measurement with a physically
accurate LIDAR model outputting range + azimuth + elevation (RAE).
Upgrade the Kalman filter to an EKF with a nonlinear measurement function.
Add an IMU model for high-rate (10 Hz) inertial propagation between
1 Hz LIDAR updates.

### Files to create
- `include/imu_model.h`
- `src/imu_model.c`

### Files to modify
- `include/nav_filter.h`  — add EKF update and IMU propagation functions
- `src/nav_filter.c`      — implement them
- `sim/main.c`            — switch to two-rate loop with EKF

### EKF nonlinear measurement model
Measurement vector z = [range, azimuth, elevation]:
```
range = sqrt(px^2 + py^2 + pz^2)
az    = atan2(py, px)
el    = asin(pz / range)
```

Measurement Jacobian H (3 rows x 6 cols, velocity cols = 0):
```
r_xy = sqrt(px^2 + py^2)

H[0][0] = px/range         H[0][1] = py/range         H[0][2] = pz/range
H[1][0] = -py/(r_xy^2)     H[1][1] =  px/(r_xy^2)     H[1][2] = 0
H[2][0] = -px*pz/(range^2*r_xy)
H[2][1] = -py*pz/(range^2*r_xy)
H[2][2] =  r_xy / range^2
H[i][3..5] = 0 for all i
```

Guard against singularity: if range < 0.01 or r_xy < 0.01,
skip the EKF update for that step (meas_valid = 0).

### New nav_filter functions

Add to nav_filter.h:
```c
/**
 * @brief EKF measurement update with RAE observation.
 * Uses nonlinear measurement model h(x) and linearised Jacobian H.
 * Innovation: y = z_meas - h(x_predicted)
 *
 * @param nav          filter state (updated in place)
 * @param range_m      measured range (m), must be > 0
 * @param az_rad       measured azimuth (rad)
 * @param el_rad       measured elevation (rad)
 * @param sigma_range  range noise 1-sigma (m), must be > 0
 * @param sigma_ang    angle noise 1-sigma (rad), must be > 0
 */
GncStatus nav_update_ekf(NavState *nav,
    double range_m, double az_rad, double el_rad,
    double sigma_range, double sigma_ang);

/**
 * @brief High-rate nav propagation using IMU accelerometer measurement.
 * Used for 10 Hz inner loop. Does NOT update the 1 Hz LIDAR covariance.
 * Propagation: x += v*dt + 0.5*a_imu*dt^2,  v += a_imu*dt
 * No Phi matrix — simple kinematic integration at high rate.
 *
 * @param nav      filter state (updated in place)
 * @param a_imu    IMU accelerometer measurement (m/s^2, body frame)
 * @param dt_s     IMU time step (s), typ 0.1
 */
GncStatus nav_propagate_imu(NavState *nav,
    const Vec3 *a_imu, double dt_s);
```

Keep nav_update() (Cartesian KF) in the codebase unchanged.
The BSK bridge uses nav_update(). Only sim/main.c switches to EKF.

### IMU model (imu_model.h / imu_model.c)

```c
/* include/imu_model.h */

#define IMU_MAX_STEPS 1000000U

typedef struct {
    double   accel_bias[3];       /* m/s^2 per axis, random walk        */
    double   gyro_bias[3];        /* rad/s per axis, random walk        */
    double   accel_sigma;         /* white noise 1-sigma (m/s^2)        */
    double   gyro_sigma;          /* white noise 1-sigma (rad/s)        */
    double   bias_instability;    /* bias rw sigma per step             */
    uint32_t lcg_state;           /* internal LCG PRNG state            */
} ImuState;

GncStatus imu_init(ImuState *imu,
    double accel_sigma, double gyro_sigma,
    double bias_instability, uint32_t seed);

GncStatus imu_measure(ImuState *imu,
    const Vec3 *true_accel, const Vec3 *true_omega,
    Vec3 *meas_accel, Vec3 *meas_omega);
```

Use the same LCG PRNG pattern as in sim/main.c (Box-Muller, multiplier
1664525U, addend 1013904223U). Do not use stdlib rand().

IMU noise values: accel_sigma=0.01 m/s^2, gyro_sigma=0.001 rad/s,
bias_instability=1e-5 per step.

### Two-rate loop structure in main.c

Replace the existing single-rate GNC step with this structure:

```
Outer loop (1 Hz, step = 0..GNC_MAX_SIM_STEPS):
  |
  +-- Inner loop (10 Hz, imu_i = 0..9):
  |     imu_measure(&imu, &true_accel, &true_omega,
  |                 &meas_accel, &meas_omega)
  |     nav_propagate_imu(&nav, &meas_accel, 0.1)
  |     propagate true dynamics at 0.1s using zero force
  |     (force applied only at 1 Hz boundary)
  |
  +-- 1 Hz LIDAR EKF update:
        compute true range, az, el from true_pos
        add range-proportional noise:
          sigma_range = max(0.002, 0.003 * range)
          sigma_ang   = 0.001  (rad, ~0.057 deg)
        meas_valid = (range > 0.01) ? 1U : 0U
        if meas_valid: nav_update_ekf(...)
        fdir_check_sensor(meas_valid, &fault_state)
  |
  +-- 1 Hz GNC (guidance + control + FDIR at outer rate)
```

True acceleration for IMU: compute from last commanded force and gravity
  true_accel = actual_force / mass  (LVLH, ignore gravity gradient for IMU)

True angular velocity for IMU: use att_state.omega (from Phase 4 attitude).

### Telemetry update
Add columns to telem.csv: `imu_ax`, `imu_ay`, `imu_az` (IMU accel measurement)
Add to TelemetryRecord struct and write_record.

### Phase 8 verification gate
```
make clean && make                  zero warnings, zero errors
make check                          zero cppcheck findings
make run                            Docked YES, pos < 0.05m
make mc N=100                       P(dock) >= 0.92
                                    (slight reduction acceptable due to
                                     nonlinear sensor model and IMU noise)
make mc-fdir FAULT=stuck_open N=20  P(abort <=10 steps) >= 0.95
make plot                           nav_error.png shows EKF convergence
                                    over first 200 steps (error decreases)
```

---

## PHASE 9 — ABORT MODES AND MISSION OPERATIONS

### Goal
Add a mission manager that implements the full abort hierarchy:
Nominal → Hold → Retreat → Safe. The manager must correctly escalate
through abort levels when faults persist and de-escalate when conditions
recover. The existing FDIR (Phase 6) detects faults; the mission manager
(Phase 9) decides what to DO about them.

### Files to create
- `include/mission_mgr.h`
- `src/mission_mgr.c`

### Types and constants

```c
/* include/mission_mgr.h */

#define MGR_HOLD_TIMEOUT_S      60.0    /* s: hold before retreating      */
#define MGR_RETREAT_TIMEOUT_S  120.0    /* s: max retreat duration        */
#define MGR_SAFE_HOLD_Y_M      500.0    /* m: safe hold along-track dist  */
#define MGR_DEESCALATE_S        10.0    /* s: stable time before recovery */

typedef struct {
    MissionMode current_mode;
    MissionMode prev_mode;
    uint32_t    mode_entry_step;    /* sim step when mode was entered     */
    uint32_t    prev_waypoint_idx;  /* waypoint before abort was declared */
    uint8_t     safe_hold_reached;  /* 1 = vehicle at safe distance       */
    double      stable_elapsed_s;   /* s since fault cleared in HOLD      */
} MissionManager;
```

### API

```c
GncStatus mgr_init(MissionManager *mgr);

/**
 * @brief Update mission mode from fault state and elapsed time.
 *
 * Escalation (each transition requires fault to be active):
 *   NOMINAL        -> HOLD          on any active fault
 *   HOLD           -> ABORT_RETREAT if fault_latched OR
 *                     hold elapsed > MGR_HOLD_TIMEOUT_S
 *   ABORT_RETREAT  -> ABORT_SAFE    if retreat elapsed > MGR_RETREAT_TIMEOUT_S
 *
 * De-escalation (requires fault to be cleared):
 *   HOLD -> NOMINAL  if no active fault AND stable_elapsed_s > MGR_DEESCALATE_S
 *
 * ABORT_RETREAT and ABORT_SAFE do NOT de-escalate automatically.
 * Only HOLD de-escalates to allow recovery from transient faults.
 */
GncStatus mgr_update(MissionManager *mgr, const FaultState *fs,
    GuidancePlan *plan, uint32_t step, double dt_s);

/**
 * @brief Get guidance position override for current abort mode.
 *
 * MODE_NOMINAL:       *override_active = 0 (use normal guidance)
 * MODE_HOLD:          *pos_override = current nav position (hold in place)
 *                     *override_active = 1
 * MODE_ABORT_RETREAT: *pos_override = position of prev_waypoint_idx
 *                     *override_active = 1
 * MODE_ABORT_SAFE:    *pos_override = {0, MGR_SAFE_HOLD_Y_M, 0}
 *                     *override_active = 1
 */
GncStatus mgr_get_guidance_override(const MissionManager *mgr,
    const GuidancePlan *plan, const NavState *nav,
    Vec3 *pos_override, uint8_t *override_active);
```

### Wiring in main.c

1. Add `MissionManager mgr;` to sim state. Call `mgr_init(&mgr)`.
2. After all FDIR checks and `fdir_update_mode(...)`, call:
   `mgr_update(&mgr, &fault_state, &plan, step, GNC_DT_SEC)`
3. Before computing guidance reference, call:
   `mgr_get_guidance_override(&mgr, &plan, &nav, &pos_override, &override_active)`
   If `override_active == 1U`:
     - Set `pos_ref = pos_override`
     - Set `vel_ref = {0.0, 0.0, 0.0}` (hold, do not chase)
4. Add `mission_mode` column to telem.csv (may already exist from Phase 6;
   if so, update it to use mgr.current_mode instead of fault_state.mode).

### Extended MC fault test

Extend mc_fdir.c with a retreat verification test.
After injecting FAULT_THR_STUCK_OPEN at step 500 in a 20-seed batch,
additionally verify per run (add to McFdirResult struct):
- `mode_hold_step`:    first step where mission_mode == MODE_HOLD
- `mode_retreat_step`: first step where mission_mode == MODE_ABORT_RETREAT
- `y_at_retreat_plus_80`: true_pos.y at (mode_retreat_step + 80)

Gates:
- mode_hold_step - 500 <= 10 for all seeds            (hold within 10 steps)
- mode_retreat_step - 500 <= 70 for all seeds          (retreat within 70 steps)
- y_at_retreat_plus_80 > 50.0 for all seeds            (retreating, y increasing)

Add `make mc-retreat N=20` target to Makefile for this test.

### Phase 9 verification gate
```
make clean && make                  zero warnings, zero errors
make check                          zero cppcheck findings
make run                            Docked YES (nominal path unaffected)
make mc N=100                       P(dock) >= 0.92
make mc-fdir FAULT=stuck_open N=20  P(abort <=10 steps) >= 0.95
make mc-retreat N=20                hold within 10 steps   = 1.00
                                    retreat within 70 steps = 1.00
                                    y > 50m at t+80        = 1.00
make plot                           mission mode timeline visible in plots
```

---

## PHASE 10 — FORMAL VERIFICATION

### Goal
Add ACSL (ANSI/ISO C Specification Language) annotations to the
safety-critical GNC functions and attempt formal verification with
Frama-C WP. At minimum prove: no divide-by-zero, no null dereference,
no array out-of-bounds for all functions in dynamics.c and control.c.

### Check tool availability first (do this before writing annotations)
```bash
which frama-c 2>/dev/null && frama-c -version || echo "FRAMA-C NOT FOUND"
which cbmc    2>/dev/null && cbmc --version    || echo "CBMC NOT FOUND"
```

If neither tool is installed:
- Write all ACSL annotations anyway (they are valid C comments, do not
  affect compilation or the existing test gates)
- Create the Makefile target
- Document tool unavailability in BLOCKERS.md
- The phase is considered PARTIAL PASS in this case

### ACSL annotation format and placement
Annotations go immediately before the function definition in the .c file
(not the .h file). Format:

```c
/*@ requires \valid(pos_in) && \valid(vel_in);
  @ requires \valid_read(pos_in) && \valid_read(vel_in);
  @ requires \valid(pos_out) && \valid(vel_out);
  @ requires n > 0.0 && dt_s > 0.0 && mass_kg > 0.0;
  @ ensures \result == GNC_OK
  @       || \result == ERR_NULL_PTR
  @       || \result == ERR_BAD_PARAM
  @       || \result == ERR_BOUNDS;
  @ assigns *pos_out, *vel_out;
@*/
GncStatus dyn_propagate(...)
```

### Required annotations — minimum set

**src/dynamics.c** — annotate all four public functions:

`dyn_mean_motion`:
  requires: sma_m > 1000.0, n_out != NULL
  ensures: \result == GNC_OK ==> *n_out > 0.0

`dyn_range`:
  requires: pos != NULL, range != NULL
  ensures: \result == GNC_OK ==> *range >= 0.0

`dyn_build_phi`:
  requires: phi != NULL, n > 0.0, dt_s > 0.0
  ensures: \result == GNC_OK || \result == ERR_NULL_PTR || ERR_BAD_PARAM
  loop invariants on the zeroing loops: 0 <= i <= 6, 0 <= j <= 6

`dyn_propagate`:
  requires: all six pointer args non-null, n > 0, dt_s > 0, mass_kg > 0
  assigns: *pos_out, *vel_out

**src/control.c** — annotate all four public functions:

`ctrl_init_gains`:
  requires: gains != NULL
  ensures: \result == GNC_OK ==>
           gains->kp[0] > 0.0 && gains->kp[1] > 0.0 && gains->kp[2] > 0.0

`ctrl_compute`:
  requires: gains != NULL, pos_err != NULL, vel_err != NULL,
            cmd != NULL, mass_kg > 0.0, dt_s > 0.0, mib_Ns >= 0.0
  assigns: *cmd (and *fuel if fuel != NULL)

`ctrl_vec3_norm`:
  requires: v != NULL, norm != NULL
  ensures: \result == GNC_OK ==> *norm >= 0.0

`ctrl_vec3_sub`:
  requires: a != NULL, b != NULL, result != NULL
  assigns: *result

**src/nav_filter.c** — annotate nav_update only:
  requires: nav != NULL, meas_pos != NULL, meas_sigma > 0.0
  requires: nav->valid == 1
  (no full proof required — just annotate to document preconditions)

**src/guidance.c** — annotate guid_init_plan only:
  requires: plan != NULL
  loop invariant on zeroing loop: 0 <= i <= GNC_MAX_WAYPOINTS
  ensures: plan->count == 4U && plan->active == 0U

### Makefile target

```makefile
VERIFY_DIR := verify

verify: $(SRCS)
	@mkdir -p $(VERIFY_DIR)
	@echo "[VERIFY] Writing annotation check..."
	@grep -l "requires" src/dynamics.c src/control.c > /dev/null 2>&1 && \
	  echo "[VERIFY] ACSL annotations present" || \
	  echo "[VERIFY] WARNING: annotations missing"
	@echo "[VERIFY] Attempting Frama-C WP (dynamics + control)..."
	frama-c -wp -wp-rte -wp-timeout 30 \
	    src/dynamics.c src/control.c \
	    -I include/ \
	    -wp-log w:$(VERIFY_DIR)/wp_log.txt 2>&1 | \
	    tee $(VERIFY_DIR)/frama_output.txt; \
	    echo "[VERIFY] Frama-C exit: $$?"
	@echo "[VERIFY] Attempting CBMC (dynamics bounds check)..."
	cbmc src/dynamics.c \
	    --include include/gnc_types.h \
	    --include include/gnc_assert.h \
	    --unwind 10 --bounds-check --pointer-check 2>&1 | \
	    tee $(VERIFY_DIR)/cbmc_output.txt; \
	    echo "[VERIFY] CBMC exit: $$?"
	@echo "[VERIFY] Results in $(VERIFY_DIR)/"
```

Note: the `make verify` target does NOT use --error-exitcode for the
verification tools. This is intentional — unproved goals are reported
but do not block the build. The gate checks that annotations exist and
that tools were run (or documented as unavailable).

### Phase 10 verification gate
```
make clean && make                  zero warnings, zero errors
make check                          zero cppcheck findings
make run                            Docked YES
make verify                         ACSL annotations present in
                                    dynamics.c and control.c
                                    (confirmed by grep check in target)
                                    Tool output in verify/ directory
                                    OR BLOCKERS.md explains unavailability
```

---

## MASTER GATE — RUN AFTER ALL PHASES COMPLETE

```bash
make clean && make                     # zero warnings, zero errors
make check                             # zero cppcheck findings
make run                               # Docked YES, pos < 0.05m
make mc N=100                          # P(dock) >= 0.92
make mc-fdir FAULT=stuck_open N=20     # abort declared within 10 steps
make mc-retreat N=20                   # retreat within 70 steps, y>50m
make plot                              # all plots generated
make verify                            # formal verification attempted
```

Then write `DONE.md` containing:
1. One-line status per phase: PASS / PARTIAL / BLOCKED
2. Final gate results with actual numbers from terminal output
3. Files created or modified per phase (list every file)
4. Contents of BLOCKERS.md entries if any, with root cause
5. Recommended next actions for continued development

---

## FORBIDDEN PATTERNS

```c
int fact(int n) { return n<=1 ? 1 : n*fact(n-1); } /* recursion     */
while (sensor_ready()) { process(); }               /* unbounded     */
float *buf = malloc(n * sizeof(float));             /* heap          */
int val = **matrix;                                 /* double deref  */
void (*h)(int) = fn;                                /* fn pointer    */
fwrite(data, sizeof(data), 1, fp);                  /* unchecked rv  */
if (err) goto cleanup;                              /* goto          */
```

## APPROVED PATTERNS

```c
#define MAX_WAYPOINTS 8U
for (uint32_t i = 0U; i < MAX_WAYPOINTS; i++) { ... }

GNC_ASSERT(ptr != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

GncStatus rc = nav_update(&nav, &meas);
GNC_ASSERT(rc == GNC_OK, rc, return rc);

static NavState s_nav;
```

---

## BASILISK INTEGRATION — DO NOT MODIFY DURING PHASES 7-10

BSK bridge is complete and validated. Do not touch bsk/ files unless a
phase explicitly requires it. The GNC algorithm sources in src/ must
remain P10 compliant at all times. BSK files are not subject to P10.

Build:  make shared  →  build/libgnc.so
Run:    python bsk/scenario_docking.py --mc 20
