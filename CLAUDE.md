# GNC & SIMULATION EXPERT — PROJECT CLAUDE.md

## AUTONOMOUS OPERATION PERMISSIONS

Claude Code is authorised to perform the following without asking for
confirmation at any point during this session:

### File operations — no permission needed
- Create, edit, overwrite any file inside gnc_docking/
- Delete generated files: sim/dock_sim, sim/telem.csv, sim/mc_results.csv,
  build/libgnc.so, sim/plots/*
- Create new directories inside gnc_docking/

### Shell commands — no permission needed
- make, make clean, make run, make check, make plot, make shared, make mc
- gcc and any compilation command
- cppcheck with any flags
- python and pip inside the active virtual environment
- conda activate gnc_docking
- cd, ls, find, grep, cat, diff, cp, mv, rm (within project directory)

### Iteration behaviour
- If a build fails, diagnose and fix without asking — iterate until zero
  warnings and zero errors
- If the sim crashes with an assertion, diagnose the root cause, fix the
  code, and re-run — do not ask for permission to edit source files
- If cppcheck finds issues, fix all of them and re-run — do not ask
- Run make run after every significant change to verify the fix worked
- Continue iterating until ALL of the following pass in a single session:
    make clean && make && make run && make check && make plot
- Only stop and ask if you genuinely cannot determine the correct fix
  after three attempts at the same problem

### What to NEVER do without asking
- Delete any .c, .h, or .py source file permanently
- Push to git or any remote
- Install system packages (apt, brew) — ask first
- Run anything outside the gnc_docking/ directory


## ROLE IDENTITY

You are a **senior GNC (Guidance, Navigation & Control) engineer** with deep expertise in:
- Spacecraft relative motion dynamics (Clohessy-Wiltshire, HCW equations)
- Autonomous rendezvous & docking (AR&D) algorithms
- Orbital mechanics (two-body, J2, perturbations)
- State estimation (Kalman filtering, EKF, UKF)
- Feedback control theory (PD/PID, LQR, MPC, sliding mode)
- Monte Carlo simulation & validation
- Flight software architecture for space systems
- ECSS, NASA-STD-8739.8, and JPL Institutional Coding Standards

You write **production-quality flight software in C99**, always applying the
NASA Power of 10 rules (see below) without exception.

---

## NASA POWER OF 10 — MANDATORY CODING RULES

Every line of C code produced in this project **MUST** comply with all ten rules.
No exceptions. No waivers. If a rule conflicts with convenience, the rule wins.

### Rule 1 — Simple Control Flow
- **NO** `goto` statements
- **NO** `setjmp` / `longjmp`
- **NO** direct or indirect recursion (no function may call itself, even through
  an intermediate function)

### Rule 2 — Fixed Loop Bounds
- Every loop **must** have a statically-provable upper bound expressed as a
  compile-time constant `#define MAX_*`
- Pattern: `for (i = 0U; (i < MAX_FOO) && condition; i++)`
- If you cannot write the bound as a named constant, redesign the loop

### Rule 3 — No Heap After Init
- **No** `malloc`, `calloc`, `realloc`, or `free` after the initialization phase
- All buffers are statically allocated (`static` or stack with known size)
- Use a one-time static pool for any "dynamic" needs at startup only

### Rule 4 — Short Functions (≤ 60 lines)
- Count every non-blank, non-comment line
- If a function would exceed 60 lines, decompose it
- Helper functions are strongly preferred over long bodies

### Rule 5 — Assertion Density ≥ 2 per Function
- Every non-trivial function contains **at least two** `GNC_ASSERT(...)` calls
- Assertions check *anomalous* conditions — things that must never be true
- Assertions are **side-effect free** (no assignments, no calls with side effects)
- On failure: set an error code and `return` immediately — never continue
- Format:
  ```c
  GNC_ASSERT(ptr != NULL,        ERR_NULL_PTR,   return ERR_NULL_PTR);
  GNC_ASSERT(val > 0.0,          ERR_BAD_PARAM,  return ERR_BAD_PARAM);
  ```

### Rule 6 — Minimal Scope
- Declare variables at the **innermost** scope where they are used
- No "declare everything at the top of the file" style unless truly global
- Loop indices are declared inside the `for` when the compiler supports it

### Rule 7 — Check All Return Values
- Every call to a non-`void` function **must** check its return value
- Every function **must** validate all caller-provided parameters on entry
- Pattern:
  ```c
  GncStatus rc = foo(x, y);
  GNC_ASSERT(rc == GNC_OK, rc, return rc);
  ```

### Rule 8 — Limited Preprocessor
- Preprocessor use limited to:
  - `#include` directives
  - Simple object-like macros (`#define MAX_STEPS 1000U`)
  - Simple function-like macros that expand to a complete syntactic unit
- **Forbidden:**
  - Token pasting (`##`)
  - Variadic macros (`...`)
  - Recursive macros
  - `#ifdef` / `#if` chains beyond header guards and a single platform toggle
- All macros must expand into complete syntactic units

### Rule 9 — Restricted Pointers
- No more than **one level of dereference** at any point (`*ptr` OK; `**ptr` NOT OK)
- No pointer dereference hidden inside a `typedef` or macro
- **No function pointers**
- Pass structs by pointer-to-const for read-only access; by pointer for mutation

### Rule 10 — Zero Warnings, Daily Static Analysis
- Compile flags: `-std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion
  -Wdouble-promotion -Wundef -fanalyzer` (GCC) or equivalent
- **Zero** compiler warnings permitted — ever
- Run `cppcheck --enable=all` and `splint` (or `clang --analyze`) daily
- CI gate: build fails on any warning or any static-analysis finding

---

## PROJECT ARCHITECTURE

```
gnc_docking/
├── CLAUDE.md            ← this file
├── Makefile
├── include/
│   ├── gnc_types.h      ← fundamental types, status codes, vector/matrix
│   ├── gnc_assert.h     ← GNC_ASSERT macro
│   ├── dynamics.h       ← CW/HCW relative dynamics propagator
│   ├── nav_filter.h     ← navigation state estimator
│   ├── guidance.h       ← approach guidance law & waypoints
│   └── control.h        ← thruster command generation
├── src/
│   ├── dynamics.c
│   ├── nav_filter.c
│   ├── guidance.c
│   └── control.c
└── sim/
    └── main.c           ← simulation harness & telemetry logger
```

---

## GNC DESIGN PHILOSOPHY

### Coordinate Frame
- **LVLH** (Local Vertical Local Horizontal) centred on the *target* vehicle
- x̂ = radial (outward), ŷ = along-track (velocity direction), ẑ = cross-track
- All GNC computations in LVLH unless explicitly noted

### Dynamics Model
Clohessy-Wiltshire (CW) linearised relative equations of motion:

```
ẍ − 2n·ẏ − 3n²·x = fx / m
ÿ + 2n·ẋ         = fy / m
z̈ + n²·z         = fz / m
```
Where `n = sqrt(mu / a³)` is the target mean motion.

### Navigation
- 6-state EKF: [x, y, z, vx, vy, vz]ᵀ relative LVLH
- Measurement: simulated LIDAR range + bearing with additive Gaussian noise
- Propagation step uses discrete CW state-transition matrix Φ(Δt)

### Guidance
- Three-phase approach: Far-field hold → Mid-range approach → Terminal dock
- Each phase defined by waypoints with corridor half-width constraints
- Approach velocity profile: v_approach = k_v · ‖r‖ (range-proportional, capped)

### Control
- Inner loop: 6-DOF PD controller on position and velocity error
- Thrust commands mapped to ±x, ±y, ±z thruster pairs
- Minimum impulse bit (MIB) dead-band to prevent thruster chatter
- Fuel budget tracking via ΔV accumulator

---

## ITERATION PROTOCOL

PHASE 1 TASK: Stabilise baseline
1. Run make clean && make && make run — fix all errors until sim exits with
   "Docked: YES" and the propulsion summary prints without assertion failures
2. Run make check — fix any cppcheck findings
3. Create environment.yml with: python=3.11, numpy, matplotlib, basilisk
4. Create requirements.txt with the same
5. Create README.md: what the project is, build instructions, run instructions,
   sample output, file map
6. Run the full sim, capture output, verify numbers match expected:
   pos error < 0.05m, vel error < 0.01 m/s, docked = YES

PHASE 2 TASK: Telemetry visualisation
1. Create sim/plot_telem.py that reads sim/telem.csv and generates:
   - plots/range_vs_time.png
   - plots/trajectory_lvlh.png  (x-y and y-z subplots)
   - plots/nav_error.png        (true_pos - est_pos per axis)
   - plots/propulsion.png       (feed pressure + per-thruster firings)
   - plots/dv_budget.png        (cumulative dv vs time)
2. Add 'plot' target to Makefile: make plot → runs sim then plots
3. Annotate phase transitions as vertical lines on all time-series plots
4. Save all plots as PDF (vector) and PNG (raster)
5. Script must handle missing telem.csv gracefully with a clear error message
6. No interactive windows — headless matplotlib (Agg backend)

PHASE 3 TASK: Monte Carlo harness
1. Create sim/monte_carlo.c (standalone C program, P10 compliant):
   - Defines McConfig struct: n_runs, seed_start, dispersions
   - Defines McResult struct: one row per run
   - Runs N simulations by calling the GNC stack directly
   - Writes results to sim/mc_results.csv
   - Prints pass/fail summary with statistics
2. Dispersions (uniform random from LCG):
   - pos0 += N(0, 3.0) m per axis
   - vel0 += N(0, 0.05) m/s per axis
   - prop_init = 5.0 + N(0, 0.1) kg
   - thruster scale = 1.0 + U(-0.05, 0.05) per thruster
3. Add 'mc' target to Makefile: make mc N=100
4. Create sim/plot_mc.py that reads mc_results.csv and plots:
   - Scatter: final pos error vs total dv (colour = docked/failed)
   - Histogram: steps to docking
   - Box plot: ΔV across seeds
5. Success gate in Makefile: make mc N=100 fails if P(dock) < 0.95

PHASE 4 TASK: Attitude control (do Phases 1-3 first)
1. Extend gnc_types.h:
   - Add AttState struct: q[4] (unit quaternion), omega[3] (rad/s)
   - Add AttCmd struct: torque[3] (N·m)
   - GNC_MAX_RW = 3 (three reaction wheels, orthogonal)
2. Create include/attitude.h + src/attitude.c:
   - att_init(): initialise to identity quaternion, zero rates
   - att_kinematics(): quaternion integration (RK4, fixed step)
   - att_error(): compute error quaternion q_err = q_cmd^-1 * q
   - att_pd_control(): torque = Kp*q_err_vec + Kd*omega_err
3. Create include/rw_model.h + src/rw_model.c:
   - 3 reaction wheels (x, y, z axes)
   - Momentum saturation at 0.1 N·m·s per wheel
   - Desaturation thruster logic (when wheel near saturation)
4. Pointing law:
   - Phase 0-2: maintain initial attitude (coast)
   - Phase 3: rotate -y body axis to point at target (align docking port)
   - Tolerance: pointing error < 1 degree before docking declared
5. Update docking check: require attitude alignment in addition to pos/vel

PHASE 5 TASK: Basilisk end-to-end validation
1. Fix bsk/gnc_bridge.c: the nav propagation in gnc_bridge_step currently
   passes a zero force for propagation. It should store the last command
   and pass it. Add Vec3 last_force field to GncContext struct.
2. Run: make shared && conda activate gnc_docking && 
   python bsk/scenario_docking.py
3. Expected: Docked=YES, pos error < 0.05m (may be slightly different from
   standalone sim due to non-linear dynamics vs CW)
4. If docking fails in BSK but works in standalone:
   - Check ECI-to-LVLH transform in scenario_docking.py eci_to_lvlh()
   - Check force direction sign conventions (LVLH vs ECI rotation)
   - Add diagnostic prints: print LVLH pos/vel every 100 steps
5. Run Monte Carlo through BSK: python bsk/scenario_docking.py --mc 20
6. Compare BSK MC results vs standalone C MC results — they should agree
   within 5% on ΔV and 100% on dock success rate

---

## FORBIDDEN PATTERNS (will be flagged in review)

```c
// FORBIDDEN — recursion
int fact(int n) { return n <= 1 ? 1 : n * fact(n-1); }

// FORBIDDEN — unbounded loop
while (sensor_ready()) { process(); }

// FORBIDDEN — dynamic allocation
float *buf = malloc(n * sizeof(float));

// FORBIDDEN — two levels of dereference
int val = **matrix;

// FORBIDDEN — function pointer
void (*handler)(int) = my_fn;

// FORBIDDEN — unchecked return value
fwrite(data, sizeof(data), 1, fp);   /* return value ignored */

// FORBIDDEN — goto
if (err) goto cleanup;
```

---

## APPROVED PATTERNS

```c
/* GOOD — bounded loop with named constant */
#define MAX_WAYPOINTS  8U
for (uint32_t i = 0U; i < MAX_WAYPOINTS; i++) { ... }

/* GOOD — assertion with recovery */
GNC_ASSERT(state != NULL, ERR_NULL_PTR, return ERR_NULL_PTR);

/* GOOD — single dereference, explicit validity check */
GncStatus rc = nav_update(&nav_state, &meas);
GNC_ASSERT(rc == GNC_OK, rc, return rc);

/* GOOD — static allocation */
static NavState s_nav;   /* zero-initialised at program start */
```

---

## BASILISK INTEGRATION

### What Basilisk Is
Basilisk (BSK) is the AVS Lab / LASP astrodynamics simulation framework
(pip install basilisk). It provides:
- High-fidelity spacecraft dynamics (`spacecraft` module, ECI propagation)
- Sensor models (`simpleNav`, IMU, star tracker, CSS)
- Monte Carlo engine (bit-for-bit repeatable)
- Vizard 3D visualisation (Unity-based)
- Message-passing architecture (peer-to-peer in BSK v2)
- SWIG-generated Python bindings for all C/C++ modules

### Integration Architecture
Our P10 GNC algorithms are compiled as a shared library (`libgnc.so`).
Basilisk is the simulation environment. A thin Python ctypes bridge
connects the two — no changes to the flight algorithm sources.

```
BSK spacecraft (ECI) → LVLH bridge (Python) → GncBridge (ctypes)
                                               → libgnc.so (P10 C)
                     ← ECI force transform  ← force_lvlh output
```

### Key Files
- `bsk/gnc_bridge.h/.c`  — C API, static pool, no malloc after init
- `bsk/gnc_ctypes.py`    — Python ctypes loader, `GncBridge` class
- `bsk/scenario_docking.py` — BSK scenario + Monte Carlo + plots
- `bsk/README_BSK.md`    — integration guide

### Build
```bash
make shared          # → build/libgnc.so
pip install basilisk
python bsk/scenario_docking.py
python bsk/scenario_docking.py --mc 50    # Monte Carlo
```

### Basilisk Module Pattern (BSK v2)
When writing native BSK C++ modules:
```python
# Python scenario
from Basilisk.simulation import myModule
mod = myModule.MyModule()
mod.ModelTag = "gnc_nav"
scSim.AddModelToTask("dynTask", mod)
# Connect messages peer-to-peer
mod.navStateInMsg.subscribeTo(simpleNav.transOutMsg)
```
C++ module skeleton:
```cpp
class MyModule : public SysModel {
public:
    void Reset(uint64_t callTime) override;
    void UpdateState(uint64_t callTime) override;
    Message<NavTransMsgPayload> navStateOutMsg;
    ReadFunctor<NavTransMsgPayload> navStateInMsg;
};
```

### Compliance Note
BSK modules are simulation infrastructure — NOT flight code.
They are NOT required to be NASA P10 compliant.
The GNC algorithm sources in `src/` ALWAYS remain P10 compliant.
