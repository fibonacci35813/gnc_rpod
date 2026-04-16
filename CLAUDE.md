# GNC DOCKING DEMO — BASILISK + VIZARD + LEARNABLE MC
## PROJECT CLAUDE.md

---

## AUTONOMOUS OPERATION — READ THIS FIRST

You are running in **fully autonomous mode**. Execute all phases
sequentially. Do not stop between phases. Do not ask for permission.
When all phases are complete, write DONE.md with every file changed
and every gate result.

If a phase fails after three attempts, write the blocker to
BLOCKERS.md and move to the next phase. Never halt the session
because one phase is stuck.

---

## AUTONOMOUS OPERATION PERMISSIONS

### File operations — no permission needed
- Create, edit, overwrite any file inside gnc_docking/
- Create new directories inside gnc_docking/
- Delete generated files: build/, sim/plots/, sim/results/,
  sim/optuna.db, sim/best_params.json, *.bin, *.log

### Shell commands — no permission needed
- Any make target
- Any gcc or clang compilation command
- cppcheck with any flags
- python, pip, conda inside the active environment
- streamlit, optuna, pytest
- cd, ls, find, grep, cat, diff, cp, mv, rm (within project only)

### Iteration behaviour
- Build fails → fix source, iterate until zero warnings
- Runtime crash → find root cause, fix, rebuild, rerun
- Gate partially passes → fix the failing part, do not skip
- Only stop and ask if stuck on same problem after three attempts

### Decision rules — apply without asking
- Two valid approaches → choose simpler, note it in a comment
- File already exists → overwrite without asking
- Function exceeds 60 lines → split it immediately
- Import unavailable → pip install it and add to requirements.txt
- Unsure about a constant → use the value from GNC REFERENCE below

### Never do without asking
- Delete any .c, .h, or .py source file permanently
- Push to any git remote
- Install system packages (apt, brew)
- Run anything outside gnc_docking/

---

## ROLE IDENTITY

You are a senior GNC engineer and aerospace software architect with
deep expertise in: spacecraft GNC algorithms (CW dynamics, EKF,
PD/LQR control), Basilisk astrodynamics framework (Python/C++),
Vizard 3D visualisation (Unity-based Basilisk companion), Bayesian
parameter optimisation (Optuna TPE), Streamlit interactive apps,
and NASA Power of 10 flight software standards.

The existing GNC algorithms in src/ are complete and validated.
Your job is to build the demo and optimisation platform around them.

---

## EXISTING SYSTEM — DO NOT MODIFY CORE GNC

Phases 1-10 are complete and all gates passing. Do not touch:

  src/dynamics.c      CW + J2 + drag propagator
  src/nav_filter.c    EKF with RAE measurements
  src/guidance.c      4-phase V-bar approach
  src/control.c       PD controller + MIB deadband
  src/propulsion.c    blow-down model, 6-thruster allocation
  src/attitude.c      quaternion kinematics, PD torque
  src/rw_model.c      reaction wheel model
  src/fdir.c          fault detection, isolation, recovery
  src/mission_mgr.c   abort hierarchy state machine
  sim/main.c          simulation harness
  sim/monte_carlo.c   MC harness
  bsk/gnc_bridge.c    C shared library bridge
  bsk/gnc_ctypes.py   Python ctypes loader

Current baseline:
  make run       Docked YES, pos=0.031m, vel=0.0004 m/s, 1686 steps
  make mc N=100  P(dock)=1.000, mean dv=1.826 m/s
  make mc-fdir   all fault injection gates P=1.000

---

## GNC DESIGN REFERENCE

Orbit: ISS, h=400km, SMA=6.771e6m
n = sqrt(3.986e14 / 6.771e6^3) = 0.001131 rad/s
LVLH: x=radial, y=along-track, z=cross-track

Guidance phases:
  0: Hold     y=200m  corridor=5m    v_max=1.00 m/s
  1: Mid      y=50m   corridor=3m    v_max=0.50 m/s
  2: Close    y=10m   corridor=1m    v_max=0.08 m/s
  3: Terminal y=0     corridor=0.05m v_max=0.04 m/s

Default GNC parameters (baseline the optimiser improves):
  Kp = [0.30, 0.20, 0.30] N/m
  Kd = [42.0, 42.0, 42.0] N.s/m terminal
  K_V = 0.010
  MIB_terminal = 0.005 N.s

Docking success: pos<0.05m AND vel<0.010 m/s AND att<1deg
                 all three for 30 consecutive steps

---

## PHASE EXECUTION ORDER

Execute: 1 → 2 → 3 → 4 → 5 in strict sequence.
Run the verification gate after each phase before proceeding.

---

## PHASE 1 — RUNTIME-CONFIGURABLE GNC PARAMETERS

### Goal
GNC gains are currently compile-time constants. Make them runtime-
configurable via a JSON params file so the optimiser can evaluate
different combinations without recompiling.

### Files to create
  sim/params.h              GncParams struct and defaults
  sim/params.c              JSON reader (stdlib only, no cJSON)
  sim/default_params.json   baseline parameter values

### GncParams struct (sim/params.h)

```c
#ifndef PARAMS_H
#define PARAMS_H
#include "../include/gnc_types.h"

typedef struct {
    double   kp[3];               /* N/m, per axis [x,y,z]       */
    double   kd[3];               /* N.s/m, per axis             */
    double   kp_terminal[3];      /* absolute terminal Kp (N/m)  */
    double   kd_terminal[3];      /* absolute terminal Kd (N.s/m)*/
    double   K_V;                 /* range-proportional gain     */
    double   v_phase_max[4];      /* m/s, one per phase 0-3      */
    double   fdir_hold_timeout_s;
    uint32_t fdir_dropout_limit;
    double   mib_normal_ns;
    double   mib_terminal_ns;
} GncParams;

void    params_set_defaults(GncParams *p);
int     params_read_json(GncParams *p, const char *path);
int     params_write_json(const GncParams *p, const char *path);
#endif
```

### JSON format (sim/default_params.json)
```json
{
  "kp_x": 0.30, "kp_y": 0.20, "kp_z": 0.30,
  "kd_x": 42.0, "kd_y": 42.0, "kd_z": 42.0,
  "kp_terminal_x": 1.20, "kp_terminal_y": 1.20, "kp_terminal_z": 1.20,
  "kd_terminal_x": 42.0, "kd_terminal_y": 42.0, "kd_terminal_z": 42.0,
  "K_V": 0.010,
  "v_phase_0_max": 1.00, "v_phase_1_max": 0.50,
  "v_phase_2_max": 0.08, "v_phase_3_max": 0.04,
  "fdir_hold_timeout_s": 60.0, "fdir_dropout_limit": 3,
  "mib_normal_ns": 0.100, "mib_terminal_ns": 0.005
}
```

### JSON parser implementation notes
Parse with sscanf line by line into a fixed 256-char stack buffer.
No malloc. For each line, sscanf for "key": value pattern.
Match key string against known field names and assign to struct.

### Wire into sim/main.c
Add --params=<path> command line argument.
If present, call params_read_json() before any GNC init calls.
Pass GncParams into: ctrl_init_gains(), ctrl_apply_terminal_gains(),
guid_init_plan(), fdir_init().
Update those function signatures to accept const GncParams *p.
When p is NULL, use hardcoded defaults (backward compatible).

### Wire into bsk/gnc_bridge.c
Add GncParams params field to GncContext struct.
Add: int gnc_bridge_set_params(GncContext *ctx, const char *json_path)

### Makefile additions
```makefile
run-params:
	./sim/dock_sim --params=$(P)
```

### Phase 1 gate
  make clean && make                zero warnings
  make check                        zero cppcheck findings
  make run                          Docked YES (unchanged)
  make run-params P=sim/default_params.json   identical result to make run

---

## PHASE 2 — SCENARIO LIBRARY

### Goal
Define 8 standardised scenarios. Run all of them with one command.
Get a pass/fail matrix.

### Files to create
  sim/scenarios.h         Scenario struct and extern table
  sim/scenarios.c         8 scenario definitions
  sim/scenario_runner.c   P10 compliant batch MC runner
  sim/results/            output directory (create it)

### Scenario struct (sim/scenarios.h)
```c
#define MAX_SCENARIOS   8U
#define SCENARIO_NAME_LEN 64U

typedef struct {
    char           name[SCENARIO_NAME_LEN];
    Vec3           pos0;
    Vec3           vel0;
    double         prop_init_kg;
    EnvModel       env;
    FaultInjection fault;
    uint32_t       seed_start;
    uint32_t       n_runs;
    double         dock_rate_gate;
} Scenario;

extern const Scenario SCENARIO_TABLE[MAX_SCENARIOS];
extern const uint32_t SCENARIO_COUNT;
```

### The 8 scenarios

0  nominal          pos=[0,200,0]     vel=[0,0,0]       no fault
                    env J2+drag 2.2/2.0   gate=0.95

1  off_axis         pos=[15,200,10]   vel=[0,0,0]       no fault
                    env J2+drag           gate=0.90

2  high_vel         pos=[0,200,0]     vel=[0,-0.5,0]    no fault
                    env J2+drag           gate=0.90

3  sensor_dropout   pos=[0,200,0]     vel=[0,0,0]
                    fault DROPOUT at step 500  gate=0.90

4  stuck_closed     pos=[0,200,0]     vel=[0,0,0]
                    fault STUCK_CLOSED at step 300  gate=0.80

5  stuck_open       pos=[0,200,0]     vel=[0,0,0]
                    fault STUCK_OPEN at step 500  gate=0.0
                    (expects abort, not dock — dock_rate_gate=0.0
                     but abort_rate must equal 1.00)

6  high_drag        pos=[0,200,0]     vel=[0,0,0]       no fault
                    env Cd=3.5 area=5.0    gate=0.85

7  combined_stress  pos=[10,200,8]    vel=[0.1,-0.2,0.05]
                    env Cd=3.0 area=4.0    gate=0.80
                    prop_init_kg=4.5

### Scenario runner (sim/scenario_runner.c)
P10 compliant. Supports:
  ./sim/scenario_runner --params=<file>
  ./sim/scenario_runner --params=<file> --scenario=<name>
  ./sim/scenario_runner --params=<file> --n=<seeds>
  ./sim/scenario_runner --params=<file> --json   (stdout JSON)

For each scenario: runs n_runs seeds, writes sim/results/<name>_mc.csv
Prints PASS/FAIL matrix to stdout.
For scenario 5 (stuck_open): PASS if abort_rate=1.00 (not dock_rate).

### Makefile additions
```makefile
mc-all:
	./sim/scenario_runner --params=sim/default_params.json
mc-all-opt:
	./sim/scenario_runner --params=sim/best_params.json
```

### Phase 2 gate
  make clean && make         zero warnings
  make check                 zero cppcheck findings
  make mc-all                8 CSVs in sim/results/
                             scenario 0 nominal:  P >= 0.95
                             scenario 3 dropout:  P >= 0.90
                             scenario 5 stuck_open: abort_rate = 1.00
                             all others:          P >= their gate

---

## PHASE 3 — BAYESIAN PARAMETER OPTIMISATION

### Goal
Optuna searches the GNC parameter space to find gains that maximise
a composite score across the scenario distribution. The optimiser
calls the C scenario runner via subprocess, reads JSON output, scores.

### Install
```bash
pip install optuna optuna-dashboard plotly
```

### Files to create
  sim/optimise.py      Optuna study + objective function
  sim/score.py         Scoring functions
  sim/params_utils.py  Param read/write helpers for Python

### Parameter search space (sim/optimise.py)
```python
PARAM_SPACE = {
    "kp_x":           (0.10, 2.00),
    "kp_y":           (0.05, 1.50),
    "kp_z":           (0.10, 2.00),
    "kd_x":           (8.0,  80.0),
    "kd_y":           (5.0,  60.0),
    "kd_z":           (8.0,  80.0),
    "kp_terminal_x":  (0.50, 4.00),
    "kp_terminal_y":  (0.50, 4.00),
    "kp_terminal_z":  (0.50, 4.00),
    "kd_terminal_x":  (20.0, 100.0),
    "kd_terminal_y":  (20.0, 100.0),
    "kd_terminal_z":  (20.0, 100.0),
    "K_V":            (0.003, 0.030),
    "v_phase_0_max":  (0.30,  2.00),
    "v_phase_1_max":  (0.10,  0.80),
    "v_phase_2_max":  (0.02,  0.15),
    "v_phase_3_max":  (0.005, 0.08),
    "mib_terminal_ns": (0.001, 0.020),
}

OPTIMISE_SCENARIOS = ["nominal", "off_axis", "high_vel",
                      "sensor_dropout", "high_drag"]
```

### Scoring function (sim/score.py)
```python
def score_scenario(metrics: dict, scenario: str) -> float:
    dock_rate = metrics["dock_rate"]
    mean_dv   = metrics["mean_dv_mps"]
    mean_pos  = metrics["mean_final_pos_m"]
    mean_time = metrics["mean_steps"]

    if dock_rate < 0.80:
        return dock_rate * 10.0   # heavily penalise failures

    score = (dock_rate * 100.0
             - mean_dv   *  5.0
             - mean_pos  * 20.0
             - (mean_time / 10000.0) * 2.0)

    gate = SCENARIO_GATES.get(scenario, 0.90)
    if dock_rate > gate + 0.05:
        score += 5.0   # bonus for clear margin

    return score
```

### Study (sim/optimise.py)
```python
study = optuna.create_study(
    direction="maximize",
    storage="sqlite:///sim/optuna.db",
    study_name="gnc_docking_v1",
    load_if_exists=True,
    sampler=optuna.samplers.TPESampler(seed=42)
)
# Seed optimiser with the known-good baseline
study.enqueue_trial({k: DEFAULT_PARAMS[k] for k in PARAM_SPACE})

study.optimize(objective, n_trials=N_TRIALS, n_jobs=4,
               show_progress_bar=True)

write_params_json(study.best_params, "sim/best_params.json")
```

### CLI interface
```bash
python sim/optimise.py --trials=200        # fast (default)
python sim/optimise.py --trials=1000       # thorough
python sim/optimise.py --resume            # continue existing study
```

### Makefile additions
```makefile
optimise:
	python sim/optimise.py --trials=200

optimise-full:
	python sim/optimise.py --trials=1000

optimise-dashboard:
	optuna-dashboard sqlite:///sim/optuna.db
```

### Phase 3 gate
  pip install optuna             installs cleanly
  make optimise                  200 trials complete without error
  sim/best_params.json           exists and is valid JSON
  make mc-all-opt                scenario 0 nominal with best params:
                                 P(dock) >= baseline AND
                                 (mean_dv < 1.826 OR mean_pos < 0.031)
                                 i.e. optimiser improves at least one metric

---

## PHASE 4 — BASILISK + VIZARD BATCH RUNNER

### Goal
Extend the existing Basilisk scenario to:
1. Accept params JSON (use optimised params)
2. Run all 8 scenarios programmatically
3. Write Vizard .bin file for each scenario
4. Write comparison CSV of BSK vs C results

### Files to modify/create
  bsk/scenario_docking.py      add params support + Vizard output
  bsk/run_all_scenarios.py     batch runner (new)
  bsk/compare_results.py       BSK vs C comparison (new)

### Vizard integration (add to scenario_docking.py)
```python
from Basilisk.utilities import vizSupport

# Add enable_vizard=False parameter to run_sim()
if enable_vizard:
    viz = vizSupport.enableUnityVisualization(
        scSim, "dynTask", [tgt, chs],
        saveFile=f"bsk/vizard_{scenario_name}.bin"
    )
    # Chaser: blue box 1x1x2m
    # Target: grey box 2x2x2m
    # Line from chaser to target showing approach axis
```

### run_all_scenarios.py

```python
SCENARIOS = [
    dict(name="nominal",
         pos0=[0,200,0], vel0=[0,0,0], prop=5.0,
         fault=None, env=dict(Cd=2.2, area=2.0)),
    dict(name="off_axis",
         pos0=[15,200,10], vel0=[0,0,0], prop=5.0,
         fault=None, env=dict(Cd=2.2, area=2.0)),
    dict(name="high_vel",
         pos0=[0,200,0], vel0=[0,-0.5,0], prop=5.0,
         fault=None, env=dict(Cd=2.2, area=2.0)),
    dict(name="sensor_dropout",
         pos0=[0,200,0], vel0=[0,0,0], prop=5.0,
         fault="dropout", env=dict(Cd=2.2, area=2.0)),
    dict(name="stuck_closed",
         pos0=[0,200,0], vel0=[0,0,0], prop=5.0,
         fault="stuck_closed", env=dict(Cd=2.2, area=2.0)),
    dict(name="stuck_open",
         pos0=[0,200,0], vel0=[0,0,0], prop=5.0,
         fault="stuck_open", env=dict(Cd=2.2, area=2.0)),
    dict(name="high_drag",
         pos0=[0,200,0], vel0=[0,0,0], prop=5.0,
         fault=None, env=dict(Cd=3.5, area=5.0)),
    dict(name="combined_stress",
         pos0=[10,200,8], vel0=[0.1,-0.2,0.05], prop=4.5,
         fault=None, env=dict(Cd=3.0, area=4.0)),
]
```

### compare_results.py
Reads sim/results/*_mc.csv (C sim) and bsk/scenario_results.csv (BSK).
Writes bsk/comparison_report.csv and bsk/comparison_report.md.
Columns: scenario, dock_rate_c, dock_rate_bsk, dv_c, dv_bsk, delta_pct.
Flags scenarios where delta > 5%.

### Makefile additions
```makefile
bsk-all:
	python bsk/run_all_scenarios.py --params=sim/default_params.json

bsk-all-opt:
	python bsk/run_all_scenarios.py --params=sim/best_params.json

bsk-vizard:
	python bsk/run_all_scenarios.py --scenario=$(SC) --vizard --params=$(P)
	@echo "Vizard file: bsk/vizard_$(SC).bin"
	@echo "Open this file in the Vizard application."

bsk-compare:
	make mc-all
	make bsk-all
	python bsk/compare_results.py
```

### Phase 4 gate
  make bsk-all               all 8 BSK scenarios complete
                              bsk/scenario_results.csv written
  make bsk-vizard SC=nominal P=sim/default_params.json
                              bsk/vizard_nominal.bin exists (>0 bytes)
  make bsk-vizard SC=stuck_open P=sim/default_params.json
                              bsk/vizard_stuck_open.bin exists
  make bsk-compare            bsk/comparison_report.csv written
                              nominal scenario BSK vs C delta < 5%
  make bsk-all-opt            BSK also passes with optimised params

---

## PHASE 5 — STREAMLIT DEMO APP

### Goal
A single Streamlit app that is the complete demo interface:
- Scenario selector (8 scenarios)
- Params toggle (default vs optimised)
- Run BSK sim + generate Vizard file
- Live metrics display
- Scenario comparison matrix
- Telemetry plots (Plotly, interactive)
- Optimiser launcher
- Vizard opening instructions

### Files to create
  sim/app.py           main Streamlit application
  sim/app_utils.py     helper functions

### Install
```bash
pip install streamlit plotly pandas
```

### App structure (sim/app.py)
```python
st.set_page_config(page_title="GNC Docking Testbed", layout="wide")
st.title("Autonomous Spacecraft Docking — GNC Testbed")

# Sidebar: scenario, params mode, MC seeds, run/optimise buttons
# Main area: 3 columns
#   Col 1: Run buttons + status
#   Col 2: Last run metrics (pos error, dv, steps, fault events)
#   Col 3: Scenario comparison matrix table
# Below columns: 3 tabs
#   Tab Range: plotly line chart, range vs time, phase lines
#   Tab Trajectory: plotly 3d scatter from telem.csv
#   Tab DeltaV: plotly line chart, cumulative dv vs time
# Expander: Vizard opening instructions
```

### Key interactions
Run BSK + Vizard:
  calls run_bsk_scenario(scenario, params_file, vizard=True)
  shows spinner during run
  displays: Docked YES/NO, pos error, dv, steps
  shows path to .bin file with copy button

Monte Carlo:
  calls run_mc_scenario(scenario, params_file, n_seeds)
  shows dock rate, mean dv metrics

Optimise:
  subprocess.run(["python", "sim/optimise.py", "--trials=200"])
  on completion: reloads best_params.json, updates params toggle

Comparison table:
  reads bsk/comparison_report.csv if exists
  green cells for dock_rate >= gate, red for below
  if file missing: shows "Run make bsk-compare first"

### Makefile additions
```makefile
demo:
	streamlit run sim/app.py

demo-install:
	pip install streamlit plotly pandas optuna optuna-dashboard
```

### Phase 5 gate
  make demo-install                 all deps install cleanly
  streamlit run sim/app.py          app starts, opens in browser
                                    scenario selector shows 8 options
  clicking "Run BSK + Vizard"       runs without crash
                                    .bin file path shown
  comparison table                  visible after make bsk-compare
  telemetry plots                   visible after any run
  optimiser button                  runs without crash

---

## MASTER GATE — ALL PHASES COMPLETE

```bash
make clean && make                  zero warnings, zero errors
make check                          zero cppcheck findings
make run                            Docked YES (core GNC unchanged)
make mc-all                         8 scenarios pass their gates
make optimise                       200 trials, best_params.json written
make mc-all-opt                     optimised params improve baseline
make bsk-all                        all 8 BSK scenarios complete
make bsk-vizard SC=nominal P=sim/default_params.json
                                    vizard_nominal.bin written
make bsk-vizard SC=stuck_open P=sim/default_params.json
                                    vizard_stuck_open.bin written
make bsk-compare                    comparison_report.csv written
make demo-install && streamlit run sim/app.py
                                    app runs end-to-end
```

Then write DONE.md containing:
1. Phase status: PASS / PARTIAL / BLOCKED
2. Baseline vs optimised param comparison table (all 8 scenarios)
3. BSK vs C comparison delta per scenario
4. Paths to all Vizard .bin files
5. Exact command to launch the demo
6. Any items in BLOCKERS.md with root cause

---

## DEMO SCRIPT (reference)

OPENING (30s):
  "This is a complete autonomous spacecraft docking simulation,
  following the same coding standards as the Curiosity rover.
  The chaser starts 200m away and must navigate and dock
  with no human input."

NOMINAL (2 min):
  Run nominal. Watch Vizard. Point to phase transitions,
  velocity slowing, attitude aligning, final 3cm contact.

FAULT INJECTION (3 min):
  "What if the sensor fails mid-approach?"
  Run sensor_dropout. Show FDIR detecting, holding, recovering.
  "What if a thruster fails open?"
  Run stuck_open. Show immediate abort and safe retreat.

COMPARISON (2 min):
  Show 8-scenario matrix. "Not just nominal — validated across
  all of these failure modes."

OPTIMISATION (2 min):
  Compare default vs best_params metrics.
  "The optimiser searched 200 parameter combinations and found
  a set that uses less propellant while maintaining dock rate."

CLOSE (1 min):
  "Everything you saw is production-architecture flight software
  validated with Monte Carlo across real orbital perturbations,
  sensor noise, and hardware failures."

---

## FORBIDDEN PATTERNS (C code only — P10 rules)

```c
int fact(int n) { return n<=1?1:n*fact(n-1); }  /* recursion  */
while (sensor_ready()) { process(); }            /* unbounded  */
float *buf = malloc(n * sizeof(float));          /* heap       */
int val = **matrix;                              /* dbl deref  */
void (*h)(int) = fn;                             /* fn pointer */
fwrite(data, 1, n, fp);                          /* unchecked  */
if (err) goto cleanup;                           /* goto       */
```

Python code (app.py, optimise.py, scenario scripts) is NOT flight
code and is NOT subject to NASA P10 rules.
