# CHANGELOG — GNC Docking Simulation

---

## [v1.1.0] — Phase 6: FDIR  ✅ PASSING

**Build:** Zero warnings (`-Werror` enforced)
**Static analysis:** `cppcheck --enable=all` → zero findings
**Standalone sim result:** Docked YES, pos=0.0295 m, vel=0.0005 m/s, ΔV=1.588 m/s
**Nominal MC (100 runs):** P(dock)=1.000
**FDIR MC stuck_open:**    P(abort ≤ 10 steps)=1.000
**FDIR MC dropout:**       P(dock)=1.000 ≥ 0.90
**FDIR MC stuck_closed:**  P(dock)=1.000 ≥ 0.80

### Phase 6 — FDIR (Fault Detection, Isolation and Recovery)
- Created `include/fdir.h`: FaultCode/MissionMode enums, FaultState struct, full API
- Created `src/fdir.c`: 7 P10-compliant functions
  - `fdir_init`, `fdir_check_thruster`, `fdir_check_sensor`, `fdir_check_nav`
  - `fdir_check_attitude`, `fdir_get_mode`, `fdir_update`
  - stuck_open detection is latched; non-latched faults auto-clear after HOLD timeout
- Wired FDIR into `sim/main.c`: checks run every step; `mission_mode`/`fault_code` added to telem.csv
- Created `sim/mc_fdir.c`: fault-injection MC harness
  - `stuck_open`: inject 0.5 N uncmd'd force on z-axis → 100/100 abort within 10 steps
  - `dropout`: 3 consecutive invalid sensor steps → 100/100 recovery+dock
  - `stuck_closed`: zero actual force for 50 steps → 100/100 dock after recovery
- Added `plot_fdir_timeline()` to `sim/plot_telem.py` (6th plot)
- Added `mc-fdir` Makefile target

---

## [v1.0.0] — All Phases Complete  ✅ PASSING

**Build:** Zero warnings (`-Werror` enforced)
**Static analysis:** `cppcheck --enable=all` → zero findings
**Standalone sim result:** Docked YES, pos=0.0295 m, vel=0.0005 m/s, ΔV=1.588 m/s
**C MC (100 runs):** P(dock)=1.000, ΔV mean=1.656 m/s
**BSK MC (20 runs):** P(dock)=1.000, ΔV mean=1.692 m/s (2.2% delta vs C — within 5% gate)

### Phase 1 — Baseline stabilisation
- `make clean && make && make run` → Docked: YES, pos < 0.05 m
- `make check` → zero cppcheck findings
- Created `environment.yml`, `requirements.txt`, `README.md`
- Fixed unmatched `--suppress=checkersReport` in Makefile (cppcheck 2.7 compat)

### Phase 2 — Telemetry visualisation
- Created `sim/plot_telem.py` (Agg backend, headless)
  - 5 plots: range_vs_time, trajectory_lvlh, nav_error, propulsion, dv_budget
  - Phase transition vertical lines inferred from range thresholds
  - PDF + PNG output to sim/plots/
- Added `make plot` target (run sim → plot)

### Phase 3 — Monte Carlo harness
- Created `sim/monte_carlo.c` (P10 compliant, MAX_RUNS=2000 fixed bound)
  - Dispersions: pos0 ± N(0,3m), vel0 ± N(0,0.05 m/s), thr_scale U(±5%)
  - Deterministic LCG with per-run seed to decorrelate runs
  - Writes sim/mc_results.csv; prints pass/fail gate at P(dock) ≥ 0.95
- Created `sim/plot_mc.py` (scatter, step histogram, ΔV box plot)
- Added `make mc N=100` target with pass/fail gate

### Phase 4 — Attitude control
- Extended `gnc_types.h`: `AttState` (quaternion + omega), `AttCmd` (torque), `GNC_MAX_RW=3`
- Created `include/attitude.h` + `src/attitude.c`:
  - `att_init()`, `att_kinematics()` (RK4), `att_error()`, `att_pd_control()`
  - `att_docking_cmd()`: aligns body -y to point toward target for Phase 3
  - `att_check_aligned()`: 1-degree tolerance check
  - **Symplectic Euler** integration order (q first, ω second) for stability at dt=1s
    — "ω-first" order has eigenvalue |λ|=1.28 > 1 (unstable); symplectic gives |λ|=0.71
- Created `include/rw_model.h` + `src/rw_model.c`:
  - 3-axis reaction wheels, ±0.1 N·m·s saturation, desaturation bleed
- Updated `sim/main.c`: coast in phases 0-2, active pointing in phase 3
  - Docking check now requires: pos ∧ vel ∧ attitude alignment
  - Phase 3 att_err < 1 degree at docking (0.504 deg at contact)

### Phase 5 — Basilisk end-to-end validation
- Fixed `bsk/gnc_bridge.c`: added `Vec3 last_force` to `GncContext`; nav propagation
  now passes the last commanded force instead of zero — improves Kalman consistency
- Added `gnc_bridge_free()` API + Python binding + `__del__` in `GncBridge` to
  return pool slots after each MC run (was exhausting 4-slot pool after 4 seeds)
- BSK single run: Docked=YES, range=3.1 cm, ΔV=1.620 m/s
- BSK MC 20 seeds: 20/20 docked, mean ΔV=1.692 m/s
- C vs BSK: ΔV difference 2.2% (< 5% gate ✓), dock rate 100% both (✓)

---

## [v0.5.0] — Final Release  ✅ PASSING

**Build:** Zero warnings (`-Werror` enforced)
**Static analysis:** `cppcheck --enable=all` → zero findings
**Result:**

```
Steps run        : 1652       (27 min simulated mission time)
Docked           : YES
Final pos error  : 0.0295 m   (tolerance: 0.050 m) — 41% margin
Final vel error  : 0.0005 m/s (tolerance: 0.010 m/s)
Total DeltaV     : 1.588 m/s
Propellant used  : 0.368 kg
Thruster fires   : 2900
```

**Changes from v0.4:**
- Removed intermediate y=0.20m waypoint (v0.4 phase 4). Root cause of
  oscillation: 8 cm corridor but 60 cm braking distance at 0.04 m/s.
  Physically impossible to satisfy -> perpetual oscillation at ~0.3 m.
- Reverted to clean 4-phase plan. GUID_K_V=0.010 profile provides
  natural geometric deceleration: 4m -> 0.04 m/s, 0.1m -> 0.001 m/s.
- Lowered GUID_V_CREEP from 0.002 -> 0.001 m/s.
  Entry speed into 5cm corridor ~0.001 m/s -> braking distance ~12mm.
- Removed dead GUID_V_FINAL_MAX constant.

---

## [v0.4.0] — Dwell-gated docking + API wiring

**Result:** Docked YES, pos=0.0325m, vel=0.0005 m/s, steps=6509
(Phase 3 too slow — ultra-slow 0.002 m/s from 10m. Fixed in v0.5.)

**Changes from v0.3:**
- SIM_DOCK_DWELL_STEPS=30: both pos+vel within tolerance for 30
  consecutive steps before declaring docked.
- Wired ctrl_vec3_norm() and guid_corridor_check() into docking check.
- Approach velocities: FAR_MAX 0.30->1.0, MID_MAX 0.15->0.50, K_V 0.003->0.010.
- GUID_V_TERM_MAX set to 0.002 m/s to eliminate terminal overshoot.

---

## [v0.3.0] — Static analysis clean

**Result:** Docked YES, pos=0.049m (barely inside), steps=4825
**Build:** Zero warnings. cppcheck: Zero findings.

**Changes from v0.2:**
- Removed dead KH matrix block in nav_update (leftover stub).
- Loop counter declarations moved inside for-loop scopes (Rule 6).
- Removed redundant range_to_wp double-computation in guidance.c.
- Added -Werror to Makefile.

---

## [v0.2.0] — Terminal refinement

**Result:** Docked YES (true state), pos=0.049m, steps=4825

**Changes from v0.1:**
- Sensor model: flat 0.5m -> range-proportional sigma(r)=max(0.002,0.003r).
- ctrl_apply_terminal_gains(): Kp x4, Kd x3 for overdamped final phase.
- ControlCmd.prop_step_kg field: clean per-step propellant burn-down.
- Docking check moved to TRUE state (was using Kalman estimate).

---

## [v0.1.0] — Initial draft

**Result:** Docked YES (nav estimate), true pos error 0.40m >> tolerance.
**Build:** Zero warnings on first compile.

Issues: flat sensor noise >> tolerance; docking on estimate not true state;
no terminal gain tightening; propellant tracking bug.

---

## NASA Power of 10 Final Compliance

Rule 1  No goto/setjmp/recursion              PASS
Rule 2  All loops have fixed named bounds      PASS
Rule 3  No malloc/free after init              PASS
Rule 4  All functions <= 60 lines             PASS
Rule 5  >= 2 GNC_ASSERT per function          PASS
Rule 6  Variables at smallest scope           PASS
Rule 7  All return values checked             PASS
Rule 8  Preprocessor: includes + macros only  PASS
Rule 9  Max 1 pointer deref; no fn ptrs       PASS
Rule 10 -Werror, zero warnings, cppcheck      PASS
