# CHANGELOG — GNC Docking Simulation

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
