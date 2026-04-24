# RPOD Kernel Metrics and Scenario Gates

## Purpose

This document defines the evaluation framework for the closed-loop RPOD kernel.
It answers one question:

**How do we know whether the simulator, algorithms, and mission logic are good enough to build on?**

The goal of this file is to prevent uncontrolled feature growth and to ensure that every algorithm, simulator upgrade, and integration step is judged against the same mission-relevant criteria.

This document applies to:
- deterministic nominal runs
- Monte Carlo campaigns
- fault-injection campaigns
- future 6-DOF upgrades
- future PIL and HIL validation

---

## Evaluation Philosophy

The kernel is not judged by whether it "runs."
The kernel is judged by whether it can repeatedly satisfy mission-relevant RPOD constraints.

Every major change should be evaluated against:
1. safety
2. docking success
3. terminal precision
4. propellant efficiency
5. robustness to uncertainty and faults
6. repeatability across scenario packs

A new algorithm or simulator feature is only better if it improves one or more of these without causing unacceptable regression elsewhere.

---

## Mission Context

Reference mission:
- cooperative chaser-target RPOD demonstration
- circular LEO scenario
- CW/HCW relative dynamics baseline
- docking along a predefined approach corridor
- phased approach with hold points
- terminal docking with low closing velocity

The evaluation framework is intentionally mission-shaped rather than generic.

---

## Primary Success Metrics

### 1. Docking Success

**Definition**
A run is counted as a successful dock if the vehicle reaches docking conditions and satisfies the terminal dwell logic.

**Current pass condition**
- Docked = YES
- terminal dwell achieved

**Why it matters**
This is the top-level mission outcome metric.

---

### 2. Final Position Error

**Definition**
Euclidean norm of terminal relative position error at docking confirmation.

**Symbol**
`e_pos_final`

**Formula**
`e_pos_final = ||r_final||`

**Current gate**
- `e_pos_final < 0.05 m`

**Stretch gate**
- `e_pos_final < 0.02 m`

**Why it matters**
Physical docking is meaningless if final geometric error is too high for interface capture.

---

### 3. Final Relative Velocity Error

**Definition**
Euclidean norm of terminal relative velocity at docking confirmation.

**Symbol**
`e_vel_final`

**Formula**
`e_vel_final = ||v_final||`

**Current gate**
- `e_vel_final < 0.01 m/s`

**Stretch gate**
- `e_vel_final < 0.005 m/s`

**Why it matters**
Docking success must occur at low impact speed.

---

### 4. Total Delta-V

**Definition**
Integrated delta-v consumed during the run.

**Symbol**
`dv_total`

**Current nominal reference**
- baseline nominal run approximately `1.43 m/s`

**Initial monitoring rule**
- track as a comparison metric for all algorithm changes
- no regression > 15% in nominal case unless justified by robustness gain

**Why it matters**
The mission must remain physically realistic and propellant-aware.

---

### 5. Mission Time to Dock

**Definition**
Elapsed time from simulation start to confirmed docking.

**Symbol**
`t_dock`

**Initial monitoring rule**
- track for all runs
- no regression > 20% in nominal case unless justified by safety improvement

**Why it matters**
Very slow convergence can indicate poor guidance or over-conservative control.

---

### 6. Hold-Point Compliance

**Definition**
Whether the vehicle reaches each hold region with acceptable position and speed before advancing.

**Measurements**
- count of hold-point violations
- count of premature phase advances
- corridor excursions during approach

**Current gate**
- zero hold-point violations in nominal deterministic run

**Why it matters**
A docking mission should not be judged only by final contact. The approach behavior must also be disciplined.

---

### 7. Abort / Retreat Success

**Definition**
Whether the system safely aborts or retreats under designated fault scenarios.

**Current gate**
- required for fault scenarios designed to trigger safe abort

**Why it matters**
A mission-quality RPOD kernel is not only a docking kernel. It must also fail safely.

---

## Secondary Diagnostic Metrics

These metrics do not define mission success by themselves, but must be tracked.

### Navigation Metrics
- RMS position estimation error
- RMS velocity estimation error
- maximum terminal navigation error
- filter convergence time
- covariance consistency trends

### Control Metrics
- thruster fire count
- command saturation count
- minimum impulse bit activity rate
- attitude error during terminal ingress
- control duty cycle / chatter behavior

### Robustness Metrics
- success rate across Monte Carlo seeds
- sensitivity to initial off-axis error
- sensitivity to measurement dropout
- sensitivity to velocity dispersion
- sensitivity to injected disturbances

### Software / Validation Metrics
- deterministic reproducibility by seed
- build success with zero warnings
- static analysis pass status
- parameter file load success
- scenario runner reproducibility

---

## Deterministic Baseline Gates

The deterministic nominal run is the first gate for any structural change.

### Nominal deterministic pass criteria
- build succeeds with zero warnings
- simulation completes without crash or assertion failure
- Docked = YES
- final position error < 0.05 m
- final velocity error < 0.01 m/s
- zero hold-point violations
- no unexpected abort

### Deterministic regression rule
If a code change breaks the deterministic nominal case, it should not proceed to Monte Carlo evaluation until fixed.

---

## Scenario Library

Every algorithm or simulator change should be evaluated on the same standard scenario pack.

### Scenario 1. Nominal
Purpose: baseline reference case.

### Scenario 2. Off-Axis Initial Condition
Purpose: test lateral/cross-track convergence and corridor discipline.

### Scenario 3. High Initial Relative Velocity
Purpose: test guidance and control authority under more aggressive approach conditions.

### Scenario 4. Sensor Dropout
Purpose: test navigation robustness and estimator resilience.

### Scenario 5. Stuck-Open / Abort Case
Purpose: test safe abort logic and mission manager response.

### Scenario 6. High Drag / Disturbance Case
Purpose: test robustness to model mismatch and disturbance effects.

### Scenario 7. Combined Stress Case
Purpose: test system behavior when multiple dispersions stack together.

### Scenario 8. Retreat Verification
Purpose: confirm that retreat logic is stable and safe when docking is not permitted.

---

## Monte Carlo Gates

Monte Carlo is used to judge robustness rather than best-case behavior.

### Initial Monte Carlo campaign size
- development gate: `N = 20`
- intermediate gate: `N = 100`
- major validation gate: `N >= 500`

### Initial docking-rate gates
These are the first working thresholds and can be tightened later.

- nominal: `dock_rate >= 0.95`
- off-axis: `dock_rate >= 0.90`
- high-velocity: `dock_rate >= 0.90`
- sensor-dropout: `dock_rate >= 0.90`
- high-drag: `dock_rate >= 0.85`
- combined-stress: `dock_rate >= 0.80`
- stuck-open / forced-abort case: `abort_rate = 1.00`
- retreat verification: `safe_retreat_rate = 1.00`

### Monte Carlo regression rule
A new algorithm may only replace the baseline if:
- it meets all mandatory safety gates
- it does not materially reduce docking rate in any core scenario
- any increase in delta-v or time-to-dock is justified by improved robustness or safety

---

## Fault-Injection Gates

Fault scenarios should be judged on safety before performance.

### Required behavior under designated abort faults
- abort is triggered when intended
- retreat remains dynamically stable
- no false docking declaration occurs
- no corridor violation persists without safing action

### Mandatory fault checks
- stuck-open / actuator fault proxy
- measurement dropout
- off-axis state estimate degradation
- guidance refusal / mission manager retreat command

---

## Acceptance Rules for Algorithm Comparison

When comparing guidance, navigation, or control variants, every candidate must be judged on the same scenario pack and seed policy.

### Candidate comparison scorecard
Each candidate should be summarized with:
- nominal pass or fail
- dock_rate by scenario
- abort_rate by scenario
- mean final position error
- mean final velocity error
- mean delta-v
- mean time-to-dock
- notes on failure modes

### Decision rule
A candidate is accepted only if it is:
- at least as safe as baseline
- no worse than baseline on mandatory gates
- meaningfully better on one of:
  - docking rate
  - terminal precision
  - delta-v
  - robustness
  - failure containment

---

## Versioned Evaluation Levels

### Level 0 — Kernel Sanity
- clean build
- deterministic nominal docking
- telemetry logging
- no crash/assertion failure

### Level 1 — Scenario Robustness
- standard scenario pack active
- Monte Carlo supported
- docking-rate gates tracked

### Level 2 — Safety and FDIR
- abort / retreat logic validated
- fault-injection scenario pack active
- mission manager responses measurable

### Level 3 — 6-DOF Upgrade Readiness
- attitude and translational coupling tracked
- sensor-frame effects included
- docking-axis alignment explicitly measured

### Level 4 — PIL Readiness
- loop timing measurable
- parameter loading stable
- repeatable embedded execution possible

### Level 5 — HIL Readiness
- command/sensor interfaces stable
- scenario framework fixed
- pass/fail logic independent of visualization layer

---

## Reporting Format

Every major run or algorithm comparison should produce a compact report with:
- commit hash
- branch name
- scenario name
- seed count
- pass/fail result
- dock_rate
- abort_rate
- mean and worst-case terminal errors
- mean delta-v
- mean time-to-dock
- notes on violations or anomalies

This can later become a machine-generated report format.

---

## Immediate Next Implementation Tasks

1. Ensure `docs/metrics.md` is treated as the source of truth for evaluation.
2. Tie scenario runner outputs directly to these metric names.
3. Ensure deterministic nominal run remains the first gate.
4. Standardize Monte Carlo output fields across all scenarios.
5. Add explicit reporting for:
   - hold-point violations
   - abort cause
   - terminal corridor compliance
   - navigation error summary
6. Use these gates before introducing further algorithm complexity.

---

## Current Baseline Snapshot

Current observed nominal deterministic behavior from the salvaged kernel:
- Docked = YES
- final position error ≈ `0.0312 m`
- final velocity error ≈ `0.0004 m/s`
- total delta-v ≈ `1.4280 m/s`
- mission time ≈ `1686 s`

This acts as the first baseline reference until replaced by a newer accepted baseline.

---

## Final Principle

The simulator should not grow by feature accumulation.
It should grow by **validated capability layers**.

Every new algorithm, disturbance model, sensor model, or integration layer must answer:

**Does it improve validated RPOD capability under this evaluation framework?**

If not, it is not yet progress.
