# RPOD Kernel Metrics and Scenario Gates

## 1. Purpose
- Why this file exists
- What problem it solves
- Why evaluation must be standardized before adding complexity

## 2. Scope
- What this document applies to
- What it does not cover yet
- Current simulator level it governs

## 3. Mission Context
- Reference mission being evaluated
- Chaser-target architecture
- Orbit / dynamics assumptions
- Cooperative docking assumptions
- Current docking concept

## 4. Evaluation Philosophy
- What “good” means in this repo
- Why passing one nominal run is not enough
- Safety first, then success, then efficiency
- Rule: features are only progress if they improve validated capability

## 5. Baseline System Under Evaluation
- Current dynamics model
- Current sensor model
- Current estimator
- Current guidance logic
- Current controller
- Current FDIR / mission manager assumptions
- Current attitude / docking assumptions

## 6. Primary Success Metrics
### 6.1 Docking Success
- Definition
- Pass condition
- Why it matters

### 6.2 Final Position Error
- Definition
- Formula / variable
- Threshold
- Stretch threshold
- Why it matters

### 6.3 Final Relative Velocity Error
- Definition
- Formula / variable
- Threshold
- Stretch threshold
- Why it matters

### 6.4 Total Delta-V
- Definition
- How measured
- Baseline reference
- Regression rule
- Why it matters

### 6.5 Time to Dock
- Definition
- How measured
- Regression rule
- Why it matters

### 6.6 Hold-Point Compliance
- Definition
- Violations tracked
- Pass threshold
- Why it matters

### 6.7 Corridor Compliance
- Definition
- What counts as excursion
- How reported
- Why it matters

### 6.8 Abort / Retreat Success
- Definition
- Pass condition
- Why it matters

## 7. Secondary Diagnostic Metrics
### 7.1 Navigation Metrics
- RMS position estimation error
- RMS velocity estimation error
- Max terminal nav error
- Filter convergence behavior
- Covariance consistency

### 7.2 Control Metrics
- Thruster fire count
- Saturation count
- MIB activity / chatter
- Control effort
- Duty cycle

### 7.3 Attitude Metrics
- Terminal attitude error
- Max attitude error during ingress
- Docking-axis alignment error

### 7.4 Robustness Metrics
- Success rate over seeds
- Sensitivity to off-axis states
- Sensitivity to dropout
- Sensitivity to velocity dispersion
- Sensitivity to disturbances

### 7.5 Software / Validation Metrics
- Clean build
- Zero warnings
- Static analysis status
- Deterministic reproducibility
- Parameter loading success

## 8. Deterministic Baseline Gates
### 8.1 Nominal Deterministic Gate
- Exact pass/fail conditions

### 8.2 Deterministic Regression Rule
- What must not regress before MC is allowed

## 9. Scenario Library
### 9.1 Nominal
- Purpose
- Key conditions

### 9.2 Off-Axis Initial Condition
- Purpose
- Key stress introduced

### 9.3 High Initial Relative Velocity
- Purpose
- Key stress introduced

### 9.4 Sensor Dropout
- Purpose
- Key stress introduced

### 9.5 Stuck-Open / Abort Case
- Purpose
- Key stress introduced

### 9.6 High Drag / Disturbance Case
- Purpose
- Key stress introduced

### 9.7 Combined Stress Case
- Purpose
- Key stress introduced

### 9.8 Retreat Verification
- Purpose
- Key stress introduced

## 10. Seed Policy
- How random seeds are chosen
- Whether seeds are fixed or versioned
- How fairness across algorithm comparisons is maintained

## 11. Monte Carlo Gates
### 11.1 Development Campaign Size
- N=20 or similar

### 11.2 Intermediate Campaign Size
- N=100 or similar

### 11.3 Validation Campaign Size
- N>=500 or similar

### 11.4 Scenario-Specific Docking Rate Gates
- Nominal
- Off-axis
- High velocity
- Dropout
- High drag
- Combined stress

### 11.5 Abort-Rate Gates
- Stuck-open / forced abort
- Retreat verification

### 11.6 Monte Carlo Regression Rules
- When a new algorithm is acceptable
- When it must be rejected

## 12. Fault-Injection Gates
### 12.1 Safety-First Principle
- Safety before docking performance

### 12.2 Required Fault Responses
- Abort trigger
- Retreat stability
- No false dock declaration
- No persistent unsafe corridor violation

### 12.3 Mandatory Fault Cases
- Actuator fault proxy
- Sensor dropout
- Estimate degradation
- Mission-manager forced retreat

## 13. Algorithm Comparison Rules
### 13.1 Comparison Protocol
- Same scenarios
- Same seeds
- Same reporting format

### 13.2 Scorecard Fields
- Pass/fail
- Dock rate
- Abort rate
- Mean final position error
- Mean final velocity error
- Mean delta-v
- Mean time to dock
- Notes on failure modes

### 13.3 Acceptance Rule
- What conditions must be met for a new algorithm to replace baseline

## 14. Reporting Format
### 14.1 Required Metadata
- Commit hash
- Branch name
- Scenario name
- Parameter file
- Seed count

### 14.2 Required Result Fields
- Pass/fail
- Dock rate
- Abort rate
- Final errors
- Delta-v
- Time to dock
- Violations

### 14.3 Failure Logging
- Why runs failed
- How failures should be categorized

## 15. Versioned Capability Levels
### 15.1 Level 0 — Kernel Sanity
### 15.2 Level 1 — Scenario Robustness
### 15.3 Level 2 — FDIR and Safe Abort
### 15.4 Level 3 — 6-DOF Readiness
### 15.5 Level 4 — PIL Readiness
### 15.6 Level 5 — HIL Readiness

## 16. Current Accepted Baseline
- Current accepted deterministic baseline numbers
- Date / commit reference
- Notes on what this baseline includes

## 17. Known Gaps
- Metrics not yet emitted automatically
- Missing scenario support
- Missing nav / control diagnostics
- Gaps before 6-DOF / PIL / HIL use

## 18. Immediate Next Tasks
- What must be implemented next so this document becomes executable rather than aspirational

## 19. Change Control for This File
- When thresholds may be changed
- Who/what justifies changing gates
- Rule against casually relaxing standards