# Autonomous Docking Simulation — Complete

## Build Summary

All 11 phases implemented and verified. Monte Carlo gate PASSED.

## Verification Results

| Metric | Result | Requirement |
|--------|--------|-------------|
| P(dock) | **1.000 (50/50)** | ≥ 0.90 |
| Mean DV | 5.951 m/s | < 20 m/s |
| Mean PosErr | 23.6 mm | < 50 mm |
| Mean Steps | 519 steps (8.7 min) | < 180 min |

## Main Simulation (main_sim.m)

Nominal run docked at ~441 s with pos=0.031 m, vel=0.0004 m/s, att=0.93°, ΔV=5.742 m/s.

## File Structure (30 files)

```
docking_sim/
├── environment/   Rx Ry Rz oe2rv eci_to_lvlh j2_accel drag_accel srp_accel two_body eom_perturbed
├── sensors/       lidar_rae imu_model
├── navigation/    ekf_init ekf_predict ekf_update
├── guidance/      guidance_law
├── control/       pd_controller
├── attitude/      quat_multiply rot2quat quat_kinematics attitude_control rw_model
├── propulsion/    blowdown_model
├── fdir/          fdir_check
├── mission/       mission_manager
├── plots/         plot_all_telemetry.m
├── main_sim.m
├── run_single_sim.m
├── run_monte_carlo.m
└── animate_docking.m
```

## Key Design Decisions

- **EKF navigation**: 6-state (pos+vel) with CW STM, LIDAR RAE updates (σ=max(2mm, 0.3%range))
- **4-phase guidance**: hold→far approach→near approach→terminal, K_V=0.01
- **PD control**: gain-scheduled normal (Kp=0.3/0.2/0.3, Kd=20) vs terminal (Kp=2, Kd=200)
- **Attitude**: quaternion PD with reaction wheels (Kp=0.5, Kd=10, I_body=50 kg⋅m²)
- **FDIR**: 4 codes (STUCK_OPEN, STUCK_CLOSED, DROPOUT, NAV_DIVERGE), suppressed in terminal zone
- **Mission manager**: NOMINAL→HOLD→RETREAT→SAFE with exits on fault-clear

## Critical Bug Fixes Found During Development

1. **Gain vector shape**: Kp/Kd must be column vectors `[;]` not row vectors `[,]` to prevent implicit broadcasting creating 3×3 force matrices.
2. **Attitude integrator instability**: Using `alpha = tau/I_body` (I_body=50) not `tau/(3*I_rw)` (I_rw=0.1); the latter gave α=1.67 rad/s² → 96°/step oscillation.
3. **Attitude reference 180° flip**: When `pos_est` oscillates through the origin (±Y crossing), `dir_tgt = pos_est/|pos_est|` flips 180°. Fixed by locking to `[0;1;0]` for `|pos_est| < 0.10 m`.
4. **FDIR false escalation**: DROPOUT/NAV_DIVERGE suppressed when `|pos_est| < 0.15 m` (terminal zone where LIDAR has limited range).
5. **HOLD override in terminal zone**: Mission override bypassed when `|pos_est| < 1.0 m` to allow terminal guidance to complete.

## Date Completed
2026-04-21
