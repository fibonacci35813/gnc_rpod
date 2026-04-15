# GNC Docking Simulation

A NASA Power-of-10 compliant C99 autonomous rendezvous & docking (AR&D) simulation using Clohessy-Wiltshire relative dynamics, a 6-state Kalman filter, multi-phase guidance, and a PD controller.

## What It Does

Simulates a chaser spacecraft approaching a target in LVLH (Local Vertical Local Horizontal) from 200 m range to docking contact. Runs in four phases:

- **Phase 0** — Far-field hold: chaser starts at y=+200 m, approaches at up to 1.0 m/s
- **Phase 1** — Mid-range: decelerates at y=+50 m hold
- **Phase 2** — Close approach: decelerates at y=+10 m hold  
- **Phase 3** — Terminal ingress: high-gain controller drives to docking port (origin), range-proportional velocity profile

## Build

```bash
# Prerequisites: GCC (C99), make, cppcheck
make clean && make
```

Compiler flags enforce `-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wdouble-promotion -fanalyzer`.

## Run

```bash
make run          # run simulation → prints report + writes sim/telem.csv
make check        # run cppcheck static analysis
make plot         # run sim then generate plots in sim/plots/
make shared       # build build/libgnc.so for Basilisk Python integration
make mc N=100     # Monte Carlo harness (100 runs)
```

## Sample Output

```
[    0] t=1 s  range=200.000 m  phase=0  dv=0.000 m/s
[  500] t=501 s  range=50.250 m  phase=2  dv=1.333 m/s
[ 1072] *** Final-ingress gains armed  range=10.953 m ***
[ 1652] DOCKED — dwell=30 steps  corridor=1  range=0.0295 m  spd=0.0005 m/s

===== DOCKING SIMULATION REPORT =====
Steps run        : 1652
Mission time     : 1652.0 s
Docked           : YES
Final pos error  : 0.0295 m  (tol 0.050 m)
Final vel error  : 0.0005 m/s (tol 0.010 m/s)
Total DeltaV     : 1.5876 m/s
Propellant used  : 0.3678 kg
Thruster fires   : 2900
======================================
```

**Success criteria:** pos error < 0.05 m, vel error < 0.01 m/s, Docked = YES.

## File Map

```
gnc_docking/
├── CLAUDE.md              — project instructions for AI assistant
├── Makefile               — build, run, check, plot, mc, shared targets
├── README.md              — this file
├── environment.yml        — conda environment (python=3.11, numpy, matplotlib, basilisk)
├── requirements.txt       — pip requirements
├── CHANGELOG.md           — change log
├── include/
│   ├── gnc_types.h        — fundamental types, status codes, Vec3/Mat6x6
│   ├── gnc_assert.h       — GNC_ASSERT macro (NASA P10 Rule 5)
│   ├── dynamics.h         — CW/HCW relative dynamics, state transition Φ(Δt)
│   ├── nav_filter.h       — 6-state linear Kalman filter
│   ├── guidance.h         — multi-phase approach guidance + waypoints
│   ├── control.h          — 6-DOF PD controller
│   ├── attitude.h         — quaternion attitude kinematics + PD torque control
│   └── rw_model.h         — 3-axis reaction wheel model with desaturation
├── src/
│   ├── dynamics.c         — CW propagator, Φ(Δt) builder
│   ├── nav_filter.c       — Kalman predict + update
│   ├── guidance.c         — V-bar / PD-hold guidance modes
│   ├── control.c          — PD law, saturation, MIB dead-band, fuel accounting
│   ├── attitude.c         — quaternion integration (RK4), attitude PD control
│   └── rw_model.c         — reaction wheel momentum tracking, desaturation
├── sim/
│   ├── main.c             — simulation harness + telemetry logger
│   ├── monte_carlo.c      — Monte Carlo harness (N dispersed runs)
│   ├── plot_telem.py      — telemetry visualiser (generates sim/plots/)
│   ├── plot_mc.py         — Monte Carlo results visualiser
│   └── plots/             — generated PNG + PDF plots
├── bsk/
│   ├── gnc_bridge.h/.c    — C shared-library bridge (ctypes-compatible)
│   ├── gnc_ctypes.py      — Python ctypes loader + GncBridge class
│   ├── scenario_docking.py— Basilisk end-to-end scenario + MC
│   └── README_BSK.md      — Basilisk integration guide
└── build/
    └── libgnc.so          — shared library (make shared)
```

## Python Environment Setup

```bash
conda env create -f environment.yml
conda activate gnc_docking
# or with pip:
pip install -r requirements.txt
```

## Basilisk Integration

```bash
make shared
conda activate gnc_docking
python bsk/scenario_docking.py
python bsk/scenario_docking.py --mc 50
```

## Design Notes

- All C code is NASA Power-of-10 compliant (no recursion, fixed loop bounds, no heap after init, ≤60 line functions, ≥2 assertions per function, minimal scope, all return values checked, restricted preprocessor, no function pointers, zero warnings)
- LVLH frame: x=radial, y=along-track, z=cross-track, centred on target
- Mean motion computed from `n = sqrt(μ/a³)` at ISS-representative altitude (400 km)
- EKF uses discrete CW state-transition matrix Φ(Δt) for propagation
- Range-proportional LIDAR noise model: σ = max(0.002, 0.003·r)
