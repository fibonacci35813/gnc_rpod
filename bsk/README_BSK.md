# Basilisk Integration Guide

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  BASILISK LAYER  (simulation environment)                        │
│                                                                  │
│  ┌────────────┐    ECI state    ┌──────────────┐                │
│  │   target   │ ──────────────► │  LVLH bridge │               │
│  │ spacecraft │                 │  (Python)    │               │
│  └────────────┘                 └──────┬───────┘               │
│  ┌────────────┐    ECI state           │ LVLH pos/vel           │
│  │  chaser    │ ──────────────►        ▼                        │
│  │ spacecraft │◄── ECI force ── ┌──────────────┐               │
│  └────────────┘                 │  GncBridge   │               │
│   extForceTorque                │  (ctypes)    │               │
│                                 └──────┬───────┘               │
└─────────────────────────────────┬──────┘────────────────────────┘
                                  │ C API calls
              ┌───────────────────▼──────────────────────┐
              │  libgnc.so  (our NASA P10-compliant GNC)  │
              │                                           │
              │  nav_filter.c  →  Kalman filter           │
              │  guidance.c    →  4-phase V-bar guidance  │
              │  control.c     →  PD + MIB controller     │
              │  dynamics.c    →  CW Φ(Δt) propagator     │
              └───────────────────────────────────────────┘
```

## Quick Start

### 1. Install Basilisk
```bash
pip install bsk
# Optional runtime tools:
pip install matplotlib numpy
```

### 2. Build the shared library
```bash
# In gnc_docking/
make shared
# → build/libgnc.so
```

### 3. Run the scenario
```bash
# Single run (seed 42)
python bsk/scenario_docking.py

# Specify seed
python bsk/scenario_docking.py --seed 7

# 50-run Monte Carlo
python bsk/scenario_docking.py --mc 50

# Single run, no plots
python bsk/scenario_docking.py --no-plot
```

### 4. Output
- Console: per-100-step progress + final report
- `bsk/plots/docking_results.pdf`: range, phase, ΔV, LVLH trajectory

## File Map

| File | Purpose |
|------|---------|
| `bsk/gnc_bridge.h` | C API (4 functions, primitive types only) |
| `bsk/gnc_bridge.c` | Bridge implementation; static 4-slot context pool |
| `bsk/gnc_ctypes.py` | Python ctypes bindings; `GncBridge` class |
| `bsk/scenario_docking.py` | Basilisk scenario + Monte Carlo + plotter |

## Design Decisions

### Why ctypes instead of BSK C++ module wrappers?
Our GNC code is pure C99 (NASA P10). Writing proper BSK C++ SWIG modules
would require:
- CMake integration (conflicts with our simple Makefile)
- Violating P10 in the wrapper layer (dynamic allocation, virtual functions)
- SWIG `.i` files for each module

The ctypes bridge avoids all of this. The GNC algorithms stay 100% P10
compliant; the bridge layer is explicitly "simulation infrastructure" —
held to a lower standard than flight code.

### Coordinate handling
Basilisk propagates both spacecraft in ECI (J2000). The Python scenario
computes the LVLH relative state each step using the full non-linear
transformation (not CW linearisation). Our GNC internal state uses
CW-linearised dynamics for propagation, which is valid for ‖Δr‖ << a.

### Sensor model
The Basilisk `simpleNav` module is deliberately NOT used here — it adds
attitude navigation noise that is irrelevant for translation-only docking.
Instead we apply our custom range-proportional noise:
    σ(r) = max(0.002, 0.003 × r)   (metres)

This matches the sensor model in `sim/main.c` exactly, so Monte Carlo
results are directly comparable between the standalone sim and BSK.

## Monte Carlo

Running 50 seeds checks robustness against sensor noise variations.
Expected outcomes:
- P(dock success) > 95%
- ΔV spread: < ±0.3 m/s across seeds
- Range at docking: < 0.05 m (100% of successful runs)

## Extending

To add J2 perturbations, change the `GravBodyData` call:
```python
from Basilisk.simulation import gravityEffector
# (see Basilisk docs: scenario_BasicOrbit.py)
```

To record Vizard data for 3D replay:
```python
from Basilisk.utilities import vizSupport
viz = vizSupport.enableUnityVisualization(scSim, "dynTask", [tgt, chs])
```
Then run: `python bsk/scenario_docking.py --vizard`
