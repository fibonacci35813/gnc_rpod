# Basilisk SIL Learning Path

## 1. What Basilisk Is Doing Here

Basilisk is the truth simulator. In this repo it propagates the target and chaser spacecraft states in inertial space and logs the resulting position and velocity histories.

## 2. What `rpod_gnc` Is Responsible For

`rpod_gnc` is the RPOD stack built around that truth model. Its job is to transform truth states into relative states, add sensing abstractions, generate references, compute control commands, apply actuator limits, and evaluate the behavior.

## 3. Current Implemented Flow

Current flow:

`Basilisk target/chaser truth -> ECI states -> LVLH relative state -> validation metrics`

Today, the repo can:

- run an open-loop two-spacecraft Basilisk truth simulation
- convert inertial states into target-relative LVLH coordinates
- compute basic range metrics on the resulting relative motion

## 4. Next Planned Flow

Next flow:

`truth -> sensor -> guidance -> controller -> actuator`

This stage is already scaffolded as standalone modules, but not yet fed back into Basilisk truth propagation.

## 5. Final SIL Flow

Final SIL flow:

`truth -> sensor -> estimator -> guidance -> controller -> actuator -> truth`

That closes the loop: noisy measurements drive an estimator, the guidance and control stack generates commands, actuators apply limits, and those commands affect the next truth state.

## 6. File Map

- `src/rpod_gnc/bsk/`: Basilisk-facing adapters, frame transforms, and truth runners
- `src/rpod_gnc/sensors/`: sensor models built on truth relative states
- `src/rpod_gnc/guidance/`: reference generation such as hold-point commands
- `src/rpod_gnc/control/`: control laws such as the translational PD controller
- `src/rpod_gnc/actuators/`: actuator models and command limiting
- `src/rpod_gnc/validation/`: metrics and validation summaries

## 7. Check Commands

General checks:

```bash
PYTHONPATH=src python scripts/run_relative_state_check.py
PYTHONPATH=src python scripts/run_frame_check.py
PYTHONPATH=src python scripts/run_noisy_sensor_check.py
PYTHONPATH=src python scripts/run_guidance_check.py
PYTHONPATH=src python scripts/run_pd_controller_check.py
PYTHONPATH=src python scripts/run_actuator_check.py
```

Basilisk availability check:

```bash
PYTHONPATH=src python scripts/run_bsk_basic_check.py
```

Basilisk-dependent flow checks:

```bash
PYTHONPATH=src python scripts/run_basilisk_two_spacecraft_demo.py
PYTHONPATH=src python scripts/run_stack_open_loop_check.py
```
