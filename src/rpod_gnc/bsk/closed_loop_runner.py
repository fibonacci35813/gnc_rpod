"""Closed-loop Basilisk SIL runner for early RPOD integration testing."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from Basilisk.simulation import extForceTorque
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody

from rpod_gnc.actuators.ideal_accel import IdealAccelActuator, IdealAccelActuatorConfig
from rpod_gnc.bsk.frames import eci_to_lvlh_frame, lvlh_force_to_eci, relative_state_eci_to_lvlh
from rpod_gnc.control.pd_controller import PDController, PDControllerConfig
from rpod_gnc.guidance.hold_point import HoldPointGuidance, HoldPointGuidanceConfig
from rpod_gnc.sensors.noisy_relative_pose import (
    NoisyRelativePoseSensor,
    NoisyRelativePoseSensorConfig,
)
from rpod_gnc.bsk.two_spacecraft_runner import make_spacecraft


@dataclass(frozen=True)
class ClosedLoopConfig:
    duration_s: float = 300.0
    dt_s: float = 1.0
    chaser_mass_kg: float = 50.0
    initial_offset_lvlh_m: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -10.0, 0.0], dtype=float)
    )
    hold_position_lvlh_m: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -0.5, 0.0], dtype=float)
    )
    position_noise_std_m: float = 0.05
    dropout_probability: float = 0.0
    sensor_seed: int = 7
    kp: float = 0.002
    kd: float = 0.08
    max_accel_mps2: float = 0.02


@dataclass(frozen=True)
class ClosedLoopResult:
    """Closed-loop state history and command logs.

    State histories are sampled at `t = 0, dt, ..., duration`.
    Control histories correspond to each propagated interval and therefore
    have one fewer sample than the state histories.
    """

    times_s: np.ndarray
    relative_positions_lvlh_m: np.ndarray
    relative_velocities_lvlh_mps: np.ndarray
    ranges_m: np.ndarray
    accel_commands_lvlh_mps2: np.ndarray
    applied_accels_lvlh_mps2: np.ndarray
    delta_v_steps_mps: np.ndarray

    @property
    def total_delta_v_mps(self) -> float:
        return float(np.sum(self.delta_v_steps_mps))


def _as_vector(vector: np.ndarray) -> np.ndarray:
    return np.asarray(vector, dtype=float).reshape(3)


def _stack_vectors(history: list[np.ndarray]) -> np.ndarray:
    if not history:
        return np.zeros((0, 3), dtype=float)
    return np.vstack(history)


def _validate_config(config: ClosedLoopConfig) -> tuple[np.ndarray, np.ndarray, int]:
    if config.duration_s < 0.0:
        raise ValueError("duration_s must be non-negative.")
    if config.dt_s <= 0.0:
        raise ValueError("dt_s must be positive.")
    if config.chaser_mass_kg <= 0.0:
        raise ValueError("chaser_mass_kg must be positive.")

    step_count_float = config.duration_s / config.dt_s
    step_count = int(round(step_count_float))
    if not np.isclose(step_count_float, step_count):
        raise ValueError("duration_s must be an integer multiple of dt_s.")

    return (
        _as_vector(config.initial_offset_lvlh_m),
        _as_vector(config.hold_position_lvlh_m),
        step_count,
    )


def _initial_orbit_state() -> tuple[np.ndarray, np.ndarray]:
    oe = orbitalMotion.ClassicElements()
    oe.a = 7000e3
    oe.e = 0.0
    oe.i = np.deg2rad(51.6)
    oe.Omega = 0.0
    oe.omega = 0.0
    oe.f = 0.0

    target_position_eci_m, target_velocity_eci_mps = orbitalMotion.elem2rv(
        orbitalMotion.MU_EARTH,
        oe,
    )
    return np.array(target_position_eci_m), np.array(target_velocity_eci_mps)


def _compute_relative_state(
    target_position_eci_m: np.ndarray,
    target_velocity_eci_mps: np.ndarray,
    chaser_position_eci_m: np.ndarray,
    chaser_velocity_eci_mps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return relative_state_eci_to_lvlh(
        target_position_eci_m,
        target_velocity_eci_mps,
        chaser_position_eci_m,
        chaser_velocity_eci_mps,
    )


def run_closed_loop_demo(config: ClosedLoopConfig | None = None) -> ClosedLoopResult:
    """Run the first closed-loop Basilisk SIL milestone.

    The controller uses the current LVLH relative state to compute a command
    that is applied over the next integration step through `extForceTorque`.
    """

    if config is None:
        config = ClosedLoopConfig()

    initial_offset_lvlh_m, hold_position_lvlh_m, step_count = _validate_config(config)

    sim = SimulationBaseClass.SimBaseClass()
    process_name = "rpod_process"
    task_name = "rpod_task"

    process = sim.CreateNewProcess(process_name)
    process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(config.dt_s)))

    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True

    target_position_eci_m, target_velocity_eci_mps = _initial_orbit_state()
    rotation_eci_to_lvlh = eci_to_lvlh_frame(
        target_position_eci_m,
        target_velocity_eci_mps,
    )
    initial_offset_eci_m = rotation_eci_to_lvlh.T @ initial_offset_lvlh_m
    chaser_position_eci_m = target_position_eci_m + initial_offset_eci_m
    chaser_velocity_eci_mps = target_velocity_eci_mps.copy()

    target = make_spacecraft("target", target_position_eci_m, target_velocity_eci_mps)
    chaser = make_spacecraft("chaser", chaser_position_eci_m, chaser_velocity_eci_mps)
    target.hub.mHub = config.chaser_mass_kg
    chaser.hub.mHub = config.chaser_mass_kg

    control_effector = extForceTorque.ExtForceTorque()
    control_effector.ModelTag = "chaserControlForce"
    control_effector.extForce_N = [0.0, 0.0, 0.0]
    chaser.addDynamicEffector(control_effector)

    grav_factory.addBodiesTo(target)
    grav_factory.addBodiesTo(chaser)

    sim.AddModelToTask(task_name, control_effector, ModelPriority=10)
    sim.AddModelToTask(task_name, target)
    sim.AddModelToTask(task_name, chaser)

    target_recorder = target.scStateOutMsg.recorder(macros.sec2nano(config.dt_s))
    chaser_recorder = chaser.scStateOutMsg.recorder(macros.sec2nano(config.dt_s))
    sim.AddModelToTask(task_name, target_recorder)
    sim.AddModelToTask(task_name, chaser_recorder)

    sim.InitializeSimulation()

    sensor = NoisyRelativePoseSensor(
        NoisyRelativePoseSensorConfig(
            position_noise_std_m=config.position_noise_std_m,
            dropout_probability=config.dropout_probability,
            seed=config.sensor_seed,
        )
    )
    guidance = HoldPointGuidance(
        HoldPointGuidanceConfig(
            hold_position_lvlh_m=hold_position_lvlh_m,
        )
    )
    controller = PDController(
        PDControllerConfig(
            kp=config.kp,
            kd=config.kd,
            max_accel_mps2=config.max_accel_mps2,
        )
    )
    actuator = IdealAccelActuator(
        IdealAccelActuatorConfig(
            max_accel_mps2=config.max_accel_mps2,
        )
    )

    relative_position_lvlh_m, relative_velocity_lvlh_mps, rotation_eci_to_lvlh = (
        _compute_relative_state(
            target_position_eci_m,
            target_velocity_eci_mps,
            chaser_position_eci_m,
            chaser_velocity_eci_mps,
        )
    )

    times_s = [0.0]
    relative_positions_lvlh_m = [relative_position_lvlh_m]
    relative_velocities_lvlh_mps = [relative_velocity_lvlh_mps]
    ranges_m = [float(np.linalg.norm(relative_position_lvlh_m))]
    accel_commands_lvlh_mps2: list[np.ndarray] = []
    applied_accels_lvlh_mps2: list[np.ndarray] = []
    delta_v_steps_mps: list[float] = []

    for step_index in range(step_count):
        current_time_s = times_s[-1]
        true_position_lvlh_m = relative_positions_lvlh_m[-1]
        true_velocity_lvlh_mps = relative_velocities_lvlh_mps[-1]

        measurement = sensor.measure(
            time_s=current_time_s,
            true_position_m=true_position_lvlh_m,
        )
        estimated_position_m = (
            measurement.position_m if measurement.valid else true_position_lvlh_m
        )
        estimated_velocity_mps = true_velocity_lvlh_mps
        if measurement.velocity_mps is not None:
            estimated_velocity_mps = measurement.velocity_mps

        desired_position_m, desired_velocity_mps = guidance.get_reference(current_time_s)
        accel_command_lvlh_mps2 = controller.compute_acceleration(
            estimated_position_m=estimated_position_m,
            estimated_velocity_mps=estimated_velocity_mps,
            desired_position_m=desired_position_m,
            desired_velocity_mps=desired_velocity_mps,
        )
        actuator_output = actuator.apply(
            accel_cmd_mps2=accel_command_lvlh_mps2,
            dt_s=config.dt_s,
        )

        force_lvlh_n = config.chaser_mass_kg * actuator_output.applied_accel_mps2
        force_eci_n = lvlh_force_to_eci(force_lvlh_n, rotation_eci_to_lvlh)
        control_effector.extForce_N = force_eci_n.tolist()

        accel_commands_lvlh_mps2.append(accel_command_lvlh_mps2)
        applied_accels_lvlh_mps2.append(actuator_output.applied_accel_mps2)
        delta_v_steps_mps.append(actuator_output.delta_v_step_mps)

        next_time_s = (step_index + 1) * config.dt_s
        sim.ConfigureStopTime(macros.sec2nano(next_time_s))
        sim.ExecuteSimulation()

        target_position_eci_m = np.array(target_recorder.r_BN_N[-1])
        target_velocity_eci_mps = np.array(target_recorder.v_BN_N[-1])
        chaser_position_eci_m = np.array(chaser_recorder.r_BN_N[-1])
        chaser_velocity_eci_mps = np.array(chaser_recorder.v_BN_N[-1])

        relative_position_lvlh_m, relative_velocity_lvlh_mps, rotation_eci_to_lvlh = (
            _compute_relative_state(
                target_position_eci_m,
                target_velocity_eci_mps,
                chaser_position_eci_m,
                chaser_velocity_eci_mps,
            )
        )

        times_s.append(next_time_s)
        relative_positions_lvlh_m.append(relative_position_lvlh_m)
        relative_velocities_lvlh_mps.append(relative_velocity_lvlh_mps)
        ranges_m.append(float(np.linalg.norm(relative_position_lvlh_m)))

    return ClosedLoopResult(
        times_s=np.asarray(times_s, dtype=float),
        relative_positions_lvlh_m=_stack_vectors(relative_positions_lvlh_m),
        relative_velocities_lvlh_mps=_stack_vectors(relative_velocities_lvlh_mps),
        ranges_m=np.asarray(ranges_m, dtype=float),
        accel_commands_lvlh_mps2=_stack_vectors(accel_commands_lvlh_mps2),
        applied_accels_lvlh_mps2=_stack_vectors(applied_accels_lvlh_mps2),
        delta_v_steps_mps=np.asarray(delta_v_steps_mps, dtype=float),
    )
