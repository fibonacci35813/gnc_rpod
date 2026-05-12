#!/usr/bin/env bash

set -e

PYTHONPATH=src python scripts/run_bsk_basic_check.py
PYTHONPATH=src python scripts/run_relative_state_check.py
PYTHONPATH=src python scripts/run_frame_check.py
PYTHONPATH=src python scripts/run_basilisk_two_spacecraft_demo.py
PYTHONPATH=src python scripts/run_noisy_sensor_check.py
PYTHONPATH=src python scripts/run_guidance_check.py
PYTHONPATH=src python scripts/run_pd_controller_check.py
PYTHONPATH=src python scripts/run_actuator_check.py
PYTHONPATH=src python scripts/run_stack_open_loop_check.py
