# MATLAB Learning Demos

This folder contains MATLAB docking and attitude-control learning demos under `matlab_gnc_learning/docking_sim`.

## Run From MATLAB

Open MATLAB, change into the demo folder, then run any of the scripts below from the Command Window.
The command below assumes MATLAB is opened with the repository root as the current folder:

```matlab
cd('gnc_docking/matlab-learning/matlab_gnc_learning/docking_sim')
```

### 1. Full Autonomous Docking Simulation

Runs the main end-to-end docking simulation, generates telemetry plots, and launches the 3-D animation.

```matlab
main_sim
```

### 2. Monte Carlo Docking Analysis

Runs 50 seeded docking cases and saves histogram plots into `plots/monte_carlo_histograms.png`.

```matlab
run_monte_carlo
```

### 3. Multi-Scenario Docking Visualisation

Runs 8 starting positions and shows the high-fidelity relative-motion visualisation.

```matlab
visualise_docking_multi
```

### 4. Attitude Control Test

Runs the reaction-wheel / quaternion attitude-control test.

```matlab
test_att
```

`test_att` ends with `exit`, so it will close MATLAB when finished. If you want to run it inside an interactive MATLAB session, remove or comment out the last line in `test_att.m` first.

## Run From a Shell

From the repository root, you can also launch the demos directly with MATLAB batch mode:

```bash
matlab -batch "cd('gnc_docking/matlab-learning/matlab_gnc_learning/docking_sim'); main_sim"
matlab -batch "cd('gnc_docking/matlab-learning/matlab_gnc_learning/docking_sim'); run_monte_carlo"
matlab -batch "cd('gnc_docking/matlab-learning/matlab_gnc_learning/docking_sim'); visualise_docking_multi"
matlab -batch "cd('gnc_docking/matlab-learning/matlab_gnc_learning/docking_sim'); test_att"
```

## Demo Files

- `main_sim.m`: main autonomous docking scenario
- `run_monte_carlo.m`: Monte Carlo robustness run
- `visualise_docking_multi.m`: 8-case visual demo
- `test_att.m`: attitude-control sanity test
