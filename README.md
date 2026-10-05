# Trajectory reconstruction and sensor fusion for underwater robotics

A place to store code that will be used in my masters pre-project. This Repo will likely see many revisions and changes, and nothing here is permanent.

## Status and next steps

### Phase 1: prototype and play around to figure stuff out (current)

- [x] Simple test cases: gaussian random walk and gaussian white noise, in 1-D and 2-D
- [x] Standard KF, filtering the trajectory forward
- [x] RTS smoother, used on the path
- [x] Plot the results and evaluate filter and smoother consistency (NIS and NEES, single run)

Other interesting tests to perform when writing about results and theory:

- [ ] Mis-tune `q` or `r` in the filter while leaving the simulator alone, to show an inconsistent filter for contrast
- [ ] Monte Carlo over seeds for proper consistency bands
- [ ] Standard forward-backward smoother, to compare against the RTS smoother

### Phase 2: sophistication

- [x] Extend the current filter and smoother to be able to handle nonlinearities
- [x] Add and experiment with EKF and ERTSS
- [x] Add a 2D strapdown inertial navigation model to test how the filters deal with real nonlinearities.
- [ ] Add and experiment with UKF and URTSS
- [ ] Run experiments and document consistency, accuracy and other factors that might be of interest

Other interesting tests to perform when writing about results and theory:

- [ ] Introduce model mismatch between the filter and the simulator, to show an inconsistent filter for contrast
- [ ] Monte Carlo over seeds for proper consistency bands
- [ ] Reason about why we can expect or should not expect differences in performance between an Unscented, an Extended or an Iterative variant of a filter/smoother.

### Phase 3: towards realistic scenarios

- [ ] Implement dynamical AUV model from Fossen
- [ ] Implement a 3D stapdown INS model
- [ ] Implement sensor models for the 3D strapdown INS model
- [ ] Run experiments and document consistency, accuracy and other factors that might be of interest

### Phase 4: Iterative solutions, Graph based solutions, and batch solutions

- [ ] Experiment with a home cooked batch solver
- [ ] Experiment with ISAM2, an incremental solver
- [ ] Experiment with GTSAM, a graph based solver
- [ ] Run experiments and document consistency, accuracy and other factors that might be of interest

### Additions if time allows

- [ ] Experiment with the ESKF and ESRTSS
- [ ] Experiment with the Stonefish simulator and how to get good data from it

### Example results

A 1-D random walk seen through noisy measurements, estimated with the filter and
the smoother. Both draw a 95% confidence band; the smoother's path is closer to
the truth (RMSE 0.274 against 0.365) and its band is narrower.

![1-D filter vs smoother](results/1D/figures/4_filter_vs_smoother_500.png)

## Experiments

Each experiment lives in `results/<name>/` with its own `README.md`, `data/` and
`figures/`.

| experiment                              | what it tests                                     | result                                                |
| --------------------------------------- | ------------------------------------------------- | ----------------------------------------------------- |
| [1-D random walk](results/1D/README.md) | filter and smoother on the simplest matched model | 1000 steps, RMSE 0.365 to 0.274, all three consistent |
| [2-D random walk](results/2D/README.md) | the same with vectors, and covariance ellipses    | 200 steps, RMSE 0.545 to 0.388, all three consistent  |
| [2-D strapdown INS, GNSS only](results/INS2D/only_gnss/README.md) | nonlinear model, GNSS the only aiding sensor, all 3 trajectories | 30000 steps, consistent except one sharp-turn case |
| [2-D strapdown INS, multi-sensor fusion](results/INS2D/multi_sensor_fusion/README.md) | GNSS + magnetometer + DVL fused sequentially, all 3 trajectories | 30000 steps, consistent on all 3 trajectories |
| [2-D strapdown INS, dead reckoning](results/INS2D/dead_reckoning/README.md) | GNSS disabled entirely, magnetometer + DVL only, all 3 trajectories | 30000 steps, position-consistent; DVL NIS mildly off |
| [2-D strapdown INS, GNSS outage](results/INS2D/gnss_outage/README.md) | GNSS denied for the middle 70% of the run, matched and mistuned IMU noise | 30000 steps, consistent except magnetometer |
| [2-D strapdown INS, UKF and URTSS](results/INS2D/ukf/README.md) | the unscented filter and smoother against the EKF and ERTS, on the outage run and a heading stress run | 30000 steps, identical on the outage run, UKF RMSE 4.90 against 8.49 m on the stress run |

## Repository layout

```
lib/        headers, grouped by role; src/ mirrors this structure
  filters/      bayesian_filter (interface), kalman_filter, extended_kalman_filter
  smoothers/    bayesian_smoother (interface), rts and extended rts smoothers
  models/       motion/measurement interfaces, linear and nonlinear (INS) models
  scenarios/    one run function per scenario
  simulation.hpp, results.hpp, utilities.hpp   generators, CSV export, helpers
src/        implementations, and main.cpp which loads one config and runs it
configs/    one yaml file per scenario
scripts/    analysis: analyze_1d.py, analyze_2d.py
results/    one folder per experiment: README.md, data/, figures/
data/       scratch output of the latest run (gitignored)
figures/    scratch figures of the latest analysis (gitignored)
```

| file                                                                     | contents                                                                       |
| ------------------------------------------------------------------------ | ------------------------------------------------------------------------------ |
| `lib/filters/bayesian_filter.hpp`, `lib/smoothers/bayesian_smoother.hpp` | pure virtual interfaces                                                        |
| `lib/models/models.hpp`                                                  | `MotionModel` (F, Q) and `MeasurementModel` (H, R)                             |
| `src/filters/kalman_filter.cpp`                                          | implements the kalman filter predict/update step from Algorithm 1, Brekke 2025 |
| `src/smoothers/rauch_tung_striebel_smoother.cpp`                         | implements the backward recursion equations from Sarkka theorem 12.2           |
| `src/main.cpp`                                                           | loads one config file from `configs/` and calls the scenario's run function    |
| `src/scenarios/random_walk_scenario.cpp`                                 | the random walk scenario (any dimension), values come from the yaml config     |
| `src/simulation.cpp`                                                     | Gaussian white noise and random walk generators for the simulation             |
| `src/results.cpp`                                                           | CSV export for filter, smoother and ground truth                               |
| `scripts/analyze_1d.py`                                                  | plots and consistency statistics for the 1-D case                              |
| `scripts/analyze_2d.py`                                                  | trajectory plots with covariance ellipses and consistency statistics for 2-D   |

## Building and running

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/kalman_filtering_and_smoothing
```

`main.cpp` names the config file to run e.g `random_walk.yaml` and the
run function that uses it. To switch scenario, change the file name there and
rebuild. To add a new kind of scenario, write a `run_<name>` function (see
`lib/scenarios/scenarios.hpp`) and call it from `main.cpp`.

Requires Eigen 3, yaml-cpp and a C++20 compiler.

`data/`, `figures/` and `.venv/` are gitignored scratch space. When a run is
worth keeping, copy its CSVs to `results/<name>/data/`, run the analysis script
on that folder (`python3 scripts/analyze_2d.py results/<name>/data`) so the
figures land in `results/<name>/figures/`, and write the experiment's README.

Most of the code is handwritten by myself, the scripts and visualization is done
with a lot of heavy lifting by Anthropics Claude Code.
