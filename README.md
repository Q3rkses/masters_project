# Trajectory reconstruction and sensor fusion for underwater robotics

A place to store code that will be used in my masters pre-project. This Repo will likely see many revisions and changes, and nothing here is permanent.

## Status and next steps

### Phase 1: prototype and play around to figure stuff out (current)

- [x] Simple test cases: gaussian random walk and gaussian white noise, in 1-D and 2-D
- [x] Standard KF, filtering the trajectory forward
- [x] RTS smoother, used on the path
- [x] Plot the results and evaluate filter and smoother consistency (NIS and NEES, single run)

Next up, in order:

- [ ] Mis-tune `q` or `r` in the filter while leaving the simulator alone, to show an inconsistent filter for contrast
- [ ] Monte Carlo over seeds for proper consistency bands
- [ ] A 2D strapdown INS scenario with x,y,psi with simulated IMU and GNSS signals
- [ ] Standard forward-backward smoother, to compare against the RTS smoother

### Phase 2: sophistication

- [ ] Extend the current filter and smoother to be able to handle nonlinearities
- [ ] Add and experiment with EKF and ERTSS
- [ ] Add and experiment with UKF and URTSS
- [ ] (stretch goal) add and experiment with ESKF and ESRTSS

### Phase 3: towards realistic scenarios

- [ ] Make test cases more realistic and aligned with the goal of the project
- [ ] Implement dynamical AUV model from Fossen
- [ ] Implement models for sensors which will be used
- [ ] (stretch goal) test using the Stonefish simulator, hereunder find a good way to get GNSS and ground truth data to play with
- [ ] Adopt the existing codebase to accomodate for the new AUV model and Sensor models
- [ ] Run experiments and document consistency, accuracy and other factors that might be of interest

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

## Repository layout

```
lib/        headers: filter and smoother interfaces, models, simulation, results
src/        implementations, and main.cpp which runs one experiment
scripts/    analysis: analyze_1d.py, analyze_2d.py
results/    one folder per experiment: README.md, data/, figures/
data/       scratch output of the latest run (gitignored)
figures/    scratch figures of the latest analysis (gitignored)
```

| file                                                   | contents                                                                       |
| ------------------------------------------------------ | ------------------------------------------------------------------------------ |
| `lib/bayesian_filter.hpp`, `lib/bayesian_smoother.hpp` | pure virtual interfaces                                                        |
| `lib/models.hpp`                                       | `MotionModel` (F, Q) and `MeasurementModel` (H, R)                             |
| `src/kalman_filter.cpp`                                | implements the kalman filter predict/update step from Algorithm 1, Brekke 2025 |
| `src/rauch_tung_striebel_smoother.cpp`                 | implements the backward recursion equations from Sarkka theorem 12.2           |
| `src/simulation.cpp`                                   | Gaussian white noise and random walk generators for the simulation             |
| `src/results.cpp`                                      | CSV export for filter, smoother and ground truth                               |
| `scripts/analyze_1d.py`                                | plots and consistency statistics for the 1-D case                              |
| `scripts/analyze_2d.py`                                | trajectory plots with covariance ellipses and consistency statistics for 2-D   |

## Building and running

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/kalman_filtering_and_smoothing
```

Requires Eigen 3 and a C++20 compiler.

`data/`, `figures/` and `.venv/` are gitignored scratch space. When a run is
worth keeping, copy its CSVs to `results/<name>/data/`, run the analysis script
on that folder (`python3 scripts/analyze_2d.py results/<name>/data`) so the
figures land in `results/<name>/figures/`, and write the experiment's README.

Most of the code is handwritten by myself, the scripts and visualization is done
with a lot of heavy lifting by Anthropics Claude Code.
