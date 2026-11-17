# 3-D strapdown INS, base case: Iterated extended Kalman filter and smoother

[Back to the base case](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Unscented Kalman filter and URTSS](../unscented/README.md) | [All estimators compared](../comparison/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter prior is centred on the true initial state.

The iterated extended Kalman filter (IEKF) relinearises each measurement update at the improved estimate until it settles. The iterated extended Kalman smoother (IEKS) starts from the IEKF output and relinearises the motion and measurement models at the whole smoothed trajectory until it settles. Both use Levenberg-Marquardt damping, starting at $\lambda = 10^{-2}$, multiplying it by 10 after a rejected step and dividing it by 10 after an accepted one.

The truth and the prior mean start at the origin, level, heading north at 2 m/s, with zero biases. Prior standard deviations: 0.63 m in position, 0.2 m/s in velocity, 0.2 rad in attitude.

The state, motion model, sensors and parameters are given in the [base case overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 0.25 to 1 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 0.25 to 1 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/base_case/iterated
for run in results/INS3D/base_case/iterated/*/; do
    python3 scripts/analyze_ins_3d.py "$run/data"
done
```

Every number below is taken from the run's `stats.txt`.

## Results

### Figure-eight

![1_scenario](figure_eight/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](figure_eight/figures/2_path.png)

*The true path with the IEKF and IEKS estimates in 3D, and the depth against time.*

![3_errors](figure_eight/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](figure_eight/figures/4_uncertainty.png)

*The position and attitude standard deviation the IEKF and IEKS report, on a log scale.*

![5_biases](figure_eight/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](figure_eight/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IEKF | 0.912 | 0.730 | 1.75 | 2.13 | 1.79 | 4.0 | 98.4% |
| IEKS | 0.685 | 0.441 | 1.26 | 1.51 | 1.26 | 3.2 | 98.6% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.538 | [2.440, 3.617] | consistent |
| NIS, magnetometer | 1 | 0.803 | [0.873, 1.136] | underconfident |
| NIS, DVL | 3 | 2.300 | [2.773, 3.236] | underconfident |
| NIS, altimeter | 1 | 1.016 | [0.877, 1.131] | consistent |
| NEES, full state, IEKF | 15 | 8.838 | [13.949, 16.089] | underconfident |
| NEES, position, IEKF | 3 | 2.298 | [2.486, 3.561] | underconfident |
| NEES, attitude, IEKF | 3 | 1.311 | [2.685, 3.332] | underconfident |
| NEES, full state, IEKS | 15 | 9.762 | [12.490, 17.737] | underconfident |
| NEES, position, IEKS | 3 | 2.853 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.756 | [1.737, 4.602] | consistent |

### Four-corner

![1_scenario](rounded_rectangle/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](rounded_rectangle/figures/2_path.png)

*The true path with the IEKF and IEKS estimates in 3D, and the depth against time.*

![3_errors](rounded_rectangle/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](rounded_rectangle/figures/4_uncertainty.png)

*The position and attitude standard deviation the IEKF and IEKS report, on a log scale.*

![5_biases](rounded_rectangle/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](rounded_rectangle/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IEKF | 1.024 | 0.477 | 1.99 | 2.27 | 1.72 | 4.0 | 99.0% |
| IEKS | 0.613 | 0.399 | 1.19 | 1.40 | 1.14 | 3.2 | 99.7% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.507 | [2.372, 3.701] | consistent |
| NIS, magnetometer | 1 | 0.800 | [0.874, 1.134] | underconfident |
| NIS, DVL | 3 | 2.317 | [2.772, 3.237] | underconfident |
| NIS, altimeter | 1 | 1.013 | [0.877, 1.131] | consistent |
| NEES, full state, IEKF | 15 | 8.315 | [13.979, 16.056] | underconfident |
| NEES, position, IEKF | 3 | 2.099 | [2.525, 3.516] | underconfident |
| NEES, attitude, IEKF | 3 | 1.222 | [2.681, 3.337] | underconfident |
| NEES, full state, IEKS | 15 | 8.732 | [12.550, 17.665] | underconfident |
| NEES, position, IEKS | 3 | 2.455 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.434 | [1.760, 4.565] | underconfident |
