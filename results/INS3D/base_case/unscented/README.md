# 3-D strapdown INS, base case: Unscented Kalman filter and URTSS

[Back to the base case](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Iterated extended Kalman filter and smoother](../iterated/README.md) | [All estimators compared](../comparison/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter prior is centred on the true initial state.

The unscented Kalman filter (UKF) carries the mean and covariance through the models with sigma points on the error state, with $\alpha = 1$, $\beta = 2$, $\kappa = 0$. The unscented Rauch-Tung-Striebel smoother (URTSS) runs backward over the stored UKF output.

The truth and the prior mean start at the origin, level, heading north at 2 m/s, with zero biases. Prior standard deviations: 0.63 m in position, 0.2 m/s in velocity, 0.2 rad in attitude.

The state, motion model, sensors and parameters are given in the [base case overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 0.25 to 1 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 0.25 to 1 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/base_case/unscented
for run in results/INS3D/base_case/unscented/*/; do
    python3 scripts/analyze_ins_3d.py "$run/data"
done
```

Every number below is taken from the run's `stats.txt`.

## Results

### Figure-eight

![1_scenario](figure_eight/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](figure_eight/figures/2_path.png)

*The true path with the UKF and URTSS estimates in 3D, and the depth against time.*

![3_errors](figure_eight/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](figure_eight/figures/4_uncertainty.png)

*The position and attitude standard deviation the UKF and URTSS report, on a log scale.*

![5_biases](figure_eight/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](figure_eight/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UKF | 0.944 | 0.742 | 1.83 | 2.22 | 1.79 | 4.0 | 98.3% |
| URTSS | 0.708 | 0.423 | 1.31 | 1.56 | 1.34 | 3.2 | 98.5% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.567 | [2.443, 3.613] | consistent |
| NIS, magnetometer | 1 | 0.805 | [0.873, 1.135] | underconfident |
| NIS, DVL | 3 | 2.263 | [2.770, 3.239] | underconfident |
| NIS, altimeter | 1 | 1.010 | [0.876, 1.132] | consistent |
| NEES, full state, UKF | 15 | 9.005 | [13.910, 16.130] | underconfident |
| NEES, position, UKF | 3 | 2.353 | [2.480, 3.569] | underconfident |
| NEES, attitude, UKF | 3 | 1.317 | [2.686, 3.332] | underconfident |
| NEES, full state, URTSS | 15 | 10.202 | [12.644, 17.554] | underconfident |
| NEES, position, URTSS | 3 | 2.897 | [0.216, 9.348] | consistent |
| NEES, attitude, URTSS | 3 | 1.989 | [1.647, 4.753] | consistent |

### Four-corner

![1_scenario](rounded_rectangle/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](rounded_rectangle/figures/2_path.png)

*The true path with the UKF and URTSS estimates in 3D, and the depth against time.*

![3_errors](rounded_rectangle/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](rounded_rectangle/figures/4_uncertainty.png)

*The position and attitude standard deviation the UKF and URTSS report, on a log scale.*

![5_biases](rounded_rectangle/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](rounded_rectangle/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UKF | 1.046 | 0.494 | 2.06 | 2.32 | 1.72 | 4.1 | 98.9% |
| URTSS | 0.631 | 0.378 | 1.25 | 1.45 | 1.16 | 3.1 | 99.7% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.531 | [2.377, 3.695] | consistent |
| NIS, magnetometer | 1 | 0.801 | [0.874, 1.134] | underconfident |
| NIS, DVL | 3 | 2.281 | [2.769, 3.240] | underconfident |
| NIS, altimeter | 1 | 1.006 | [0.877, 1.131] | consistent |
| NEES, full state, UKF | 15 | 8.391 | [13.968, 16.069] | underconfident |
| NEES, position, UKF | 3 | 2.137 | [2.518, 3.523] | underconfident |
| NEES, attitude, UKF | 3 | 1.221 | [2.683, 3.334] | underconfident |
| NEES, full state, URTSS | 15 | 8.787 | [12.783, 17.392] | underconfident |
| NEES, position, URTSS | 3 | 2.487 | [0.216, 9.348] | consistent |
| NEES, attitude, URTSS | 3 | 1.493 | [1.745, 4.589] | underconfident |
