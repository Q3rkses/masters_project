# 3-D strapdown INS, poor initialization: Unscented Kalman filter and URTSS

[Back to the poor initialization](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Iterated extended Kalman filter and smoother](../iterated/README.md) | [All estimators compared](../comparison/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter starts from a prior far from the truth. In the `*_sparse_magnetometer` runs the magnetometer fires every 30 to 60 s instead of every 0.25 to 1 s, so the yaw error of the prior is corrected only rarely.

The unscented Kalman filter (UKF) carries the mean and covariance through the models with sigma points on the error state, with $\alpha = 1$, $\beta = 2$, $\kappa = 0$. The unscented Rauch-Tung-Striebel smoother (URTSS) runs backward over the stored UKF output.

The truth starts at the origin, level, heading north at 2 m/s. The prior mean is off by $(3, -3, 1)$ m in position, $(-0.5, 0.5, 0.2)$ m/s in velocity and by the rotation vector $(0.3, -0.3, 1.5)$ rad in attitude, 89° in total. Each offset is one standard deviation of the prior.

The state, motion model, sensors and parameters are given in the [poor initialization overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 0.25 to 1 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 0.25 to 1 s |
| [`figure_eight_sparse_magnetometer`](figure_eight_sparse_magnetometer/) | figure-eight | 30 to 60 s |
| [`rounded_rectangle_sparse_magnetometer`](rounded_rectangle_sparse_magnetometer/) | four-corner | 30 to 60 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/poor_initialization/unscented
for run in results/INS3D/poor_initialization/unscented/*/; do
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
| UKF | 1.022 | 0.766 | 1.89 | 5.26 | 5.76 | 89.4 | 97.7% |
| URTSS | 0.721 | 0.425 | 1.32 | 1.79 | 1.63 | 15.0 | 97.3% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.654 | [2.433, 3.625] | overconfident |
| NIS, magnetometer | 1 | 1.205 | [0.648, 1.427] | consistent |
| NIS, DVL | 3 | 2.519 | [2.682, 3.336] | underconfident |
| NIS, altimeter | 1 | 1.055 | [0.873, 1.135] | consistent |
| NEES, full state, UKF | 15 | 12.028 | [11.772, 18.613] | consistent |
| NEES, position, UKF | 3 | 2.492 | [2.480, 3.568] | consistent |
| NEES, attitude, UKF | 3 | 2.321 | [1.950, 4.272] | consistent |
| NEES, full state, URTSS | 15 | 11.982 | [11.401, 19.085] | consistent |
| NEES, position, URTSS | 3 | 3.138 | [0.216, 9.348] | consistent |
| NEES, attitude, URTSS | 3 | 2.272 | [0.216, 9.348] | consistent |

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
| UKF | 1.122 | 0.506 | 2.07 | 5.64 | 5.63 | 89.1 | 98.5% |
| URTSS | 0.641 | 0.380 | 1.25 | 1.45 | 1.41 | 12.5 | 98.8% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.591 | [2.359, 3.716] | consistent |
| NIS, magnetometer | 1 | 1.147 | [0.694, 1.361] | consistent |
| NIS, DVL | 3 | 2.449 | [2.713, 3.301] | underconfident |
| NIS, altimeter | 1 | 1.024 | [0.878, 1.130] | consistent |
| NEES, full state, UKF | 15 | 10.479 | [11.296, 19.221] | underconfident |
| NEES, position, UKF | 3 | 2.227 | [2.509, 3.534] | underconfident |
| NEES, attitude, UKF | 3 | 2.095 | [1.940, 4.287] | consistent |
| NEES, full state, URTSS | 15 | 10.123 | [10.972, 19.648] | underconfident |
| NEES, position, URTSS | 3 | 2.641 | [0.216, 9.348] | consistent |
| NEES, attitude, URTSS | 3 | 1.750 | [0.276, 8.894] | consistent |

### Figure-eight, sparse magnetometer

![1_scenario](figure_eight_sparse_magnetometer/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](figure_eight_sparse_magnetometer/figures/2_path.png)

*The true path with the UKF and URTSS estimates in 3D, and the depth against time.*

![3_errors](figure_eight_sparse_magnetometer/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](figure_eight_sparse_magnetometer/figures/4_uncertainty.png)

*The position and attitude standard deviation the UKF and URTSS report, on a log scale.*

![5_biases](figure_eight_sparse_magnetometer/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](figure_eight_sparse_magnetometer/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UKF | 10.445 | 5.505 | 22.63 | 36.18 | 21.65 | 89.4 | 49.6% |
| URTSS | 8.546 | 4.656 | 18.80 | 19.40 | 9.68 | 61.5 | 41.9% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 18.669 | [1.338, 5.321] | overconfident |
| NIS, magnetometer | 1 | 11.363 | [0.001, 5.024] | overconfident |
| NIS, DVL | 3 | 2.229 | [2.795, 3.212] | underconfident |
| NIS, altimeter | 1 | 0.995 | [0.873, 1.136] | consistent |
| NEES, full state, UKF | 15 | 90.279 | [12.529, 17.690] | overconfident |
| NEES, position, UKF | 3 | 30.744 | [1.423, 5.156] | overconfident |
| NEES, attitude, UKF | 3 | 20.320 | [1.765, 4.557] | overconfident |
| NEES, full state, URTSS | 15 | 83.215 | [6.262, 27.488] | overconfident |
| NEES, position, URTSS | 3 | 36.575 | [0.216, 9.348] | overconfident |
| NEES, attitude, URTSS | 3 | 21.134 | [0.216, 9.348] | overconfident |

### Four-corner, sparse magnetometer

![1_scenario](rounded_rectangle_sparse_magnetometer/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](rounded_rectangle_sparse_magnetometer/figures/2_path.png)

*The true path with the UKF and URTSS estimates in 3D, and the depth against time.*

![3_errors](rounded_rectangle_sparse_magnetometer/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](rounded_rectangle_sparse_magnetometer/figures/4_uncertainty.png)

*The position and attitude standard deviation the UKF and URTSS report, on a log scale.*

![5_biases](rounded_rectangle_sparse_magnetometer/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](rounded_rectangle_sparse_magnetometer/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UKF | 10.006 | 5.858 | 20.40 | 30.55 | 20.36 | 89.2 | 47.9% |
| URTSS | 6.833 | 3.922 | 14.65 | 15.27 | 9.28 | 62.5 | 47.6% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 18.084 | [1.324, 5.349] | overconfident |
| NIS, magnetometer | 1 | 8.789 | [0.009, 4.177] | overconfident |
| NIS, DVL | 3 | 2.231 | [2.796, 3.211] | underconfident |
| NIS, altimeter | 1 | 0.991 | [0.872, 1.136] | consistent |
| NEES, full state, UKF | 15 | 72.632 | [12.607, 17.598] | overconfident |
| NEES, position, UKF | 3 | 19.544 | [1.856, 4.415] | overconfident |
| NEES, attitude, UKF | 3 | 17.907 | [1.793, 4.513] | overconfident |
| NEES, full state, URTSS | 15 | 65.737 | [6.262, 27.488] | overconfident |
| NEES, position, URTSS | 3 | 22.285 | [0.216, 9.348] | overconfident |
| NEES, attitude, URTSS | 3 | 21.109 | [0.216, 9.348] | overconfident |
