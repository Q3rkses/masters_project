# 3-D strapdown INS, poor initialization: Iterated extended Kalman filter and smoother

[Back to the poor initialization](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Unscented Kalman filter and URTSS](../unscented/README.md) | [All estimators compared](../comparison/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter starts from a prior far from the truth. In the `*_sparse_magnetometer` runs the magnetometer fires every 30 to 60 s instead of every 0.25 to 1 s, so the yaw error of the prior is corrected only rarely.

The iterated extended Kalman filter (IEKF) relinearises each measurement update at the improved estimate until it settles. The iterated extended Kalman smoother (IEKS) starts from the IEKF output and relinearises the motion and measurement models at the whole smoothed trajectory until it settles. Both use Levenberg-Marquardt damping, starting at $\lambda = 10^{-2}$, multiplying it by 10 after a rejected step and dividing it by 10 after an accepted one.

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
./build/kalman_filtering_and_smoothing results/INS3D/poor_initialization/iterated
for run in results/INS3D/poor_initialization/iterated/*/; do
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
| IEKF | 0.978 | 0.745 | 1.79 | 4.37 | 5.54 | 89.4 | 98.3% |
| IEKS | 0.686 | 0.442 | 1.26 | 1.51 | 1.26 | 3.2 | 98.2% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.555 | [2.437, 3.621] | consistent |
| NIS, magnetometer | 1 | 0.826 | [0.875, 1.133] | underconfident |
| NIS, DVL | 3 | 2.474 | [2.703, 3.312] | underconfident |
| NIS, altimeter | 1 | 1.016 | [0.876, 1.132] | consistent |
| NEES, full state, IEKF | 15 | 9.222 | [14.053, 15.977] | underconfident |
| NEES, position, IEKF | 3 | 2.353 | [2.487, 3.560] | underconfident |
| NEES, attitude, IEKF | 3 | 1.513 | [2.703, 3.312] | underconfident |
| NEES, full state, IEKS | 15 | 9.766 | [12.487, 17.739] | underconfident |
| NEES, position, IEKS | 3 | 2.875 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.750 | [1.739, 4.600] | consistent |

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
| IEKF | 1.081 | 0.488 | 2.00 | 4.37 | 5.47 | 89.1 | 99.0% |
| IEKS | 0.614 | 0.400 | 1.19 | 1.40 | 1.13 | 3.2 | 99.5% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.522 | [2.365, 3.709] | consistent |
| NIS, magnetometer | 1 | 0.818 | [0.876, 1.132] | underconfident |
| NIS, DVL | 3 | 2.470 | [2.716, 3.298] | underconfident |
| NIS, altimeter | 1 | 1.012 | [0.877, 1.131] | consistent |
| NEES, full state, IEKF | 15 | 8.747 | [14.054, 15.976] | underconfident |
| NEES, position, IEKF | 3 | 2.144 | [2.523, 3.518] | underconfident |
| NEES, attitude, IEKF | 3 | 1.426 | [2.701, 3.315] | underconfident |
| NEES, full state, IEKS | 15 | 8.728 | [12.544, 17.672] | underconfident |
| NEES, position, IEKS | 3 | 2.475 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.425 | [1.762, 4.563] | underconfident |

### Figure-eight, sparse magnetometer

![1_scenario](figure_eight_sparse_magnetometer/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](figure_eight_sparse_magnetometer/figures/2_path.png)

*The true path with the IEKF and IEKS estimates in 3D, and the depth against time.*

![3_errors](figure_eight_sparse_magnetometer/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](figure_eight_sparse_magnetometer/figures/4_uncertainty.png)

*The position and attitude standard deviation the IEKF and IEKS report, on a log scale.*

![5_biases](figure_eight_sparse_magnetometer/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](figure_eight_sparse_magnetometer/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IEKF | 2.743 | 1.365 | 7.53 | 8.56 | 7.73 | 89.4 | 97.2% |
| IEKS | 3.061 | 1.791 | 6.53 | 7.01 | 2.40 | 4.9 | 98.9% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.595 | [2.266, 3.836] | consistent |
| NIS, magnetometer | 1 | 0.308 | [0.156, 2.611] | consistent |
| NIS, DVL | 3 | 2.264 | [2.795, 3.212] | underconfident |
| NIS, altimeter | 1 | 0.995 | [0.872, 1.137] | consistent |
| NEES, full state, IEKF | 15 | 10.112 | [13.464, 16.618] | underconfident |
| NEES, position, IEKF | 3 | 2.269 | [2.437, 3.620] | underconfident |
| NEES, attitude, IEKF | 3 | 1.823 | [2.489, 3.558] | underconfident |
| NEES, full state, IEKS | 15 | 8.100 | [12.601, 17.605] | underconfident |
| NEES, position, IEKS | 3 | 2.302 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.537 | [1.586, 4.857] | underconfident |

### Four-corner, sparse magnetometer

![1_scenario](rounded_rectangle_sparse_magnetometer/figures/1_scenario.png)

*The true path with the body $x$ (forward) and $z$ (down) axes, and when each sensor fired.*

![2_path](rounded_rectangle_sparse_magnetometer/figures/2_path.png)

*The true path with the IEKF and IEKS estimates in 3D, and the depth against time.*

![3_errors](rounded_rectangle_sparse_magnetometer/figures/3_errors.png)

*Position error per axis and attitude error about each body axis, with each estimator's 95% band. Shaded where GNSS is denied.*

![4_uncertainty](rounded_rectangle_sparse_magnetometer/figures/4_uncertainty.png)

*The position and attitude standard deviation the IEKF and IEKS report, on a log scale.*

![5_biases](rounded_rectangle_sparse_magnetometer/figures/5_biases.png)

*True and estimated accelerometer and gyro biases.*

![6_consistency](rounded_rectangle_sparse_magnetometer/figures/6_consistency.png)

*NIS of every fix per sensor against its 95% interval, and the running mean of the full-state NEES.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IEKF | 3.249 | 0.864 | 7.29 | 8.66 | 7.36 | 89.1 | 97.6% |
| IEKS | 2.525 | 1.994 | 4.75 | 4.97 | 2.31 | 5.4 | 99.4% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.300 | [2.264, 3.838] | consistent |
| NIS, magnetometer | 1 | 0.225 | [0.224, 2.347] | consistent |
| NIS, DVL | 3 | 2.273 | [2.797, 3.210] | underconfident |
| NIS, altimeter | 1 | 0.994 | [0.872, 1.137] | consistent |
| NEES, full state, IEKF | 15 | 9.324 | [13.517, 16.559] | underconfident |
| NEES, position, IEKF | 3 | 2.098 | [2.472, 3.578] | underconfident |
| NEES, attitude, IEKF | 3 | 1.602 | [2.532, 3.507] | underconfident |
| NEES, full state, IEKS | 15 | 7.807 | [12.748, 17.433] | underconfident |
| NEES, position, IEKS | 3 | 2.328 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.319 | [1.654, 4.740] | underconfident |
