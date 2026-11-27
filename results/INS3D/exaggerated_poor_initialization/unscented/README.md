# 3-D strapdown INS, exaggerated poor initialization: Unscented Kalman filter and URTSS

[Back to the exaggerated poor initialization](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Iterated extended Kalman filter and smoother](../iterated/README.md) | [All estimators compared](../comparison/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter starts from a prior far from the truth, and the magnetometer fires only every 30 to 60 s instead of every 0.25 to 1 s, so the yaw error of the prior is corrected only rarely.

The unscented Kalman filter (UKF) carries the mean and covariance through the models with sigma points on the error state, with $\alpha = 1$, $\beta = 2$, $\kappa = 0$. The unscented Rauch-Tung-Striebel smoother (URTSS) runs backward over the stored UKF output.

The truth starts at the origin, level, heading north at 2 m/s. The prior mean is off by $(3, -3, 1)$ m in position, $(-0.5, 0.5, 0.2)$ m/s in velocity and by the rotation vector $(0.3, -0.3, 1.5)$ rad in attitude, 89° in total. Each offset is one standard deviation of the prior.

The state, motion model, sensors and parameters are given in the [exaggerated poor initialization overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 30 to 60 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 30 to 60 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/exaggerated_poor_initialization/unscented
for run in results/INS3D/exaggerated_poor_initialization/unscented/*/; do
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
| UKF | 10.329 | 5.369 | 22.51 | 35.88 | 21.49 | 89.4 | 50.0% |
| URTSS | 8.418 | 4.455 | 18.63 | 19.19 | 9.65 | 61.5 | 43.1% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 18.134 | [1.349, 5.300] | overconfident |
| NIS, magnetometer | 1 | 11.108 | [0.001, 5.024] | overconfident |
| NIS, DVL | 3 | 2.230 | [2.796, 3.212] | underconfident |
| NIS, altimeter | 1 | 0.997 | [0.873, 1.136] | consistent |
| NEES, full state, UKF | 15 | 88.094 | [12.535, 17.683] | overconfident |
| NEES, position, UKF | 3 | 29.093 | [1.447, 5.109] | overconfident |
| NEES, attitude, UKF | 3 | 20.054 | [1.768, 4.553] | overconfident |
| NEES, full state, URTSS | 15 | 81.044 | [6.262, 27.488] | overconfident |
| NEES, position, URTSS | 3 | 34.566 | [0.216, 9.348] | overconfident |
| NEES, attitude, URTSS | 3 | 21.059 | [0.216, 9.348] | overconfident |

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
| UKF | 9.991 | 5.860 | 20.38 | 30.58 | 20.37 | 89.2 | 47.9% |
| URTSS | 6.929 | 3.852 | 14.89 | 15.48 | 9.36 | 62.5 | 47.7% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 18.097 | [1.324, 5.349] | overconfident |
| NIS, magnetometer | 1 | 8.880 | [0.009, 4.197] | overconfident |
| NIS, DVL | 3 | 2.229 | [2.799, 3.208] | underconfident |
| NIS, altimeter | 1 | 1.001 | [0.872, 1.137] | consistent |
| NEES, full state, UKF | 15 | 72.750 | [12.608, 17.597] | overconfident |
| NEES, position, UKF | 3 | 19.320 | [1.859, 4.410] | overconfident |
| NEES, attitude, UKF | 3 | 18.000 | [1.793, 4.512] | overconfident |
| NEES, full state, URTSS | 15 | 65.752 | [6.262, 27.488] | overconfident |
| NEES, position, URTSS | 3 | 22.171 | [0.216, 9.348] | overconfident |
| NEES, attitude, URTSS | 3 | 21.278 | [0.216, 9.348] | overconfident |
