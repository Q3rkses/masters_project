# 3-D strapdown INS, exaggerated poor initialization: Iterated extended Kalman filter and smoother

[Back to the exaggerated poor initialization](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Unscented Kalman filter and URTSS](../unscented/README.md) | [All estimators compared](../comparison/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter starts from a prior far from the truth, and the magnetometer fires only every 30 to 60 s instead of every 0.25 to 1 s, so the yaw error of the prior is corrected only rarely.

The iterated extended Kalman filter (IEKF) relinearises each measurement update at the improved estimate until it settles. The iterated extended Kalman smoother (IEKS) starts from the IEKF output and relinearises the motion and measurement models at the whole smoothed trajectory until it settles. Both use Levenberg-Marquardt damping, starting at $\lambda = 10^{-2}$, multiplying it by 10 after a rejected step and dividing it by 10 after an accepted one.

The truth starts at the origin, level, heading north at 2 m/s. The prior mean is off by $(3, -3, 1)$ m in position, $(-0.5, 0.5, 0.2)$ m/s in velocity and by the rotation vector $(0.3, -0.3, 1.5)$ rad in attitude, 89° in total. Each offset is one standard deviation of the prior.

The state, motion model, sensors and parameters are given in the [exaggerated poor initialization overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 30 to 60 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 30 to 60 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/exaggerated_poor_initialization/iterated
for run in results/INS3D/exaggerated_poor_initialization/iterated/*/; do
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
| IEKF | 2.752 | 1.418 | 7.58 | 8.26 | 7.73 | 89.4 | 97.2% |
| IEKS | 3.023 | 1.766 | 6.46 | 6.91 | 2.41 | 4.9 | 99.0% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.604 | [2.267, 3.834] | consistent |
| NIS, magnetometer | 1 | 0.307 | [0.154, 2.619] | consistent |
| NIS, DVL | 3 | 2.269 | [2.795, 3.212] | underconfident |
| NIS, altimeter | 1 | 0.999 | [0.872, 1.137] | consistent |
| NEES, full state, IEKF | 15 | 10.204 | [13.468, 16.613] | underconfident |
| NEES, position, IEKF | 3 | 2.299 | [2.439, 3.618] | underconfident |
| NEES, attitude, IEKF | 3 | 1.826 | [2.490, 3.557] | underconfident |
| NEES, full state, IEKS | 15 | 7.879 | [12.628, 17.574] | underconfident |
| NEES, position, IEKS | 3 | 2.257 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.480 | [1.603, 4.828] | underconfident |

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
| IEKF | 3.265 | 0.862 | 7.26 | 8.65 | 7.35 | 89.1 | 97.5% |
| IEKS | 2.545 | 1.919 | 4.82 | 5.03 | 2.31 | 5.5 | 99.4% |

| statistic | dof | average | 95% interval | verdict |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.291 | [2.271, 3.829] | consistent |
| NIS, magnetometer | 1 | 0.211 | [0.165, 2.572] | consistent |
| NIS, DVL | 3 | 2.275 | [2.798, 3.209] | underconfident |
| NIS, altimeter | 1 | 1.005 | [0.871, 1.138] | consistent |
| NEES, full state, IEKF | 15 | 9.370 | [13.515, 16.561] | underconfident |
| NEES, position, IEKF | 3 | 2.103 | [2.471, 3.580] | underconfident |
| NEES, attitude, IEKF | 3 | 1.623 | [2.537, 3.501] | underconfident |
| NEES, full state, IEKS | 15 | 7.515 | [12.757, 17.422] | underconfident |
| NEES, position, IEKS | 3 | 2.295 | [0.216, 9.348] | consistent |
| NEES, attitude, IEKS | 3 | 1.286 | [1.676, 4.704] | underconfident |
