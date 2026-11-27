# 3-D strapdown INS, exaggerated poor initialization: all estimators compared

[Back to the exaggerated poor initialization](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Unscented Kalman filter and URTSS](../unscented/README.md) | [Iterated extended Kalman filter and smoother](../iterated/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter starts from a prior far from the truth, and the magnetometer fires only every 30 to 60 s instead of every 0.25 to 1 s, so the yaw error of the prior is corrected only rarely.

The EKF with the ERTS, the UKF with the URTSS and the IEKF with the IEKS are run on the same data, so every estimate below has seen exactly the same IMU readings and fixes.

The truth starts at the origin, level, heading north at 2 m/s. The prior mean is off by $(3, -3, 1)$ m in position, $(-0.5, 0.5, 0.2)$ m/s in velocity and by the rotation vector $(0.3, -0.3, 1.5)$ rad in attitude, 89° in total. Each offset is one standard deviation of the prior.

The state, motion model, sensors and parameters are given in the [exaggerated poor initialization overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 30 to 60 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 30 to 60 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/exaggerated_poor_initialization/comparison
for run in results/INS3D/exaggerated_poor_initialization/comparison/*/; do
    python3 scripts/analyze_ins_3d.py "$run/data"
done
```

Every number below is taken from the run's `stats.txt`.

## Results

### Figure-eight

![1_scenario](figure_eight/figures/1_scenario.png)

*The true path with the body axes, and when each sensor fired.*

![2_variants](figure_eight/figures/2_variants.png)

*From the top: position error of the filters, position error of the smoothers, attitude error of the filters and attitude error of the smoothers. Shaded where GNSS is denied.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EKF | 7.517 | 3.880 | 17.03 | 25.10 | 17.72 | 89.4 | 55.3% |
| UKF | 10.329 | 5.369 | 22.51 | 35.88 | 21.49 | 89.4 | 50.0% |
| IEKF | 2.752 | 1.418 | 7.58 | 8.26 | 7.73 | 89.4 | 97.2% |
| ERTS | 6.429 | 3.523 | 14.03 | 14.43 | 7.05 | 44.2 | 51.1% |
| URTSS | 8.418 | 4.455 | 18.63 | 19.19 | 9.65 | 61.5 | 43.1% |
| IEKS | 3.023 | 1.766 | 6.46 | 6.91 | 2.41 | 4.9 | 99.0% |

| statistic | dof | EKF / ERTS | UKF / URTSS | IEKF / IEKS |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 14.228 (overconfident) | 18.134 (overconfident) | 3.604 (consistent) |
| NIS, magnetometer | 1 | 5.947 (overconfident) | 11.108 (overconfident) | 0.307 (consistent) |
| NIS, DVL | 3 | 2.324 (underconfident) | 2.230 (underconfident) | 2.269 (underconfident) |
| NIS, altimeter | 1 | 1.002 (consistent) | 0.997 (consistent) | 0.999 (consistent) |
| NEES, full state, filter | 15 | 57.086 (overconfident) | 88.094 (overconfident) | 10.204 (underconfident) |
| NEES, position, filter | 3 | 16.108 (overconfident) | 29.093 (overconfident) | 2.299 (underconfident) |
| NEES, attitude, filter | 3 | 12.330 (overconfident) | 20.054 (overconfident) | 1.826 (underconfident) |
| NEES, full state, smoother | 15 | 55.842 (overconfident) | 81.044 (overconfident) | 7.879 (underconfident) |
| NEES, position, smoother | 3 | 20.225 (overconfident) | 34.566 (overconfident) | 2.257 (consistent) |
| NEES, attitude, smoother | 3 | 9.678 (overconfident) | 21.059 (overconfident) | 1.480 (underconfident) |

### Four-corner

![1_scenario](rounded_rectangle/figures/1_scenario.png)

*The true path with the body axes, and when each sensor fired.*

![2_variants](rounded_rectangle/figures/2_variants.png)

*From the top: position error of the filters, position error of the smoothers, attitude error of the filters and attitude error of the smoothers. Shaded where GNSS is denied.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EKF | 7.743 | 5.569 | 15.68 | 22.15 | 17.30 | 89.1 | 55.3% |
| UKF | 9.991 | 5.860 | 20.38 | 30.58 | 20.37 | 89.2 | 47.9% |
| IEKF | 3.265 | 0.862 | 7.26 | 8.65 | 7.35 | 89.1 | 97.5% |
| ERTS | 5.583 | 3.571 | 11.74 | 12.19 | 7.14 | 47.0 | 51.7% |
| URTSS | 6.929 | 3.852 | 14.89 | 15.48 | 9.36 | 62.5 | 47.7% |
| IEKS | 2.545 | 1.919 | 4.82 | 5.03 | 2.31 | 5.5 | 99.4% |

| statistic | dof | EKF / ERTS | UKF / URTSS | IEKF / IEKS |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 14.771 (overconfident) | 18.097 (overconfident) | 3.291 (consistent) |
| NIS, magnetometer | 1 | 5.002 (overconfident) | 8.880 (overconfident) | 0.211 (consistent) |
| NIS, DVL | 3 | 2.338 (underconfident) | 2.229 (underconfident) | 2.275 (underconfident) |
| NIS, altimeter | 1 | 1.005 (consistent) | 1.001 (consistent) | 1.005 (consistent) |
| NEES, full state, filter | 15 | 53.453 (overconfident) | 72.750 (overconfident) | 9.370 (underconfident) |
| NEES, position, filter | 3 | 13.595 (overconfident) | 19.320 (overconfident) | 2.103 (underconfident) |
| NEES, attitude, filter | 3 | 11.829 (overconfident) | 18.000 (overconfident) | 1.623 (underconfident) |
| NEES, full state, smoother | 15 | 51.747 (overconfident) | 65.752 (overconfident) | 7.515 (underconfident) |
| NEES, position, smoother | 3 | 16.481 (overconfident) | 22.171 (overconfident) | 2.295 (consistent) |
| NEES, attitude, smoother | 3 | 10.177 (overconfident) | 21.278 (overconfident) | 1.286 (underconfident) |
