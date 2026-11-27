# 3-D strapdown INS, poor initialization: all estimators compared

[Back to the poor initialization](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Unscented Kalman filter and URTSS](../unscented/README.md) | [Iterated extended Kalman filter and smoother](../iterated/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter starts from a prior far from the truth.

The EKF with the ERTS, the UKF with the URTSS and the IEKF with the IEKS are run on the same data, so every estimate below has seen exactly the same IMU readings and fixes.

The truth starts at the origin, level, heading north at 2 m/s. The prior mean is off by $(3, -3, 1)$ m in position, $(-0.5, 0.5, 0.2)$ m/s in velocity and by the rotation vector $(0.3, -0.3, 1.5)$ rad in attitude, 89° in total. Each offset is one standard deviation of the prior.

The state, motion model, sensors and parameters are given in the [poor initialization overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 0.25 to 1 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 0.25 to 1 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/poor_initialization/comparison
for run in results/INS3D/poor_initialization/comparison/*/; do
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
| EKF | 1.008 | 0.786 | 1.84 | 4.43 | 5.52 | 89.4 | 96.9% |
| UKF | 1.021 | 0.763 | 1.87 | 5.26 | 5.76 | 89.4 | 97.6% |
| IEKF | 0.980 | 0.749 | 1.79 | 4.37 | 5.54 | 89.4 | 98.1% |
| ERTS | 0.726 | 0.435 | 1.33 | 1.59 | 1.32 | 9.2 | 96.7% |
| URTSS | 0.722 | 0.425 | 1.32 | 1.79 | 1.65 | 15.1 | 97.4% |
| IEKS | 0.693 | 0.447 | 1.27 | 1.52 | 1.27 | 3.2 | 98.3% |

| statistic | dof | EKF / ERTS | UKF / URTSS | IEKF / IEKS |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.584 (consistent) | 3.646 (overconfident) | 3.548 (consistent) |
| NIS, magnetometer | 1 | 0.828 (underconfident) | 1.203 (consistent) | 0.824 (underconfident) |
| NIS, DVL | 3 | 2.660 (consistent) | 2.518 (underconfident) | 2.472 (underconfident) |
| NIS, altimeter | 1 | 1.087 (consistent) | 1.055 (consistent) | 1.016 (consistent) |
| NEES, full state, filter | 15 | 10.528 (underconfident) | 12.209 (consistent) | 9.369 (underconfident) |
| NEES, position, filter | 3 | 2.614 (consistent) | 2.495 (consistent) | 2.360 (underconfident) |
| NEES, attitude, filter | 3 | 1.420 (underconfident) | 2.334 (consistent) | 1.520 (underconfident) |
| NEES, full state, smoother | 15 | 12.815 (consistent) | 12.194 (consistent) | 9.985 (underconfident) |
| NEES, position, smoother | 3 | 3.082 (consistent) | 3.130 (consistent) | 2.887 (consistent) |
| NEES, attitude, smoother | 3 | 1.855 (consistent) | 2.303 (consistent) | 1.773 (consistent) |

### Four-corner

![1_scenario](rounded_rectangle/figures/1_scenario.png)

*The true path with the body axes, and when each sensor fired.*

![2_variants](rounded_rectangle/figures/2_variants.png)

*From the top: position error of the filters, position error of the smoothers, attitude error of the filters and attitude error of the smoothers. Shaded where GNSS is denied.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EKF | 1.140 | 0.505 | 2.12 | 4.37 | 5.46 | 89.1 | 98.1% |
| UKF | 1.174 | 0.498 | 2.17 | 5.64 | 5.64 | 89.1 | 98.4% |
| IEKF | 1.137 | 0.502 | 2.10 | 4.37 | 5.47 | 89.1 | 98.9% |
| ERTS | 0.617 | 0.376 | 1.22 | 1.41 | 1.23 | 8.4 | 98.3% |
| URTSS | 0.615 | 0.379 | 1.22 | 1.43 | 1.43 | 12.5 | 98.8% |
| IEKS | 0.592 | 0.407 | 1.15 | 1.35 | 1.15 | 3.3 | 99.4% |

| statistic | dof | EKF / ERTS | UKF / URTSS | IEKF / IEKS |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.552 (consistent) | 3.600 (consistent) | 3.529 (consistent) |
| NIS, magnetometer | 1 | 0.822 (underconfident) | 1.145 (consistent) | 0.817 (underconfident) |
| NIS, DVL | 3 | 2.606 (underconfident) | 2.438 (underconfident) | 2.460 (underconfident) |
| NIS, altimeter | 1 | 1.066 (consistent) | 1.024 (consistent) | 1.013 (consistent) |
| NEES, full state, filter | 15 | 9.480 (underconfident) | 10.618 (underconfident) | 8.825 (underconfident) |
| NEES, position, filter | 3 | 2.346 (underconfident) | 2.271 (underconfident) | 2.192 (underconfident) |
| NEES, attitude, filter | 3 | 1.341 (underconfident) | 2.105 (consistent) | 1.432 (underconfident) |
| NEES, full state, smoother | 15 | 11.188 (consistent) | 10.260 (underconfident) | 8.775 (underconfident) |
| NEES, position, smoother | 3 | 2.536 (consistent) | 2.544 (consistent) | 2.396 (consistent) |
| NEES, attitude, smoother | 3 | 1.581 (consistent) | 1.807 (consistent) | 1.456 (underconfident) |
