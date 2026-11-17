# 3-D strapdown INS, base case: all estimators compared

[Back to the base case](../README.md) | [Extended Kalman filter and ERTS](../extended/README.md) | [Unscented Kalman filter and URTSS](../unscented/README.md) | [Iterated extended Kalman filter and smoother](../iterated/README.md)

## Problem and setup

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. GNSS is denied from 45 s to 255 s. The filter prior is centred on the true initial state.

The EKF with the ERTS, the UKF with the URTSS and the IEKF with the IEKS are run on the same data, so every estimate below has seen exactly the same IMU readings and fixes.

The truth and the prior mean start at the origin, level, heading north at 2 m/s, with zero biases. Prior standard deviations: 0.63 m in position, 0.2 m/s in velocity, 0.2 rad in attitude.

The state, motion model, sensors and parameters are given in the [base case overview](../README.md).

| run | trajectory | magnetometer period |
| --- | --- | --- |
| [`figure_eight`](figure_eight/) | figure-eight | 0.25 to 1 s |
| [`rounded_rectangle`](rounded_rectangle/) | four-corner | 0.25 to 1 s |

## Reproduction

From the project root, after building:

```bash
./build/kalman_filtering_and_smoothing results/INS3D/base_case/comparison
for run in results/INS3D/base_case/comparison/*/; do
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
| EKF | 0.931 | 0.752 | 1.79 | 2.17 | 1.78 | 4.0 | 98.3% |
| UKF | 0.944 | 0.742 | 1.83 | 2.22 | 1.79 | 4.0 | 98.3% |
| IEKF | 0.912 | 0.730 | 1.75 | 2.13 | 1.79 | 4.0 | 98.4% |
| ERTS | 0.708 | 0.424 | 1.31 | 1.56 | 1.35 | 3.2 | 98.6% |
| URTSS | 0.708 | 0.423 | 1.31 | 1.56 | 1.34 | 3.2 | 98.5% |
| IEKS | 0.685 | 0.441 | 1.26 | 1.51 | 1.26 | 3.2 | 98.6% |

| statistic | dof | EKF / ERTS | UKF / URTSS | IEKF / IEKS |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.548 (consistent) | 3.567 (consistent) | 3.538 (consistent) |
| NIS, magnetometer | 1 | 0.805 (underconfident) | 0.805 (underconfident) | 0.803 (underconfident) |
| NIS, DVL | 3 | 2.300 (underconfident) | 2.263 (underconfident) | 2.300 (underconfident) |
| NIS, altimeter | 1 | 1.015 (consistent) | 1.010 (consistent) | 1.016 (consistent) |
| NEES, full state, filter | 15 | 8.829 (underconfident) | 9.005 (underconfident) | 8.838 (underconfident) |
| NEES, position, filter | 3 | 2.341 (underconfident) | 2.353 (underconfident) | 2.298 (underconfident) |
| NEES, attitude, filter | 3 | 1.302 (underconfident) | 1.317 (underconfident) | 1.311 (underconfident) |
| NEES, full state, smoother | 15 | 10.536 (underconfident) | 10.202 (underconfident) | 9.762 (underconfident) |
| NEES, position, smoother | 3 | 2.913 (consistent) | 2.897 (consistent) | 2.853 (consistent) |
| NEES, attitude, smoother | 3 | 2.058 (consistent) | 1.989 (consistent) | 1.756 (consistent) |

### Four-corner

![1_scenario](rounded_rectangle/figures/1_scenario.png)

*The true path with the body axes, and when each sensor fired.*

![2_variants](rounded_rectangle/figures/2_variants.png)

*From the top: position error of the filters, position error of the smoothers, attitude error of the filters and attitude error of the smoothers. Shaded where GNSS is denied.*

| estimator | RMSE position [m] | median [m] | 95th percentile [m] | max [m] | RMSE attitude [°] | max attitude [°] | true position inside 95% ellipsoid |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EKF | 1.029 | 0.492 | 2.02 | 2.29 | 1.72 | 4.1 | 99.0% |
| UKF | 1.046 | 0.494 | 2.06 | 2.32 | 1.72 | 4.1 | 98.9% |
| IEKF | 1.024 | 0.477 | 1.99 | 2.27 | 1.72 | 4.0 | 99.0% |
| ERTS | 0.637 | 0.378 | 1.26 | 1.46 | 1.18 | 3.2 | 99.7% |
| URTSS | 0.631 | 0.378 | 1.25 | 1.45 | 1.16 | 3.1 | 99.7% |
| IEKS | 0.613 | 0.399 | 1.19 | 1.40 | 1.14 | 3.2 | 99.7% |

| statistic | dof | EKF / ERTS | UKF / URTSS | IEKF / IEKS |
| --- | --- | --- | --- | --- |
| NIS, GNSS | 3 | 3.514 (consistent) | 3.531 (consistent) | 3.507 (consistent) |
| NIS, magnetometer | 1 | 0.801 (underconfident) | 0.801 (underconfident) | 0.800 (underconfident) |
| NIS, DVL | 3 | 2.319 (underconfident) | 2.281 (underconfident) | 2.317 (underconfident) |
| NIS, altimeter | 1 | 1.011 (consistent) | 1.006 (consistent) | 1.013 (consistent) |
| NEES, full state, filter | 15 | 8.298 (underconfident) | 8.391 (underconfident) | 8.315 (underconfident) |
| NEES, position, filter | 3 | 2.111 (underconfident) | 2.137 (underconfident) | 2.099 (underconfident) |
| NEES, attitude, filter | 3 | 1.217 (underconfident) | 1.221 (underconfident) | 1.222 (underconfident) |
| NEES, full state, smoother | 15 | 9.276 (underconfident) | 8.787 (underconfident) | 8.732 (underconfident) |
| NEES, position, smoother | 3 | 2.519 (consistent) | 2.487 (consistent) | 2.455 (consistent) |
| NEES, attitude, smoother | 3 | 1.588 (underconfident) | 1.493 (underconfident) | 1.434 (underconfident) |
