# 3-D strapdown INS, base case

[Back to the overview](../../../README.md) | [Next: poor initialization](../poor_initialization/README.md)

An underwater vehicle estimates its pose in three dimensions from an IMU, aided by GNSS, a magnetometer, a DVL and an altimeter. The attitude is a unit quaternion. GNSS is lost for most of the run. The extended, unscented and iterated filters and their smoothers are each run on the same data, and then compared.

|               |                                  |
| ------------- | -------------------------------- |
| **Run**       | 30000 timesteps (300 s at $\Delta t = 0.01$ s), seed 42 |
| **Cases**     | figure-eight and four-corner, for each of `extended/`, `unscented/`, `iterated/` and `comparison/` |
| **Reproduce** | see [Reproduction](#reproduction) |

## Problem formulation

### State and motion model

The state collects position $p$ and velocity $v$ in the navigation frame (north, east, down), the unit quaternion $q$ that rotates the body frame to the navigation frame, and the accelerometer and gyro biases,

$$
x = \begin{bmatrix} p^\top & v^\top & q^\top & b_a^\top & b_g^\top \end{bmatrix}^\top \in \mathbb{R}^{16}.
$$

The covariance lives on the 15-dimensional error state $\delta x = [\delta p,\ \delta v,\ \delta\theta,\ \delta b_a,\ \delta b_g]$, where the attitude error is a rotation vector in the body frame, $q_\text{true} = q \otimes \mathrm{Exp}(\delta\theta)$.

The IMU reading $u_k = (a_k, \omega_k)$ is corrected with the estimated biases, $\tilde a_k = a_k - b_{a,k}$ and $\tilde\omega_k = \omega_k - b_{g,k}$. With $R_{k+1/2}$ the attitude at the middle of the step and $g = [0,\ 0,\ 9.81]^\top$,

$$
\begin{aligned}
p_{k+1} &= p_k + \Delta t\, v_k + \tfrac{1}{2}\Delta t^2 \left(R_{k+1/2}\,\tilde a_k + g\right), \\
v_{k+1} &= v_k + \Delta t \left(R_{k+1/2}\,\tilde a_k + g\right), \\
q_{k+1} &= q_k \otimes \mathrm{Exp}(\Delta t\, \tilde\omega_k), \\
b_{a,k+1} &= e^{-\Delta t / T_a}\, b_{a,k}, \qquad b_{g,k+1} = e^{-\Delta t / T_g}\, b_{g,k},
\end{aligned}
$$

with process noise $w_k \sim \mathcal{N}(0, Q_k)$ on the error state. The biases are first-order Gauss-Markov processes with time constant $T$ and stationary variance $\sigma_b^2$.

### Sensors

| sensor | measurement | noise variance | period |
| --- | --- | --- | --- |
| GNSS | $z = p$ | $0.20$ m² per axis | 1 to 3 s, denied from 45 s to 255 s |
| magnetometer | yaw of $R(q)$, $z = \operatorname{atan2}(R_{21}, R_{11})$ | $3\cdot10^{-3}$ rad² | 0.25 to 1 s |
| DVL | body-frame velocity, $z = R(q)^\top v$ | $10^{-2}$ (m/s)² per axis | 0.2 to 1 s |
| altimeter | vertical position, $z = p_z$ | $10^{-2}$ m² | 0.2 to 1 s |

Each sensor fires at random intervals drawn uniformly from its period range. The schedule is drawn once, so every estimator sees the same fixes.

### Simulation parameters

| parameter | value |
| --- | --- |
| time step $\Delta t$, steps, duration | 0.01 s, 30000, 300 s |
| accelerometer bias: variance $\sigma_{b_a}^2$, time constant $T_a$ | $5\cdot10^{-2}$ (m/s²)², 1000 s, every axis |
| gyro bias: variance $\sigma_{b_g}^2$, time constant $T_g$ | $5\cdot10^{-5}$ (rad/s)², 1000 s, every axis |
| white noise intensity, accelerometer / gyro | $5\cdot10^{-4}$ (m/s²)²·s / $5\cdot10^{-4}$ rad²/s |
| seed | 42 |

The filters are given the same IMU noise parameters the data is simulated with.

### Trajectories

Both paths are flown at 2 m/s in a plane tilted 0.15 rad about the $x$ axis, so the depth changes along the loop. The vehicle rolls into each turn as in a coordinated turn, building up the roll over 5 m of path.

| trajectory | geometry |
| --- | --- |
| four-corner (`rounded_rectangle`) | $80 \times 50$ m rectangle with corner radius 10 m, left turns only |
| figure-eight (`figure_eight`) | two circles of radius 20 m with centres 70 m apart, joined by two crossing straights, one left and one right turn per lap |

### Estimators

| folder | filter | smoother |
| --- | --- | --- |
| `extended/` | extended Kalman filter (EKF) | extended Rauch-Tung-Striebel smoother (ERTS) |
| `unscented/` | unscented Kalman filter (UKF), $\alpha = 1$, $\beta = 2$, $\kappa = 0$ | unscented Rauch-Tung-Striebel smoother (URTSS) |
| `iterated/` | iterated extended Kalman filter (IEKF) | iterated extended Kalman smoother (IEKS), run over the IEKF |
| `comparison/` | all three | all three |

The IEKF and IEKS use Levenberg-Marquardt damping, starting at $\lambda = 10^{-2}$, multiplying it by 10 after a rejected step and dividing it by 10 after an accepted one, with at most 50 accepted iterations.

### Consistency

The NIS of each sensor and the NEES of the full state, the position and the attitude are averaged over time and compared with the 95% interval of that average. Above the interval the estimator is overconfident, below it underconfident.

### Initial condition

The filter prior is centred on the true initial state, at the origin, level, heading north at 2 m/s:

| | value |
| --- | --- |
| prior mean $\hat x_0$ | $p = 0$, $v = [2,\ 0,\ 0]^\top$, $q = [1,\ 0,\ 0,\ 0]^\top$, biases 0 |
| prior covariance $\hat P_0$ | $\mathrm{diag}(0.4\,I_3,\ 0.04\,I_3,\ 0.04\,I_3,\ 0.004\,I_3,\ 0.0004\,I_3)$ |

The true initial state is the prior mean, not a draw from the prior.

## Reproduction

Build once, then simulate every case of this page and analyse each one:

```bash
cmake -S . -B build && cmake --build build
./build/kalman_filtering_and_smoothing results/INS3D/base_case
for case in results/INS3D/base_case/*/*/; do
    python3 scripts/analyze_ins_3d.py "$case/data"
done
```

The executable runs every `config.yaml` it finds below the folder it is given and writes each case's `data/`. The analysis writes `stats.txt`, which holds every number for that case, and the figures in `figures/`. Adding `--pdf` to the analysis also writes each figure as a PDF.

`data/` holds `truth.csv` with the true state, and per estimator `<prefix>_filter.csv` and `<prefix>_smoother.csv` with the mean and full covariance at every step and `<prefix>_<sensor>_fixes.csv` with the measurement, innovation and $S$ of every fix. The prefix is `ekf`, `ukf` or `ieks`.

## Results

Each method has its own page with the setup, the figures and the numbers of its runs (figure-eight and four-corner).

| page | estimators |
| --- | --- |
| [extended](extended/README.md) | EKF and ERTS |
| [unscented](unscented/README.md) | UKF and URTSS |
| [iterated](iterated/README.md) | IEKF and IEKS |
| [comparison](comparison/README.md) | all three on the same data |

## Related results

- [Poor initialization](../poor_initialization/README.md): the same scenario with a filter prior far from the truth.
- [2-D strapdown INS, IEKS](../../INS2D/ieks/README.md): the planar version of the same estimators.
