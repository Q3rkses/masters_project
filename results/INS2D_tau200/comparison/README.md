# 2-D strapdown INS, bias time constant 1000 s against 200 s

The IMU biases are first-order Gauss-Markov processes, simulated as

$$b_k = a\, b_{k-1} + w_k, \qquad a = e^{-\Delta t/\tau}, \qquad w_k \sim \mathcal{N}\!\left(0,\ \sigma^2 (1 - a^2)\right),$$

starting at $b_0 = 0$, so the variance grows as $\sigma^2 \left(1 - e^{-2t/\tau}\right)$. With $\tau = 1000$ s it reaches only 45% of $\sigma^2$ by the end of a 300 s run, and the bias behaves like a random walk. With $\tau = 200$ s it reaches 78% at 150 s and 95% by 300 s, so the envelope visibly flattens within one run without the bias wandering too fast. Every case in `results/INS2D` was rerun with $\tau = 200$ s in both the simulator and the filter. The stationary variances $\sigma^2$ and everything else were left unchanged. All numbers are from the matched white-noise variant, over the same 21 seeds (22 to 42).

Made by `scripts/compare_ins_2d_time_constants.py 200`.

## Figures

**`1_bias_process`** shows five simulated truth gyro-bias paths for each time constant, with the $\pm 2\sigma(t)$ envelope.

**`2_filter_bias_sigma`** shows the EKF's predicted gyro-bias standard deviation over one run (seed 42). The process now feeds in $2\sigma^2/\tau$, five times more uncertainty per second than at 1000 s, so the estimate levels off higher. That is about $0.005$ rad/s with GNSS only and $0.003$ rad/s with the magnetometer and DVL.

**`3_rmse_over_time`** shows the Monte Carlo position RMSE over time. GNSS only: the EKF error is higher throughout (time-averaged RMSE 0.72 m to 0.85 m). Heading stress: the UKF drifts faster in the outage (6.6 m to 7.8 m). The ordinary GNSS outage with DVL and magnetometer is the control and does not move.

**`4_anees`** shows position ANEES with its 95% t-interval for a selection of cases and estimators.

**`changes.csv`** holds every RMSE, outage RMSE, ANEES and ANIS of both runs side by side, with their consistency verdicts.

## Results for both time constants

| case | estimator | RMSE [m] 1000 / 200 s | ANEES 1000 / 200 s |
|---|---|---|---|
| multi-sensor fusion, figure-eight | EKF | 0.62 / 0.63 | 1.97 / 1.98 |
| GNSS outage, four-corner | EKF | 1.21 / 1.21 | 1.99 / 1.98 |
| GNSS only, figure-eight | EKF | 0.72 / 0.85 | 2.07 / 2.14 |
| heading stress, four-corner | UKF | 6.59 / 7.78 | 2.52 / 3.01 |
| heading stress, four-corner | URTSS | 2.79 / 3.31 | 2.35 / 2.69 |
| heading stress, four-corner | IEKF | 8.81 / 10.23 | 4.50 / 5.48 |
| heading stress, four-corner | IEKS | 2.56 / 2.83 | 1.95 / 2.04 |

## Reading

Where the DVL and magnetometer observe the bias, the position results do not move. With GNSS only the EKF error rises by 13 cm and its ANEES is flagged mildly overconfident. Under heading stress the UKF and URTSS go from consistent to overconfident, the IEKF gets worse, and the IEKS stays consistent. In the mistuned GNSS outage the DVL ANIS drops from 1.97 to 1.93, which is flagged underconfident.
