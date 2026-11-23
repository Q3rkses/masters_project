# 1-D random walk

[Back to the overview](../../README.md) | [Next: 2-D random walk](../2D/README.md)

The first end-to-end test of the filter and the smoother, on the simplest model
there is. The filter is given exactly the model the data was simulated from (a
_matched_ model), so if the implementation is right the estimates should be
accurate and consistent. If they are not, the bug is in the code, not in a
modelling choice.

|               |                                                               |
| ------------- | ------------------------------------------------------------- |
| **Status**    | done, consistent over a Monte Carlo of 21 seeds               |
| **Run**       | Monte Carlo over seeds 22 to 42, 21 runs of 1000 timesteps     |
| **Reproduce** | see [Reproduce](#reproduce)                                   |

## Setup

A 1-D random walk observed directly:

$$
x_k = x_{k-1} + w_k, \quad w_k \sim \mathcal{N}(0, q)
\qquad \Rightarrow \qquad F = 1,\ Q = q
$$

$$
z_k = x_k + v_k, \quad v_k \sim \mathcal{N}(0, r)
\qquad \Rightarrow \qquad H = 1,\ R = r
$$

| parameter        | value    | meaning                    |
| ---------------- | -------- | -------------------------- |
| $q$              | 0.05     | process noise variance     |
| $r$              | 0.5      | measurement noise variance |
| $\hat x_0$, $P_0$ | 0.0, 1.0 | prior mean and variance    |
| timesteps        | 1000     |                            |

- $q$ and $r$ are declared once in `config.yaml` and handed to both the
  simulator and the filter, so the filter's model matches the truth by
  construction.
- The true initial state is drawn from the prior $\mathcal{N}(\hat x_0, P_0)$,
  so the prior is honest from the first step.
- The Kalman filter runs predict then update at every timestep, starting from
  the prior. The RTS smoother then runs backward over the stored filter output,
  starting from the last filtered estimate.

### Monte Carlo

A single run says little about consistency, because the time average of NEES
from one realisation scatters widely. Over the 21 seeds used here the filter's
single-run ANEES ranges from 0.81 to 1.14. The experiment is therefore repeated
for $M = 21$ seeds (22 to 42), each a full independent realisation of the truth,
the measurements and the prior draw, with everything else in the configuration
unchanged.

For each run $i$ and timestep $k$, with estimation error $\tilde x_k^i = x_k^i - \hat x_k^i$
and innovation $\nu_k^i$ with covariance $S_k$,

$$
\epsilon_k^i = \frac{(\tilde x_k^i)^2}{P_k}, \qquad
\eta_k^i = \frac{(\nu_k^i)^2}{S_k},
$$

and the run-averaged statistics are

$$
\bar\epsilon_k = \frac{1}{M}\sum_{i=1}^{M} \epsilon_k^i, \qquad
\bar\eta_k = \frac{1}{M}\sum_{i=1}^{M} \eta_k^i .
$$

Because the runs are independent, $M\bar\epsilon_k \sim \chi^2_{M n}$ under a
consistent filter, with $n = 1$ here. Every timestep therefore gets an exact 95%
band $[\chi^2_{M,\,0.025},\ \chi^2_{M,\,0.975}]/M = [0.49,\ 1.69]$, with no
correction for autocorrelation over time. The same holds for NIS with $m = 1$.

The overall averages over all runs and timesteps are tested with the spread of
the per-run averages. These are independent across runs, so a 95% $t$-interval
over the 21 values is a valid interval for the overall average, which should
contain 1.

$P_k$ and $S_k$ do not depend on the data in a linear Kalman filter, so they are
identical in every run.

## Reproduce

To regenerate the figures and statistics from the stored data:

```bash
python3 scripts/monte_carlo_1d.py --skip-run results/1D/monte_carlo
```

To regenerate the data as well:

```bash
cmake -S . -B build && cmake --build build
python3 scripts/monte_carlo_1d.py results/1D/monte_carlo   # writes one seed_<n>/ per seed, then the figures
```

The script copies `results/1D/config.yaml` into `monte_carlo/seed_<n>/config.yaml`
with only the seed changed, runs the executable on all of them, and writes
`monte_carlo/stats.txt` and `monte_carlo/figures/`. Add `--pdf` for vector
figures.

The example run shown in the trajectory figures is the seed 42 case in this
folder, and is regenerated with

```bash
./build/kalman_filtering_and_smoothing results/1D     # writes results/1D/data/
python3 scripts/analyze_1d.py results/1D/data         # writes results/1D/figures/
```

## Data

Every `monte_carlo/seed_<n>/data/` (and the example run in `data/`) holds four
CSV files with one row per timestep and a shared `timestep` column, so they can
be joined on it. The column layout is documented in `lib/results.hpp`.

| file                    | columns                                                                                     |
| ----------------------- | ------------------------------------------------------------------------------------------- |
| `truth.csv`             | `timestep, x_true_0`                                                                        |
| `measurement_fixes.csv` | `timestep, z_0, innovation_0, S_0_0`                                                        |
| `filter.csv`            | `timestep, x_predicted_0, P_predicted_0_0, x_updated_0, innovation_0, S_0_0, P_updated_0_0` |
| `smoother.csv`          | `timestep, x_smoothed_0, P_smoothed_0_0`                                                    |

`S` is the innovation covariance, kept so that NIS can be computed later.

## Results

### What one run looks like

The trajectory figures show a single realisation (seed 42), since 21 overlaid
trajectories would be unreadable. Its errors are typical of the Monte Carlo set,
with filter RMSE 0.365 and smoother RMSE 0.274 against the means of 0.358 and
0.276 below.

The problem: a hidden random walk and the noisy measurements of it.

![simulation](figures/1_simulation_500.png)

The Kalman filter with its 95% confidence band. It tracks, but lags at the turns
and inherits visible jitter from the measurements, because each estimate only
knows the past.

![filter](figures/2_filter_500.png)

The RTS smoother over the same data. Each estimate uses the whole record, so it
is smoother, closer to truth, and reports a narrower band.

![smoother](figures/3_smoother_500.png)

Side by side, at three zoom levels. The smoother's band is about 76% as wide as
the filter's (its variance is about 58% of the filter's).

![filter vs smoother, first 150 timesteps](figures/4_filter_vs_smoother_150.png)

![filter vs smoother, first 500 timesteps](figures/4_filter_vs_smoother_500.png)

![filter vs smoother, all 1000 timesteps](figures/4_filter_vs_smoother_1000.png)

### Accuracy over 21 runs

Mean and standard deviation across the runs of each run's time-averaged error.

|          | RMSE            | MAE             | mean $P$ |
| -------- | --------------- | --------------- | -------- |
| filter   | 0.3581 ± 0.0153 | 0.2857 ± 0.0122 | 0.1354   |
| smoother | 0.2759 ± 0.0098 | 0.2200 ± 0.0075 | 0.0783   |

The smoother has the lower RMSE in all 21 runs, on average 22.9% lower, and its
mean $P$ is 42% lower.

The RMSE across the runs at each timestep, $\sqrt{\frac{1}{M}\sum_i (\tilde x_k^i)^2}$,
should scatter around the standard deviation the estimator reports, $\sqrt{P_k}$.
It does, for both the filter and the smoother, including the initial transient
and the smoother's rise to the filter's value at the final step.

![RMSE across runs against sqrt(P)](monte_carlo/figures/mc_rmse_vs_sigma.png)

### Filter vs smoother variance

![filter vs smoother variance](figures/6_variance.png)

The smoother's reported variance is never larger than the filter's, and the two
are equal at the final timestep, matching Figure 12.2 in Sarkka's _Bayesian
Filtering and Smoothing_ for this exact example (a Gaussian random walk).
It's a property of the RTS recursion itself, since the smoother's covariance
update only ever subtracts a positive semi-definite term from the filter's, so
it is a check on the implementation, not on the data. It is the same in every
run.

## Consistency

NEES averaged over the 21 runs at every timestep, for the filter and the
smoother, against the exact 95% band. The dashed line is the expected value
$\mathbb{E}[\chi^2_1] = 1$.

![NEES averaged over runs](monte_carlo/figures/mc_nees.png)

NIS averaged over the 21 runs, against the same band.

![NIS averaged over runs](monte_carlo/figures/mc_nis.png)

About 95% of the timesteps should land inside the band if the estimator is
consistent, and they do.

| statistic     | timesteps inside [0.49, 1.69] |
| ------------- | ----------------------------- |
| NIS           | 94.5%                         |
| NEES filter   | 95.9%                         |
| NEES smoother | 94.0%                         |

Averaged over all runs and timesteps, with the 95% $t$-interval from the spread
of the per-run averages:

| statistic      | value  | 95% interval     | per-run range    | verdict                    |
| -------------- | ------ | ---------------- | ---------------- | -------------------------- |
| ANIS           | 0.9815 | [0.9598, 1.0032] | [0.8660, 1.0728] | consistent                 |
| ANEES filter   | 0.9488 | [0.9121, 0.9856] | [0.8053, 1.1437] | slightly low, see below    |
| ANEES smoother | 0.9741 | [0.9429, 1.0052] | [0.8446, 1.0911] | consistent                 |

The filter's ANEES sits just below its interval for this set of seeds. This is
the seed set and not the filter. A check over 200 other seeds (1000 to 1199)
gives a filter ANEES of $1.0005 \pm 0.0053$, a smoother ANEES of
$1.0040 \pm 0.0055$ and an ANIS of $0.9987 \pm 0.0033$ (mean ± standard error),
all on 1. Seeds 22 to 42 happen to land about three standard errors low, which
also shows how much a 21-run Monte Carlo can still scatter.

## Takeaways and limits

- With a matched model, the filter and the smoother are both consistent over the
  Monte Carlo, at every timestep and on average, and the smoother is more
  accurate and more confident in every run, as it should be. That is good
  evidence that the predict/update step and the RTS backward recursion are
  implemented correctly.
- The measured error across runs follows $\sqrt{P_k}$ over the whole run, so the
  covariance the estimators report is the covariance of their actual error.
- One run is not enough to judge consistency. Single-run ANEES values here span
  0.81 to 1.14, so a single run can look over- or underconfident by chance.
  Even 21 runs leave visible scatter in the overall average.
