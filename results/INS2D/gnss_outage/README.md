# 2-D strapdown INS, GNSS outage

[Back to the overview](../../../README.md) | [Previous: dead reckoning](../dead_reckoning/README.md) | [Next: UKF and URTSS](../ukf/README.md)

A single long GNSS outage in the middle of an otherwise GNSS-aided run,
rather than a constant sensor mix for the whole run like the other three
scenarios. GNSS pings at the same fast rate as the [GNSS-only
scenario](../only_gnss/README.md) (uniform(1, 3)s) for the first 15% and
last 15% of the run, and is denied entirely for the 70% in between.
Magnetometer and DVL stay active throughout, same rates as [multi-sensor
fusion](../multi_sensor_fusion/README.md), so the outage is dead reckoning
aided by heading and body-frame velocity, not totally unaided.

Run twice: once with the filter's IMU noise belief matched to the truth,
once deliberately mistuned (1.6x gyro, 1.8x accel), to see whether the
mismatch shows up differently during an outage than it does with GNSS
constantly available.

|               |                                                   |
| ------------- | ------------------------------------------------- |
| **Status**    | done, consistent except magnetometer               |
| **Run**       | 30000 timesteps (5 minutes at dt=0.01s), seed 42   |
| **Reproduce** | see [Reproduce](#reproduce)                        |

## Setup

Same shared parameters as [GNSS only](../only_gnss/README.md#setup), on two
trajectories (`rounded_rectangle` in `matched` and `mistuned`, `figure_eight`
in `figure_eight_matched` and `figure_eight_mistuned`), with:

| sensor       | period            | denied window | fixes per run |
| ------------ | ------------------ | -------------- | -------------- |
| GNSS         | uniform(1, 3)s      | 45-255s        | 45             |
| Magnetometer | uniform(0.25, 1)s   | never          | ~470           |
| DVL          | uniform(0.2, 1)s    | never          | ~490           |

`matched`: `filter_gyro_noise_variance` 5e-5, `filter_accelerometer_noise_variance`
[5e-2, 5e-2], same as true.
`mistuned`: `filter_gyro_noise_variance` 8e-5, `filter_accelerometer_noise_variance`
[9e-2, 9e-2], true stays the same as `matched`.
`figure_eight_matched` and `figure_eight_mistuned`: the same two settings on the
figure-eight path (see [GNSS only](../only_gnss/README.md#setup) for its
geometry), with the same sensors and the same denied window.

## Reproduce

```bash
python3 scripts/analyze_ins_2d.py results/INS2D/gnss_outage/<matched|mistuned|figure_eight_matched|figure_eight_mistuned>/data
```

To regenerate the data, copy
`results/INS2D/gnss_outage/<matched|mistuned|figure_eight_matched|figure_eight_mistuned>/config.yaml` over
`configs/strapdown_ins_2D.yaml`, then follow the same steps as
[GNSS only's reproduce section](../only_gnss/README.md#reproduce).

## Results

### Variance

![Filter vs smoother variance, matched](matched/figures/ins_6_variance.png)

Log scale, since the outage pushes the filter's variance up almost two
orders of magnitude before GNSS returns. The shape is the clean textbook
case: the filter's variance grows through the whole outage (bounded by
magnetometer/DVL, not unbounded the way pure dead reckoning would be, see
the [dead reckoning scenario](../dead_reckoning/README.md)) and snaps back
down hard the moment GNSS resumes. The smoother, anchored by GNSS on both
sides of the outage, peaks in the middle and stays below the filter at
every single timestep, the same invariant checked for the random walk
case. The mistuned run looks near-identical at this scale (image in
`mistuned/figures/ins_6_variance.png`); a 1.6-1.8x IMU noise mismatch is
small next to the outage's own effect on variance.

### Bias

![Sensor bias, matched](matched/figures/ins_7_bias.png)

Accelerometer bias (both axes) tracks the true value closely through the
entire run, outage included, magnetometer and DVL give the filter enough
to keep learning it even with GNSS gone. Gyro bias is noisier and drifts
further from truth than the other two, consistent with it being the
hardest of the three biases to observe in this setup (see the
[GNSS-only](../only_gnss/README.md) and [short-gap random
walk](../../2D/README.md) scenarios for the same pattern). The mistuned
run's bias plot looks very similar; see
`mistuned/figures/ins_7_bias.png`.

### Consistency

**matched**

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.9989 | [1.4349, 2.6574] | consistent   |
| ANIS magnetometer       | 0.8573 | [0.8682, 1.1410] | INCONSISTENT |
| ANIS DVL                | 1.9935 | [1.8296, 2.1779] | consistent   |
| ANEES position filter   | 1.4470 | [1.2959, 2.8544] | consistent   |
| ANEES position smoother | 1.5674 | [0.0506, 7.3778] | consistent   |

RMSE: filter 1.00m, smoother 0.71m.

**mistuned**

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.9969 | [1.4434, 2.6459] | consistent   |
| ANIS magnetometer       | 0.8487 | [0.8668, 1.1426] | INCONSISTENT |
| ANIS DVL                | 1.9253 | [1.8274, 2.1803] | consistent   |
| ANEES position filter   | 1.4555 | [1.3109, 2.8323] | consistent   |
| ANEES position smoother | 1.5343 | [0.0506, 7.3778] | consistent   |

RMSE: filter 1.01m, smoother 0.70m.

Magnetometer ANIS is the one surprise, mildly inconsistent in both runs,
and just under its band both times (0.857 and 0.849 against bands starting
around 0.87). The [dead reckoning
scenario](../dead_reckoning/README.md#consistency-read-carefully) gives the
same result (0.81 on all three trajectories), so the filter is a touch
underconfident about the magnetometer's `R`, not overconfident. Worth a
look in a future retuning pass.

### Figure-eight

The same outage and the same two settings on the figure-eight path.

![Filter vs smoother variance, figure-eight matched](figure_eight_matched/figures/ins_6_variance.png)

The variance has the same shape as on the rounded rectangle: the filter's
grows through the outage, here by about a factor of 40 (from about 0.1 to
4.5), and snaps back down at 255 s when GNSS resumes. The smoother peaks at
about 1 in the middle and stays below the filter at every timestep (checked
for both runs).

![Sensor bias, figure-eight matched](figure_eight_matched/figures/ins_7_bias.png)

Both accelerometer biases are tracked closely through the whole run, outage
included. The gyro bias is the weak one again: the EKF swings between about
-0.006 and 0.010 rad/s in the first 1000 steps and then follows the true
level only loosely, and the ERTS estimate is nearly flat around 0.002-0.0035
and does not follow the rise of the true bias to about 0.007 near step 19000.

**figure_eight_matched**

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.8012 | [1.4325, 2.6608] | consistent   |
| ANIS magnetometer       | 0.8593 | [0.8682, 1.1410] | INCONSISTENT |
| ANIS DVL                | 1.9876 | [1.8297, 2.1778] | consistent   |
| ANEES position filter   | 2.3378 | [1.1958, 3.0076] | consistent   |
| ANEES position smoother | 1.7506 | [0.0506, 7.3778] | consistent   |

RMSE: filter 1.42m, smoother 0.68m.

**figure_eight_mistuned**

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.7956 | [1.4384, 2.6527] | consistent   |
| ANIS magnetometer       | 0.8495 | [0.8669, 1.1425] | INCONSISTENT |
| ANIS DVL                | 1.9217 | [1.8275, 2.1801] | consistent   |
| ANEES position filter   | 2.3365 | [1.1938, 3.0108] | consistent   |
| ANEES position smoother | 1.7878 | [0.0506, 7.3778] | consistent   |

RMSE: filter 1.43m, smoother 0.70m.

The consistency story is the same as on the rounded rectangle: consistent
everywhere except the magnetometer ANIS, just under its band in both runs
(0.859 and 0.850 against lower edges of 0.868 and 0.867). The filter's RMSE is
higher than on the rounded rectangle (1.42-1.43m against 1.00-1.01m), while
the smoother's is about the same (0.68-0.70m against 0.70-0.71m). The
[dead reckoning](../dead_reckoning/README.md#consistency-read-carefully)
page saw the error grow with how much the vehicle turns, and the figure-eight
turns for about 60% of its length, which fits but was not tested here.

## Takeaways and limits

- The variance plot is the clearest illustration in this whole results set
  of what the smoother buys over the filter: identical sensors, identical
  data, and the smoother's peak uncertainty mid-outage is still below the
  filter's uncertainty at almost every other point in the run.
- The deliberate IMU mismatch (1.6x gyro, 1.8x accel) doesn't show up as a
  meaningfully different consistency story here, both runs are consistent
  in the same places and inconsistent in the same place (magnetometer).
  Either the mismatch is too small to matter at this outage length, or its
  effect is swamped by the outage itself.
- Single seed, two trajectories (rounded rectangle and figure-eight). The
  outage window (45-255s) and GNSS rate were chosen by hand, not swept.
