# 2-D strapdown INS, dead reckoning

[Back to the overview](../../../README.md) | [Previous: multi-sensor fusion](../multi_sensor_fusion/README.md)

Third of three scenarios sharing one setup (see
[GNSS only](../only_gnss/README.md#setup) for the shared parameters): GNSS
disabled for the whole run. Magnetometer and DVL active. No
absolute position reference anywhere in the run, genuine dead reckoning,
aided only by heading and body-frame velocity corrections.

|               |                                                  |
| ------------- | ------------------------------------------------ |
| **Status**    | done, consistent in position; DVL NIS mildly off |
| **Run**       | 30000 timesteps (5 minutes at dt=0.01s), seed 42 |
| **Reproduce** | see [Reproduce](#reproduce)                      |

## Setup

Same shared parameters as [GNSS only](../only_gnss/README.md#setup). This
scenario's own sensor periods:

| sensor       | period            | fixes per run |
| ------------ | ----------------- | ------------- |
| GNSS         | disabled (1e5 s)  | 0             |
| Magnetometer | uniform(0.25, 1)s | ~370          |
| DVL          | uniform(0.2, 1)s  | ~390          |

## Reproduce

```bash
python3 scripts/analyze_ins_2d.py results/INS2D/dead_reckoning/<trajectory>/data
```

To regenerate the data, copy
`results/INS2D/dead_reckoning/<trajectory>/config.yaml` over
`configs/strapdown_ins_2D.yaml`, then follow the same steps as
[GNSS only's reproduce section](../only_gnss/README.md#reproduce).

## Results

### rounded_rectangle

![Sensor fixes](rounded_rectangle/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](rounded_rectangle/figures/ins_2_filter_vs_smoother.png)
![NIS](rounded_rectangle/figures/ins_4_nis.png)

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS magnetometer       | 0.9637 | [0.8809, 1.1265] | consistent   |
| ANIS DVL                | 1.6756 | [1.8195, 2.1889] | INCONSISTENT |
| ANEES position filter   | 1.0967 | [0.3612, 5.0133] | consistent   |
| ANEES position smoother | 1.0877 | [0.0506, 7.3778] | consistent   |

RMSE: filter 1.53m, smoother 1.52m. (GNSS: no fixes recorded.)

### circle

![Sensor fixes](circle/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](circle/figures/ins_2_filter_vs_smoother.png)
![NIS](circle/figures/ins_4_nis.png)

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS magnetometer       | 0.9633 | [0.8808, 1.1267] | consistent   |
| ANIS DVL                | 1.6747 | [1.8181, 2.1905] | INCONSISTENT |
| ANEES position filter   | 1.2811 | [0.2822, 5.3640] | consistent   |
| ANEES position smoother | 1.2734 | [0.0506, 7.3778] | consistent   |

RMSE: filter 1.63m, smoother 1.62m. (GNSS: no fixes recorded.)

### straight_into_turn

![Sensor fixes](straight_into_turn/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](straight_into_turn/figures/ins_2_filter_vs_smoother.png)
![NIS](straight_into_turn/figures/ins_4_nis.png)

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS magnetometer       | 0.9634 | [0.8808, 1.1266] | consistent   |
| ANIS DVL                | 1.6713 | [1.8186, 2.1899] | INCONSISTENT |
| ANEES position filter   | 1.8161 | [0.1875, 5.9050] | consistent   |
| ANEES position smoother | 1.8138 | [0.0506, 7.3778] | consistent   |

RMSE: filter 2.14m, smoother 2.13m. (GNSS: no fixes recorded.)

## Consistency, read carefully

Large RMSE (1.5-2.1m, by far the worst of the three scenarios) is the
expected signature of dead reckoning, not a sign of a broken filter
there is nothing here to correct an accumulating position error against.
What actually matters for consistency is whether the filter's own `P`
honestly reflects that growing error, and `ANEES position filter` says yes
on all three trajectories (comfortably inside its band every time): the
filter knows it's unsure, by the right amount.

## Takeaways and limits

- Position/heading accuracy degrades with trajectory complexity even
  though nothing else does: RMSE climbs from 1.53m (rectangle) to 1.63m
  (circle) to 2.14m (straight_into_turn, the sharpest turn of the three).
  The filter's own uncertainty estimate (ANEES) stays consistent
  throughout regardless, which is the real point of this scenario.
- The DVL-underconfidence finding is consistent across all three
  trajectories, which is some evidence it's a property of the sensor
  tuning rather than a one-off.
- Single seed per trajectory, matched IMU model, no deliberate mismatch
  test yet.
