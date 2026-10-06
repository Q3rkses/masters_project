# 2-D strapdown INS, multi-sensor fusion

[Back to the overview](../../../README.md) | [Previous: GNSS only](../only_gnss/README.md) | [Next: dead reckoning](../dead_reckoning/README.md)

Second of three scenarios sharing one setup (see
[GNSS only](../only_gnss/README.md#setup) for the shared parameters): GNSS,
magnetometer and DVL all active and fused sequentially within a timestep.

|               |                                                  |
| ------------- | ------------------------------------------------ |
| **Status**    | done, consistent on all three trajectories       |
| **Run**       | 30000 timesteps (5 minutes at dt=0.01s), seed 42 |
| **Reproduce** | see [Reproduce](#reproduce)                      |

## Setup

Same shared parameters as [GNSS only](../only_gnss/README.md#setup). This
scenario's own sensor periods:

| sensor       | period            | fixes per run (typical) |
| ------------ | ----------------- | ----------------------- |
| GNSS         | uniform(10, 20)s  | ~20                     |
| Magnetometer | uniform(0.25, 1)s | ~480                    |
| DVL          | uniform(0.2, 1)s  | ~490                    |

GNSS period is longer here than in the GNSS-only scenario (10-20s instead
of 1-3s) since it no longer has to carry the whole run on its own.

## Reproduce

```bash
python3 scripts/analyze_ins_2d.py results/INS2D/multi_sensor_fusion/<trajectory>/data
```

To regenerate the data, copy
`results/INS2D/multi_sensor_fusion/<trajectory>/config.yaml` over
`configs/strapdown_ins_2D.yaml`, then follow the same steps as
[GNSS only's reproduce section](../only_gnss/README.md#reproduce).

## Results

### rounded_rectangle

![Sensor fixes](rounded_rectangle/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](rounded_rectangle/figures/ins_2_filter_vs_smoother.png)
![NIS](rounded_rectangle/figures/ins_4_nis.png)

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.9977 | [1.1494, 3.0823] | consistent   |
| ANIS magnetometer       | 0.8981 | [0.8823, 1.1250] | consistent   |
| ANIS DVL                | 1.8918 | [1.8152, 2.1936] | consistent   |
| ANEES position filter   | 2.5121 | [0.9895, 3.3601] | consistent   |
| ANEES position smoother | 2.3270 | [0.0506, 7.3778] | consistent   |

RMSE: filter 0.68m, smoother 0.46m.

### circle

![Sensor fixes](circle/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](circle/figures/ins_2_filter_vs_smoother.png)
![NIS](circle/figures/ins_4_nis.png)

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.8193 | [0.9278, 3.4770] | consistent   |
| ANIS magnetometer       | 0.8999 | [0.8823, 1.1250] | consistent   |
| ANIS DVL                | 1.8897 | [1.8165, 2.1922] | consistent   |
| ANEES position filter   | 2.2638 | [1.1404, 3.0971] | consistent   |
| ANEES position smoother | 2.4637 | [0.0506, 7.3778] | consistent   |

RMSE: filter 0.66m, smoother 0.47m.

### straight_into_turn

![Sensor fixes](straight_into_turn/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](straight_into_turn/figures/ins_2_filter_vs_smoother.png)
![NIS](straight_into_turn/figures/ins_4_nis.png)

|                         | value  | band             | verdict      |
| ----------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS               | 1.7876 | [1.3568, 2.7661] | consistent   |
| ANIS magnetometer       | 0.9010 | [0.8826, 1.1246] | consistent   |
| ANIS DVL                | 1.8984 | [1.8164, 2.1923] | consistent   |
| ANEES position filter   | 1.8771 | [1.0419, 3.2654] | consistent   |
| ANEES position smoother | 1.6521 | [0.0506, 7.3778] | consistent   |

RMSE: filter 0.57m, smoother 0.40m.

Magnetometer ANIS (0.90 on all three) sits close to the lower edge of its
band (about 0.88), the same slight underconfidence about the magnetometer's
`R` seen in the [dead reckoning](../dead_reckoning/README.md) and
[outage](../gnss_outage/README.md) scenarios.

## Takeaways and limits

- All three sensors, all three trajectories: every consistency check
  passes, as it does for GNSS only.
- Filter RMSE is 0.57-0.68m against 0.68-0.70m for GNSS only, with ~19 GNSS
  fixes instead of ~149. Magnetometer and DVL carry the accuracy that the
  dense GNSS fixes carried. The smoother is worse than in GNSS only
  (0.40-0.47m against 0.21-0.22m), likely because it has far fewer
  absolute position fixes to anchor on.
- Single seed per trajectory, matched IMU model, no deliberate mismatch
  test yet.

## Next

[Dead reckoning](../dead_reckoning/README.md) removes GNSS entirely, the
opposite extreme from this scenario.
