# 2-D strapdown INS, GNSS only

[Back to the overview](../../../README.md) | [Previous: 2-D random walk](../../2D/README.md) | [Next: multi-sensor fusion](../multi_sensor_fusion/README.md)

First of three scenarios sharing one setup (seed, IMU, trajectories), each
testing a different sensor combination: this one GNSS alone, magnetometer
and DVL disabled for the whole run. A sensor is disabled by setting its
`*_period_min`/`*_period_max` both far longer than the run itself, so its
uniformly-distributed next-fire time never lands inside the session, the
same trick used for GNSS in the
[dead reckoning scenario](../dead_reckoning/README.md).

Unlike the earlier (now removed) one-trajectory-per-folder results, every
scenario here is run on all three trajectories (`rounded_rectangle`,
`circle`, `straight_into_turn`) and reported together in this one README.

|               |                                                   |
| ------------- | ------------------------------------------------- |
| **Status**    | done, consistent except one turn-heavy case        |
| **Run**       | 30000 timesteps (5 minutes at dt=0.01s), seed 42   |
| **Reproduce** | see [Reproduce](#reproduce)                        |

## Setup

Shared across all three scenarios in this set (this one, multi-sensor
fusion, and dead reckoning) and all three trajectories within each:

| parameter | value | meaning |
| --- | --- | --- |
| gyro bias instability | 5e-5 (rad/s)^2 | true and filter belief matched, no deliberate mismatch |
| accel bias instability | 5e-2 (m/s^2)^2, both axes | true and filter belief matched |
| gyro / accel bias time constant | 1000s | slow enough to look like a near-constant offset over one run |
| GNSS position noise | 0.20 m^2 | 1-sigma ~0.45m |
| magnetometer heading noise | 3e-3 rad^2 | 1-sigma ~3deg |
| DVL body-frame velocity noise | 1e-2 (m/s)^2, both axes | |

This scenario's own sensor periods:

| sensor       | period            | fixes per run (typical) |
| ------------ | ------------------ | ------------------------ |
| GNSS         | uniform(1, 3)s      | ~150                     |
| Magnetometer | disabled (1e5 s)    | 0                        |
| DVL          | disabled (1e5 s)    | 0                        |

Trajectories: `rounded_rectangle` (80x50m, 10m corner radius, looped),
`circle` (40m radius, looped), `straight_into_turn` (296m straight, 90deg
turn of 5m radius, then another 296m straight, not looped, length chosen so
the whole shape completes right around the end of the run). All three start
at the origin heading +x, at a constant 2 m/s.

## Reproduce

```bash
python3 scripts/analyze_ins_2d.py results/INS2D/only_gnss/<trajectory>/data
```

To regenerate the data, copy `results/INS2D/only_gnss/<trajectory>/config.yaml`
over `configs/strapdown_ins_2D.yaml`, then:

```bash
cmake --build build
./build/kalman_filtering_and_smoothing
cp data/*.csv results/INS2D/only_gnss/<trajectory>/data/
python3 scripts/analyze_ins_2d.py results/INS2D/only_gnss/<trajectory>/data
```

## Results

### rounded_rectangle

![Sensor fixes](rounded_rectangle/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](rounded_rectangle/figures/ins_2_filter_vs_smoother.png)
![NIS](rounded_rectangle/figures/ins_4_nis.png)

|                        | value  | band             | verdict    |
| ---------------------- | ------ | ---------------- | ---------- |
| ANIS GNSS              | 1.9679 | [1.6991, 2.3251] | consistent |
| ANEES position filter  | 1.8826 | [1.4533, 2.6326] | consistent |
| ANEES position smoother| 1.7055 | [0.0506, 7.3778] | consistent |

RMSE: filter 0.69m, smoother 0.26m.

### circle

![Sensor fixes](circle/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](circle/figures/ins_2_filter_vs_smoother.png)
![NIS](circle/figures/ins_4_nis.png)

|                        | value  | band             | verdict    |
| ---------------------- | ------ | ---------------- | ---------- |
| ANIS GNSS              | 1.9853 | [1.6986, 2.3256] | consistent |
| ANEES position filter  | 1.8765 | [1.4979, 2.5735] | consistent |
| ANEES position smoother| 1.7225 | [0.0506, 7.3778] | consistent |

RMSE: filter 0.68m, smoother 0.26m.

### straight_into_turn

![Sensor fixes](straight_into_turn/figures/ins_1_sensor_fixes.png)
![Filter vs smoother](straight_into_turn/figures/ins_2_filter_vs_smoother.png)
![NIS](straight_into_turn/figures/ins_4_nis.png)

|                        | value  | band             | verdict      |
| ---------------------- | ------ | ---------------- | ------------ |
| ANIS GNSS              | 3.6455 | [1.2468, 2.9283] | INCONSISTENT |
| ANEES position filter  | 3.8392 | [1.2977, 2.8518] | INCONSISTENT |
| ANEES position smoother| 5.2963 | [0.0506, 7.3778] | consistent   |

RMSE: filter 0.99m, smoother 0.45m.

The only inconsistent case in this scenario. With only GNSS correcting
position and nothing correcting heading between fixes, the EKF has to
dead-reckon through the entire 90deg turn on the IMU model alone. The
filter (not the smoother, which sees the whole record both ways) ends up
overconfident through that stretch. The rectangle's and circle's turns are
much gentler (10m and 40m radius against this track's 5m), so this looks
like a turn-sharpness effect specific to this trajectory, not a general
problem with the scenario.

## Takeaways and limits

- GNSS alone, well-tuned, holds up fine on gentle curvature (rectangle,
  circle) but goes inconsistent through a sharp, unaided turn
  (straight_into_turn). Exactly the gap the other two scenarios in this
  set are meant to cover with magnetometer and DVL aiding.
- Single seed per trajectory, matched IMU model (see Setup), no deliberate
  mismatch test yet.

## Next

[Multi-sensor fusion](../multi_sensor_fusion/README.md) adds magnetometer
and DVL back in to see whether that sharp-turn inconsistency goes away.
