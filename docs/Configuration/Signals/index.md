# Signal Timing

| | |
|:--|:--|
| File | `config/scenarios/<name>/signals.yaml` |
| Web viewer | Settings → **Signals** |
| Unity export | *V2X → Config → Export Simulation And Signal Timing To Config* |

```yaml
intersections:
- name: Kakeiken              # TrafficIntersection object (see Kashiwa Scene Names)
  enabled: true               # false = keep the scene timing
  steps:
  - duration_s: 35
    lights: V1 green, V2 red, V3 green, P1 red, P2 green
  - duration_s: 3
    lights: P2 green flashing
  # … one entry per step, in order; the cycle repeats
```

Each intersection runs its lighting steps in order and repeats them. `duration_s` sets how long each step lasts. `lights` describes what the step shows (`V1` = vehicle group 1, `P1` = pedestrian group 1); it is written by the exporter and not read. The number of steps and what they show come from the scene, so steps cannot be added or removed here. Signal info (remaining time) and the V2X signal topics follow the new timing.
