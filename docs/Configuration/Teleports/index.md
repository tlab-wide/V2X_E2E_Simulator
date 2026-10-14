# Teleports

| | |
|:--|:--|
| File | `config/scenarios/<name>/teleports.yaml` |
| Web viewer | Settings → **Teleports** |
| Unity export | *V2X → Config → Export Pedestrians And Teleports To Config* |

A teleport gate moves the ego vehicle when it drives into the trigger box.

```yaml
teleports:
- name: First gate
  enabled: true
  trigger: {x: 4431.903, y: 73109.227, z: 19.707, heading_deg: 70.4}
  trigger_size: {x: 5.11, y: 2, z: 6.92}      # width, height, length (m)
  strategy: lane_midpoint                     # lane_midpoint | random_point
  clearance_radius_m: 15                      # NPCs this close to the target are removed
  delay_s: 0
  random_delay_s: [0, 0]
  lane_midpoint:
    lanelets: [1034, 1118, 1113, 1104, 1086, 1093, 1082, 1071]   # candidate lanes
    midpoint_lanelet: 328
    midpoint_fraction: 0.5
    cruise_speed_kmh: 30
    acceleration_mps2: 2
    arrival_window_s: [0, 90]
    spacing_m: 10
    include_endpoints: true
  # random_point: random_points: [{x: …, y: …, z: …, heading_deg: …}]
```

| Field | Meaning |
|:--|:--|
| `trigger`, `trigger_size` | Box the ego drives into: centre and heading in the map frame; size `x` = width, `y` = height, `z` = length (m) |
| `strategy` | `lane_midpoint`: pick a start point on the listed lanes (below). `random_point`: one of `random_points`, chosen at random. |
| `clearance_radius_m` | NPC vehicles closer than this to the target are removed before the ego arrives |
| `delay_s`, `random_delay_s` | Wait before the jump: `delay_s` plus a random value in [min, max] |
| `lane_midpoint.lanelets` | Lanes the ego may be placed on; candidate points every `spacing_m` along them (`include_endpoints`: also the lane ends) |
| `lane_midpoint.midpoint_lanelet`, `midpoint_fraction` | Reference point the ego should reach: this far along that lane (0 = start, 1 = end) |
| `lane_midpoint.arrival_window_s` | A target time is drawn at random in [min, max]. The candidate whose estimated driving time to the reference point is closest wins. |
| `lane_midpoint.cruise_speed_kmh`, `acceleration_mps2` | Used for that estimate: accelerate from 0 to cruise speed, then drive on |

In the web viewer, **Pick target lanes on map** toggles lanes in `lanelets`.
