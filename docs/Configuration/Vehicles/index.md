# Vehicle Traffic

| | |
|:--|:--|
| File | `config/scenarios/<name>/vehicles.yaml` |
| Web viewer | Settings → **Vehicles** |
| Unity export | *V2X → Config → Export Vehicle Traffic To Config* |

```yaml
lanes_root: AutowareSimulator/Enviroment /TrafficLanes
template_traffic_manager: ScenarioSelector/Cool4 Kakeiken Overal Evaluation /New NPC - Kashiwa High
seed: 0
max_vehicles: 200
spawn_distance_to_ego_m: 30          # no spawning closer than this to the ego vehicle
spawn_clearance_m: 4                 # free space needed around the spawn point
maximum_spawns: 4000                 # 0 = unlimited
behaviour:
  deadlock_timeout_s: 8              # standing while yielding this long -> takes the right of way
  forced_pass_s: 6                   # ... for this long
  forced_pass_radius_m: 70
  stuck_despawn_s: 180               # standing still this long for any reason -> removed (0 = never)
  standstill_speed_mps: 0.3
vehicle_types:
- {prefab: Hatchback Variant, weight: 1}
- {prefab: SmallCar Variant, weight: 1}
- {prefab: Taxi Variant, weight: 1}
- {prefab: Van Variant, weight: 1}
spawn_points:
- name: lanelet779
  lanelet: 779
  vehicles_per_minute: 5.15          # ± 25 % random spacing
  position: {x: 4352.18, y: 73301.3, z: 18.62}
branches:                            # where a lane splits
- from_lanelet: 1034
  to:
  - {lanelet: 1088, ratio: 0.33}
  - {lanelet: 1089, ratio: 0.67}     # normalised; next lanes that are not listed get 0
```

- **Lanelet numbers** are the names of the simulator's traffic lanes (`lanelet<number>` under `lanes_root`). In the web viewer, click a lane in **Settings → Vehicles** to see its number and next lanes and to add a spawn point or branch ratios.
- One traffic manager runs the traffic. The `template_traffic_manager` of the scene supplies the ego vehicle, layers and vehicle dynamics, and the scene's other traffic managers are switched off. `vehicle_types` may use any NPC prefab that a traffic manager of the scene knows.
- A vehicle is created only when the spawn point is clear. The next one is scheduled from `vehicles_per_minute`.
- `behaviour.forced_pass_radius_m`: at most one vehicle within this distance gets the right of way at a time (the one that has waited longest), so two blocked vehicles at the same intersection do not both go. `behaviour.standstill_speed_mps`: below this speed a vehicle counts as standing.
- Right of way: vehicles follow the lane rules of the scene. If a vehicle has waited to yield for `deadlock_timeout_s` while the others also wait, it gets the right of way. Vehicles that have not moved for `stuck_despawn_s` are removed. Vehicles also leave at the end of their route.
