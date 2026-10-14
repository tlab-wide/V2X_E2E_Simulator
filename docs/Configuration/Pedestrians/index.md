# Pedestrians and Cyclists

| | |
|:--|:--|
| File | `config/scenarios/<name>/pedestrians.yaml` |
| Web viewer | Settings → **Pedestrians** |
| Unity export | *V2X → Config → Export Pedestrians And Teleports To Config* |

```yaml
max_active: 5                        # agents walking at the same time (0 = unlimited)
snap_to_ground: true
paths:
- id: Path2
  spawns_per_minute: 0.133
  cyclist_ratio: 0.29                # share of cyclists: 0 = only pedestrians, 1 = only cyclists
  enabled: true
  waypoints:
  - {x: 4480.579, y: 72960.906, z: 23}
  - {x: 4487.901, y: 72948.555, z: 23}
  - {x: 4490.751, y: 72946.922, z: 23, traffic_light: Kakeiken_P_0x30_1}   # waits here for green
  - {x: 4508.019, y: 72955.133, z: 23}
```

Agents start at the first waypoint, walk the path and are removed at the last one. Traffic-light names follow the scene naming (see [Kashiwa Scenes](../../ProjectGuide/KashiwaScenes/index.md)).
