# Sensor Types

| | |
|:--|:--|
| File | `config/defaults/sensor_types.yaml` |
| Web viewer | Settings → **Defaults** |
| Unity export | *V2X → Config → Export Scene To Config* |

These defaults are shared by all scenarios.

```yaml
pseudo_sensor_types:
- name: lidar_60x22deg_60m
  kind: lidar            # lidar | camera
  h_fov_deg: 60
  v_fov_deg: 22
  range_m: 60
```

An RSU sensor refers to a type by `name` and may override `h_fov_deg`, `v_fov_deg` and `range_m`. `raw_sensor_types` lists the LiDAR and camera prefabs that `raw_sensors.yaml` can create.
