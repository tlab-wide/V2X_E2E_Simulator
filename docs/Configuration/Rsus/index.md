# RSUs and Pseudo Sensors

| | |
|:--|:--|
| File | `config/scenarios/<name>/rsus.yaml` |
| Web viewer | Settings → **RSUs & sensors** |
| Unity export | *V2X → Config → Export Scene To Config* |

```yaml
rsus:
- name: Ganken
  rsu_id: 0x12020001      # 32-bit hex, unique; used in object IDs and information_source_list
  location: Ganken
  enabled: true
  sensors:
  - name: RSU_Lidar 1
    type: lidar_60x22deg_60m
    sensor_id: 1          # 0-255, unique within the RSU
    enabled: true
    pose: {x: 4240.481, y: 73444.969, z: 25.433, heading_deg: 35.5, pitch_deg: -13.8, roll_deg: 0}
    # optional: range_m: 80, h_fov_deg: 90, v_fov_deg: 30
    # optional own topics (no network copies):
    # object_info: {topic: /v2x/rsu1/lidar1/object_info, ground_truth_topic: /v2x/rsu1/lidar1/object_info_groundtruth}
```

To add a sensor, add an entry. To remove one, delete it or set `enabled: false`. Line-of-sight checks use exactly the sensors of this file.

## Object-info publishers (CooL4 DM)

```yaml
object_info_publishers:
- name: all_rsus
  rsus: [all]                        # RSU names, or all
  topic: /v2x/object_info
  ground_truth_topic: /v2x/object_info_groundtruth
  hz: 10
  noise: {position: Default, rotation: Default, dimension: Default, probability: Default}
  network: [DSRC, Cellular]          # adds /v2x/object_info/DSRC and /v2x/object_info/Cellular

per_sensor_object_info:              # rate and noise of the per-sensor topics
  hz: 10
  noise: {position: Default, rotation: Default, dimension: Default, probability: Default}
```

A publisher merges what the sensors of its RSUs see. An object seen by several sensors is sent once and lists every sensor in `information_source_list`. For the message content and the ID layout, see [Object Info](../../DMProtocols/CooL4/ObjectInfo/index.md).

## Autoware object and traffic-signal publishers

```yaml
autoware_object_publishers:          # autoware_auto_perception_msgs/PredictedObjects
- name: rsu1
  rsus: [Ganken]
  topic: /v2x/rsu1/predicted_object
  station_id: 1
  hz: 10
  noise: {position: Default, rotation: Default, dimension: Default, probability: Default}
  network: [DSRC, Cellular]

traffic_signal_publishers:           # v2x_msgs/V2XSignals
- name: rsu1
  station_id: 1
  intersections: [Ganken]            # TrafficIntersection names in the scene
  topics: [/v2x/rsu1/traffic_signals, /v2x/traffic_signals]
  hz: 10
  signal_id: relation                # relation = Lanelet2 regulatory-element IDs, way = traffic-light way IDs
```
