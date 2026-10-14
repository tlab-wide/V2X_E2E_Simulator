# Raw Sensors

| | |
|:--|:--|
| File | `config/scenarios/<name>/raw_sensors.yaml` |
| Web viewer | Settings → **Raw sensors** |
| Unity export | *V2X → Config → Export Raw Sensors To Config* |

These are LiDARs (RGL) and cameras that publish point clouds and images.

```yaml
raw_sensors:
- name: BusAutowareFinal/URDF/base_link/sensor_kit_base_link/velodyne_left_base_link/VelodyneVLP16
  kind: lidar                    # lidar | camera
  mount: ego                     # ego | scene | world
  vehicle: BusAutowareFinal
  object: URDF/base_link/sensor_kit_base_link/velodyne_left_base_link/VelodyneVLP16
  enabled: true
  hz: 10
  frame_id: world
  point_cloud_topics:
  - {topic: lidar_left/pointcloud, format: Pcl24, publish: true}
  - {topic: lidar_left/pointcloud_ex, format: PointXYZIRCAEDT, publish: true}
- name: CameraSensor2
  kind: camera
  mount: scene
  object: AutowareSimulator/Cooperative Autonomous Driving/RSUs/Publishers/Int#2 Raw Sensors/Cameras/CameraSensor2
  enabled: true
  hz: 10
  frame_id: traffic_light_left_camera/camera_optical_link
  image_topic: /rsu/intersection2/camera_2/camera_raw
  camera_info_topic: /rsu/intersection2/camera_2/camera_info
  width: 480
  height: 270
```

| `mount` | Meaning |
|:--|:--|
| `ego` | a sensor on an ego vehicle (`object` = path below the vehicle; `[i]` tells same-named sensors apart) |
| `scene` | a sensor already in the scene (`object` = scene path) |
| `world` | a new sensor created from a `type` of `raw_sensor_types` at `pose` |

Sensors of the scene that are not listed keep their scene setup. Set `enabled: false` to switch one off.
