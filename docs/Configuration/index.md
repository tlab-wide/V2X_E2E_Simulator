# Scenario Configuration

From v3.2 the simulator builds its RSUs, pseudo sensors, publishers, traffic, pedestrians, teleports and raw sensors from YAML files, and sets the ego vehicle, time scale, seed, sun, fog, loggers and signal timing from them, in `config/` when Play starts. The Unity scene keeps the map, the roads, the traffic lights and the vehicle and sensor prefabs. To change a scenario you edit the files instead of the scene. You can edit them by hand or in the [web viewer](../Tools/WebViewer/index.md#settings) **Settings** tab.

!!! note "When changes apply"
    The files are read once, when Play starts (or when a built simulator starts). Saved changes apply on the next run.

## Folder layout

```text
config/
├── default_scenario.txt          name of the scenario that is loaded (one line)
├── defaults/                     shared by all scenarios
│   ├── sensor_types.yaml         pseudo-sensor types (FOV, range) and raw-sensor prefabs
│   ├── noise.yaml                noise profiles
│   └── network.yaml              network profiles (DSRC, Cellular, …)
├── scenarios/
│   └── kashiwa_cool4/
│       ├── scenario.yaml         lists the files of this scenario
│       ├── simulation.yaml       ego vehicle, time scale, seed, sun, fog, loggers
│       ├── signals.yaml          traffic-signal step durations
│       ├── rsus.yaml             RSUs, pseudo sensors, object-info / Autoware / signal publishers
│       ├── vehicles.yaml         NPC vehicle traffic
│       ├── pedestrians.yaml      pedestrian and cyclist paths
│       ├── teleports.yaml        teleport gates for the ego vehicle
│       └── raw_sensors.yaml      LiDARs and cameras that publish raw data
├── maps/traffic_lanes.json       simulator lanes for the web viewer (generated)
└── factory/                      shipped copy of every file ("Reset to default")
```

`scenario.yaml` names the files of its scenario. If a key is missing, that part keeps the setup of the scene:

```yaml
name: kashiwa_cool4
description: exported from V2X_E2E_Simulator_v3.1.1
rsus: rsus.yaml
vehicles: vehicles.yaml
pedestrians: pedestrians.yaml
teleports: teleports.yaml
raw_sensors: raw_sensors.yaml
simulation: simulation.yaml
signals: signals.yaml
```

## Choosing the scenario

The first of these that is set is used:

1. Command-line argument `--scenario <name>` (built simulator).
2. The **Scenario** field of the *Config Bootstrap* object in the scene.
3. `config/default_scenario.txt`.

`--config-root <path>` reads the configuration from another folder. By default it is `config/` next to `Assets/` in the editor, and next to the `<name>_Data` folder in a build.

!!! warning "Errors"
    The simulator checks all files before it builds anything. If there is an error (a missing file, an unknown sensor type, a duplicate ID, …), it logs every problem in the Console as `[Config] …`. It then runs with the scene's own setup. The web viewer runs the same checks before it saves.

## Coordinates

Positions use the **map frame** of Autoware and the Lanelet2 map: MGRS-local x (east) and y (north), and z (height), in metres. These are the coordinates you see in RViz.

| Field | Meaning |
|:--|:--|
| `x`, `y`, `z` | map frame, metres |
| `heading_deg` | direction the object faces, clockwise from north (0 = north, 90 = east) |
| `pitch_deg` | positive looks up, negative looks down |
| `roll_deg` | roll around the viewing direction |

Fields of view (`h_fov_deg`, `v_fov_deg`) are **full** angles. A 60° sensor sees 30° to each side.

## Simulation (`simulation.yaml`)

```yaml
time_scale: 1                 # simulation speed, 1 = real time (> 0, at most 100)
seed: 0                       # one seed for everything random; leave empty for a different run every time
ego:
  vehicle: BusAutowareFinal   # an ego vehicle of the scene (tag Ego): BusAutowareFinal, Lexus RX450h 2015 Sample Sensor
  pose: {x: 4769.09, y: 73083.82, z: 18.38, heading_deg: 241}   # start pose; omit to keep the scene pose
lighting:
  time_of_day: '17:30'        # local time; empty = the sun of the scene
  date: '2025-06-21'
  latitude_deg: 35.9
  longitude_deg: 139.94
  utc_offset_h: 9
  # sun_elevation_deg: 40     # or place the sun directly (both values)
  # sun_azimuth_deg: 180
  sun_intensity_lux: 100000
  color_temperature_k: 6570
  night_intensity_lux: 0.5    # moonlight when the sun is below the horizon
weather:
  fog_visibility_m: 0         # e.g. 150 for thick fog; 0 = no fog
  fog_height_m: 60
loggers:
  output_dir: SimulationOutput
  npc_area:                   # NPC vehicles and pedestrians inside the log area (LogArea)
    enabled: true
    cars_csv: Scenario_car.csv
    humans_csv: Scenario_human.csv
    every_n_frames: 2         # 0 = every interval_s instead
    interval_s: 0.05
  ego:                        # state of the ego vehicle (LogAutonomous)
    enabled: false
    csv: Autonomous.csv
    spawn_validation_csv: AutonomousSpawnValidation.csv
    prefix: Auto
    interval_s: 0.005
```

- **Ego vehicle.** The chosen vehicle is switched on and the other ego vehicles are switched off. The traffic manager, the ScenarioSelector teleport keys and the *Selected Car* dropdown follow it. Its raw sensors are set up from `raw_sensors.yaml` before it starts.
- **Seed.** It seeds Unity's random numbers, the traffic manager (it replaces the `seed` of `vehicles.yaml`), line-of-sight UUIDs and the object numbers of object info. Physics and frame timing still differ a little between runs, so runs are similar but not bit-identical.
- **Sun.** `time_of_day` places the sun from the date and the position (NOAA formulas). When the sun is below the horizon, a dim light opposite it (`night_intensity_lux`) keeps the scene readable. The camera exposure of the scene is automatic.
- **Fog.** It adds an HDRP fog to the scene's global volume. Visibility is the distance where objects fade out (mean free path × 3). It affects cameras, not the pseudo sensors or the RGL LiDARs.
- **Loggers.** Files are written to `output_dir` (relative to the project folder, or the build folder) as `<name>_<yyyyMMdd_HHmmss>.csv`. See [Loggers](../Components/Logger/index.md).
- **Time scale.** It is also passed to the ROS clock, so `/clock` and the timestamps follow it.

## Signal timing (`signals.yaml`)

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

## Defaults

### Pseudo-sensor types (`defaults/sensor_types.yaml`)

```yaml
pseudo_sensor_types:
- name: lidar_60x22deg_60m
  kind: lidar            # lidar | camera
  h_fov_deg: 60
  v_fov_deg: 22
  range_m: 60
```

An RSU sensor refers to a type by `name` and may override `h_fov_deg`, `v_fov_deg` and `range_m`. `raw_sensor_types` lists the LiDAR and camera prefabs that `raw_sensors.yaml` can create.

### Noise profiles (`defaults/noise.yaml`)

| Field | Meaning |
|:--|:--|
| `distribution` | `uniform`: a random value in [−amplitude, +amplitude] per axis. `gaussian`: `gaussian_mean + gaussian_std_dev × amplitude × N(0,1)` |
| `x_m`, `y_m`, `z_m` | amplitude per axis (m) |

`NoNoise` (all zero) is used for ground truth.

### Network profiles (`defaults/network.yaml`)

A network profile publishes a delayed, lossy copy of a topic as `<topic><topic_suffix>`, for example `/v2x/object_info/DSRC`.

| Field | Meaning |
|:--|:--|
| `delay_ms`, `delay_variance_ms` | latency and its random ± part |
| `packet_loss`, `packet_loss_variance` | probability (0–1) that a message is dropped |
| `ground_truth` | also make a copy of the ground-truth topic |

## RSUs and pseudo sensors (`rsus.yaml`)

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

### Object-info publishers (CooL4 DM)

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

A publisher merges what the sensors of its RSUs see. An object seen by several sensors is sent once and lists every sensor in `information_source_list`. For the message content and the ID layout, see [Object Info](../DMProtocols/CooL4/ObjectInfo/index.md).

### Autoware object and traffic-signal publishers

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

## Vehicle traffic (`vehicles.yaml`)

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
- Right of way: vehicles follow the lane rules of the scene. If a vehicle has waited to yield for `deadlock_timeout_s` while the others also wait, it gets the right of way. Vehicles that have not moved for `stuck_despawn_s` are removed. Vehicles also leave at the end of their route.

## Pedestrians and cyclists (`pedestrians.yaml`)

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

Agents start at the first waypoint, walk the path and are removed at the last one. Traffic-light names follow the scene naming (see [Kashiwa Scenes](../ProjectGuide/KashiwaScenes/index.md)).

## Teleports (`teleports.yaml`)

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

## Raw sensors (`raw_sensors.yaml`)

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

## Editing in Unity

The *V2X → Config* menu has:

- **Add Config Bootstrap To Scene**: adds the object that loads the configuration. It is already in the v3.2 scene.
- **Export Scene To Config**: writes the scene's RSUs, sensors and publishers to `rsus.yaml` and the defaults.
- **Export Vehicle Traffic / Pedestrians And Teleports / Raw Sensors To Config**: writes the other files from the scene.
- **Export Simulation And Signal Timing To Config**: writes `simulation.yaml` (the active ego, its pose, sun and loggers of the scene) and `signals.yaml`.
- **Export Traffic Lanes For The Web Viewer**: writes `config/maps/traffic_lanes.json`. Run it again after you change the lanes in the scene.

!!! warning
    An export overwrites the file it writes, including entries you added by hand.
