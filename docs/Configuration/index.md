# Scenario Configuration

From v3.2 the simulator builds its RSUs, pseudo sensors, publishers, traffic, pedestrians, teleports and raw sensors from YAML files, and sets the ego vehicle, time scale, seed, sun, fog, loggers and signal timing from them, in `config/` when Play starts. The Unity scene keeps the map, the roads, the traffic lights and the vehicle and sensor prefabs. To change a scenario you edit the files instead of the scene. You can edit them by hand or in the [web viewer](../Tools/WebViewer/index.md#settings) **Settings** tab.

!!! note "When changes apply"
    The files are read once, when Play starts (or when a built simulator starts). Saved changes apply on the next run.

## Configuration files

| Page | File | Sets |
|:--|:--|:--|
| [Simulation](Simulation/index.md) | `simulation.yaml` | ego vehicle and start pose, time scale, seed, sun, fog, loggers |
| [Signal Timing](Signals/index.md) | `signals.yaml` | step durations of the traffic signals |
| [RSUs and Pseudo Sensors](Rsus/index.md) | `rsus.yaml` | RSUs, sensors, object-info / Autoware-object / traffic-signal publishers |
| [Vehicle Traffic](Vehicles/index.md) | `vehicles.yaml` | spawn points, branch ratios, vehicle types, right of way, removal of stuck vehicles |
| [Pedestrians and Cyclists](Pedestrians/index.md) | `pedestrians.yaml` | walking and cycling paths, spawn rates, cyclist share |
| [Teleports](Teleports/index.md) | `teleports.yaml` | teleport gates for the ego vehicle |
| [Raw Sensors](RawSensors/index.md) | `raw_sensors.yaml` | LiDARs and cameras that publish raw data |
| [Sensor Types](SensorTypes/index.md) | `defaults/sensor_types.yaml` | pseudo-sensor types (FOV, range) and raw-sensor prefabs |
| [Noise Profiles](Noise/index.md) | `defaults/noise.yaml` | detection noise |
| [Network Profiles](Network/index.md) | `defaults/network.yaml` | delayed / lossy copies (DSRC, Cellular) |

## How to change the configuration

There are three ways. All of them change the same files, so you can mix them.

| Way | Use it for |
|:--|:--|
| **Web viewer → Settings** (`tools/web-viewer/run.sh`, open <http://localhost:8080>, top bar **Settings**) | Everyday changes. Lists and forms for every file, with editing on the map. **Save** checks everything first. See [Web Viewer → Settings](../Tools/WebViewer/index.md#settings). |
| **Edit the YAML files** in `config/` with any text editor | Many similar changes at once, or with version control. Errors show in the Unity Console as `[Config] …` when Play starts. |
| **Unity menu *V2X → Config → Export …*** | Copy what you built in the scene into the files (see [Editing in Unity](#editing-in-unity)) |

After a change, press Play again (or restart the built simulator). To undo, use **Reset to default** in the web viewer (or copy the file back from `config/factory/`).

### Common tasks

| Task | In the web viewer (Settings) | In the files |
|:--|:--|:--|
| Make a new scenario | **New…** copies the current one; **Make default** loads it at Play | Copy a folder in `config/scenarios/`, write its name in `config/default_scenario.txt` (or pick it on the *Config Bootstrap* object, or start with `--scenario <name>`) |
| Use the Lexus instead of the bus | Simulation → *vehicle* | [`simulation.yaml`](Simulation/index.md) → `ego.vehicle` |
| Start the ego somewhere else | Simulation → **Place on map**, then drag; set *heading* | [`simulation.yaml`](Simulation/index.md) → `ego.pose` |
| Run faster than real time | Simulation → *time scale* | [`simulation.yaml`](Simulation/index.md) → `time_scale` |
| Repeat the same traffic | Simulation → *random seed* | [`simulation.yaml`](Simulation/index.md) → `seed` |
| Evening, night, fog | Simulation → *time of day*, *fog visibility* | [`simulation.yaml`](Simulation/index.md) → `lighting.time_of_day`, `weather.fog_visibility_m` |
| Turn CSV logging on or off | Simulation → Loggers | [`simulation.yaml`](Simulation/index.md) → `loggers` |
| Change a signal cycle | Signals → intersection → step durations | [`signals.yaml`](Signals/index.md) → `steps[].duration_s` |
| Add, move or remove an RSU sensor | RSUs & sensors → RSU → **Place on map** / drag / × | [`rsus.yaml`](Rsus/index.md) → `rsus[].sensors` |
| Change a sensor's range or field of view | RSUs & sensors → sensor → *range*, *h FOV*, *v FOV* (or Defaults → sensor type) | [`rsus.yaml`](Rsus/index.md) (per sensor) or [`defaults/sensor_types.yaml`](SensorTypes/index.md) (per type) |
| Change sensor noise or network delay | Defaults → Noise / Network profiles; RSUs → publisher → noise, network copies | [`defaults/noise.yaml`](Noise/index.md), [`defaults/network.yaml`](Network/index.md), [`rsus.yaml`](Rsus/index.md) publishers |
| Publish a new object-info topic | RSUs & sensors → Object-info publishers → **+ Add** | [`rsus.yaml`](Rsus/index.md) → `object_info_publishers` |
| More or less traffic | Vehicles → spawn point → *vehicles / minute*; *max vehicles* | [`vehicles.yaml`](Vehicles/index.md) → `spawn_points`, `max_vehicles` |
| Add a spawn point or a turn ratio | Vehicles → click a lane → **Spawn vehicles here** / **Set branch ratios** | [`vehicles.yaml`](Vehicles/index.md) → `spawn_points`, `branches` |
| Add a pedestrian or cyclist route | Pedestrians → **+ Add**, click the points on the map | [`pedestrians.yaml`](Pedestrians/index.md) → `paths` |
| Move or add a teleport gate | Teleports → drag the gate / **Place gate on map** | [`teleports.yaml`](Teleports/index.md) |
| Switch a LiDAR or camera on or off, change its topic | Raw sensors → sensor → *enabled*, topics | [`raw_sensors.yaml`](RawSensors/index.md) |

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

## Editing in Unity

The *V2X → Config* menu has:

- **Add Config Bootstrap To Scene**: adds the object that loads the configuration. It is already in the v3.2 scene.
- **Export Scene To Config**: writes the scene's RSUs, sensors and publishers to `rsus.yaml` and the defaults.
- **Export Vehicle Traffic / Pedestrians And Teleports / Raw Sensors To Config**: writes the other files from the scene.
- **Export Simulation And Signal Timing To Config**: writes `simulation.yaml` (the active ego, its pose, sun and loggers of the scene) and `signals.yaml`.
- **Export Traffic Lanes For The Web Viewer**: writes `config/maps/traffic_lanes.json`. Run it again after you change the lanes in the scene.

!!! warning
    An export overwrites the file it writes, including entries you added by hand.
