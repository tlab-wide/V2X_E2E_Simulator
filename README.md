# V2X E2E Simulator

![V2X E2E Simulator](https://github.com/hoosh-ir/V2X_E2E_Simulator/assets/32614364/2e273eb2-89f0-47f3-9e6f-81648e3cd807)

V2X E2E Simulator is an end-to-end Vehicle-to-Everything (V2X) simulation platform built on top of [AWSIM](https://github.com/tier4/AWSIM) — the leading scene simulator for [Autoware](https://github.com/autowarefoundation/autoware).

**Full documentation: [https://tlab-wide.github.io/V2X_E2E_Simulator/](https://tlab-wide.github.io/V2X_E2E_Simulator/)**

**Download** (Unity package, Kashiwa maps, Autoware vehicle files): fill in the [request form](https://docs.google.com/forms/d/e/1FAIpQLSfsFGAvp8J0iLGfcQQ8r6o2x72SBmsddg-B-ngfVQoC4tF0iw/viewform); the download link is sent to your email.

---

## Features

- V2X cooperative perception via roadside unit sensors with noise and network delay simulation
- Advanced traffic management: per-lane spawn rates, branch weights, and zebra crossing yield behaviour
- Teleportation system with ETA-based lane targeting for controlled experiments
- Pedestrian and cyclist simulation with waypoint paths and pool-based spawning
- End-to-end CSV data logging of NPC positions, ego vehicle state, and spawn selection
- CooL4 DM protocol output: object info and signal info (with remaining time), each also with DSRC / Cellular network delay
- Local web viewer (`tools/web-viewer`): live topic health and all published information on the Lanelet2 map
- Full ROS2 integration compatible with Autoware
- Open source — Ubuntu 22.04 and Windows 10/11

## Quick Start

1. Clone the repository:

   ```bash
   git clone https://github.com/tlab-wide/V2X_E2E_Simulator.git
   ```

2. Open the project in **Unity 6000.0.50f1** (Unity 6).
3. The scene, the Kashiwa map and the 3D models are not in the repository. Request **`V2X_E2E_Simulator_3_1_1.unitypackage`** with the [request form](https://docs.google.com/forms/d/e/1FAIpQLSfsFGAvp8J0iLGfcQQ8r6o2x72SBmsddg-B-ngfVQoC4tF0iw/viewform) (the download link is sent to your email; file list on the [Download page](https://tlab-wide.github.io/V2X_E2E_Simulator/latest/GettingStarted/Download/)) and import it with `Assets → Import Package → Custom Package…`.
4. Open the scene `Assets/V2X/Scenes/V2X_E2E_Simulator_v3.1.1.unity` and press Play.

See the [Getting Started guide](https://tlab-wide.github.io/V2X_E2E_Simulator/GettingStarted/QuickStartDemo/) for full setup instructions (ROS 2, Autoware, maps).

## CooL4 DM output

The simulator publishes the CooL4 data-linkage formats ([`dm_*_msgs`](https://github.com/tlab-wide/v2x_e2e_msgs)) directly:

| Topic | Type | Content |
|:--|:--|:--|
| `/signal_info` | `dm_signal_info_msgs/SignalInfoArray` | Signal state and remaining time of the five bus-route intersections, 10 Hz |
| `/signal_info/DSRC`, `/signal_info/Cellular` | same | The same with DSRC (100 ms, 5 % loss) / Cellular (400 ms) network delay |
| `/v2x/object_info` | `dm_object_info_msgs/ObjectInfoArray` | Objects detected by all RSU sensors, with sensor noise |
| `/v2x/object_info/DSRC`, `/v2x/object_info/Cellular` | same | The same with DSRC / Cellular network delay |
| `/v2x/object_info_groundtruth` | same | The same objects without noise |
| `/v2x/rsu…/sensor…/object_info[_groundtruth]` | same | One sensor each (with noise / without noise), no network delay |

Details: [DM Protocols](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/DMProtocols/) · [Topic List](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/DMProtocols/CooL4/Topics/)

## Intersection and traffic-light names

The signalised intersections of the Kashiwa scene (`Enviroment copy/intersectionsFixed/` in the environment prefab) and their traffic lights are named after the CooL4 DM signal sheet and the Lanelet2 map. The names are plain ASCII (romaji), so they are safe in logs, configs and code.

**Intersections.** The five bus-route intersections use their DM names. The others are named after the nearest OpenStreetMap landmark (OSM names only 東大前 itself).

| GameObject | Place | DM CRP ID | Approaches (方路, bearing from the centre) |
|:--|:--|:--|:--|
| `Todaimae` | 東大前 | `0x00000001` | 1 = 66° (E), 2 = 246° (W), 3 = 311° (N) |
| `Ganken` | がん研 | `0x000c1001` | 1 = 64° (E), 2 = 150° (S), 3 = 236° (W) |
| `Kakeiken` | 科警研 | `0x000c3001` | 1 = 65° (E), 2 = 160° (S), 3 = 347° (N) |
| `Wakashiba` | 若柴 | `0x00000002` | 1 = 15° (N), 2 = 100° (E), 3 = 190° (S), 4 = 280° (W) |
| `Ekimae` | 駅前 | `0x000c2001` | 1 = 83° (E), 2 = 180° (S), 3 = 0° (N) |
| `LalaportKashiwanoha` | ららぽーと柏の葉 | – | 1 = 354°, 2 = 91°, 3 = 173°, 4 = 272° |
| `ParkCityIchibangai` | パークシティ柏の葉キャンパス一番街 | – | 1 = 6°, 2 = 187°, 3 = 277° |
| `KobanMae` | 柏の葉キャンパス交番前 | – | 1 = 6°, 2 = 187°, 3 = 280° |
| `WakashibaRyokuchi` | 若柴緑地 | – | 1 = 6°, 2 = 97°, 3 = 187°, 4 = 278° |
| `KashiwanohaKoko` | 柏の葉高校前 | – | 1 = 338°, 2 = 67°, 3 = 159°, 4 = 248° |
| `KashiwanohaKoen` | 柏の葉公園 | – | 1 = 333°, 2 = 156°, 3 = 248° |
| `ToyofutaShogakko` | 十余二小学校 | – | 1 = 357°, 2 = 90°, 3 = 175°, 4 = 272° |
| `TodaiKashiwaWest` | 東京大学柏キャンパス西側 | – | 1 = 342°, 2 = 72°, 3 = 161°, 4 = 253° |
| `GankenHigashiByoin` | 国立がん研究センター東病院前 | – | 1 = 316°, 2 = 49°, 3 = 129°, 4 = 231° |

**Approach IDs (方路).** For the five DM intersections they are the IDs of the DM signal sheet. For the others they are numbered clockwise, starting with the leg closest to north (1 = that leg, 2 = the next leg clockwise, …).

**Traffic lights.** Each light carries the CooL4 signal IDs it shows (spec §3.6.3: vehicle ID = inbound 方路 × 0x10 + outbound 方路, pedestrian ID = crossed 方路 × 0x10):

```text
<Intersection>_V_0x<k>1-0x<k><N>_<i>    vehicle head for traffic entering from 方路 k (N = number of approaches)
<Intersection>_P_0x<k>0_<i>             pedestrian head of the crosswalk across 方路 k
```

`<i>` numbers the heads of the same approach (1, 2, …, clockwise around the centre). Examples: `Kakeiken_V_0x21-0x23_1` is the first vehicle head for traffic entering 科警研 from 方路2 (signal IDs 0x21–0x23). `Wakashiba_P_0x40_2` is the second pedestrian head of the crosswalk across 若柴 方路4.

The RSU sensor groups under `RSUs/Pseudo Sensors setup` follow the RSU numbers of the route: `RSU1_Ganken`, `RSU2_Kakeiken`, `RSU3_Midblock` (between 科警研 and 若柴), `RSU4_Wakashiba`, plus the variants `RSU1_Ganken_CooL4`, `RSU2_Kakeiken_CooL4` and `RSU2_Kakeiken_Alt`.

Names are used as references in a few places. Keep them in sync when renaming:
- `DMSignalInfoPublisher` → **Traffic Intersection Name** of each intersection
- pedestrian path configs (`Assets/Configs/pedestrian_spawn*.json`) and `WaypointSystem` waypoints → `trafficLightId`
- web viewer configs (`tools/web-viewer/config/traffic_lights.yaml` → `unity_intersection`, `sensors.yaml` → `scene_object`)

---

## License

This project is a derivative of **[AWSIM](https://github.com/tier4/AWSIM)** by TIER IV, Inc.

- **AWSIM code**: [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
- **AWSIM assets**: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)
- **V2X original code** (`/V2X` directory): [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) — Copyright 2025 T-LAB (Wide Research Group)
- **V2X original assets**: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)

See [`LICENSE`](./LICENSE) for full details.

(c) 2022–2025 Tsukada LAB — T-LAB Wide Research Group
