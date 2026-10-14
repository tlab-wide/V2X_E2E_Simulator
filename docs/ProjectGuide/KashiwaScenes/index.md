# Kashiwa Scene Names

The signalised intersections of the Kashiwa scene (`Enviroment copy/intersectionsFixed/` in the environment prefab) and their traffic lights are named after the CooL4 DM signal sheet and the Lanelet2 map. The names are plain ASCII (romaji), so they are safe in logs, configs and code.

## Intersections

The five bus-route intersections use their DM names. The others are named after the nearest OpenStreetMap landmark (OSM names only 東大前 itself).

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

## Approach IDs (方路)

For the five DM intersections they are the IDs of the DM signal sheet. For the others they are numbered clockwise, starting with the leg closest to north (1 = that leg, 2 = the next leg clockwise, …).

## Traffic lights

Each light carries the CooL4 signal IDs it shows (spec §3.6.3: vehicle ID = inbound 方路 × 0x10 + outbound 方路, pedestrian ID = crossed 方路 × 0x10):

```text
<Intersection>_V_0x<k>1-0x<k><N>_<i>    vehicle head for traffic entering from 方路 k (N = number of approaches)
<Intersection>_P_0x<k>0_<i>             pedestrian head of the crosswalk across 方路 k
```

`<i>` numbers the heads of the same approach (1, 2, …, clockwise around the centre). Examples: `Kakeiken_V_0x21-0x23_1` is the first vehicle head for traffic entering 科警研 from 方路2 (signal IDs 0x21–0x23). `Wakashiba_P_0x40_2` is the second pedestrian head of the crosswalk across 若柴 方路4.

## RSUs

The RSU sensor groups under `RSUs/Pseudo Sensors setup` follow the RSU numbers of the route: `RSU1_Ganken_CooL4` (RSU ID `0x12020001`), `RSU2_Kakeiken_CooL4` (`0x12020002`, `0x12020003`), `RSU3_Midblock` (`0x12020004`, between 科警研 and 若柴) and `RSU4_Wakashiba` (`0x12020005`). Sensor IDs are unique within each RSU.

## Where the names are used

Names are used as references in a few places. Keep them in sync when renaming:
- `DMSignalInfoPublisher` → **Traffic Intersection Name** of each intersection
- pedestrian path configs (`Assets/Configs/pedestrian_spawn*.json`) and `WaypointSystem` waypoints → `trafficLightId`
- web viewer configs (`tools/web-viewer/config/traffic_lights.yaml` → `unity_intersection`, `sensors.yaml` → `scene_object`)
