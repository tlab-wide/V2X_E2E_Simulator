# Object Info (物標情報)

Object information describes the position and state of objects on and around the road: vehicles, pedestrians and cyclists (spec §3.3). In the simulator, roadside pseudo sensors detect these objects. The `ObjectInfo` component then publishes them as **recognised object information** (認識物標情報), the way a cooperative roadside unit (協調型路側機) would.

## How it works

```text
 NPC vehicles / pedestrians / cyclists          (ground truth in Unity)
          │
          ▼
 Pseudo sensors (MockSensor: LiDAR / camera FOV + line-of-sight checks), each with a sensor ID
          │  GetSeenObjects()
          ▼
 ObjectInfo component                                            every 1 / Hz s
          │  1. collect every object seen by its sensors, by the sensors of its Sources (other
          │     ObjectInfo components) and of its Rsu Sensors groups, each with its RSU ID; ego excluded
          │  2. build one ObjectInfo per object, twice:
          │       with noise    → topic              e.g. /v2x/object_info
          │       ground truth  → topicGroundTruth   e.g. /v2x/object_info_groundtruth
          │  3. optional delayed / lossy copies of the noisy topic (/DSRC, /Cellular)
          ▼
 dm_object_info_msgs/ObjectInfoArray
```

An object seen by several sensors is sent **once**, with every sensor that saw it in `information_source_list`. The ground truth contains the same objects with the same IDs, without position, rotation, size or existence noise.

## Message format

`dm_object_info_msgs/ObjectInfoArray` holds `ObjectInfo[] array`. The table lists every field the simulator fills; all other fields keep their "unknown" default (see [conventions](../index.md#unknown-values-and-optional-fields)).

| Field | Spec item | Value published by the simulator |
|:--|:--|:--|
| `id.value` | 物標 ID [required] | Same as `information_source_list[0]` (see [Object ID layout](#object-id-layout)) |
| `time.value` | 情報取得時刻 [required] | Last update time of the sensor of `information_source_list[0]`, as `DE_TimestampIts` ([epoch note](../index.md#timestamp-de_timestampits)) |
| `object_class[0].id.value` | 物標種別 | `1` VEHICLE if the object is an `NPCVehicle`, else `2` PERSON (pedestrians and cyclists) |
| `object_class[0].confidence.value` | 分類信頼度 | `101` |
| `existency.value` | 存在信頼度 | Ground truth: `101` (certain). Noisy: `0.8 × 101` with the probability noise applied. |
| `object_location.geodetic_srid.value` | 空間参照系 | `4326` (WGS84) |
| `object_location.latitude.value` | 緯度 | 1e-7 deg, see [coordinates](#coordinates) |
| `object_location.longitude.value` | 経度 | 1e-7 deg |
| `object_location.altitude.value` | 標高 | 0.01 m (`(ROS z + MGRS offset z) × 100`) |
| `orientation.value.value` | 物標の向き | Heading, 0.0125° clockwise from north |
| `direction.value.value` | 移動方向 | Direction of the velocity vector, 0.0125° from north. Falls back to the heading when the object is not moving (< 1 cm/s). |
| `speed.value.value` | 速さ | 0.01 m/s. Vehicles: rigidbody speed. Pedestrians/cyclists: their walking/riding speed (`ISpeed`). |
| `size.length/width/height.value.value` | 物標のサイズ | 0.01 m. Vehicles: bounding box of the `NPCVehicle`. Others: fixed 0.5 × 0.5 × 1.7 m. |
| `information_source_list[]` | 情報源のリスト [required] | One object ID per sensor that saw the object, sorted by RSU ID, then sensor ID |

Noise (position, rotation, size, existence probability) uses the profiles named in the component, defined in the [Noise Setting](../../../Components/PseudoSensors/NoiseSetting/index.md). The ground-truth message skips noise.

### Coordinates

Unity positions are converted to ROS (ENU), shifted by the UTM origin of the Kashiwa map, and converted to latitude/longitude with GeographicLib:

| Parameter | Value |
|:--|:--|
| UTM zone | 54, northern hemisphere |
| Origin easting | 404 542.530 m |
| Origin northing | 3 972 957.923 m |

Only the latitude/longitude/altitude representation of `DF_Location` is filled. The plane-rectangular, CRP+offset, lane-count and lane-ID representations, and the position accuracy, stay unknown.

## Object ID layout

Spec §3.3.3 packs every object ID into 64 bits. The top two bits say who assigned it:

```text
 bits 63-62  meaning                                       remaining 62 bits
 00          cooperative roadside unit itself              reserved (30) | device ID (32)
 01          object's own pseudonym ID (自己物標)            reserved (12) | pseudonym ID (50)
 10          object recognised by a roadside unit           number assigned by the RSU (30) | device ID (32)
 11          object recognised by a vehicle                 number assigned by the vehicle (12) | pseudonym ID (50)
```

The simulator plays roadside units, so it always uses **`10`**. It splits the 30-bit "number assigned by the RSU" into a sensor ID and an object number:

```text
  63 62 61 ......... 56 55 ....... 48 47 ........... 32 31 ............................ 0
 ┌─────┬──────────────┬─────────────┬─────────────────┬────────────────────────────────┐
 │ 1 0 │  0 (6 bit)   │ sensor ID   │ object number   │ RSU / device ID (rsuId)        │
 │     │              │  (8 bit)    │  (16 bit)       │  e.g. 0x12020002               │
 └─────┴──────────────┴─────────────┴─────────────────┴────────────────────────────────┘
```

- **RSU ID**: the `Rsu Id` field of the component the sensor belongs to, a hex string such as `0x12020002`. A combined publisher keeps each source's RSU ID.
- **Sensor ID**: the `Sensor Id` of the `MockSensor` that saw the object.
- **Object number**: a random number (1–65 535) drawn once per object (keyed by the object's UUID) from one table shared by all `ObjectInfo` components. So the same object has the same number on every topic and at every intersection, and a number is never given to two objects (the table starts over after 65 535 objects).

`information_source_list` holds one such ID per sensor that saw the object, sorted by RSU ID and then sensor ID. `id` copies the first one. It changes only when the set of sensors that see the object changes so that a different sensor comes first.

!!! note "Differences from the spec"
    - The spec expects `information_source_list` to contain the *observing device's* ID (`00` + device ID). The simulator puts the per-sensor recognised-object IDs there instead, which lets a consumer tell which sensors saw the object.
    - The list is not capped at 4 entries.

## Publishing rate

The **Hz** field is the publishing rate: `ObjectInfo.Update()` publishes every `1 / Hz` seconds of simulation time, carrying the remainder over so the average stays at **Hz** even when the frame rate does not divide it. All publishers of the Kashiwa scene use **10 Hz**. Verify with `ros2 topic hz`.

QoS: best effort, volatile, keep-last 1.

## Components

| Component | Script | Use |
|:--|:--|:--|
| **ObjectInfo** | `Assets/V2X/Script/CustomMsg/ObjectInfo.cs` | DM object publisher. Takes a list of `MockSensor`s and/or other `ObjectInfo` components as sources. |

### ObjectInfo Inspector

| Field | Description |
|:--|:--|
| **Sensors** | Pseudo sensors whose detections are merged into this component's output (reported with this component's **Rsu Id**) |
| **Sources** | Other `ObjectInfo` components whose sensors are merged in as well, each reported with that component's **Rsu Id**. Used for one message covering several RSUs. |
| **Rsu Sensors** | Further RSUs (an **Rsu Id** and its **Sensors**) merged into this message only. For RSUs that do not publish their own topics. |
| **Hz** | Publishing rate (see [Publishing rate](#publishing-rate)) |
| **Topic** | Noisy output |
| **Topic Ground Truth** | Noise-free output. Set it to `None` or leave it empty to disable. |
| **Rsu Id** | 32-bit device ID as a hex string, used in the object IDs |
| **Position / Rotation / Dimension / Probability Noise Name** | Noise profile names from the Noise Setting |
| **Enable Delayed Messages**, **Message Delays Configs** | [Delayed copies](../index.md#delayed-copies-network-model) of the noisy topic (or of the ground-truth topic for profiles marked *Is Ground Truth*) |

### Adding a DM object publisher

1. Place or reuse pseudo sensors ([Pseudo Sensor setup](../../../Components/PseudoSensors/PseudoSensorsSetup/index.md)) and give each a unique **Sensor Id**.
2. Add an **ObjectInfo** component under `RSUs/Publishers`, fill **Sensors**, and set **Rsu Id** and the topics.
3. Optionally enable delayed messages.

## Setup in the Kashiwa scene

Under `RSUs/Publishers/CooL4 Object Info (DM)` in `V2X_E2E_Simulator_v3.1.1.unity`, all at 10 Hz:

| Publisher | Topics | Use |
|:--|:--|:--|
| **all_rsu_publisher** | `/v2x/object_info`, `/v2x/object_info/DSRC`, `/v2x/object_info/Cellular`, `/v2x/object_info_groundtruth` | Every object seen by any RSU sensor below, with each sensor's RSU ID. The 10 Kakeiken sensors come in through **Sources**, the other RSUs through **Rsu Sensors**. This is the DM-style output, with network-delayed copies. |
| **indivisual_publisher/…** (10 publishers, one per Kakeiken sensor) | `/v2x/rsu<RSU ID>/sensor<N>/object_info`, `…/object_info_groundtruth` | One sensor each, without delayed copies. For tracking research. |

The RSUs and their sensors:

| RSU | RSU ID | Sensors (sensor ID) |
|:--|:--|:--|
| がん研 (Ganken), `RSU1_Ganken_CooL4` | `0x12020001` | 4 LiDARs `RSU_Lidar 1`–`4` (1–4) |
| 科警研 (Kakeiken), `RSU2_Kakeiken_CooL4` | `0x12020002` | Pole 6, 7, 9: camera 6 (1), camera 7 (2), LiDAR 7 (3), camera 9 (4), LiDAR 9 (5) |
| 科警研 (Kakeiken), `RSU2_Kakeiken_CooL4` | `0x12020003` | Pole 2, 3, 8: camera 2 (1), LiDAR 2 (2), camera 3 (3), LiDAR 3 (4), camera 8 (5) |
| Midblock between 科警研 and 若柴, `RSU3_Midblock` | `0x12020004` | 1 LiDAR (1) |
| 若柴 (Wakashiba), `RSU4_Wakashiba` | `0x12020005` | cameras c1–c4 (1–4), LiDAR s1 (5) |

A pseudo sensor only detects objects when it is listed in the active scenario (**Cooperative Autonomous Driving** → *Rsu Lidar* / *Rsu Cameras*), which runs the line-of-sight checks. Add new RSU sensors there as well.

See the [Topic List](../Topics/index.md) for every topic name.
