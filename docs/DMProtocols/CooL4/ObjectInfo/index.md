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
 ObjectInfo component (one per RSU, or one per sensor)          every 1 / (2 × Hz) s
          │  1. collect every object seen by any of its sensors (ego excluded)
          │  2. build one ObjectInfo per object, twice:
          │       ground truth  → topicGroundTruth   e.g. /v2x/rsu12020002/sensor1/object_info
          │       with noise    → topic              e.g. /v2x/rsu12020002/sensor1/object_info_noise
          │  3. optional delayed / lossy copies of the noisy topic (/DSRC, /Cellular)
          ▼
 dm_object_info_msgs/ObjectInfoArray
```

An object seen by several sensors of the same component is sent **once**, with every sensor that saw it in `information_source_list`.

## Message format

`dm_object_info_msgs/ObjectInfoArray` holds `ObjectInfo[] array`. The table lists every field the simulator fills; all other fields keep their "unknown" default (see [conventions](../index.md#unknown-values-and-optional-fields)).

| Field | Spec item | Value published by the simulator |
|:--|:--|:--|
| `id.value` | 物標 ID [required] | Same as `information_source_list[0]` (see [Object ID layout](#object-id-layout)) |
| `time.value` | 情報取得時刻 [required] | Last update time of the first sensor that saw the object, as `DE_TimestampIts` ([epoch note](../index.md#timestamp-de_timestampits)) |
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
| `information_source_list[]` | 情報源のリスト [required] | One object ID per sensor that saw the object |

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

- **RSU ID**: the component's `Rsu Id` field, a hex string such as `0x12020002`.
- **Sensor ID**: the `Sensor Id` of the `MockSensor` that saw the object.
- **Object number**: lower 16 bits of a random number drawn once per object (keyed by the object's UUID). It is stable while the object exists.

`information_source_list` holds one such ID per sensor that saw the object. `id` copies the first one, so the same object seen by a different first sensor gets a different `id`.

!!! note "Differences from the spec"
    - The spec expects `information_source_list` to contain the *observing device's* ID (`00` + device ID). The simulator puts the per-sensor recognised-object IDs there instead, which lets a consumer tell which sensors saw the object.
    - The list is not capped at 4 entries.
    - The object number is 16 bits, so two objects can collide in rare cases.

## Publishing rate

The **Hz** field sets half the publishing rate. `ObjectInfo.Update()` publishes every `1 / (2 × Hz)` seconds, so `Hz = 10` produces about **20 Hz**, and `Hz = 1` produces about **2 Hz**. Verify with `ros2 topic hz`.

QoS: best effort, volatile, keep-last 1.

## Components

| Component | Script | Use |
|:--|:--|:--|
| **ObjectInfo** | `Assets/V2X/Script/CustomMsg/ObjectInfo.cs` | Main DM object publisher. Takes a list of `MockSensor`s. |
| ObjectInfoPublisher *(deprecated)* | `Assets/V2X/Script/CustomMsg/ObjectInfoPublisher.cs` | Publishes everything inside a `GroundTruthArea` without sensors. Single topic, no delay support. |

### ObjectInfo Inspector

| Field | Description |
|:--|:--|
| **Sensors** | Pseudo sensors whose detections are merged into this component's output |
| **Hz** | Half of the publishing rate (see [Publishing rate](#publishing-rate)) |
| **Is Ground Truth** | Not used by `ObjectInfo`. Both the noisy and the ground-truth topic are always published. It is kept for the deprecated `ObjectInfoPublisher`. |
| **Topic** | Noisy output |
| **Topic Ground Truth** | Noise-free output. Set it to `None` or leave it empty to disable. |
| **Rsu Id** | 32-bit device ID as a hex string, used in the object IDs |
| **Position / Rotation / Dimension / Probability Noise Name** | Noise profile names from the Noise Setting |
| **Enable Delayed Messages**, **Message Delays Configs** | [Delayed copies](../index.md#delayed-copies-network-model) of the noisy topic (or of the ground-truth topic for profiles marked *Is Ground Truth*) |

### Adding a DM object publisher

1. Place or reuse pseudo sensors ([Pseudo Sensor setup](../../../Components/PseudoSensors/PseudoSensorsSetup/index.md)) and give each a unique **Sensor Id**.
2. Add an **ObjectInfo** component under `RSUs/Publishers`, fill **Sensors**, and set **Rsu Id** and the topics.
3. Optionally enable delayed messages.

## Setups in the Kashiwa scene

Under `RSUs/Publishers` in `V2X_E2E_Simulator_v3.1.1.unity`:

| Setup | Default | What it publishes |
|:--|:--|:--|
| **Int#2 CooL4 Setup for Data Fusion (DM)** | active | One topic pair per sensor at Intersection #2 (科警研). RSU `0x12020002` sensors 1–5 and RSU `0x12020003` sensors 1–5, plus an all-sensor ground-truth source. |
| **BasicSetup / RSU2** | active | Intersection #2 (科警研) merged: `/v2x/object_info_2[_noise]` (Hz 1) |
| **BasicSetup / RSU1** | inactive | Intersection #1 (がん研) merged: `/v2x/object_info_1[_noise]` |
| **All Intersection CooL4 Setup (DM)** | inactive | One merged topic pair per intersection: `/v2x/rsu1`, `rsu2_1`, `rsu2_2`, `rsu3`, `rsu4` (RSU IDs `0x12020001`–`0x12020005`) |

See the [Topic List](../Topics/index.md) for every topic name.
