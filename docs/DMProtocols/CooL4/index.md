# CooL4 DM Protocols

**CooL4** (自動運転レベル４等先進モビリティサービス研究開発・社会実装プロジェクト テーマ 4) defines a *data-linkage platform* (データ連携 PF). It collects dynamic information from vehicles and roadside equipment and hands it to automated-driving systems. Its API specification, *CooL4 データ連携 PF API 仕様 v0.8.1*, defines four kinds of dynamic information:

| Information | Japanese | Purpose | In the simulator |
|:--|:--|:--|:--|
| Object information | 物標情報 | Position and state of vehicles, pedestrians, … | **Published:** [Object Info](ObjectInfo/index.md) |
| Signal information | 信号情報 | Current colour of each traffic signal and when it changes | **Published:** [Signal Info](SignalInfo/index.md) |
| Free-space information | フリースペース情報 | Areas known to be free of objects | Message package included; not covered here |
| Sensor information | センサー情報 | Sensor mounting position and detection area | Message package included; not covered here |

The ROS 2 packages `dm_object_info_msgs`, `dm_signal_info_msgs`, … are a direct transcription of the spec's *logical* data formats. Each data element (DE) or data frame (DF) of the spec becomes a small message with a `value` field and named constants.

## How the simulator plays the roles of the platform

```text
 Real deployment                               Simulator
 ───────────────                               ─────────
 Roadside sensors (LiDAR, cameras)        ≙    Pseudo sensors (MockSensor) on RSU poles
 Cooperative roadside unit (協調型路側機)   ≙    ObjectInfo component, one per RSU / sensor
 Traffic signal controller                ≙    TrafficIntersection lighting sequences
 DM messenger / ROS2DMI                   ≙    DMSignalInfoPublisher (/signal_info)
```

Every DM message the simulator sends is built from the simulation's **ground truth**. Object positions, sizes and speeds can be perturbed with the [noise settings](../../Components/PseudoSensors/NoiseSetting/index.md) to mimic sensor error. Messages can also be delayed or dropped with the built-in network model (see [Delayed copies](#delayed-copies-network-model)).

## Conventions shared by all CooL4 messages

### Units

The spec fixes a small set of units (§3.2, §5). The simulator converts from Unity (metres, degrees, seconds) as follows:

| Quantity | DM unit | Conversion in the simulator | Unknown value |
|:--|:--|:--|:--|
| Length / size | 0.01 m | `metres × 100` | 65535 (`ObjectDimensionValue`) |
| Azimuth (heading, direction) | 0.0125° clockwise from north | `degrees × 80` | 28800 |
| Speed | 0.01 m/s | `m/s × 100` | 16383 |
| Latitude / longitude | 0.1 µdeg (1e-7 deg) | `deg × 10 000 000` | 900000001 / 1800000001 |
| Altitude | 0.01 m | `metres × 100` | 800001 |
| Remaining signal time | 0.1 s | `seconds × 10` | 65535 |

### Timestamp (`DE_TimestampIts`)

`TimestampIts.value` is the number of milliseconds since 2004‑01‑01 00:00. The spec counts from **UTC**. The simulator subtracts `1072882800000` from the current Unix time in milliseconds, which is 2004‑01‑01 00:00 **JST**. As a result, simulator timestamps run 9 hours (32 400 000 ms) ahead of a strict UTC reading.

All DM publishers in the simulator (object info and signal info) use the same helper, `ObjectInfo.GetITSTimeInMilliseconds()`, so timestamps are consistent with each other.

!!! warning "Converting to UTC"
    To recover UTC Unix time from a simulator timestamp, add `1072882800000`, not `1072915200000`. If the DM side assumes the spec's UTC epoch, account for the 9 h offset.

### Unknown values and optional fields

The spec makes most fields optional, and every optional field has an "unknown" value. Fields the simulator does not fill keep the default from the `.msg` file, which is the unknown value where one is defined. For example, `MinTimeToChange.value` defaults to `65535` and `AzimuthValue.value` defaults to `28800`.

### Identifiers

| Identifier | Size | Used in | Defined in |
|:--|:--|:--|:--|
| Object ID (物標 ID) | 64 bit | `ObjectInfo.id`, `information_source_list` | spec §3.3.3, see [Object Info](ObjectInfo/index.md#object-id-layout) |
| CRP ID (交差点 ID) | 32 bit | `SignalInfo.crp_id` | assigned by the traffic administrator |
| Signal ID (信号灯器 ID) | 8 bit | `SignalInfo.signal_id_list` | spec §3.6.3, see [Signal Info](SignalInfo/index.md#signal-id-layout) |

## Delayed copies (network model)

Each DM publisher can also publish **delayed and lossy copies** of its messages. The general profiles live on the `NetworkSimulator` component (the **Noise and Delay Profile** object in the scene), and each publisher can add its own. Each profile appends a suffix to the topic name:

| Suffix | Delay | Packet loss |
|:--|:--|:--|
| `/DSRC` | 100 ms | 5 % |
| `/Cellular` | 400 ms | 0 % |

For example, `/v2x/rsu12020002/sensor1/object_info_noise` is also published as `/v2x/rsu12020002/sensor1/object_info_noise/DSRC` and `.../Cellular`. Turn this on or off per component with **Enable Delayed Messages**.

## Where to go next

- [Object Info](ObjectInfo/index.md): how detected objects become `ObjectInfoArray` messages, field by field.
- [Signal Info](SignalInfo/index.md): how traffic lights become `SignalInfoArray` messages, including remaining time and the Kashiwa signal ID table.
- [Topic List](Topics/index.md): every DM topic in the Kashiwa scene, with type, rate and QoS.
