# CooL4 DM Topic List

All CooL4 DM topics of the Kashiwa scene (`V2X_E2E_Simulator_v3.1.1.unity`), with the scene's default settings. "Delayed copies" are the `/DSRC` (100 ms, 5 % loss) and `/Cellular` (400 ms) variants described in [Delayed copies](../index.md#delayed-copies-network-model).

## Signal information

Type: `dm_signal_info_msgs/msg/SignalInfoArray`. Publisher: **DM Signal Info Publisher** prefab. Rate: 10 Hz. QoS: reliable, volatile, keep-last 1. Details: [Signal Info](../SignalInfo/index.md).

| Topic | Content |
|:--|:--|
| `/signal_info` | **All five intersections** in one array. This is the topic DM consumes. |
| `/signal_info/DSRC` | Same, delayed 100 ms with 5 % packet loss |
| `/signal_info/Cellular` | Same, delayed 400 ms |

The delayed copies use the general profiles of the `NetworkSimulator` (see [Delayed copies](../index.md#delayed-copies-network-model)); turn them off with **Enable Delayed Messages** on the DM Signal Info Publisher.

## Object information

Type: `dm_object_info_msgs/msg/ObjectInfoArray`. Publisher: `ObjectInfo`, built at Play from `rsus.yaml` of the scenario (see [Scenario Configuration](../../../Configuration/Rsus/index.md#object-info-publishers-cool4-dm)). QoS: best effort, volatile, keep-last 1. Rate: 10 Hz ([publishing rate](../ObjectInfo/index.md#publishing-rate)). Details: [Object Info](../ObjectInfo/index.md).

"With noise" means position, rotation, size and existence noise; the ground truth has none. The RSUs:

| RSU | RSU ID | Sensors (sensor ID) |
|:--|:--|:--|
| がん研 (Ganken), `RSU1_Ganken_CooL4` | `0x12020001` | 4 LiDARs `RSU_Lidar 1`–`4` (1–4) |
| 科警研 (Kakeiken), `RSU2_Kakeiken_CooL4` | `0x12020002` | Pole 6, 7, 9: camera 6 (1), camera 7 (2), LiDAR 7 (3), camera 9 (4), LiDAR 9 (5) |
| 科警研 (Kakeiken), `RSU2_Kakeiken_CooL4` | `0x12020003` | Pole 2, 3, 8: camera 2 (1), LiDAR 2 (2), camera 3 (3), LiDAR 3 (4), camera 8 (5) |
| Midblock between 科警研 and 若柴, `RSU3_Midblock` | `0x12020004` | 1 LiDAR (1) |
| 若柴 (Wakashiba), `RSU4_Wakashiba` | `0x12020005` | cameras c1–c4 (1–4), LiDAR s1 (5) |


### All RSUs in one message

| Topic | Content |
|:--|:--|
| `/v2x/object_info` | Every object seen by any RSU sensor (all five RSUs above), with noise. Each object lists the sensors (RSU ID + sensor ID) that saw it. |
| `/v2x/object_info/DSRC` | Same, delayed 100 ms with 5 % packet loss |
| `/v2x/object_info/Cellular` | Same, delayed 400 ms |
| `/v2x/object_info_groundtruth` | The same objects without noise |

### One topic pair per sensor (tracking research)

Only the 10 Kakeiken sensors; no delayed copies.

| Topic prefix | RSU ID | Sensor |
|:--|:--|:--|
| `/v2x/rsu12020002/sensor1/` | 0x12020002 | Pole 6 |
| `/v2x/rsu12020002/sensor2/` | 0x12020002 | Pole 7 camera |
| `/v2x/rsu12020002/sensor3/` | 0x12020002 | Pole 7 LiDAR |
| `/v2x/rsu12020002/sensor4/` | 0x12020002 | Pole 9 camera |
| `/v2x/rsu12020002/sensor5/` | 0x12020002 | Pole 9 LiDAR |
| `/v2x/rsu12020003/sensor1/` | 0x12020003 | Pole 2 camera |
| `/v2x/rsu12020003/sensor2/` | 0x12020003 | Pole 2 LiDAR |
| `/v2x/rsu12020003/sensor3/` | 0x12020003 | Pole 3 camera |
| `/v2x/rsu12020003/sensor4/` | 0x12020003 | Pole 3 LiDAR |
| `/v2x/rsu12020003/sensor5/` | 0x12020003 | Pole 8 camera |

Each prefix carries `object_info` (with noise) and `object_info_groundtruth` (without noise).

Object numbers are shared by all these topics: the same object has the same object number everywhere ([Object ID layout](../ObjectInfo/index.md#object-id-layout)).

## Inspecting the topics

For decoded output (units, light names, IDs split into RSU / sensor / object), see [Reading the Messages](../Reading/index.md).

```console
# list every DM topic
ros2 topic list -t | grep -E "dm_(object|signal)_info_msgs"

# one signal snapshot of the whole route
ros2 topic echo /signal_info --once

# rate check
ros2 topic hz /v2x/object_info
```
