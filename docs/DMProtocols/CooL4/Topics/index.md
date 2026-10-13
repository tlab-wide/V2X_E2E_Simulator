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

Type: `dm_object_info_msgs/msg/ObjectInfoArray`. Publisher: `ObjectInfo` (`RSUs/Publishers/CooL4 Object Info (DM)`). QoS: best effort, volatile, keep-last 1. The actual rate is twice the **Hz** field, about 20 Hz ([why](../ObjectInfo/index.md#publishing-rate)). Details: [Object Info](../ObjectInfo/index.md).

All sensors are at 科警研 (Kakeiken), on two RSUs: `0x12020002` and `0x12020003`. "With noise" means position, rotation, size and existence noise; the ground truth has none.

### All RSUs in one message

| Topic | Content |
|:--|:--|
| `/v2x/object_info` | Every object seen by any of the 10 sensors, with noise. Each object lists the sensors (RSU ID + sensor ID) that saw it. |
| `/v2x/object_info/DSRC` | Same, delayed 100 ms with 5 % packet loss |
| `/v2x/object_info/Cellular` | Same, delayed 400 ms |
| `/v2x/object_info_groundtruth` | The same objects without noise |

### One topic pair per sensor (tracking research)

No delayed copies.

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

```console
# list every DM topic
ros2 topic list -t | grep -E "dm_(object|signal)_info_msgs"

# one signal snapshot of the whole route
ros2 topic echo /signal_info --once

# rate check
ros2 topic hz /v2x/object_info
```
