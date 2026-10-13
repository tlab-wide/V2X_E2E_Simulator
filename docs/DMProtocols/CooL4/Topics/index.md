# CooL4 DM Topic List

All CooL4 DM topics of the Kashiwa scene (`V2X_E2E_Simulator_v3.1.0.unity`), with the scene's default settings. "Delayed copies" are the `/DSRC` (100 ms, 5 % loss) and `/Cellular` (400 ms) variants described in [Delayed copies](../index.md#delayed-copies-network-model).

## Signal information

Type: `dm_signal_info_msgs/msg/SignalInfoArray`. Publisher: **DM Signal Info Publisher** prefab. Rate: 10 Hz. QoS: reliable, volatile, keep-last 1. Details: [Signal Info](../SignalInfo/index.md).

| Topic | Content |
|:--|:--|
| `/signal_info` | **All five intersections** in one array. This is the topic DM consumes. |
| `/v2x/todaimae/signal_info` | 東大前 (UTokyo), CRP 1, `crossroad 14` |
| `/v2x/rsu1/signal_info` | がん研 (Ganken), CRP 790529, `crossroad1` |
| `/v2x/rsu2/signal_info` | 科警研 (Kakeiken), CRP 798721, `crossroad2` |
| `/v2x/rsu4/signal_info` | 若柴 (Wakashiba), CRP 2, `crossroad3` |
| `/v2x/rsu5/signal_info` | 駅前 (Station), CRP 794625, `crossroad4` |

Delayed copies (for example `/signal_info/DSRC`) are published only when **Enable Delayed Messages** is on (off by default).

## Object information

Type: `dm_object_info_msgs/msg/ObjectInfoArray`. Publisher: `ObjectInfo`. QoS: best effort, volatile, keep-last 1. The actual rate is twice the **Hz** field ([why](../ObjectInfo/index.md#publishing-rate)). Details: [Object Info](../ObjectInfo/index.md).

Every source publishes a pair: `…/object_info` (ground truth) and `…/object_info_noise` (with sensor noise). Delayed copies are made from the noisy topic, for example `…/object_info_noise/DSRC`.

### Active by default

**Int#2 CooL4 Setup for Data Fusion (DM)**: Intersection #2 (科警研), one source per sensor, about 20 Hz.

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

Each prefix carries `object_info`, `object_info_noise`, `object_info_noise/DSRC` and `object_info_noise/Cellular`.

| Topic | Content |
|:--|:--|
| `/v2x/rsu_all/object_info_groundtruth` | Ground truth of the whole sensor set of Intersection #2 |
| `/v2x/rsu_all/object_info_groundtruth_noise` | Same, with noise |
| `/v2x/object_info_2`, `/v2x/object_info_2_noise` | RSU2 (Intersection #2) merged over all its sensors, about 2 Hz |

### Inactive by default

Enable the GameObject to publish these.

| Setup | Topics | RSU ID |
|:--|:--|:--|
| BasicSetup / RSU1 | `/v2x/object_info_1`, `/v2x/object_info_1_noise` | 0x1100 |
| All Intersection CooL4 Setup (DM) / Intersection 1 | `/v2x/rsu1/object_info[_noise]` | 0x12020001 |
| All Intersection CooL4 Setup (DM) / Intersection 2-1 | `/v2x/rsu2_1/object_info[_noise]` | 0x12020002 |
| All Intersection CooL4 Setup (DM) / Intersection 2-2 | `/v2x/rsu2_2/object_info[_noise]` | 0x12020003 |
| All Intersection CooL4 Setup (DM) / Intersection 3 | `/v2x/rsu3/object_info[_noise]` | 0x12020004 |
| All Intersection CooL4 Setup (DM) / Intersection 4 | `/v2x/rsu4/object_info[_noise]` | 0x12020005 |

## Inspecting the topics

```console
# list every DM topic
ros2 topic list -t | grep -E "dm_(object|signal)_info_msgs"

# one signal snapshot of the whole route
ros2 topic echo /signal_info --once

# rate check
ros2 topic hz /v2x/rsu12020002/sensor1/object_info_noise
```
