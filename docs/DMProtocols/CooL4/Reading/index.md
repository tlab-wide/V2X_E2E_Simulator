# Reading the Messages

This page shows how to look at the CooL4 DM topics while the simulator runs, and how to read the content: the units, the light codes, and how object and signal IDs are built. The field-by-field specification is on the [Object Info](../ObjectInfo/index.md) and [Signal Info](../SignalInfo/index.md) pages.

## Three ways to look at the topics

All three need ROS 2 Humble and the built [v2x_e2e_msgs](https://github.com/tlab-wide/v2x_e2e_msgs) workspace sourced, with the same middleware as the simulator:

```console
source /opt/ros/humble/setup.bash
source ~/v2x_e2e_msgs/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
```

| Tool | Command | Shows |
|:--|:--|:--|
| `ros2 topic echo` | `ros2 topic echo --once /v2x/object_info` | Raw message: integers in spec units, every field |
| `dm_echo.py` | `python3 tools/dm-echo/dm_echo.py` | Decoded text: degrees, metres, km/h, light colours, IDs split into RSU / sensor / object |
| [Web viewer](../../../Tools/WebViewer/index.md) | `tools/web-viewer/run.sh`, open <http://localhost:8080> | Objects and signal states on the map. Click one for its decoded fields. |

`dm_echo.py` takes topic names (default `/v2x/object_info` and `/signal_info`) and `--once`:

```console
python3 tools/dm-echo/dm_echo.py --once /v2x/object_info
python3 tools/dm-echo/dm_echo.py /signal_info /v2x/object_info/DSRC
```

## Object info

### An example

One pedestrian seen by three sensors at 科警研 (Kakeiken), as `dm_echo.py` prints it:

```text
- object 43436 (person), 14:02:33.331, existence 75/101
    position  35.8960759, 139.9420145, altitude 18.58 m
    heading   168.7 deg, moving to 164.6 deg at 1.41 m/s (5.1 km/h)
    size      0.50 x 0.50 x 1.70 m (length x width x height)
    id        0x8003a9ac12020002 = object 43436 seen by Kakeiken_A / RSU_Lidar 7 (RSU 0x12020002, sensor 3)
    also      0x8004a9ac12020002 = object 43436 seen by Kakeiken_A / RSU_camera 9 (RSU 0x12020002, sensor 4)
    also      0x8004a9ac12020003 = object 43436 seen by Kakeiken_B / RSU_Lidar 3 (RSU 0x12020003, sensor 4)
```

The same object in `ros2 topic echo` (fields left at their unknown value are omitted here):

| Raw field | Raw value | Conversion | Meaning |
|:--|--:|:--|:--|
| `id.value` | 9224403018286497794 | hex `0x8003a9ac12020002`, see [IDs](#object-ids) | object 43436, RSU `0x12020002`, sensor 3 |
| `time.value` | 719071353331 | ms since 2004‑01‑01 00:00 JST ([note](../index.md#timestamp-de_timestampits)) | 14:02:33.331 JST |
| `object_class[0].id.value` | 2 | 1 = vehicle, 2 = person (pedestrians and cyclists) | person |
| `existency.value` | 75 | out of 101; the ground-truth topic always has 101 | 74 % |
| `object_location.latitude.value` | 358960759 | ÷ 10⁷ | 35.8960759° |
| `object_location.longitude.value` | 1399420145 | ÷ 10⁷ | 139.9420145° |
| `object_location.altitude.value` | 1858 | ÷ 100 | 18.58 m |
| `orientation.value.value` | 13497 | × 0.0125 | 168.7° (clockwise from north) |
| `direction.value.value` | 13168 | × 0.0125 | moving towards 164.6° |
| `speed.value.value` | 141 | ÷ 100 | 1.41 m/s = 5.1 km/h |
| `size.length / width / height` | 50 / 50 / 170 | ÷ 100 | 0.5 × 0.5 × 1.7 m |
| `information_source_list[]` | 3 IDs | one per sensor that sees the object | the three sensors above |

### Object IDs

Every object ID is 64 bits. The simulator acts as a roadside unit, so the top two bits are always `10` (*object recognised by a roadside unit*, spec §3.3.3):

| Bits | 63–62 | 61–56 | 55–48 | 47–32 | 31–0 |
|:--|:--|:--|:--|:--|:--|
| Field | owner | reserved | sensor ID | object number | RSU ID |
| `0x8003a9ac12020002` | `10` | `000000` | `0x03` = 3 | `0xa9ac` = 43436 | `0x12020002` = Kakeiken_A |
| `0x8004a9ac12020003` | `10` | `000000` | `0x04` = 4 | `0xa9ac` = 43436 | `0x12020003` = Kakeiken_B |

- **Object number**: one random number per object (1–65 535). It is the same on every topic and in every intersection while the object exists. It is the part to use to **track** an object.
- **Sensor ID + RSU ID**: which sensor saw it. A different sensor gives a different ID for the same object.
- `information_source_list` holds one ID per sensor that sees the object, sorted by RSU ID, then sensor ID. `id` is the first of them.

To split an ID in Python:

```python
def split_object_id(value):
    return {
        'owner':         value >> 62,               # 2 = recognised by RSU
        'sensor_id':     (value >> 48) & 0xFF,
        'object_number': (value >> 32) & 0xFFFF,
        'rsu_id':        value & 0xFFFFFFFF,
    }

split_object_id(9224403018286497794)
# {'owner': 2, 'sensor_id': 3, 'object_number': 43436, 'rsu_id': 302120962}   (302120962 = 0x12020002)
```

### RSU and sensor IDs (Kashiwa)

The IDs come from `config/scenarios/kashiwa_cool4/rsus.yaml` (see [Scenario Configuration](../../../Configuration/Rsus/index.md)). Every sensor publishes to the combined topics `/v2x/object_info` and `/v2x/object_info_groundtruth`, and to their `/DSRC` and `/Cellular` copies. The ten Kakeiken sensors also have their own pair of topics: `…/object_info` (with noise) and `…/object_info_groundtruth` (without). These have no network copies.

| RSU | `rsu_id` (hex) | decimal | `sensor_id` | Sensor | Kind | Own topic |
|:--|:--|--:|--:|:--|:--|:--|
| Ganken | `0x12020001` | 302120961 | 1 | RSU_Lidar 1 | lidar 60°, 60 m | — |
| Ganken | `0x12020001` | 302120961 | 2 | RSU_Lidar 2 | lidar 60°, 60 m | — |
| Ganken | `0x12020001` | 302120961 | 3 | RSU_Lidar 3 | lidar 60°, 60 m | — |
| Ganken | `0x12020001` | 302120961 | 4 | RSU_Lidar 4 | lidar 60°, 60 m | — |
| Kakeiken_A | `0x12020002` | 302120962 | 1 | RSU_camera 6 | camera 84.8°, 75 m | `/v2x/rsu12020002/sensor1/object_info` |
| Kakeiken_A | `0x12020002` | 302120962 | 2 | RSU_camera 7 | camera 84.8°, 50 m | `/v2x/rsu12020002/sensor2/object_info` |
| Kakeiken_A | `0x12020002` | 302120962 | 3 | RSU_Lidar 7 | lidar 70.4°, 150 m | `/v2x/rsu12020002/sensor3/object_info` |
| Kakeiken_A | `0x12020002` | 302120962 | 4 | RSU_camera 9 | camera 84.8°, 50 m | `/v2x/rsu12020002/sensor4/object_info` |
| Kakeiken_A | `0x12020002` | 302120962 | 5 | RSU_Lidar 9 | lidar 70.4°, 150 m | `/v2x/rsu12020002/sensor5/object_info` |
| Kakeiken_B | `0x12020003` | 302120963 | 1 | RSU_camera 2 | camera 84.8°, 50 m | `/v2x/rsu12020003/sensor1/object_info` |
| Kakeiken_B | `0x12020003` | 302120963 | 2 | RSU_Lidar 2 | lidar 70.4°, 150 m | `/v2x/rsu12020003/sensor2/object_info` |
| Kakeiken_B | `0x12020003` | 302120963 | 3 | RSU_camera 3 | camera 84.8°, 50 m | `/v2x/rsu12020003/sensor3/object_info` |
| Kakeiken_B | `0x12020003` | 302120963 | 4 | RSU_Lidar 3 | lidar 70.4°, 150 m | `/v2x/rsu12020003/sensor4/object_info` |
| Kakeiken_B | `0x12020003` | 302120963 | 5 | RSU_camera 8 | camera 84.8°, 75 m | `/v2x/rsu12020003/sensor5/object_info` |
| Midblock | `0x12020004` | 302120964 | 1 | RSU_Lidar | lidar 90°, 120 m | — |
| Wakashiba | `0x12020005` | 302120965 | 1 | RSU_camera (4) _ 1 _ 360 | camera 360°, 15 m | — |
| Wakashiba | `0x12020005` | 302120965 | 2 | RSU_camera (4) _2 _ 360 | camera 360°, 15 m | — |
| Wakashiba | `0x12020005` | 302120965 | 3 | RSU_camera (4) _ 4_camera | camera 80°, 50 m | — |
| Wakashiba | `0x12020005` | 302120965 | 4 | RSU_camera (4) | camera 360°, 50 m | — |
| Wakashiba | `0x12020005` | 302120965 | 5 | RSU_sensor (4) _ 3 Lidar | lidar 80°, 121 m | — |

Angles are the full horizontal field of view. If you change the RSUs in the configuration, this table no longer applies; `dm_echo.py` always uses the current `rsus.yaml`.

## Signal info

### An example

The four signal groups of 東大前 (UTokyo), as `dm_echo.py` prints them:

```text
- 東大前 (UTokyo) (crp 0x00000001) vehicle [0x11 0x12 0x13] = approach 1 -> 1, approach 1 -> 2, approach 1 -> 3
    green for another 55.1 s -> red for 5.0 s -> green for 20.0 s -> red for 5.0 s
- 東大前 (UTokyo) (crp 0x00000001) vehicle [0x21 0x22 0x23] = approach 2 -> 1, approach 2 -> 2, approach 2 -> 3
    green for another 55.1 s -> yellow for 5.0 s -> red for 25.0 s
- 東大前 (UTokyo) (crp 0x00000001) vehicle [0x33 0x31 0x32] = approach 3 -> 3, approach 3 -> 1, approach 3 -> 2
    red for another 80.1 s -> yellow for 5.0 s
- 東大前 (UTokyo) (crp 0x00000001) pedestrian [0x30] = crossing approach 3
    green for another 15.1 s -> green flashing for 5.0 s -> red for 65.0 s
```

The first group in `ros2 topic echo`:

| Raw field | Raw value | Meaning |
|:--|:--|:--|
| `crp_id.value` | 1 | intersection 東大前 (UTokyo), see [Kashiwa signal IDs](../SignalInfo/index.md#kashiwa-signal-ids) |
| `signal_id_list[].value` | 17 18 19 0 0 0 0 0 | `0x11 0x12 0x13`, padded with `0` to 8 entries |
| `time.value` | 719071173525 | ms since 2004‑01‑01 00:00 JST |
| `signal_light_info_list[0]` | `main_light` 5, `min/max_time_to_change` 551 | green **now**, changes in 55.1 s |
| `signal_light_info_list[1]` | `main_light` 3, 50 | then red for 5.0 s |
| `signal_light_info_list[2]` | `main_light` 5, 200 | then green for 20.0 s |
| `count_down_stop_flag.value` | 0 | the time counts down (1 = the light never changes; the time is then 65535 = unknown) |

### Signal IDs

A signal ID is one byte: which approach (方路, 1–8) the traffic comes from and where it goes.

| Signal ID | High 4 bits | Low 4 bits | Meaning |
|:--|:--|:--|:--|
| `0x12` (18) | 1 | 2 | vehicles from approach 1 to approach 2 |
| `0x31` (49) | 3 | 1 | vehicles from approach 3 to approach 1 |
| `0x30` (48) | 3 | 0 | pedestrians crossing approach 3 |
| `0x00` | — | — | unknown / padding |

The signal IDs listed in one `SignalInfo` always show the same light. The approach numbers of each intersection, with their directions, are on the [Signal Info](../SignalInfo/index.md#kashiwa-signal-ids) page.

### Light codes and times

| `main_light` | Vehicle signal | Pedestrian signal |
|:--|:--|:--|
| 1 | dark | dark |
| 2 | red flashing | red flashing |
| 3 | red | red |
| 5 | green | green |
| 7 | **yellow** | **green flashing** (青点滅) |
| 9 | yellow flashing | yellow flashing |

- `min_time_to_change` / `max_time_to_change` are in **0.1 s**. They are equal because the signals run on a fixed cycle.
- Entry `[0]` gives the time left for the current light. The following entries give how long each next light lasts.
- `arrow_light` is a bit field: `0x40` left, `0x10` straight, `0x04` right. The Kashiwa signals have no arrows, so it is `0`.

## The other topics

| Topic | Type | Read it as |
|:--|:--|:--|
| `/v2x/object_info/DSRC`, `/v2x/object_info/Cellular` | same as `/v2x/object_info` | the same content, 100 ms or 400 ms later; DSRC drops 5 % of messages |
| `/v2x/object_info_groundtruth` | same | no position, size or existence noise; `existency` = 101 |
| `/signal_info/DSRC`, `/signal_info/Cellular` | same as `/signal_info` | delayed copies |

The full list is on the [Topic List](../Topics/index.md) page.
