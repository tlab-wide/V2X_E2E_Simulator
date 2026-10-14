# Signal Info (信号情報)

The simulator publishes the state of its traffic signals in the CooL4 signal-information format (`dm_signal_info_msgs`, spec §3.6). For every configured intersection it publishes:

- which logical signals (信号灯器) are being described: the **CRP ID** and **signal ID list**
- the colour they show now: **main light** and **green arrows**
- **how long that colour remains**, plus the upcoming colours and their durations

All five signalised intersections on the Kashiwa-no-ha bus route are published **continuously**, wherever the ego vehicle is. DM receives the same stream a roadside DM messenger would send, so the old `signal_info_converter_node` (`/v2x/traffic_signals` → `/signal_info`) is no longer needed.

## How it works

```text
 TrafficIntersection (one per intersection, in the Kashiwa environment prefab)
 │  lightingSequences: [ {intervalSec, group → bulbs}, ... ]  (looped)
 │  remembers: current sequence index, the time it started, current bulbs of every group
 │
 │  TryGetGroupTimeline(group)  ──►  [ (bulbs now, time left in this step),
 │                                     (bulbs in next step, step length), ... one full cycle ]
 ▼
 DMSignalInfoPublisher  (every 1/Hz s)
 │  for each intersection → for each configured signal group:
 │     1. bulbs → MainLightIndication + ArrowLightIndication
 │     2. merge consecutive steps with the same light → remaining time of each light
 │     3. build SignalInfo { crp_id, signal_id_list, time, …, signal_light_info_list }
 ▼
 /signal_info                       SignalInfoArray, all intersections   (consumed by DM)
 /signal_info/DSRC, /…/Cellular     delayed / lossy copies (network model)
```

The light state comes straight from the lighting sequence that drives the Unity traffic lights. The colour the DM consumer receives is therefore always the colour shown in the scene and obeyed by the NPC vehicles.

!!! info "Why not read the TrafficLight objects?"
    A `TrafficLight` only knows its current bulbs. The **remaining time** (残秒数) exists only in the intersection's lighting sequence. Pedestrian phases (groups 10–13) exist only in the sequence too: at these intersections they have no visible lights. Reading the sequence gives both.

## Message format

```text
dm_signal_info_msgs/SignalInfoArray
└─ SignalInfo[] array
   ├─ dm_object_info_msgs/CrpId        crp_id                 交差点 ID (uint32)
   ├─ SignalId[]                       signal_id_list         信号灯器 ID list (uint8 each, 1–8)
   ├─ dm_object_info_msgs/TimestampIts time                   情報生成時刻 (ms)
   ├─ SignalState                      state                  信号状態情報
   ├─ SpecificControlFlags             specific_control_flags 特定制御動作中フラグ
   ├─ EventCount                       event_count            イベントカウンタ
   ├─ CountDownStopFlag                count_down_stop_flag   カウントダウン停止フラグ (OFF=0, ON=1)
   └─ SignalLightInfo[]                signal_light_info_list 灯色出力情報 (1–12), [0] = now
      ├─ MainLightIndication   main_light          主灯色表示
      ├─ ArrowLightIndication  arrow_light         青矢信号表示 (bit field)
      ├─ MinTimeToChange       min_time_to_change  最小残秒数 (0.1 s)
      └─ MaxTimeToChange       max_time_to_change  最大残秒数 (0.1 s)
```

### Field mapping

| Field | Value published by the simulator |
|:--|:--|
| `crp_id.value` | CRP ID of the intersection (see [Kashiwa signal IDs](#kashiwa-signal-ids)) |
| `signal_id_list[]` | Signal IDs of one signal group. Padded with `0` (`SignalId.UNKNOWN`) to **8 elements**, like the DM messenger (ROS2DMI) output. Turn **Pad Signal Id List** off to send only the real IDs. More than 8 IDs are split into several `SignalInfo` (spec §3.6.2). |
| `time.value` | Generation time, `DE_TimestampIts` (see [timestamp note](../index.md#timestamp-de_timestampits)) |
| `state.value` | `0`. Under consideration in the spec (SIP-adus). |
| `specific_control_flags.value` | `0`. Under consideration in the spec. |
| `event_count.value` | `0`. The fixed-time signals never change in a way that time alone cannot predict. |
| `count_down_stop_flag.value` | `OFF (0)` while the remaining time counts down. `ON (1)` when the light never changes in the cycle. The remaining time is then unknown and does not count down. |
| `signal_light_info_list[0]` | Current light. `min/max_time_to_change` = time until it changes. |
| `signal_light_info_list[1..]` | Following lights in order. `min/max_time_to_change` = how long each is shown. |

Because the simulated signals are fixed-time, the remaining times are exact, so `min_time_to_change == max_time_to_change` (spec: "残秒数が確定している場合には，最小残秒数と最大残秒数に同じ値を格納する").

### Light colour (`main_light`)

| Bulb shown in Unity | Vehicle signal | Pedestrian signal |
|:--|:--|:--|
| no bulb lit | `1` DARK | `1` DARK |
| red, solid | `3` RED_LIGHT | `3` RED_LIGHT |
| red, flashing | `2` FLASHING_RED | `2` FLASHING_RED |
| green, solid | `5` PERMISSIVE_GREEN | `5` PERMISSIVE_GREEN |
| green, flashing | `5` PERMISSIVE_GREEN | `7` (青点滅, flashing green) |
| yellow, solid | `7` PERMISSIVE_YELLOW | `7` (青点滅, flashing green) |
| yellow, flashing | `9` FLASHING_YELLOW | `9` FLASHING_YELLOW |

Value `7` means *yellow* for vehicle signals and *flashing green* for pedestrian signals (spec §5, `DE_MainLightIndication`). Values 4, 6 and 8 are never used.

### Green arrows (`arrow_light`)

Bit field of `DE_青矢信号表示方向`: bit 6 left, bit 4 straight, bit 2 right. A lit `LEFT_ARROW_BULB`, `UP_ARROW_BULB` or `RIGHT_ARROW_BULB` sets the matching bit. The Kashiwa signals have no arrow bulbs, so `arrow_light` is always `0` there.

## Remaining time (残秒数)

`TrafficIntersection` records which lighting step is active and when it started. For a signal group the publisher:

1. takes the group's bulbs for the current step and **the next steps of one full cycle**. A step that does not mention the group keeps its previous bulbs.
2. converts each step to a `main_light` / `arrow_light` pair, and **merges consecutive steps with the same light**.
3. publishes the first merged entry with the **time left**: the rest of the current step plus the following steps with the same light. The next entries carry their **full durations**.
4. drops the last entry if it shows the current light again, because the look-ahead window cuts it short. It caps the list at 12 entries (spec maximum).
5. if the light never changes during the cycle, publishes a single entry with `min/max_time_to_change = 65535` (UNKNOWN) and `count_down_stop_flag = ON`.

**Worked example.** がん研 (`Ganken`), 方路1 (group 0), 10 s after the cycle starts, with the environment prefab's sequence (the Kashiwa scene overrides it, so live values differ): G 50 s → Y 3 s → R (7+3+3+30+3+3 s) → G …

| `signal_light_info_list` | `main_light` | `min/max_time_to_change` | Meaning |
|:--|:--|:--|:--|
| `[0]` | 5 (green) | 400 | green for another 40.0 s |
| `[1]` | 7 (yellow) | 30 | then yellow for 3.0 s |
| `[2]` | 3 (red) | 490 | then red for 49.0 s |

The remaining time is measured in simulation time (`Time.time`). It slows down and speeds up with the simulation time scale, exactly like the lights themselves.

## Signal ID layout

A signal ID identifies a **logical** signal: the go/stop indication for one inbound lane to one outbound lane, not a physical signal head (spec §3.6.3). It is built from the **approach IDs** (方路 ID, 1–8) of the intersection:

```text
 vehicle signal     [ inbound approach (4 bit) | outbound approach (4 bit) ]   e.g. 0x12 = from 方路1 to 方路2
 pedestrian signal  [ crossed approach (4 bit) | 0000                     ]   e.g. 0x30 = crossing 方路3
 0x00 = unknown, 0xFF = always allowed
```

Signal IDs that always show the same state are sent together in one `signal_id_list`. All movements from one approach therefore share one `SignalInfo`, for example `[0x11, 0x12, 0x13]` for everything entering from 方路1.

## Kashiwa signal IDs

The IDs below follow *東大_柏の葉実験環境について_信号情報について抜粋*, the signal ID sheet of the Kashiwa-no-ha experiment environment. Approach directions are given north-up. Entries marked **sheet** match the DM messenger output in the sheet, including the order inside the list. Entries marked **spec** are not in the sheet; they are built with the §3.6.3 rule so that every approach and crossing is covered.

Each row also names the Unity `TrafficIntersection` (in `Enviroment copy/intersectionsFixed/`) and the lighting-sequence group the state comes from. Pedestrian groups follow the vehicle phase they run with: a crosswalk is green together with the traffic that runs parallel to it.

### 東大前 (UTokyo) — CRP `0x00000001` (1) — `Todaimae`

| Signal group | Approach | Group | Signal IDs (hex) | Signal IDs (dec) | Source |
|:--|:--|:--|:--|:--|:--|
| 方路1, east (駅→東大 turns right) | vehicle | 0 | 0x11 0x12 0x13 | 17 18 19 | sheet |
| 方路2, west | vehicle | 2 | 0x21 0x22 0x23 | 33 34 35 | spec |
| 方路3, north (東大→駅 turns left) | vehicle | 1 | 0x33 0x31 0x32 | 51 49 50 | sheet |
| crossing 方路3 | pedestrian | 10 | 0x30 | 48 | spec |
| crossing 方路1, 2 | pedestrian | 11 | 0x10 0x20 | 16 32 | spec |

### がん研 (Ganken) — CRP `0x000c1001` (790529) — `Ganken` / RSU1

| Signal group | Approach | Group | Signal IDs (hex) | Signal IDs (dec) | Source |
|:--|:--|:--|:--|:--|:--|
| crossing 方路1, 3 | pedestrian | 11 | 0x10 0x30 | 16 48 | sheet |
| 方路1, east | vehicle | 0 | 0x11 0x12 0x13 | 17 18 19 | sheet |
| crossing 方路2 | pedestrian | 10 | 0x20 | 32 | sheet |
| 方路2, south (駅→東大 turns left) | vehicle | 1 | 0x22 0x23 0x21 | 34 35 33 | sheet |
| 方路3, west (東大→駅 turns right) | vehicle | 2 | 0x33 0x31 0x32 | 51 49 50 | sheet |

### 科警研 (Kakeiken) — CRP `0x000c3001` (798721) — `Kakeiken` / RSU2

| Signal group | Approach | Group | Signal IDs (hex) | Signal IDs (dec) | Source |
|:--|:--|:--|:--|:--|:--|
| crossing 方路1 | pedestrian | 10 | 0x10 | 16 | sheet |
| 方路1, east (駅→東大 turns right) | vehicle | 1 | 0x11 0x12 0x13 | 17 18 19 | sheet |
| crossing 方路2, 3 | pedestrian | 11 | 0x20 0x30 | 32 48 | sheet |
| 方路2, south | vehicle | 0 | 0x22 0x23 0x21 | 34 35 33 | sheet |
| 方路3, north (東大→駅 turns left) | vehicle | 2 | 0x33 0x31 0x32 | 51 49 50 | sheet |

### 若柴 (Wakashiba) — CRP `0x00000002` (2) — `Wakashiba` / RSU4

| Signal group | Approach | Group | Signal IDs (hex) | Signal IDs (dec) | Source |
|:--|:--|:--|:--|:--|:--|
| 方路1, north | vehicle | 0 | 0x11 0x12 0x13 0x14 | 17 18 19 20 | sheet |
| 方路2, east | vehicle | 1 | 0x22 0x23 0x24 0x21 | 34 35 36 33 | sheet |
| 方路3, south (駅→東大 turns left) | vehicle | 0 | 0x33 0x34 0x31 0x32 | 51 52 49 50 | sheet |
| 方路4, west (東大→駅 turns right) | vehicle | 1 | 0x44 0x41 0x42 0x43 | 68 65 66 67 | sheet |
| crossing 方路2, 4 | pedestrian | 10 | 0x20 0x40 | 32 64 | spec |
| crossing 方路1, 3 | pedestrian | 11 | 0x10 0x30 | 16 48 | spec |

!!! note "若柴 in the field"
    The sheet notes that the IDs a bus should look at here (方路3 and 方路4) were not output during the L2 bus runs, and that the L4 bus currently refers to the opposite side. The simulator always publishes all four approaches. To reproduce the field behaviour, delete the 方路3 / 方路4 entries in the Inspector.

### 駅前 (Station) — CRP `0x000c2001` (794625) — `Ekimae` / RSU5

| Signal group | Approach | Group | Signal IDs (hex) | Signal IDs (dec) | Source |
|:--|:--|:--|:--|:--|:--|
| crossing 方路1 | pedestrian | 10 | 0x10 | 16 | sheet |
| 方路1, east (駅→東大 turns right) | vehicle | 1 | 0x11 0x12 0x13 | 17 18 19 | sheet |
| crossing 方路2, 3 | pedestrian | 11 | 0x20 0x30 | 32 48 | sheet |
| 方路2, south | vehicle | 0 | 0x22 0x23 0x21 | 34 35 33 | sheet |
| 方路3, north (東大→駅 turns left) | vehicle | 0 | 0x33 0x31 0x32 | 51 49 50 | sheet |

In the Kashiwa scene the south lights (`Ekimae_V_0x21-0x23_1`, `_2`) of `Ekimae` are moved into group 0 (together with the north lights) and group 2 is never lit, so 方路2 reads group 0.

### How the intersections were matched

The Unity intersections were matched to the sheet in two steps (before v3.1.1 their GameObjects were called `crossroad1` … `crossroad 14`; they now carry the names below):

1. Each intersection's traffic-light Lanelet2 IDs (`TrafficLightLaneletID`) were placed on the Kashiwa Lanelet2 map. From south-east to north-west: 駅前 `Ekimae` (35.8945, 139.9498) → 若柴 `Wakashiba` (35.8979, 139.9495) → 科警研 `Kakeiken` (35.8960, 139.9418) → がん研 `Ganken` (35.9007, 139.9390) → 東大前 `Todaimae` (35.9001, 139.9376).
2. The legs were numbered with the bus turns given in the sheet. For example, at 駅前 the 駅→東大 bus turns right from 方路1 towards 若柴 in the north, so 方路1 is the east leg.

The resulting leg layout matches every diagram in the sheet. It also matches the pedestrian groupings the sheet gives for がん研, 科警研 and 駅前.

## Using it in a scene

The publisher is a single component, `DMSignalInfoPublisher` (`Assets/V2X/Script/CustomMsg/DMSignalInfoPublisher.cs`). It is shipped ready-configured as a prefab:

1. Drag **`Assets/V2X/Prefabs/DM/DM Signal Info Publisher.prefab`** into the scene, for example under `RSUs/Publishers`.
2. Save the scene and press Play. `/signal_info` appears right away.

It does not depend on an RSU GameObject being active. The intersections are found by GameObject name when play starts, so it works with the RSU objects disabled.

### Inspector

| Field | Default | Description |
|:--|:--|:--|
| **Hz** | 10 | Publishing rate (1–50 Hz) |
| **Topic** | `/signal_info` | All intersections in one `SignalInfoArray`. This is the topic DM subscribes to. |
| **Pad Signal Id List** | on | Pad `signal_id_list` with `0` to 8 elements (ROS2DMI style) |
| **Intersections** | Kashiwa route | One entry per intersection (see below) |
| **Enable Delayed Messages** | on | Also publish `/signal_info/DSRC`, `/signal_info/Cellular`, … copies ([network model](../index.md#delayed-copies-network-model)) |
| **Message Delays Configs** | empty | Extra delay profiles for this component only |
| **QoS Settings** | reliable, volatile, keep-last 1 | QoS of all topics of this component |

Each **Intersection** entry has:

| Field | Description |
|:--|:--|
| **Label** | Free text, e.g. `がん研 (Ganken)` |
| **Traffic Intersection** | Optional direct reference. When empty, the intersection is looked up by **Traffic Intersection Name**. |
| **Traffic Intersection Name** | GameObject name of the `TrafficIntersection`, e.g. `Ganken` |
| **Crp Id** | CRP ID as an unsigned integer (`0x000c1001` = 790529) |
| **Signals** | Signal groups. Each has a **Label**, **Is Pedestrian**, the lighting-sequence **Traffic Light Group** (0–3 vehicle, 10–13 pedestrian) and the **Signal Ids**. |

!!! tip "Restoring the defaults"
    The Kashiwa configuration is the component's default value. Right-click the component and choose **Reset** to restore it after editing.

### Adding another intersection

1. Find the intersection's `TrafficIntersection` and note its GameObject name and the group numbers in its **Traffic Light Groups** / **Lighting Sequences**.
2. Get the CRP ID and approach IDs from the traffic administrator, or assign them with the §3.6.3 rule.
3. Add an **Intersections** entry with one signal group per approach and per crosswalk group.

## Example output

The values below are illustrative; the timestamp is shortened. The array holds every signal group of the five intersections; shown here is がん研, 方路1, 10 s into the cycle:

```console
$ ros2 topic echo /signal_info --once
array:
- ...                       # 東大前 (CRP 1) groups come first
- crp_id:
    value: 790529
  signal_id_list:
  - value: 17
  - value: 18
  - value: 19
  - value: 0
  - value: 0
  - value: 0
  - value: 0
  - value: 0
  time:
    value: 719000000000
  state:
    value: 0
  specific_control_flags:
    value: 0
  event_count:
    value: 0
  count_down_stop_flag:
    value: 0
  signal_light_info_list:
  - main_light:
      value: 5
    arrow_light:
      value: 0
    min_time_to_change:
      value: 400
    max_time_to_change:
      value: 400
  - main_light:
      value: 7
    arrow_light:
      value: 0
    min_time_to_change:
      value: 30
    max_time_to_change:
      value: 30
  - main_light:
      value: 3
    arrow_light:
      value: 0
    min_time_to_change:
      value: 490
    max_time_to_change:
      value: 490
- crp_id:
    value: 790529
  ...
```

## Known limitations

- **Fixed-time signals.** `min_time_to_change` always equals `max_time_to_change`. `event_count`, `state` and `specific_control_flags` are always `0`.
- **Groups that never change.** A group that the lighting sequence sets only once per cycle (for example a pedestrian group) never changes; it is published with an UNKNOWN remaining time and `count_down_stop_flag = ON`.
- **Scene overrides.** The Kashiwa scene overrides the lighting sequences and light groups of がん研, 科警研, 若柴 and 駅前 on top of the environment prefab, and 科警研's GameObject carries a second, disabled `TrafficIntersection`. The publisher uses the enabled component, and the approach-to-group table above matches the scene. If you change the light groups in the scene, update the publisher's signal groups too; a group that is never lit is logged once as a warning and not published.
- **Lighting sequence of 東大前.** In `Todaimae` (not overridden by the scene) one step sets group 0 twice (`g0=R g0=G g2=R`). Group 1 (東大前 方路3) therefore goes red → yellow without ever turning green. The DM output faithfully reports what the scene shows.
- **Timestamp epoch.** The epoch is JST-based, consistent with object info (see [timestamp note](../index.md#timestamp-de_timestampits)).

## Relation to the other signal topics

| Topic | Type | Content |
|:--|:--|:--|
| `/signal_info` | `dm_signal_info_msgs/SignalInfoArray` | **This page.** CooL4 format, all intersections, with remaining time. |
| `/v2x/traffic_signals`, `/v2x/rsuN/traffic_signals` (not in the v3.1.2 scene) | `v2x_msgs/V2XSignals` | Autoware `TrafficSignalArray` wrapped with the RSU station ID. Lanelet2 regulatory-element IDs, no remaining time. Published by `CustomV2I` + `CustomV2IPublisher` on active RSU objects. |
| `/v2x/traffic_signals_Awsim` | `autoware_perception_msgs/TrafficLightGroupArray` | Original AWSIM `V2I` output (lights within 150 m of the ego). Disabled in the Kashiwa scene. |
