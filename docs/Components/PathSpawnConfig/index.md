# Path Spawn Config (JSON)

## Overview

Pedestrian and cyclist paths can be defined in a JSON file instead of being placed by hand in the scene. At start-up the file is read, one runtime [`WaypointSystem`](../WaypointPed/index.md#waypointsystem) is built per path, and pooled agents with a [`WaypointFollower`](../WaypointPed/index.md#waypointfollower) are spawned onto each path at the configured rate.

```text
pedestrian_spawn.json ──► WaypointSystem per path ──► pooled WaypointFollower agents
 (ROS/MGRS coordinates)    (Unity coordinates,          (spawned at spawnPerMinute,
                            optional light constraints)  recycled at path end)
```

| Component | Status | Role |
|---|---|---|
| `PedestrianCyclistSpawner` | **Supported** — active in the `NormalTraffics` scenario of the Kashiwa scene (GameObject `ScenarioSelector/Cool4 Kakeiken Overal Evaluation/PedestrianCyclistSpawner`) | Loads the JSON, builds the paths and spawns both pedestrians and cyclists |
| `PathConfigLoader` + `PedestrianPathSpawner` / `CyclistPathSpawner` | Alternative — present in the scene but inactive | Loader builds the paths; two separate spawners use them |
| `ClearWaypointFollower` | Alternative follower | Rigidbody follower that can read a path from `PathConfigLoader` |
| `PathMetadata` | Helper | Stores `type` and `spawnPerMinute` on each generated path GameObject |

All scripts are in `Assets/V2X/Script/Paths/` (namespace `V2X.Paths`).

---

## The JSON File

Default path: `Assets/Configs/pedestrian_spawn.json`

### Schema

```json
{
  "paths": [
    {
      "id": "<unique string>",
      "type": "pedestrian | cyclist | both",
      "spawnPerMinute": 0.0,
      "waypoints": [
        { "x": 0.0, "y": 0.0, "z": 0.0, "checkLight": false, "trafficLightId": "" }
      ]
    }
  ]
}
```

### Example (excerpt of `Assets/Configs/pedestrian_spawn.json`)

```json
{
    "paths": [
        {
            "id": "Path2",
            "type": "both",
            "spawnPerMinute": 0.066,
            "waypoints": [
                ...
                {
                    "x": 4487.9013671875,
                    "y": 72948.5546875,
                    "z": 23.0,
                    "checkLight": false,
                    "trafficLightId": ""
                },
                {
                    "x": 4490.7509765625,
                    "y": 72946.921875,
                    "z": 23.0,
                    "checkLight": true,
                    "trafficLightId": "TrafficLights_A_pedestrain (1)-cross2"
                },
                ...
            ]
        }
    ]
}
```

### Field Reference

#### Path

| Field | Type | Description |
|---|---|---|
| `id` | string | **Required and unique.** Paths with an empty `id` are skipped. If two paths share an `id`, both entries spawn on the waypoints of the last one. |
| `type` | string | `pedestrian`, `cyclist` or `both` (case-insensitive). Empty or unknown values are treated as `both`. Selects which prefabs may be spawned on the path. |
| `spawnPerMinute` | float | Mean spawn rate for this path. `0` (or negative) disables the path. |
| `waypoints` | array | Ordered points. Paths with no waypoints are skipped. |

#### Waypoint

| Field | Type | Description |
|---|---|---|
| `x`, `y`, `z` | float | Position in metres in the ROS / MGRS map frame (the same frame as the Lanelet2 / point-cloud map). |
| `checkLight` | bool | If true (and `trafficLightId` is set), a `ConstraintPedestrianTrafficLight` is added at this waypoint. |
| `trafficLightId` | string | GameObject name of the pedestrian `TrafficLight` to check (matched case-insensitively). The agent waits **at this waypoint** until the light is solid green. |

!!! note "Traffic light names must be unique"
    The light is looked up by name among all `TrafficLight` components in the scene (including inactive ones) and the **first** match is used. If several lights share the name, the agent may watch the wrong one. For example, the current Kashiwa scene contains three `TrafficLight` objects named `TrafficLights_B_pedestrain (2)`, which is referenced by the bundled configs. Rename lights so that every referenced name is unique. If the light is not found, a warning is logged and the agent does not stop there.

### File Location

A relative path is resolved against the project root (`Application.dataPath/..`):

| Context | `Assets/Configs/pedestrian_spawn.json` resolves to |
|---|---|
| Unity Editor | `<project>/Assets/Configs/pedestrian_spawn.json` |
| Player build | `<build folder>/Assets/Configs/pedestrian_spawn.json` (the folder that contains the executable and its `_Data` folder) |

An absolute path is used as-is.

!!! warning "Player builds"
    The JSON is read from disk and is not packed into the build. Copy `Assets/Configs/` into the build folder, next to the executable, so that `<build folder>/Assets/Configs/pedestrian_spawn.json` exists.

### Presets

| File | Total rate |
|---|---|
| `pedestrian_spawn.json` (default; same content as the Low preset) | ≈ 47 agents per hour |
| `pedestrian_spawn-KashiwaRealPedestrains_Low_total47.json` | ≈ 47 agents per hour (sum of `spawnPerMinute` = 0.779) |
| `pedestrian_spawn-KashiwaRealPedestrains_High_total76.json` | ≈ 76 agents per hour (sum of `spawnPerMinute` = 1.273) |

All presets define the same 14 Kashiwa paths (`Path1`–`Path14`, type `both`) and differ only in `spawnPerMinute`. To use a preset, set `pathsJsonPath` on `PedestrianCyclistSpawner` to its path (or copy it over `pedestrian_spawn.json`).

---

## Coordinate Conversion

Each JSON point is converted to a Unity world position as follows:

1. **Subtract the MGRS offset** (only if `useMgrsOffset` is on): the offset is `Environment.Instance.MgrsOffsetPosition`. If there is no `Environment` in the scene, `manualOffset` is used instead (a ROS/MGRS-frame offset; ignored when it is zero).
2. **ROS → Unity axes**: `(x, y, z)` → `(-y, z, x)`.
3. **Add `worldOrigin`**: if a `worldOrigin` Transform is assigned, its position is added (a Unity-frame translation).
4. **Ground snapping** (if `snapWaypointsToGround` is on): a ray is cast downward from `groundRayHeight` metres above the point, over a distance of `2 × groundRayHeight`, against `groundMask`. On a hit, the height is replaced by the hit point. On a miss, the converted height is kept.

Saving applies the inverse conversion (Unity → ROS: `(x, y, z)` → `(z, -x, y)`, then add the offset).

---

## PedestrianCyclistSpawner (supported)

### Inspector Fields

| Field | Default | Description |
|---|---|---|
| `pedestrianPrefabs` | — | Pedestrian prefabs (must have a `WaypointFollower`) |
| `cyclistPrefabs` | — | Cyclist prefabs (must have a `WaypointFollower`) |
| `activeParent` | — | Optional parent for spawned agents |
| `maxExtraInstancesPerSpawn` | 5 | When no compatible inactive clone exists, a new clone is created. `0` forbids creating clones (and since the pool starts empty, nothing spawns) |
| `maxActive` | 0 | Max concurrently active agents across all paths of this spawner. `0` = unlimited. The Kashiwa scene uses **5** |
| `initialSpawnDelayRange` | (0, 2) | Random delay before a path's first spawn (seconds). The Kashiwa scene uses (0, 0) |
| `worldOrigin` | — | Optional Unity-frame anchor added after conversion |
| `useMgrsOffset` | true | Subtract the MGRS offset of `Environment` |
| `manualOffset` | (0, 0, 0) | ROS/MGRS-frame offset used when `useMgrsOffset` is on but no `Environment` exists |
| `snapWaypointsToGround` | true | Snap waypoints to the ground with a downward raycast |
| `groundMask` | Everything | Layers hit by the ground ray |
| `groundRayHeight` | 3 | Ray start height above the waypoint (m) |
| `npcPaths` | — | Paths loaded from the JSON (filled at runtime; editable in the Scene view) |
| `pathsJsonPath` | `Assets/Configs/pedestrian_spawn.json` | JSON file to load / save |

The JSON is loaded in `Start`. The context menu provides **Load NPC Paths From JSON** (reload and restart all spawn loops) and **Save NPC Paths To JSON** (overwrite `pathsJsonPath` with `npcPaths`).

### Spawn Timing

For each enabled path, a separate spawn loop runs:

- **Mean interval** = `60 / spawnPerMinute` seconds (minimum 0.1 s).
- **First spawn**: after a random delay from `initialSpawnDelayRange`. If that delay is 0, a random phase in `[0, interval)` is used instead, so paths do not spawn in sync.
- **Following spawns**: each wait is drawn from a Gaussian around the mean interval (standard deviation 10 %), clamped to 0.5×–1.5× the mean.
- If `maxActive` is reached, the loop waits one mean interval and checks again.

### Pooling and Recycling

- Agents are created as clones of a random prefab from the lists allowed by the path `type` (`pedestrian` → `pedestrianPrefabs`, `cyclist` → `cyclistPrefabs`, `both` → both lists), and kept in a pool.
- An inactive clone is reused only on a **compatible** path: a cyclist clone is never reused on a `pedestrian` path, and a pedestrian clone is never reused on a `cyclist` path. Any clone can be reused on a `both` path.
- On spawn, the agent is placed at the first waypoint (`ForceStartAtFirstWaypoint`), receives a new [`ScenarioHumanId`](../WaypointPed/index.md#scenariohumanid), and is activated.
- Generated paths do not loop. At the last waypoint, `WaypointFollower` deactivates the agent, which returns it to the pool.

The generated path objects (`Path_<id>` with `wp` children, under `RuntimePaths_PedCyclist`) are hidden from the Hierarchy.

### Editing Paths in the Scene View

`PedestrianCyclistSpawnerEditor` (`Assets/V2X/Editor/`) adds a **Path Editing** section to the Inspector once `npcPaths` contains paths:

| Control | Description |
|---|---|
| `Selected Path` | Path to show in the Scene view |
| `Handle Size` | Size of the waypoint handles |
| `Show Labels` | Show `<id>:<index>` labels |

The selected path is drawn as lines between its waypoints (green = `pedestrian`, cyan = `cyclist`, yellow = `both`) with a sphere handle at each waypoint. Dragging a handle converts its position back to ROS/MGRS and updates `npcPaths` (with Undo). Use **Save NPC Paths To JSON** to write the change to the file.

!!! note
    `npcPaths` is serialized with the scene, so handles are available in Edit mode too, but `Start` always replaces `npcPaths` with the JSON content. Save to JSON for an edit to take effect. During Play, moving handles does not move the running waypoint systems; they are rebuilt only when the paths are loaded again.

---

## Alternative: PathConfigLoader + Pedestrian/CyclistPathSpawner

In this setup, `PathConfigLoader` loads the JSON in `Awake` and builds the waypoint systems. `PedestrianPathSpawner` and `CyclistPathSpawner` read them in `Start` (finding a loader in the scene if none is assigned). In the Kashiwa scene these objects exist under the `NormalTraffics` scenario but are **inactive**, and the loader's `jsonConfigPath` is set to `Assets/Configs/paths.json`.

| Spawner | Paths used | Prefabs |
|---|---|---|
| `PedestrianPathSpawner` | `pedestrian` and `both` | `pedestrianPrefabs`; on `both` paths also `cyclistPrefabsForBoth` (cyclist clones are reused only on `both` paths) |
| `CyclistPathSpawner` | `cyclist` only | `cyclistPrefabs` |

Both spawners have `activeParent`, `maxExtraInstancesPerSpawn` (5 / 3), `maxActive` (0) and `initialSpawnDelayRange` (0, 2), with the same timing rules as above (`maxActive` is counted per spawner).

### PathConfigLoader Fields

| Field | Default | Description |
|---|---|---|
| `jsonConfigPath` | `Assets/Configs/pedestrian_spawn.json` | JSON file (relative to project root, or absolute) |
| `runtimeParent` | — | Parent for generated paths. If empty, a `RuntimePaths` GameObject is created |
| `instantiateEditableInScene` | false | If true, generated `Path_<id>` objects are normal scene objects that can be moved; otherwise they are hidden (`HideAndDontSave`) |
| `worldOrigin` | — | Optional Unity-frame anchor added after conversion |
| `useMgrsOffset` | true | Subtract the MGRS offset of `Environment` |
| `manualOffset` | (0, 0, 0) | ROS/MGRS-frame offset used when no `Environment` exists |
| `snapWaypointsToGround` | true | Snap waypoints to the ground |
| `groundMask` | Everything | Layers hit by the ground ray |
| `groundRayHeight` | 3 | Ray start height above the waypoint (m) |

The Inspector has **Load Paths Now** and **Save Paths Now** buttons (also available as the context menu items **Load Paths JSON** / **Save Paths JSON**).

**Save** writes the current positions of the `wp` children of each `Path_<id>` object back to the JSON. It also picks up `WaypointSystem` objects that were duplicated in the scene: the id is the object name without the `Path_` prefix. `type` and `spawnPerMinute` are read from the object's `PathMetadata` component. `checkLight` / `trafficLightId` are kept only where a `ConstraintPedestrianTrafficLight` with a resolved light exists on the waypoint.

!!! warning
    If `runtimeParent` is not set, **Save** collects every `WaypointSystem` in the scene, including hand-made ones. Set `runtimeParent` before saving.

### PathMetadata

`PathMetadata` (`type`, `spawnPerMinute`) is added to every generated `Path_<id>` object by both `PathConfigLoader` and `PedestrianCyclistSpawner`. With `instantiateEditableInScene` enabled, edit it in the Inspector to change a path's type or rate before saving.

### ClearWaypointFollower

`ClearWaypointFollower` is an alternative rigidbody follower (acceleration/deceleration, look-ahead steering, optional per-waypoint waits and `NoLoop` / `Loop` / `PingPong` modes). With `pathSource = PathLoader` and a `pathId`, it reads the path from `PathConfigLoader` (found in the scene if not assigned). It uses the loader's **converted** waypoint positions (the children of the generated `WaypointSystem`), so the offset and ground snapping match the loader, and takes only `checkLight` / `trafficLightId` from the JSON definition. It is not used by the spawners above.

---

## Changing the JSON at Runtime

The **Traffic settings** panel of the UI (`TrafficSettingsUI`) has a pedestrian path input field. Its **Apply** button calls `SetJsonConfigPath(...)` and `LoadPaths()` on the first `PathConfigLoader` it finds, including inactive ones.

!!! warning
    This field targets `PathConfigLoader` only. It does **not** affect `PedestrianCyclistSpawner`, which is the spawner used in the Kashiwa scene. Already running `PedestrianPathSpawner` / `CyclistPathSpawner` instances also keep the paths they read in `Start`. To change the pedestrian config of the supported spawner, set `pathsJsonPath` in the Inspector (or replace the file) before Play.

---

## Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| No pedestrians or cyclists spawn | The JSON was not found (`Paths JSON not found at ...` warning). In a player build, copy `Assets/Configs/` next to the executable. Also check that the `NormalTraffics` scenario is selected |
| A specific path never spawns | `spawnPerMinute` is `0` or negative, `id` is empty, or the path has no waypoints |
| Fewer agents than expected | `maxActive` is reached (the Kashiwa scene uses 5), or the prefab list for the path `type` is empty, or `maxExtraInstancesPerSpawn` is `0` |
| Agents appear in the wrong place or far away | Offset settings: keep `useMgrsOffset` on when the JSON uses MGRS map coordinates and the scene has an `Environment`. Without an `Environment`, set `manualOffset`. Make sure `worldOrigin` is not set unintentionally |
| Agents float or sink | Ground snapping missed: check `groundMask` and increase `groundRayHeight` if the ground is more than `groundRayHeight` away from the converted height |
| `Traffic light '<id>' not found` warning | `trafficLightId` does not match any `TrafficLight` GameObject name. The agent will not stop at that waypoint |
| Agent waits at the wrong light or never crosses | The light name is not unique and another light with the same name was matched. Rename the lights so the referenced name is unique |
