# Zebra Crossing System

## Overview

The Zebra Crossing system makes NPC vehicles automatically yield to pedestrians and cyclists at crosswalks. It consists of two components:

- **`ZebraCrossingArea`** — attached to an individual crosswalk trigger. Detects occupants and exposes a static query API for NPC vehicles.
- **`ZebraCrossingConfig`** — optional parent component. Defines a shared whitelist of occupant types for all child `ZebraCrossingArea` objects.

---

## How It Works

### Full Interaction Pipeline

```mermaid
sequenceDiagram
    participant P as Pedestrian / Cyclist
    participant ZA as ZebraCrossingArea (Trigger)
    participant CS as NPCVehicleCognitionStep
    participant ST as NPCVehicleInternalState
    participant DS as NPCVehicleDecisionStep

    P->>ZA: OnTriggerEnter
    ZA->>ZA: Add root to occupantRoots (if whitelisted)

    Note over CS: Every FixedUpdate — after ground/obstacle jobs complete
    CS->>ZA: TryGetBlockedStop(frontCenterPos, forward)
    ZA-->>CS: blocked=true, distance=12.3m, area="CrosswalkNorth"
    CS->>ST: IsZebraBlocked = true
    CS->>ST: DistanceToZebraStop = 12.3
    CS->>ST: ZebraAreaName = "CrosswalkNorth"
    CS->>ST: WasZebraBlocked = true

    Note over DS: Same FixedUpdate — after Cognition
    DS->>ST: Read IsZebraBlocked, DistanceToZebraStop
    DS->>DS: distanceToStop = Min(frontVehicle, trafficLight, rightOfWay, zebraStop)
    DS->>ST: SpeedMode = STOP (or SUDDEN_STOP / ABSOLUTE_STOP)

    P->>ZA: OnTriggerExit (or occupant destroyed / deactivated)
    ZA->>ZA: Remove root from occupantRoots
    CS->>ZA: TryGetBlockedStop → blocked=false
    CS->>ST: IsZebraBlocked = false
    CS->>ST: WasZebraBlocked = false
    DS->>ST: SpeedMode follows the remaining stop conditions
```

The zebra occupancy is evaluated in the **cognition** step; the **decision** step only consumes the stored distance.

### Where the Code Lives

| Code location | What it does |
|---|---|
| `ZebraCrossingArea.OnTriggerEnter/Exit` | Maintains the set of current occupants |
| `ZebraCrossingArea.IsOccupied` | Drops occupants that were destroyed or deactivated inside the trigger (they never receive `OnTriggerExit`) |
| `NPCVehicleCognitionStep.UpdateZebraCrossingAwareness()` | Queries every active area for every NPC vehicle each FixedUpdate |
| `NPCVehicleInternalState` fields | Stores the result between Cognition and Decision steps |
| `NPCVehicleDecisionStep.UpdateSpeedMode()` | Includes `DistanceToZebraStop` in the minimum stop distance calculation |

### NPC Vehicle State Fields

These fields are written by the Cognition step and read by the Decision step each frame:

| Field | Type | Description |
|---|---|---|
| `IsZebraBlocked` | `bool` | True when a blocked crossing is detected ahead |
| `DistanceToZebraStop` | `float` | Distance to the stop point (metres), `float.MaxValue` if not blocked |
| `ZebraAreaName` | `string` | Name of the matched `ZebraCrossingArea`, `null` if not blocked |
| `WasZebraBlocked` | `bool` | Previous frame's value — used to log the first frame of a stop |

All fields are reset at the start of every cognition update and re-filled only if a blocked crossing is found.

### How the Stop Distance Is Used

In `NPCVehicleDecisionStep`, the zebra stop distance competes with all other stop conditions:

```csharp
var distanceToStopPointByZebra = state.IsZebraBlocked
    ? state.DistanceToZebraStop
    : float.MaxValue;

var distanceToStopPoint = Mathf.Min(
    distanceToStopPointByFrontVehicle,
    distanceToStopPointByTrafficLight,
    distanceToStopPointByRightOfWay,
    distanceToStopPointByZebra        // ← zebra crossing
);
```

The resulting `distanceToStopPoint` then determines the speed mode:

| Condition | Speed Mode |
|---|---|
| `distance ≤ absoluteStopDistance` | `ABSOLUTE_STOP` |
| `distance ≤ suddenStopDistance` | `SUDDEN_STOP` |
| `distance ≤ stopDistance` | `STOP` |
| `distance ≤ slowDownDistance` or vehicle is turning | `SLOW` |
| otherwise | `NORMAL` |

Where the distances depend on current speed and the deceleration values in `NPCVehicleConfig`:

$$
d_\text{stoppable} = \frac{v^2}{2 \cdot a}
$$

plus a fixed margin (`MinStopDistance` = 1.5 m: ×1 for absolute, ×2 for sudden, ×3 for normal stop; `slowDownDistance` = `stopDistance` + 4 × 1.5 m).

So a faster NPC vehicle needs more distance to stop and may enter `SUDDEN_STOP` or `ABSOLUTE_STOP` if the crossing is detected late.

---

## ZebraCrossingArea

### Setup

1. Create an empty GameObject positioned over the crosswalk.
2. Add a **Collider** component and enable **Is Trigger**.
3. Scale the collider to cover the full crossing surface.
4. Attach the **ZebraCrossingArea** script.
5. Assign a **Stop Point** transform — place this just before the stop line where vehicles should halt.

!!! warning "Rigidbody required"
    Unity only sends trigger events if the occupant or the area has a **Rigidbody** (kinematic is fine). Without one, occupants are never detected.

!!! note "One stop point per approach direction"
    Each area has a single stop point, and detection is purely geometric (distance and angle to the stop point, no lane matching). An area therefore serves one approach direction; for a two-way road, add one area with its own stop point per direction.

### Occupant Detection

The area automatically identifies the following types:

| Type | Detection method |
|---|---|
| `NPCPedestrian` | `GetComponentInParent<NPCPedestrian>()` |
| `SimpleCyclistMovement` | Full type name match on a `MonoBehaviour` in the parent hierarchy |
| `BicycleMoveRotate` | Full type name match on a `MonoBehaviour` in the parent hierarchy |
| `V2X.Paths.ClearWaypointFollower` | Full type name match on a `MonoBehaviour` in the parent hierarchy |
| `WaypointFollower` (global namespace) | Full type name match on a `MonoBehaviour` in the parent hierarchy |

The occupant's **root transform** (`transform.root`) is stored, not the collider's transform, so compound colliders on a single character count as one occupant. Occupants parented under a common scene object share that root and therefore count as a single occupant.

### Whitelist Filtering

If a `ZebraCrossingConfig` parent is present with non-empty lists, only occupants matching the whitelist are counted:

- **`allowedPrefabs`** — exact match of the normalized names (`(Clone)`, `(clone)`, `_clone`, `_Clone` removed, then trimmed).
- **`allowedRootNameSubstrings`** — case-insensitive substring of the normalized root name.

If both lists are empty, all detected types are allowed.

### Inspector Fields

| Field | Default | Description |
|---|---|---|
| `stopPoint` | This object | Transform where vehicles should stop. Defaults to the area's own position if unset. |
| `maxDetectionDistance` | 40 m | How far ahead a vehicle will look for this crossing |
| `forwardAngleTolerance` | 80° | Angular cone (half-angle) in front of vehicle for detection |
| `stopBuffer` | 1.5 m | Extra gap kept between the vehicle and the stop point |
| `enableDebugLogs` | false | Log occupancy changes, ignored colliders and stop decisions to the Console |

### Static Query API

It is called once per `FixedUpdate` by the cognition step for every NPC vehicle not marked for despawn:

```csharp
bool blocked = ZebraCrossingArea.TryGetBlockedStop(
    vehiclePosition,   // vehicle's front-center world position
    vehicleForward,    // vehicle's forward direction (Y is ignored)
    out float distance,           // distance to the stop point (minus buffer)
    out ZebraCrossingArea area    // the matched area, or null
);
```

The method checks all active, occupied `ZebraCrossingArea` instances and returns the closest one that:

1. Has its stop point within `maxDetectionDistance` (horizontal distance)
2. Has its stop point within the `forwardAngleTolerance` cone in front of the vehicle

`distance` is the horizontal distance to the stop point minus `stopBuffer`, clamped to ≥ 0. If `blocked == true`, the cognition step stores it in `DistanceToZebraStop`; the decision step then includes it in the minimum stop distance, which yields `STOP`, `SUDDEN_STOP` or `ABSOLUTE_STOP` (or `SLOW` while still far away) depending on speed.

### Gizmo Visualization

When the GameObject is selected in the Scene view:

- **Green wireframe** — crossing is clear (no occupants)
- **Red wireframe** — crossing is occupied
- **Cyan sphere** — stop point position

Enable `enableDebugLogs` to see occupant enter/exit events in the Console, plus an NPC-level message when a vehicle starts stopping:

```
[NPC Zebra] Vehicle 'Bus_01' stopping for zebra 'CrosswalkNorth' at ~12.3m.
```

This message fires on the first frame an NPC detects the blocking state (`WasZebraBlocked` transition `false → true`). The code also contains a "cleared zebra" message, but it never fires in the current implementation (`ZebraAreaName` is reset to `null` before the check), so nothing is logged when the crossing clears.

!!! note
    With `enableDebugLogs` on, `TryGetBlockedStop` also logs a `[ZebraCrossingArea] Blocked ...` line for every vehicle matching the area, every frame — use it only for short debugging sessions.

---

## ZebraCrossingConfig

### Purpose

Place this component on a **parent GameObject** that groups multiple `ZebraCrossingArea` children. All child areas inherit its whitelist settings.

```
ZebraCrossingConfig (parent)
├── ZebraCrossingArea (north crosswalk)
├── ZebraCrossingArea (south crosswalk)
└── ZebraCrossingArea (east crosswalk)
```

### Fields

| Field | Description |
|---|---|
| `allowedPrefabs` | Only occupants whose root name matches one of these prefab names are counted |
| `allowedRootNameSubstrings` | Only occupants whose root name contains one of these strings (case-insensitive) are counted |

### Examples

```
allowedRootNameSubstrings: ["Human", "Cyclist", "Pedestrian"]
```

This ensures that vehicles that enter the trigger zone accidentally (e.g., due to overlapping colliders) are not counted as occupants.

---

## Complete Setup Example

```
Intersection (GameObject)
└── ZebraCrossingConfig
    ├── allowedRootNameSubstrings: ["Human", "Bicycle"]
    │
    ├── CrosswalkNorth (GameObject)
    │   ├── BoxCollider [Is Trigger = true]
    │   ├── ZebraCrossingArea
    │   │   └── stopPoint → StopLineNorth (Transform)
    │   └── StopLineNorth (child Transform)
    │
    └── CrosswalkEast (GameObject)
        ├── BoxCollider [Is Trigger = true]
        ├── ZebraCrossingArea
        │   └── stopPoint → StopLineEast (Transform)
        └── StopLineEast (child Transform)
```

!!! warning "Collider Must Be a Trigger"
    The `ZebraCrossingArea` script calls `Reset()` automatically to set `isTrigger = true` when first added. If you replace the collider later, make sure to re-enable the trigger flag.

!!! tip "Placement"
    Place the collider so it covers the full painted area of the crosswalk, not just the curb edge. Pedestrians need to be inside the zone for the system to work correctly.
