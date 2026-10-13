# TrafficSpawnConfig

## Overview

`TrafficSpawnConfig` is a static utility that loads traffic configuration from a JSON file and makes it available to the traffic simulation system at runtime. It controls two things:

1. **Per-lane spawn rates** — the maximum number of vehicles per minute each lane may spawn (enforced by `TrafficManager`).
2. **Branch weights** — the probability of NPC vehicles turning onto each downstream lane.

The state is **static and global**: it is shared by all `TrafficManager`s, and the last one to initialize (or load a config) wins.

It also records every lane-change decision made during a simulation run and exports them as a CSV on session end, allowing you to verify that configured probabilities match observed behaviour.

---

## The JSON Configuration File

Default path: `Assets/Configs/traffic_spawn.json`

This path is configurable on the `TrafficManager` component (`spawnConfigPath` field) and at runtime from the Traffic Settings UI. A relative path is resolved against the project root (`Application.dataPath/..`); an absolute path is used as-is. The file is loaded automatically at scene start if it exists; otherwise a warning is logged and the Inspector values of the random traffic simulators are used.

!!! warning "Player builds"
    In a player build, `Application.dataPath/..` is the build folder, so the default file must be placed at `<build folder>/Assets/Configs/traffic_spawn.json` (or an absolute path must be set).

### Full Example

```json
{
  "spawnRates": [
    { "lane": "Lane_MainRoad_A",  "spawnsPerMinute": 8.0 },
    { "lane": "Lane_MainRoad_B",  "spawnsPerMinute": 6.0 },
    { "lane": "Lane_SideStreet",  "spawnsPerMinute": 2.0 }
  ],
  "branchWeights": [
    {
      "fromLane": "Lane_MainRoad_A",
      "next": [
        { "lane": "Lane_MainRoad_B",  "weight": 3.0 },
        { "lane": "Lane_SideStreet",  "weight": 1.0 }
      ]
    },
    {
      "fromLane": "Lane_MainRoad_B",
      "next": [
        { "lane": "Lane_MainRoad_A",  "weight": 1.0 }
      ]
    }
  ]
}
```

### Field Reference

#### `spawnRates`

| Field | Type | Description |
|---|---|---|
| `lane` | string | Lane GameObject name (compared case-insensitively). |
| `spawnsPerMinute` | float | Maximum spawn frequency: enforces a minimum interval of `60 / spawnsPerMinute` s between spawns on this lane. It is not a guaranteed rate. `0` disables rate limiting for this lane. |

Only spawnable lanes listed in the file get a rate limit while the file is loaded; unlisted lanes are not rate-limited.

#### `branchWeights`

| Field | Type | Description |
|---|---|---|
| `fromLane` | string | The lane where vehicles are making a turn decision (case-insensitive). |
| `next[].lane` | string | Name of a downstream lane option. |
| `next[].weight` | float | Relative probability weight. Values are normalised automatically; entries with weight ≤ 0 are ignored. |

Branch weights from the file **replace** the Inspector `branchWeights` of every random traffic simulator.

!!! note "Normalisation"
    Weights do not need to sum to 1. The system normalises them internally. A weight of `2.0` for lane B and `1.0` for lane C means B is chosen 67% of the time.

---

## Saving and Loading from TrafficManager

The `TrafficManager` Inspector provides two context menu actions:

| Action | Effect |
|---|---|
| **Save Spawn Config JSON** | Reads current spawn rates and branch weights from all `RandomTrafficSimulator` entries and writes them to the JSON file. |
| **Load Spawn Config JSON** | Reads the JSON file and applies spawn rates and branch weights back to all simulators. |

You can also call these at runtime:

```csharp
trafficManager.SetSpawnConfigPath("Assets/Configs/my_config.json");  // loads immediately
trafficManager.SaveSpawnConfigJson();
trafficManager.LoadSpawnConfigJson();
```

!!! note
    Every load (`TrafficSpawnConfig.Load`, including `SetSpawnConfigPath`, **Load Spawn Config JSON** and `Restart`) clears the branch decision statistics collected so far.

---

## Branch Decision Statistics

Every time `ChooseNextLane` picks a next lane, `TrafficSpawnConfig` records it internally, aggregated per `(from_lane, to_lane, selection_mode)`. In `TrafficManager.OnDestroy()` (after the NPC vehicle simulator is disposed), these statistics are written to a CSV file. Export errors are logged as warnings.

Default output path: `Assets/Configs/traffic_branch_decisions.csv` (relative to the project root). Ratios and averages are written with 6 decimals using the invariant culture (`.` as decimal separator), independent of the system locale.

### CSV Column Reference

| Column | Description |
|---|---|
| `from_lane` | Origin lane name |
| `to_lane` | Chosen next lane |
| `selection_mode` | How the lane was chosen (see below) |
| `decision_count` | How many times this transition occurred |
| `total_from_lane` | Total decisions made from this lane (all destinations) |
| `total_from_lane_mode` | Total decisions from this lane using this selection mode |
| `observed_ratio_from_lane` | `decision_count / total_from_lane` |
| `observed_ratio_from_lane_mode` | `decision_count / total_from_lane_mode` |
| `avg_configured_weight` | Average configured weight of the chosen lane, after normalisation over **all** `next` entries of `fromLane` in the config. `0` for uniform modes. |
| `avg_effective_probability` | Average probability the chosen lane actually had at decision time: its weight divided by the sum of weights of the configured lanes present in the current `NextLanes` (`weighted`), or `1 / NextLanes.Count` (uniform modes). |

### Selection Modes

| Mode | Meaning |
|---|---|
| `weighted` | Weights for `fromLane` matched at least one current next lane and were used |
| `uniform_not_loaded` | No config is loaded — uniform random |
| `uniform_no_config` | Config loaded, but it has no weights for this `fromLane` |
| `uniform_no_matching_weights` | Weights exist for `fromLane`, but none of their lanes is in the current `NextLanes` |
| `uniform_weighted_fallback` | Weighted draw did not select a lane (floating-point rounding edge case) — uniform random |

### Using the CSV

For `weighted` rows, compare `avg_effective_probability` against `observed_ratio_from_lane_mode` to verify that the configured weights produce the expected routing ratios. Over a long run, these should converge. `avg_configured_weight` equals `avg_effective_probability` only when all configured next lanes are available.

Example — expected vs. observed for the config `{ B: 3.0, C: 1.0 }` (illustrative values):

| to_lane | avg_configured_weight | avg_effective_probability | observed_ratio_from_lane_mode |
|---|---|---|---|
| Lane_B | 0.75 | 0.75 | 0.742 |
| Lane_C | 0.25 | 0.25 | 0.258 |

---

## Programmatic API

| Method | Description |
|---|---|
| `TryGetSpawnRate(laneName, out rate)` | Query the stored spawn rate for a lane. Not used by spawning (`TrafficManager` keeps its own per-lane lookup). |
| `ChooseNextLane(fromLane, options)` | Weighted random selection from a list of next lanes (records the decision) |
| `ExportBranchDecisionCsv(path = null)` | Write the decision statistics CSV immediately; returns the full path |
| `Load(ConfigData)` / `Load(jsonPath)` | Load from a pre-parsed config object or a JSON file (clears previous state and statistics) |
| `ReadJson(path)` | Parse a JSON file and return a `ConfigData` object (`null` if missing or invalid) |
| `VerboseBranchLogging` | Static flag, `false` by default. When `true`, every weighted decision logs two lines to the Console. |

### Example: Manual Lane Selection

```csharp
TrafficLane next = TrafficSpawnConfig.ChooseNextLane(currentLane, currentLane.NextLanes);
```

If no weights are configured for `currentLane`, the method falls back to uniform random selection transparently.

---

## Tips

!!! tip "Iterate Without Reopening Unity"
    Edit `traffic_spawn.json` in any text editor while the project is closed, then use **Load Spawn Config JSON** from the context menu when you reopen it — or set the path via script and call `LoadSpawnConfigJson()` at runtime for live reloading.

!!! tip "Start from the Current Setup"
    Use **Save Spawn Config JSON** to capture the current Inspector configuration as a JSON baseline, then modify the file from there.

!!! warning "Lane Names Must Be Unique"
    `TrafficManager` validates that all `TrafficLane` objects have unique names. Duplicate names will cause incorrect rate/weight lookup. The validation runs at startup and logs errors to the Console.
