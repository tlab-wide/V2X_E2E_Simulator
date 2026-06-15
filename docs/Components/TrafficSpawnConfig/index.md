# TrafficSpawnConfig

## Overview

`TrafficSpawnConfig` is a static utility that loads traffic configuration from a JSON file and makes it available to the traffic simulation system at runtime. It controls two things:

1. **Per-lane spawn rates** — how many vehicles per minute each lane should spawn.
2. **Branch weights** — the probability of NPC vehicles turning onto each downstream lane.

It also records every lane-change decision made during a simulation run and exports them as a CSV on session end, allowing you to verify that configured probabilities match observed behaviour.

---

## The JSON Configuration File

Default path: `Assets/Configs/traffic_spawn.json`

This path is configurable on the `TrafficManager` component (`spawnConfigPath` field). The file is loaded automatically at scene start if it exists.

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
| `lane` | string | Lane name (case-insensitive). Must match the lane's GameObject name exactly. |
| `spawnsPerMinute` | float | Target spawn frequency. `0` disables rate limiting for this lane. |

#### `branchWeights`

| Field | Type | Description |
|---|---|---|
| `fromLane` | string | The lane where vehicles are making a turn decision (case-insensitive). |
| `next[].lane` | string | Name of a downstream lane option. |
| `next[].weight` | float | Relative probability weight. Values are normalised automatically. |

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

---

## Branch Decision Statistics

Every time a vehicle makes a lane-change decision, `TrafficSpawnConfig` records it internally. On `TrafficManager.OnDestroy()`, these statistics are written to a CSV file.

Default output path: `Assets/Configs/traffic_branch_decisions.csv`

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
| `avg_configured_weight` | Average weight configured for this transition |
| `avg_effective_probability` | Average probability at decision time |

### Selection Modes

| Mode | Meaning |
|---|---|
| `weighted` | Branch weight was found and used |
| `uniform_no_config` | Config was not loaded — fallback to uniform random |
| `uniform_no_matching_weights` | Config loaded but no weights for this lane |
| `uniform_weighted_fallback` | Weighted selection failed — fell back to uniform |

### Using the CSV

Compare `avg_configured_weight` against `observed_ratio_from_lane_mode` to verify that the configured weights are producing the expected routing ratios. Over a long run, these should converge.

Example — expected vs. observed for the config `{ B: 3.0, C: 1.0 }`:

| to_lane | configured_weight | expected_ratio | observed_ratio |
|---|---|---|---|
| Lane_B | 3.0 | 0.75 | 0.742 |
| Lane_C | 1.0 | 0.25 | 0.258 |

---

## Programmatic API

| Method | Description |
|---|---|
| `TryGetSpawnRate(laneName, out rate)` | Query configured spawn rate for a lane |
| `ChooseNextLane(fromLane, options)` | Weighted random selection from a list of next lanes |
| `ExportBranchDecisionCsv(path)` | Write the decision statistics CSV immediately |
| `Load(ConfigData)` | Load from a pre-parsed config object |
| `ReadJson(path)` | Parse a JSON file and return a `ConfigData` object |

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
