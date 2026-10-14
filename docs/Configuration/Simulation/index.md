# Simulation

| | |
|:--|:--|
| File | `config/scenarios/<name>/simulation.yaml` |
| Web viewer | Settings → **Simulation** |
| Unity export | *V2X → Config → Export Simulation And Signal Timing To Config* |

```yaml
time_scale: 1                 # simulation speed, 1 = real time (> 0, at most 100)
seed: 0                       # one seed for everything random; leave empty for a different run every time
ego:
  vehicle: BusAutowareFinal   # an ego vehicle of the scene (tag Ego): BusAutowareFinal, Lexus RX450h 2015 Sample Sensor
  pose: {x: 4769.09, y: 73083.82, z: 18.38, heading_deg: 241}   # start pose; omit to keep the scene pose
lighting:
  time_of_day: '17:30'        # local time; empty = the sun of the scene
  date: '2025-06-21'
  latitude_deg: 35.9
  longitude_deg: 139.94
  utc_offset_h: 9
  # sun_elevation_deg: 40     # or place the sun directly (both values)
  # sun_azimuth_deg: 180
  sun_intensity_lux: 100000
  color_temperature_k: 6570
  night_intensity_lux: 0.5    # moonlight when the sun is below the horizon
weather:
  fog_visibility_m: 0         # e.g. 150 for thick fog; 0 = no fog
  fog_height_m: 60
loggers:
  output_dir: SimulationOutput
  npc_area:                   # NPC vehicles and pedestrians inside the log area (LogArea)
    enabled: true
    cars_csv: Scenario_car.csv
    humans_csv: Scenario_human.csv
    every_n_frames: 2         # 0 = every interval_s instead
    interval_s: 0.05
  ego:                        # state of the ego vehicle (LogAutonomous)
    enabled: false
    csv: Autonomous.csv
    spawn_validation_csv: AutonomousSpawnValidation.csv
    prefix: Auto
    interval_s: 0.005
```

- **Ego vehicle.** The chosen vehicle is switched on and the other ego vehicles are switched off. The traffic manager, the ScenarioSelector teleport keys and the *Selected Car* dropdown follow it. Its raw sensors are set up from `raw_sensors.yaml` before it starts.
- **Seed.** It seeds Unity's random numbers, the traffic manager (it replaces the `seed` of `vehicles.yaml`), line-of-sight UUIDs and the object numbers of object info. Physics and frame timing still differ a little between runs, so runs are similar but not bit-identical.
- **Sun.** `time_of_day` places the sun from the date and the position (NOAA formulas). When the sun is below the horizon, a dim light opposite it (`night_intensity_lux`) keeps the scene readable. The camera exposure of the scene is automatic.
- **Fog.** It adds an HDRP fog to the scene's global volume. Visibility is the distance where objects fade out (mean free path × 3). It affects cameras, not the pseudo sensors or the RGL LiDARs.
- **Loggers.** Files are written to `output_dir` (relative to the project folder, or the build folder) as `<name>_<yyyyMMdd_HHmmss>.csv`. See [Loggers](../../Components/Logger/index.md).
- **Time scale.** It is also passed to the ROS clock, so `/clock` and the timestamps follow it.
