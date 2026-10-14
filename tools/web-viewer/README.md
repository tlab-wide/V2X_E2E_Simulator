# V2X Simulator Web Viewer

A local web page that shows which simulator topics are live and draws everything the simulator publishes on the Kashiwa Lanelet2 map. Raw sensor data is excluded. The map includes DM object info, DM signal info with remaining time, V2X and Autoware traffic signals, Autoware objects and the ego vehicle.

```console
tools/web-viewer/run.sh            # then open http://localhost:8080
tools/web-viewer/run.sh --port 9000
```

- Runs on the simulator machine only (bound to `127.0.0.1`).
- The first start creates `tools/web-viewer/.venv` with `aiohttp`. Nothing is installed system-wide.
- Needs ROS 2 Humble and the built [v2x_e2e_msgs](https://github.com/tlab-wide/v2x_e2e_msgs) workspace. By default `run.sh` looks for it at `../v2x_e2e_msgs/install` next to the simulator folder; set `MSGS_SETUP=/path/to/install/setup.bash` otherwise.
- Uses the same middleware settings as the simulator: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_LOCALHOST_ONLY=1`.

## Settings

The **Settings** tab (top bar) edits the simulator configuration in `config/` of the repository: RSUs and pseudo sensors,
object-info / Autoware / traffic-signal publishers, vehicle spawn points and branch ratios, pedestrian and cyclist paths,
teleport gates, raw sensors, and the shared sensor types, noise and network profiles.

- Items are edited in the list on the left and on the map (drag sensors, path points and gates; click lanes).
- **Save** checks all files together (same rules as the simulator) and writes them; the simulator reads them at the next Play.
- **Reset to default** restores the files of the current tab from `config/factory/`. **New…** copies the scenario; **Make default** writes `config/default_scenario.txt`.
- Vehicle lanes on the map are the simulator's own traffic lanes (`config/maps/traffic_lanes.json`, written by *V2X > Config > Export Traffic Lanes For The Web Viewer* in Unity). Their numbers are the `lanelet` values of `vehicles.yaml` and `teleports.yaml`.
- The Live view draws the RSU sensors of the default scenario. `config/sensors.yaml` of the viewer is used only when the repository has no `config/`.

## Layout

```text
run.sh                        start script (sources ROS + msgs, creates the venv)
server/main.py                aiohttp server: page, /api/config, WebSocket /ws
server/settings.py            /api/settings: read, check and write the simulator config (Settings tab)
server/monitor.py             ROS 2 node: discovers topics, measures rate, keeps the latest message
server/decoders.py            ROS messages -> GeoJSON features (DM, V2X, Autoware, poses)
server/geo.py                 map frame <-> WGS84, oriented boxes
config/traffic_lights.yaml    signal heads: positions, CooL4 CRP/signal IDs, Lanelet2 IDs (hand-editable)
config/sensors.yaml           RSU pseudo sensors, fallback when the repository has no config/
scripts/build_signal_config.py  fills signal-head positions in traffic_lights.yaml from a Lanelet2 map
maps/lanelet2.geojson         Kashiwa Lanelet2 map, pre-converted
maps/lanelet2_meta.json       map-frame offset and traffic-light positions
scripts/build_map.py          converts a Lanelet2 .osm into the two map files
web/                          the page (MapLibre GL vendored in web/vendor, works offline)
```

## Using another map

```console
python3 tools/web-viewer/scripts/build_map.py /path/to/lanelet2_map.osm
```

The Kashiwa `lanelet2_map.osm` is in `simulator_maps_v3.1.1.zip` ([request form](https://docs.google.com/forms/d/e/1FAIpQLSfsFGAvp8J0iLGfcQQ8r6o2x72SBmsddg-B-ngfVQoC4tF0iw/viewform)). The script needs `pyproj`. It overwrites `maps/lanelet2.geojson` and `maps/lanelet2_meta.json`.

See the documentation page *Tools → Web Viewer* for details.
