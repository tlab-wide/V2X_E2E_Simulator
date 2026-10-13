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

## Layout

```text
run.sh                        start script (sources ROS + msgs, creates the venv)
server/main.py                aiohttp server: page, /api/config, WebSocket /ws
server/monitor.py             ROS 2 node: discovers topics, measures rate, keeps the latest message
server/decoders.py            ROS messages -> GeoJSON features (DM, V2X, Autoware, poses)
server/geo.py                 map frame <-> WGS84, oriented boxes
config/cool4_intersections.json   where CooL4 signal IDs are drawn (CRP ID, approach bearings)
maps/lanelet2.geojson         Kashiwa Lanelet2 map, pre-converted
maps/lanelet2_meta.json       map-frame offset and traffic-light positions
scripts/build_map.py          converts a Lanelet2 .osm into the two map files
web/                          the page (MapLibre GL vendored in web/vendor, works offline)
```

## Using another map

```console
python3 tools/web-viewer/scripts/build_map.py /path/to/lanelet2_map.osm
```

The script needs `pyproj`. It overwrites `maps/lanelet2.geojson` and `maps/lanelet2_meta.json`.

See the documentation page *Tools → Web Viewer* for details.
