# Web Viewer

The web viewer is a local web page that answers two questions while the simulator runs:

1. **Which topics are live?** Every topic the simulator publishes is listed with its status, rate and last-message age.
2. **What do they say?** Everything except raw sensor data is drawn on the Kashiwa Lanelet2 map: DM object info, DM signal info with remaining time, V2X and Autoware traffic signals, Autoware objects and the ego vehicle.

It lives in `tools/web-viewer/` and runs on the simulator machine only.

![Web viewer: topic health, the bus route with signal states, and the inspector](overview.png)

## Start

```console
cd V2X_E2E_Simulator
tools/web-viewer/run.sh
```

Then open **http://localhost:8080**. Start the simulator before or after; the viewer picks up topics as they appear.

`run.sh` does the setup:

- sources ROS 2 Humble and the [v2x_e2e_msgs](https://github.com/tlab-wide/v2x_e2e_msgs) workspace, which it expects at `../v2x_e2e_msgs/install` next to the simulator folder. Override this with `MSGS_SETUP=/path/to/install/setup.bash`.
- uses the simulator's middleware settings: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `ROS_LOCALHOST_ONLY=1`.
- on the first run, creates `tools/web-viewer/.venv` with `aiohttp`, leaving the system Python untouched.

Options: `--port 9000` changes the port. The server binds to `127.0.0.1`, so the page cannot be opened from another computer.

## How it works

```text
 Simulator ──ROS 2──► server/main.py  (one Python process, rclpy + aiohttp)
                       ├─ monitor.py   discovers topics every second, subscribes to each non-raw topic,
                       │               measures rate / last-message age, keeps the latest message
                       ├─ decoders.py  turns the latest messages into map features (≤ 10 Hz),
                       │               only for the topics ticked in the page
                       └─ WebSocket /ws ──► browser: topic list (1 Hz) + map layers (≤ 10 Hz)
```

No Foxglove, rosbridge or other ROS packages are needed. The server subscribes with *best effort* QoS, which matches both reliable and best-effort publishers. Raw sensor types (`PointCloud2`, `Image`, `CameraInfo`, `LaserScan`, `Imu`, `NavSatFix`, …) are listed but never subscribed to.

## Topic panel

Topics are grouped into DM signal info, DM object info, traffic signals, Autoware objects, ego vehicle, control, clock and other.

| Status | Meaning |
|:--|:--|
| 🟢 live | Messages are arriving; the rate is measured over the last 3 s. |
| 🟠 stale | Messages arrived before but have stopped (the age is shown). |
| 🔵 no data yet | A publisher exists but nothing has been received. |
| ⚫ offline | No publisher any more (e.g. the simulator was stopped). |
| 🔴 type missing | The message type is not in the sourced workspace, so the topic cannot be decoded. |

Tick a topic to draw it on the map. Click its name to see its type, publishers, rate, message count, bandwidth and a summary of the last message. Delayed copies (`…/DSRC`, `…/Cellular`) and the per-intersection `…/signal_info` topics start unticked, because they duplicate other topics. Your choices are remembered in the browser.

## Map layers

| Data | Topic types | How it is placed |
|:--|:--|:--|
| Lanelet2 map | `maps/lanelet2.geojson` | Lanes, lane borders, crosswalks, stop lines and traffic-light positions |
| DM object info | `dm_object_info_msgs/ObjectInfoArray` | Latitude/longitude and heading from the message, drawn as boxes of the reported size |
| DM signal info | `dm_signal_info_msgs/SignalInfoArray` | One lamp per approach (方路) at its intersection, from `config/cool4_intersections.json`. Zoom in (≥ 17) to see remaining seconds and pedestrian signals. |
| Traffic signals | `v2x_msgs/V2XSignals`, Autoware `TrafficSignalArray` / `TrafficLightGroupArray` | At the Lanelet2 traffic light with that regulatory-element or way ID |
| Autoware objects | `PredictedObjects`, `DetectedObjects`, `TrackedObjects` | Map frame (`map`, `obj`) converted to lat/lon. Ego-relative frames such as `bus` are placed with the ego pose. |
| Ego vehicle | `/awsim/ground_truth/vehicle/pose` and other `PoseStamped` / `Odometry` topics in the map frame | Cyan box |

Each topic has its own colour (the swatch in the list). Layers of topics that are not live are dimmed. Click any object or signal to open the inspector. It shows the decoded fields: for example the object ID split into RSU / sensor / object number, the CooL4 light colour and the full remaining-time schedule.

![Intersection 科警研 zoomed in: signal lamps with remaining seconds, DM and Autoware objects](intersection.png)

**Background map (online)** overlays GSI tiles when the machine has internet. Everything else, including the Lanelet2 map, works offline.

## Coordinates

| Source | Conversion |
|:--|:--|
| DM messages | Already WGS84 (1e-7 deg) |
| Autoware / AWSIM `map` frame | Local coordinates of MGRS square 54SVE: `UTM(EPSG:32654) = map_xy + (400000, 3900000)`. `build_map.py` derives this offset from the Lanelet2 `local_x`/`local_y` tags (residual 0 m). |
| Ego-relative frames | Rotated and translated with the latest ego pose |

## Using another map or intersections

- **Map:** `python3 tools/web-viewer/scripts/build_map.py /path/to/lanelet2_map.osm` regenerates `maps/lanelet2.geojson` and `maps/lanelet2_meta.json`.
- **CooL4 signals:** edit `config/cool4_intersections.json`. Each intersection has its CRP ID, its centre (`[lon, lat]`) and the bearing of each approach in degrees clockwise from north. The Kashiwa values match the [Signal Info](../../DMProtocols/CooL4/SignalInfo/index.md) mapping.

## Troubleshooting

| Symptom | Check |
|:--|:--|
| "No topics yet" | Is the simulator running? Are `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` the same in both terminals? |
| A topic shows *type missing* | The type is not in the sourced `v2x_e2e_msgs` build. Rebuild the workspace, or point `MSGS_SETUP` to one that has it. |
| Autoware objects in a relative frame are not drawn | The ego pose topic `/awsim/ground_truth/vehicle/pose` must be live; the topic summary says so. |
| Port already in use | `run.sh --port 9000` |
