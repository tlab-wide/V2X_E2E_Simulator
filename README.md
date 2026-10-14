# V2X E2E Simulator

![V2X E2E Simulator](https://github.com/hoosh-ir/V2X_E2E_Simulator/assets/32614364/2e273eb2-89f0-47f3-9e6f-81648e3cd807)

V2X E2E Simulator is an end-to-end Vehicle-to-Everything (V2X) simulation platform built on top of [AWSIM](https://github.com/tier4/AWSIM) — the leading scene simulator for [Autoware](https://github.com/autowarefoundation/autoware).

**Full documentation: [https://tlab-wide.github.io/V2X_E2E_Simulator/](https://tlab-wide.github.io/V2X_E2E_Simulator/)**

**Download** (Unity package, Kashiwa maps, Autoware vehicle files): fill in the [request form](https://docs.google.com/forms/d/e/1FAIpQLSfsFGAvp8J0iLGfcQQ8r6o2x72SBmsddg-B-ngfVQoC4tF0iw/viewform); the download link is sent to your email.

---

## Features

- V2X cooperative perception via roadside unit sensors with noise and network delay simulation
- Advanced traffic management: per-lane spawn rates, branch weights, and zebra crossing yield behaviour
- Teleportation system with ETA-based lane targeting for controlled experiments
- Pedestrian and cyclist simulation with waypoint paths and pool-based spawning
- End-to-end CSV data logging of NPC positions, ego vehicle state, and spawn selection
- CooL4 DM protocol output: object info and signal info (with remaining time), each also with DSRC / Cellular network delay
- Scenario configuration in YAML: RSUs, pseudo and raw sensors, publishers, vehicle traffic, pedestrians, cyclists and teleports
- Local web viewer (`tools/web-viewer`): live topic health, everything published on the Lanelet2 map, and a Settings tab to edit the configuration
- Full ROS2 integration compatible with Autoware
- Open source — Ubuntu 22.04 and Windows 10/11

## Quick Start

1. Clone the repository:

   ```bash
   git clone https://github.com/tlab-wide/V2X_E2E_Simulator.git
   ```

2. Open the project in **Unity 6000.0.50f1** (Unity 6).
3. The scene, the Kashiwa map and the 3D models are not in the repository. Request **`V2X_E2E_Simulator_3_1_2.unitypackage`** with the [request form](https://docs.google.com/forms/d/e/1FAIpQLSfsFGAvp8J0iLGfcQQ8r6o2x72SBmsddg-B-ngfVQoC4tF0iw/viewform) (the download link is sent to your email; file list on the [Download page](https://tlab-wide.github.io/V2X_E2E_Simulator/latest/GettingStarted/Download/)) and import it with `Assets → Import Package → Custom Package…`.
4. Open the scene `Assets/V2X/Scenes/V2X_E2E_Simulator_v3.1.1.unity` and press Play.

See the [Getting Started guide](https://tlab-wide.github.io/V2X_E2E_Simulator/GettingStarted/QuickStartDemo/) for full setup instructions (ROS 2, Autoware, maps).

## Documentation

- [Getting started](https://tlab-wide.github.io/V2X_E2E_Simulator/latest/GettingStarted/Download/): download, Unity setup, ROS 2 and Autoware
- [Scenario configuration](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/Configuration/): RSUs, sensors, traffic, pedestrians and teleports from YAML files
- [CooL4 DM protocols](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/DMProtocols/): object info and signal info, [topic list](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/DMProtocols/CooL4/Topics/), [reading the messages](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/DMProtocols/CooL4/Reading/)
- [Kashiwa scene names](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/ProjectGuide/KashiwaScenes/): intersections, approaches, traffic lights and RSUs
- [Web viewer](https://tlab-wide.github.io/V2X_E2E_Simulator/dev/Tools/WebViewer/)

---

## License

This project is a derivative of **[AWSIM](https://github.com/tier4/AWSIM)** by TIER IV, Inc.

- **AWSIM code**: [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
- **AWSIM assets**: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)
- **V2X original code** (`/V2X` directory): [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) — Copyright 2025 T-LAB (Wide Research Group)
- **V2X original assets**: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)

See [`LICENSE`](./LICENSE) for full details.

(c) 2022–2025 Tsukada LAB — T-LAB Wide Research Group
