# V2X E2E Simulator

![V2X E2E Simulator](https://github.com/hoosh-ir/V2X_E2E_Simulator/assets/32614364/2e273eb2-89f0-47f3-9e6f-81648e3cd807)

V2X E2E Simulator is an end-to-end Vehicle-to-Everything (V2X) simulation platform built on top of [AWSIM](https://github.com/tier4/AWSIM) — the leading scene simulator for [Autoware](https://github.com/autowarefoundation/autoware).

**Full documentation: [https://tlab-wide.github.io/V2X_E2E_Simulator/](https://tlab-wide.github.io/V2X_E2E_Simulator/)**

---

## Features

- V2X cooperative perception via roadside unit sensors with noise and network delay simulation
- Advanced traffic management: per-lane spawn rates, branch weights, and zebra crossing yield behaviour
- Teleportation system with ETA-based lane targeting for controlled experiments
- Pedestrian and cyclist simulation with waypoint paths and pool-based spawning
- End-to-end CSV data logging of NPC positions, ego vehicle state, and spawn selection
- Full ROS2 integration compatible with Autoware
- Open source — Ubuntu 22.04 and Windows 10/11

## Quick Start

```bash
git clone https://github.com/tlab-wide/V2X_E2E_Simulator.git
```

Open the project in **Unity 2022.3 LTS** and load the scene at `Assets/AWSIM/Scenes/Main/AutowareSimulation/`.

See the [Getting Started guide](https://tlab-wide.github.io/V2X_E2E_Simulator/GettingStarted/QuickStartDemo/) for full setup instructions.

---

## License

This project is a derivative of **[AWSIM](https://github.com/tier4/AWSIM)** by TIER IV, Inc.

- **AWSIM code**: [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
- **AWSIM assets**: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)
- **V2X original code** (`/V2X` directory): [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) — Copyright 2025 T-LAB (Wide Research Group)
- **V2X original assets**: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)

See [`LICENSE`](./LICENSE) for full details.

(c) 2022–2025 Tsukada LAB — T-LAB Wide Research Group
