# NetSim

This package is a network simulator. It is designed to create simulated packet loss and delay in the incoming message topics and simulate the network status.
## Download NetSim
- [NetSim](https://drive.google.com/file/d/1tKNGgSLNZQis8rrmpWv3sMu6EuH_Lxra/view?usp=drive_link)

## Dependencies

### ROS dependency

- [ROS2 humble](https://docs.ros.org/en/humble/)
- [autoware_auto_msgs](https://github.com/tier4/autoware_auto_msgs)
- [v2x_msgs](https://drive.google.com/file/d/1DFmr8T9k_tHY_9cgSxPmSqWQ43SQW-nX/view?usp=drive_link)
- [csv2](https://github.com/p-ranav/csv2)

## Build

- Install ROS2 and clone the `v2x_msgs` and `autoware_auto_msgs` into the `src` directory of a ROS2 workspace.

- Clone the repository into your ROS2 workspace's `src` folder:
```console
git clone --recurse-submodules git@github.com:hoosh-ir/net_sim.git
```
- Source your ROS2 Humble:
```console
source /opt/ros/humble/setup.bash
```
- From the root of your workspace, use `colcon` to build the package:
```console
colcon build --symlink-install
```

## Run

- From the root of your workspace, source your ROS2 workspace:
```console
. install/setup.bash
```
- Configure the network simulator by modifying the parameters in the config file `config/net_sim_conf.yaml`.
- Launch the application:
```console
ros2 launch net_sim net_simulator.launch.py
```

## Configure

### Network Simulation Configurations

The `net_sim_conf.yaml` file located at `config` provides various configurations for the network simulator. Below, you'll find a list of the options:

- `network_status_type`: This parameter determines the type of the network status and can take the following values:
    - `0` (None): This value determines there are no delays or packet losses for the incoming topics.
    - `1` (Constant): In this mode, the simulator creates constant delay and packet loss for the message topics.
    - `2` (Heatmap): In this mode, the simulator reads a CSV file associated with a given network heatmap and creates delays and packet losses for the incoming message topics based on the heatmap values.

- `const_network_delay`: This parameter determines the amount of constant delay (in milliseconds) to create for the incoming messages (Used only when the `network_status_type` is set to `1`).

- `const_network_packet_loss`: This parameter determines the amount of constant packet loss (in percent) to create for the incoming messages (Used only when the `network_status_type` is set to `1`).

- `prediction_network_status_file`: This parameter gives the absolute address of the heatmap file for the object prediction topic (Used only when the `network_status_type` is set to `2`).

- `prediction_network_status_involved_stations`: This parameter lists the IDs of the RSUs involved in the heatmap file for the object prediction topic (Used only when the `network_status_type` is set to `2`).

- `traffic_signal_network_status_file`: This parameter gives the absolute address of the heatmap file for the traffic signal topic (Used only when the `network_status_type` is set to `2`).

- `traffic_signal_network_status_involved_stations`: This parameter lists the IDs of the RSUs involved in the heatmap file for the traffic signal topic (Used only when the `network_status_type` is set to `2`).

- `freespace_network_status_file`: This parameter gives the absolute address of the heatmap file for the freespace topic (Used only when the `network_status_type` is set to `2`).

- `freespace_network_status_involved_stations`: This parameter lists the IDs of the RSUs involved in the heatmap file for the freespace topic (Used only when the `network_status_type` is set to `2`).

### Topic Configurations

Similarly, the `topic_conf.yaml` file at the `config` folder, provides the means to configure the package for the input/output topics. The parameters are listed below:

- `input_objects_topic`
- `output_objects_topic`
- `input_signals_topic`
- `output_signals_topic`
- `input_freespaces_topic`
- `output_freespaces_topic`
- `vehicle_pose_topic`

## Network Heatmap

When simulating the network status based on some given heatmap file, the provided file must abide by the format rules below:

- The heatmap file must be a COMMA-SEPARATED VALUES file
- The heatmap file must NOT include any descriptive headers
- The values must be in the order `x,y,z,rsu_1_delay,rsu_1_packet_loss,rsu_2_delay,rsu_2_packet_loss,...`

You can refer to a sample heatmap file located at the `sample` folder.
