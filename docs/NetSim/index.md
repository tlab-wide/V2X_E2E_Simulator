# NetSim

This package is a network simulator. It is designed to create simulated packet loss and delay in the incoming message topics and simulate the network status.
## Download NetSim
- [NetSim](https://drive.google.com/file/d/1EciTv2D9Imiq3SrnpVihSfwbYY8Jbg9I/view?usp=drive_link)

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

Similarly, the `topic_conf.yaml` file in the `config` folder, provides the means to configure the package for the input/output topics. The parameters are listed below:

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

You can refer to a sample heatmap file located in the `sample` folder.

## Custom Messages

You can assign NetSim custom messages of your own. For this purpose, you need to adjust a few things in the NetSim codebase and rebuild it.

In the following sections, we'll explain the requirements and modifications to custom messages.

### Custom Message Format Requirements

Let's assume you have a message type other than those supported by NetSim. Let's name this message `CustomMessage`. To simulate the network passing such messages, you need to create a cooperative message type containing your `CustomMessage`. We call that `CooperativeCustomMessage`. The fields inside `CooperativeCustomMessage` must include:

```
uint64 station_id
uint64 sensor_type
geometry_msgs/PoseStamped station_pose
custom_package/CustomMessage custom_message
```

`station_id` is the ID of the RSU (or OBU) sending the message, `sensor_type` is the type of the sensor used to obtain the messages (not yet defined). `station_pose` is the whole pose of the transmitting RSU (or OBU) and `custom_message` is your desired message.

!!! note
    The `CustomMessage` type can be any of the ROS built-in message types or from any `custom_package` created by third parties.

### Modification to NetSim Codebase

To have NetSim transmit the external message types, you need to make NetSim identify, subscribe to and publish them. We've provided the means for you just to make slight alterations.

In the source code, under `src`, you need to modify the `net_simulator.cpp` file. There are three sections identifiable by the phrase `Custom message section`. Under each section, there are commented code snippets. You need to uncomment and modify them according to the name of your package and message type.

- The first section includes the message types. Modify according to your package and message type names
- The second section defines the variables needed to simulate the message. In this section, be wary of the name you pass to the network reporter node ("custom_message" by default). This name must repeat in `config/net_sim_conf.yaml`. Also, make sure you pass the proper input and output topic names to the sender ("/output_custom_message" by default) and the receiver ("/input_custom_message" by default) nodes.
- The last section passes the created ROS nodes to the multithreaded executor.

### Modification to NetSim Config Files

After uncommenting and modifying commented code snippets, you need to modify `config/net_sim_conf.yaml`. Again, we've provided two commented sections (search "Custom message section" in the file) for you to uncomment and modify according to your needs.

### Modification to CMakeLists.txt

In the CMakeLists.txt, refer to `Custom message section`s:
- In the first section, add your `CustomMessage` and `CooperativeCustomMessage` packages as needed.
- In the second section, link your custom packages to NetSim's libraries.

### Modification to package.xml

In `package.xml` refer to the `Custom message section`, uncomment its section and modify it according to the names of your `CustomMessage` and `CooperativeCustomMessage` packages. 

### Build NetSim

After the above steps, rebuild NetSim to reflect the changes made. Make sure to place your `CustomMessage` and `CooperativeCustomMessage` packages in the same workspace as `net_sim` to build together. Follow the build instructions to build the workspace.
