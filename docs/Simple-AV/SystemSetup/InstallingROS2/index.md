
# Installing ROS2 Humble on Ubuntu 22.04 on WSL2 

First, ensure your Ubuntu system is up to date.

```bash
sudo apt update
sudo apt upgrade
```

Install [Ros2 Humble distibution](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Follow all the steps and install ros-humble-desktop. Make sure the installations is complete by using the commands bellow.

In one terminal, source the setup file and then run a C++ talker:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

In another terminal source the setup file and then run a Python listener:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
You should see the talker saying that it’s Publishing messages and the listener saying I heard those messages. This verifies both the C++ and Python APIs are working properly. Hooray!