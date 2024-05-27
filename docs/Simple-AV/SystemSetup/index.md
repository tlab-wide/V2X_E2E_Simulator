# Environment Setup for simple-AV on Ubuntu 22.04

This guide will walk you through the steps to set up an environment in Ubuntu 22.04 for developing and running the simple-AV ROS node, including the installation of AWSIM, ROS2 Humble, and other necessary components.

## Step 1: Update and Upgrade the System

First, ensure your system is up to date.

```bash
sudo apt update
sudo apt upgrade
```

## Step 2: Install ROS2 Humble (Debian Package)

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