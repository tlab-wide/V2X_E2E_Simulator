# Communication with Awsim

If you have not installed ROS2 humble please refer to [Installing ROS2](../SystemSetup/InstallingROS2/index.md) page.

For communicating with Awsim in order to fill control/command topics and subscribe to other topics that Awsim publishes itsell we need to create a Node and write proper code. To do this we first create a workspace and package. You can also look at [ROS2 documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#create-a-workspace) for better undertanding of the process.

first install colcon build tool:

```bash
sudo apt install python3-colcon-common-extensions
```

### Creating a workspace

First, create a directory `simple_av` to contain our workspace:

```bash
mkdir -p ~/simple_av/src
cd ~/simple_av
```

At this point the workspace contains a single empty directory `src`:

```
.
└── src

1 directory, 0 files
```

In the root of the workspace, run `colcon build`. the option `--symlink-install` allows the installed files to be changed by changing the files in the source space (e.g. Python files or other non-compiled resources) for faster iteration.

```bash
colcon build --symlink-install
```

After the build is finished, we should see the build, install, and log directories:

```
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```

### Creating a package

First, source your ROS 2 installation.

```bash
source /opt/ros/humble/setup.bash
```

Make sure you are in the `src` folder before running the package creation command.

```bash
cd ~/ros2_ws/src
```

The command syntax for creating a new package in ROS 2 is:


```bash
ros2 pkg create <your_package_name> --build-type ament_python --dependencies rclpy
```


### Creating a python node file

Go to the package folder and create a .py file. Give access to it using `chmod` command.

```bash
cd ~/simple_av/src/<your_package_name>
touch node.py
chmod +x node.py
```


### Simple code for a subscriber node on `/sensing/gnss/pose` topic - Python

```python
def main():
    pass

class Sensor():
    pass
```

### Testing