# Cooperative Autoware

## Download 
[Cooperative Autoware](https://drive.google.com/file/d/1x5M7YegynHbTBnRW1qIkoA0bpzMZLOAP/view?usp=drive_link)

## Build

The repository comes as a workspace by itself. To build the workspace follow the steps below:

- Clone this repository to some desired folder:
```
git clone git@github.com:hoosh-ir/autoware.git
```

- Navigate to the root of the workspace:
```
cd autoware
```

- Execute the ansible script to download and install system dependencies:
```
./setup-dev-env.sh
```

- Install ROS dependencies:
```
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

- Build the workspace:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Memory Issues At Build Time

Building Autoware requires and enormous amount RAM (about 32GB). If you have insufficient memory, use the following commands to allocate 32GB of swap file:
```
# Optional: Check the current swapfile
free -h

# Remove the current swapfile
sudo swapoff /swapfile
sudo rm /swapfile

# Create a new swapfile
sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Optional: Check if the change is reflected
free -h
```

### Additional Options Provided

```lightweight_localisation_ndt:=true/false```
```lightweight_perception_detection:=true/false``` used along with ```pseudo_sensing_topic:=<AWSIM pseudo sensor topic>```
```lightweight_perception_traffic_light:=true/false```
```cooperative_mode:=true/false```
