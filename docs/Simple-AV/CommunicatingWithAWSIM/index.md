# Communication with Awsim

## Phase No.1 - Creating a simple ROS2 Node

If you have not installed ROS2 humble please refer to [Installing ROS2](../SystemSetup/InstallingROS2/index.md) page.

To communicate with Awsim, filling control/command topics and subscribing to other topics that Awsim publishes, we need to create a Node and write the appropriate code. To do this, we first create a workspace and package. You can also look at [ROS2 documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#create-a-workspace) for better undertanding of the process.

first install colcon build tool on Ubuntu:

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
ros2 pkg create --build-type ament_python <your_package_name>
```

The simplest possible package may have a file structure that looks like:


```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```


### Creating a python node file

Go to the package folder and create a .py file. Give access to it using `chmod` command.

```bash
cd ~/simple_av/src/my_package/my_package
touch ConnectionController.py
chmod +x ConnectionController.py
```

### Simple code for a subscriber node on `/sensing/gnss/pose` topic - Python

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('talker')
        self.get_logger().info("Node cunstructed")
        self.publisher_ = self.create_publisher(String, 'dummy', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.callback)
        self.i = 0

    def callback(self):
        msg = String()
        msg.data = "Hellow %d" % self.i
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    _node = MinimalPublisher()
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
```

### Modifications on setup.py and package.xml


Add the following code into the setup.py
```python
entry_points={
        'console_scripts': [
            'ConnectionControlNode = my_package.ConnectionController:main',
        ],
    },
```

Add the following to the package.xml
```bash
 <build_depend>rclpy</build_depend>
 <build_depend>std_msgs</build_depend>
```

### Testing

First, go to the root and build using colcon. then source the workspace

```bash
cd ~/simple_av
colcon build
source /install/setup.bash
```

Now, run the node.

![alt text](image-1.png)

![alt text](image.png)


## Phase No.2 - Subscribing to Awsim Topics

In this Phase we aim to subscribe to Awsim Topics. we try to subscibe to 2 topics from Awsim.

### 1. `/sensing/gnss/pose` Topic

<b>Step 1: Finding the topic informations</b>

Use the command bellow to get the information of desiered topic

```bash
ros2 topic info </topic_name>
```

![alt text](image-4.png)

As we can see the type of the `/sensing/gnss/pose` topic is `geometry_msgs/msg/PoseStamped`.

<b>Step 2: Creating ROS node</b>

Python code:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class GnssPoseSubscriber(Node):
    def __init__(self):
        super().__init__('gnss_pose_subscriber')
        self.get_logger().info("GnssPoseSubscriber Cunstructed")
        self.subscription = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.listener_callback,
            10
        )
        self.subscription
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Position - x: {msg.pose.position.x}, y = {msg.pose.position.y}, z = {msg.pose.position.z}')
        self.get_logger().info(f'orientation- x: {msg.pose.orientation.x}, y = {msg.pose.orientation.y}, z = {msg.pose.orientation.z}, w = {msg.pose.orientation.w}')
        self.get_logger().info("-------------------------------------------------------------------------------------")


def main(args=None):
    rclpy.init(args=args)
    _node = GnssPoseSubscriber()
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
```

setup.py:
```python
entry_points={
        'console_scripts': [
            'ConnectionControlNode = inspector.ConnectionController:main',
        ],
    },
```

package.xml:
```python
<build_depend>rclpy</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>geometry_msgs</build_depend>
```

After applying the mentioned modification, source the workplace and run the `colcon build` in workspace root. Then, run the node
```bash
ros2 run my_package ConnectionControllerNode
```

![alt text](image-2.png)

<b>Step2: Run Awsim</b>

If you have not set up the [windows/WSL connection](../SystemSetup/Windows_WSL_Connection/index.md) you may not see the results below. 

![alt text](image-5.png)


### 2. `/vehicle/status/velocity_status` Topic



