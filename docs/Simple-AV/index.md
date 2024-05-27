# Simple-AV

simple-AV is a single <b>ROS</b> node written in Python, designed to facilitate autonomous vehicle simulation by <b>directly connecting to AWSIM and replacing the Autoware framework</b>. While integrating AWSIM with Autoware is an effective approach, it often introduces substantial overhead. simple-AV aims to provide a more efficient solution by simplifying this process. 

This project focuses on ensuring seamless communication between AWSIM and the vehicle simulation tasks typically managed by Autoware, without the added complexity. By doing so, simple-AV offers a streamlined and effective method for autonomous vehicle simulation, handling all necessary tasks while minimizing setup and operational overhead.

simple-AV uses ROS Messages to communicate with AWSIM, subscribing to essential ROS topics such as sensor data and traffic light information. By processing these messages, simple-AV performs path planning and generates control commands necessary for vehicle movement, including acceleration, steering, and braking. It then publishes these control commands back to AWSIM, ensuring precise and efficient vehicle control from one point to another. 

This approach allows simple-AV to handle all aspects of vehicle control and movement, providing a comprehensive and simplified solution for autonomous vehicle simulation.