# Environment Setup for simple-AV

This guide will walk you through the steps to set up an environment in Windows 10 for launching the simple-AV ROS node, including the installation of AWSIM, ROS2 Humble, and WSL.

To develop and run simple-AV, it is essential to establish communication between AWSIM and ROS2. For this project, AWSIM is installed on Windows 10, while Ubuntu 22.04 is installed on WSL (Windows Subsystem for Linux) within Windows 10. ROS2 is then installed on Ubuntu 22.04 within WSL.

Given the crucial need for seamless communication between simple-AV and AWSIM, we ensure proper setup for the exchange of ROS topics. This involves configuring the network settings to enable ROS topics to be sent and received correctly between the Windows 10 environment and the ROS2 installation on WSL.

More specifically, this document goes through the installation process of

* [Installing WSL2 on Windows 10](../WSL/index.md)
* [Installing ROS2 Humble on Ubuntu 22.04 on WSL2](../InstallingROS2/index.md)
* [Setting up AWSIM on Window 10](../AWSIM_on_Windows/index.md)
* [Setting up Awsim and WSL2 connection](../Windows_WSL_Connection/index.md)

<b>For running AWSIM on Windows and get the topics in WSL, you don’t need to actually install ROS2 for Windows.</b>

--------------------------------------------------------------------------------------------------------------------

By following this guide, you will create a development environment that supports efficient communication between AWSIM and ROS2, enabling the successful deployment of the simple-AV ROS node.


