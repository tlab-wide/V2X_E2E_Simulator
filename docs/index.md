# Welcome to V2X E2E simulator


This project is a simulation system based on the AWSIM project, available on [GitHub](https://github.com/tier4/AWSIM). It has been enhanced with a range of sensor types and a more realistic environment. One of the key study cases in this project is the simulation of autonomous buses, which incorporates a significant number of cameras to address blind spots during autonomous driving. Examples of applications for this simulation environment include analyzing the positioning of roadside unit sensors and bus sensors to enhance the vision of autonomous buses, as well as evaluating traffic metrics such as congestion due to the presence of autonomous buses within the city.

[Our Git repository is currently available](https://github.com/tlab-wide/V2X_E2E_Simulator){.md-button .md-button--primary}

## About
This project is part of **Cool4 (Cooperative Level 4 Automated Mobility Service)**, which corresponds to **Theme 4** of the [*Road to the L4*](https://www.road-to-the-l4.go.jp/en/) initiative.

The *Road to the L4* program is a **Research, Development, Demonstration, and Deployment (RDD&D)** project funded by the **Ministry of Economy, Trade and Industry (METI) of Japan**. Its goal is to promote the practical realization of **Level 4 autonomous driving** through collaboration among government, industry, and academia.

**Cool4 (Cooperative Level 4 Automated Mobility Service)** focuses on the development and demonstration of *cooperative* autonomous driving technologies — enabling vehicles, infrastructures, and systems to share information via **V2X communication** for safe and efficient automated mobility services.

This repository hosts the **V2X End-to-End Simulator**, designed to **evaluate cooperative Level 4 autonomous driving scenarios** in **Kashiwa-no-ha** and other test sites. The simulator supports comprehensive testing and analysis of communication, perception, and decision-making modules in connected automated environments.



## Simulation sample

<video width="1920" controls autoplay muted loop>
<source src="Autonomous.mp4" type="video/mp4">
</video>


## Extra Features
* Cyclists support
* Pseudo sensors (computationally lightweight sensors)
* New types of ROS messages
* Complete setup for an Autonomous Bus compatible with Autoware
* Enhanced logging system compatible with OpenStreetMap
* Tools and techniques to improve the accuracy of 3D models by loading PCD files
* Multi-Sensor Step Scanning component that generates synchronized data from camera and LiDAR and creates corresponding ground truth positions
* Pedestrian and cyclist movement patterns compatible with traffic lights and a collision avoidance system to prevent collisions with pedestrians and vehicles
* Integration of the [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors) package with HDRP support (not available in the original library)
* Teleport feature on the Unity side for the ego vehicle
* Traffic API updates to remove vehicles in specific scenarios and handle teleport situations
* Automatic repetition of multiple configurations to test various camera and lidar setups (documentation for this feature is currently incomplete)




<!-- ## Some notes

* `mkdocs new [dir-name]` - Create a new project.
* `mkdocs serve` - Start the live-reloading docs server.
* `mkdocs build` - Build the documentation site.
* `mkdocs -h` - Print help message and exit.

## Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files. -->


<style>
    .md-content__inner p {
    text-align: justify;
    hyphens: auto;
    }
</style>
