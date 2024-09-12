# Welcome to V2X E2E simulator

This project is a simulation system based on the AWSIM project, available on [GitHub](https://github.com/tier4/AWSIM). It has been enhanced with a range of sensor types and a more realistic environment. One of the key study cases in this project is the simulation of autonomous buses, which incorporates a significant number of cameras to address blind spots during autonomous driving. Examples of applications for this simulation environment include analyzing the positioning of roadside unit sensors and bus sensors to enhance the vision of autonomous buses, as well as evaluating traffic metrics such as congestion due to the presence of autonomous buses within the city.

Our Git repository is currently available on the [Link](https://tlab-wide.github.io/V2X_E2E_Simulator/Simple-AV/SystemSetup/)

## Simulation sample

<video width="1920" controls autoplay muted loop>
<source src="Autonomous.mp4" type="video/mp4">
</video>


## Extra Features
* Cyclists
* Pseudo sensors (computationally lightweight sensors)
* Additional patterns of movement
* New types of ROS messages
* Complete setup for an Autonomous Bus compatible with Autoware
* Enhanced Logging System compatible with OpenStreetMap
* Tools and techniques to improve the accuracy of 3D models by loading PCDs


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
