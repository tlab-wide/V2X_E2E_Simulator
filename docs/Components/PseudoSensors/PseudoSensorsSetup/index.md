# How to setup RSU and OBU sensing by Pseudo sensors

<video width="1920" controls autoplay muted loop>
<source src="V2X_Pseudo_sensor.mp4" type="video/mp4">
</video>


## Installation Guide for Pseudo Sensor Setup in the Unity
1.	Add the BlindManager.
2.	Add the Scenario.
3.	Assign References:

4.	Verify that the prefabs in the RandomTraffic simulator are sourced from our prefab directory.
5.	Create an OBU_Sensor as a child of sensor_kit_base_link (this is for management purposes only).
6.	Copy the prefabs and assign them to the OBU and the sensor list in the EgoVehicle.
7.	Create a similar setup object, as shown in the video, to handle RSUs.
8.	Add the prefabs from the specified directory.
9.	Similarly, assign the sensor to the RSU files and the sensor list in the EgoVehicle.


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
