# Pseudo Sensor
Pseudo sensors are designed to reduce the number of processes compared to real sensors such as lidar. Unlike real sensors that broadcast signals in all directions, pseudo sensors use line of sight. Rays are directly sent toward target objects that already have a line of sight, as illustrated in the picture below.

<!-- ![alt text](image.png) -->
<div style="text-align: center;">
  <img src="image.png" alt="alt text" width="700" >
</div>

Each pseudo sensor requires a Mock Sensor Object and a box collider. The box collider is used to calculate ray collisions.

<!-- ![alt text](image-1.png) -->
<div style="text-align: center;">
  <img src="image-1.png" alt="alt text" width="900" >
</div>

## Fields:

* **Name** : This field is used in the log system to identify which sensors have detected objects.
* **Category** : This field categorizes sensors using a config file. If you prefer to manually set the position and features of the sensor in the Unity scene, you can ignore this field.

* **Mock Sensor Type** : This enum field determines if the sensor simulates a Camera or lidar and sets it in a bus or roadside.

* **HFOV (Horizontal Field of View)** : Specifies the horizontal field of view.

* **VFOV (Vertical Field of View)** : Specifies the vertical field of view.

* **Sceen Objects** : This field is for testing purposes and shows the detected objects during runtime.

* **Max Distnace** : Specifies the range of the rays.


## Other configuration:
At the end the pseudo sensor must be add to scenario this enable the system to detect  which object detect by which type of sensors


<!-- ![alt text](image-2.png) -->

<div style="text-align: center;">
  <img src="image-2.png" alt="alt text" width="600" >
</div>