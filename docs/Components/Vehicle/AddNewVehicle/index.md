# Adding New Vehicle 

Firstly, set up the vehicle based on the tutorial from AWSIM ([ Awsim Link ](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/
))

1. Create an empty object under the "sensorkit_base_link"" or any other object responsible for sensor data. Name the object "perception_sensor".


<div style="text-align: center;">
  <img src="image.png" alt="alt text" width="600">
</div>

<!-- ![alt text](image.png) -->


2. Add "Detected Object Autoware" to the "perception_sensor".


<div style="text-align: center;">
  <img src="image-2.png" alt="alt text" width="500">
</div>
<!-- ![alt text](image-2.png) -->


3. Under the "perception_sensor" object, you can add mock sensors in two ways::

    1. Create them from scratch based on our documentation [ link ](./../../PseudoSensors/PseudoSensors/index.md) 
    2. Use the prefabs inside the `Assets\KashiwaPackage\Prefabs\Pseudo Sensors`
)

4. You must add a reference to your sensor in the list of sensors within the "Detected Objects Autoware" component.


<div style="text-align: center;">
  <img src="image-3.png" alt="alt text" width="500">
</div>

5. You are also required to add a reference to your sensors in the scenario component as shown in the picture. Be careful when selecting the correct list; you must choose the appropriate list and drag and drop the sensor onto it.


<div style="text-align: center;">
  <img src="image-4.png" alt="alt text" width="1000">
</div>






