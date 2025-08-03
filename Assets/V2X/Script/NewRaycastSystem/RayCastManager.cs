using System.Collections.Generic;
using UnityEngine;

public class RayCastManager : Singleton<RayCastManager>
{
    [SerializeField] private int thresholdCameras = 1;
    [SerializeField] private int thresholdLidars = 1;

    [SerializeField] private float defaultFrequencySensors = 10;

    [SerializeField] private List<MockSensorRaycast> busSensors;
    [SerializeField] private List<MockSensorRaycast> rsuSensors;
    
    [SerializeField] private float maxDistanceForSensorDetection = 250;
    [SerializeField] private float zoneCheckFrequencyTrckables = 1.2f; //object check 
    
    [SerializeField] private Transform vehicleTransform;
    
    [SerializeField] private float colorUpdateFrequency = 4f;


    public int getThresholdCameras()
    {
        return this.thresholdCameras;
    }

    public int getThresholdLidars()
    {
        return this.thresholdLidars;
    }

    public float getDefaultFrequency()
    {
        return this.defaultFrequencySensors;
    }

    public float getMaxDistanceForSensorDetection()
    {
        return this.maxDistanceForSensorDetection;
    }

    public float getZoneCheckFrequencyTrakables()
    {
        return this.zoneCheckFrequencyTrckables;
    }

    public List<MockSensorRaycast> getBusSensors()
    {
        return this.busSensors;
    }

    public List<MockSensorRaycast> getRsuSensors()
    {
        return this.rsuSensors;
    }
}