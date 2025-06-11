using System;
using AWSIM;
using UnityEngine;

public class ConstraintTraffic : MonoBehaviour, IConstraintWayPoint
{
    [SerializeField] private bool state = false; //just for debug
    [SerializeField] private TrafficLight trafficLight;

    public bool CheckState()
    {
        if (trafficLight is null || !trafficLight.gameObject.activeSelf || !trafficLight.gameObject.activeInHierarchy)
        {
            return true;
        }

        var trafficLightBulbData = trafficLight.GetBulbData();
        //Fill TrafficSignal with bulbData
        byte color = 0;
        foreach (var bulbData in trafficLightBulbData)
        {
            if (isBulbTurnOn(bulbData.Status))
            {
                color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
            }
        }

        //when the traffic light is red we can pass the street 
        if (color == autoware_perception_msgs.msg.TrafficSignalElement.RED)
        {
            state = true;
            return true;
        }
        else
        {
            state = false;
            return false;
        }
    }

    private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
    {
        return bulbStatus == TrafficLight.BulbStatus.SOLID_ON || bulbStatus == TrafficLight.BulbStatus.FLASHING;
    }
}

[SerializeField]
public interface IConstraintWayPoint
{
    public bool CheckState();
}