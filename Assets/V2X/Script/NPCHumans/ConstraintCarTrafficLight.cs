using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

public class ConstraintCarTrafficLight : MonoBehaviour , IConstraintWayPoint
{
    [SerializeField] private bool state;
    [SerializeField] private TrafficLight trafficLight;

    public bool CheckState()
    {
 //       Debug.Log("checked");
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
   //             Debug.Log(bulbData.Color);
                color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
            }
        }

        //when the traffic light is red we can pass the street 
        if (color == autoware_perception_msgs.msg.TrafficSignalElement.GREEN)
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
