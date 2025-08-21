using AWSIM;
using UnityEngine;
public class ConstraintPedestrianTrafficLight : MonoBehaviour,IConstraintWayPoint
{
    [SerializeField] private bool lastCheckedstate;
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
        bool IsSolid = false;
        foreach (var bulbData in trafficLightBulbData)
        {
            if (isBulbTurnOn(bulbData.Status))
            {
                //             Debug.Log(bulbData.Color);
                color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
                IsSolid = true;
                break;
            }
        }

        if (!IsSolid)
        {
            return false;
        }

        //when the traffic light is red we can pass the street 
        if (color == autoware_perception_msgs.msg.TrafficSignalElement.GREEN)
        {
            lastCheckedstate = true;
            return true;
        }
        else
        {
            // Debug.Log($"by color {color.ToString()}");
            lastCheckedstate = false;
            return false;
        }
    }

    private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
    {
        return bulbStatus == TrafficLight.BulbStatus.SOLID_ON;
    }
}
