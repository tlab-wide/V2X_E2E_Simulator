using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineOfSightManager : Singleton<LineOfSightManager>
{


    [SerializeField] private Transform busLidar;
    [SerializeField] private Transform crossRoadLidar;
    
    [SerializeField] private int busLidarThreshold = 3;
    [SerializeField] private int crossRoadLidarThreshold = 2;

    public Transform getBusLidar()
    {
        return busLidar;
    }

    public Transform getCrossRoadLidar()
    {
        return crossRoadLidar;
    }

    public bool CheckIsBusLidar(Transform transform)
    {
        return transform == busLidar;
    }

    public bool CheckIsCrossRoadLidar(Transform transform)
    {
        return transform == crossRoadLidar;
    }


    public int getBusLidarThreshold()
    {
        return busLidarThreshold;
    }

    public int getCrossRoadLidarThreshold()
    {
        return crossRoadLidarThreshold;
    }
    
    
    public void setBusLidarThreshold(string number)
    {
        busLidarThreshold = int.Parse(number);
    }

    public void setCrossroadThreshold(string number)
    {
        crossRoadLidarThreshold = int.Parse(number);
    }
}
