using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

public class VehicleKiller : MonoBehaviour
{
    
    private void OnTriggerEnter(Collider other)
    {
        WaypointFollower waypointFollower = other.GetComponent<WaypointFollower>();
        if (waypointFollower != null)
        {
            waypointFollower = other.GetComponentInParent<WaypointFollower>();
        }

        if (waypointFollower != null)
        {
            Destroy(waypointFollower.gameObject);   
        }
    }
}
