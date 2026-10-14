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
        if (waypointFollower == null)   // the collider may sit on a child object
        {
            waypointFollower = other.GetComponentInParent<WaypointFollower>();
        }

        if (waypointFollower != null)
        {
            Destroy(waypointFollower.gameObject);   
        }
    }
}
