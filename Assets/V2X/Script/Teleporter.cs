using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

public class Teleporter : MonoBehaviour
{
    [SerializeField] Transform teleporter_pivot;

    
    private void OnTriggerEnter(Collider other)
    {
        Vehicle autonomousVehicle = other.GetComponentInParent<Vehicle>();
        if (autonomousVehicle != null)
        {
            autonomousVehicle.transform.position = teleporter_pivot.position;
            autonomousVehicle.transform.rotation = teleporter_pivot.rotation;
            autonomousVehicle.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
            autonomousVehicle.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
            
        }
        
    }
}
