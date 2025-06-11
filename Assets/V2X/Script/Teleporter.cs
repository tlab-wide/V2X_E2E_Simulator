using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

public class Teleporter : MonoBehaviour
{
    [SerializeField] Transform teleporter_pivot;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

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
