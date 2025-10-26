using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using AWSIM.TrafficSimulation;
using UnityEngine;

public class Teleporter : MonoBehaviour
{
    [SerializeField] Transform teleporter_pivot;
    [SerializeField] float clearanceRadius = 10f;
    [SerializeField] float teleportDelay = 0.5f;

    private List<TrafficManager> trafficManagers;
    private HashSet<Vehicle> teleportingVehicles = new HashSet<Vehicle>();

    private void Awake()
    {
        // Find all active TrafficManagers
        trafficManagers = new List<TrafficManager>(FindObjectsOfType<TrafficManager>());
    }

    private void OnTriggerEnter(Collider other)
    {
        Vehicle autonomousVehicle = other.GetComponentInParent<Vehicle>();
        if (autonomousVehicle != null && !teleportingVehicles.Contains(autonomousVehicle))
        {
            StartCoroutine(TeleportWithClearance(autonomousVehicle));
        }
    }

    private IEnumerator TeleportWithClearance(Vehicle vehicle)
    {
        // Mark vehicle as being teleported to prevent re-triggering
        teleportingVehicles.Add(vehicle);

        // Phase 1: Remove vehicles around teleport destination
        foreach (TrafficManager trafficManager in trafficManagers)
        {
            trafficManager.RemoveVehiclesInRadius(teleporter_pivot.position, clearanceRadius);
        }

        // Wait for removal to complete
        yield return new WaitForSeconds(teleportDelay);

        // we have to catch on fix update to move properly
        yield return new WaitForFixedUpdate();
        
        // Phase 2: Teleport the vehicle
        vehicle.transform.position = teleporter_pivot.position;
        vehicle.transform.rotation = teleporter_pivot.rotation;
        
        Rigidbody rb = vehicle.GetComponent<Rigidbody>();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Extra safety: wait a bit more
        yield return new WaitForSeconds(0.5f);

        // Remove from tracking set
        teleportingVehicles.Remove(vehicle);
    }
}