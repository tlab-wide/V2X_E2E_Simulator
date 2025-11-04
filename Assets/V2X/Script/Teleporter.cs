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

        // Get Rigidbody reference
        Rigidbody rb = vehicle.GetComponent<Rigidbody>();


        // Phase 1: Remove vehicles around teleport destination
        foreach (TrafficManager trafficManager in trafficManagers)
        {
            trafficManager.RemoveVehiclesInRadius(teleporter_pivot.position, clearanceRadius);
        }

        // Wait for removal to complete
        yield return new WaitForSeconds(teleportDelay);


        // Wait for fixed update
        yield return new WaitForFixedUpdate();
        // Disable physics temporarily
        if (rb != null)
        {
            rb.isKinematic = true;
        }

        vehicle.transform.SetPositionAndRotation(teleporter_pivot.position, teleporter_pivot.rotation);
        yield return new WaitForFixedUpdate();
        // Reset vehicle state
        vehicle.transform.SetPositionAndRotation(teleporter_pivot.position, teleporter_pivot.rotation);
        yield return new WaitForFixedUpdate();

        // Phase 2: Teleport the vehicle (do ALL at once)
        vehicle.transform.SetPositionAndRotation(teleporter_pivot.position, teleporter_pivot.rotation);

        // Re-enable physics and reset velocities
        if (rb != null)
        {
            rb.isKinematic = false;
            rb.position = teleporter_pivot.position;
            rb.rotation = teleporter_pivot.rotation;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Wait for physics to stabilize
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // Remove from tracking set
        teleportingVehicles.Remove(vehicle);
    }
}