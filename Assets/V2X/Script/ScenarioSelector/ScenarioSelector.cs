using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using AWSIM;
using AWSIM.TrafficSimulation;

public class ScenarioSelector : MonoBehaviour
{
    // [SerializeField] private TMP_Dropdown dropdown;

    [Header("Scenarios")]
    [SerializeField] private List<Transform> scenarios;
    [Tooltip("Scenario index to activate on Start. Set -1 to skip auto-activation.")]
    [SerializeField] private int defaultScenarioIndex = 1;

    [Header("Vehicles")]
    [SerializeField] private Transform bus;
    [SerializeField] private Transform car;

    [Header("Start Points")]
    [SerializeField] private Transform startPoint1;
    [SerializeField] private Transform startPoint2;
    [SerializeField] private Transform startPoint3;
    [SerializeField] private Transform startPoint4;

    [Header("Teleport Settings")]
    [SerializeField] private float clearanceRadius = 10f;
    [SerializeField] private float teleportDelay = 0.5f;

    private Transform targetTransform;

    // Cache all active TrafficManagers (like in Teleporter)
    private List<TrafficManager> trafficManagers;
    private HashSet<Transform> teleporting = new HashSet<Transform>();

    void Awake()
    {
        trafficManagers = new List<TrafficManager>(FindObjectsOfType<TrafficManager>());
    }

    void Start()
    {
        DeactivateAllScenarios();
        if (defaultScenarioIndex >= 0)
            ActivateScenario(defaultScenarioIndex);

        targetTransform = bus.gameObject.activeInHierarchy ? bus.transform : car.transform;
    }

    // --- Public API ----------------------------------------------------------
    // Call this with any vehicle transform and a target pose to teleport with clearance.
    public void Teleport(Transform vehicle, Transform destination)
    {
        if (vehicle == null || destination == null) return;
        StartCoroutine(TeleportWithClearance(vehicle.GetComponent<Vehicle>(), destination.position, destination.rotation));
    }

    // If you want to feed a raw position/rotation instead of a Transform:
    public void Teleport(Transform vehicle, Vector3 position, Quaternion rotation)
    {
        if (vehicle == null) return;
        StartCoroutine(TeleportWithClearance(vehicle.GetComponent<Vehicle>() , position, rotation));
    }
    // ------------------------------------------------------------------------

    private IEnumerator TeleportWithClearance(Vehicle vehicle, Vector3 position, Quaternion rotation)
    {
        // Prevent re-entrancy on the same vehicle
        if (teleporting.Contains(vehicle.transform)) yield break;
        teleporting.Add(vehicle.transform);

        // Get Rigidbody reference
        var rb = vehicle.GetComponent<Rigidbody>();
        // Phase 1: clear destination area
        foreach (var tm in trafficManagers)
        {
            if (tm != null)
                tm.RemoveVehiclesInRadius(position, clearanceRadius);
        }

        // Wait for removal to take effect
        yield return new WaitForSeconds(teleportDelay);
    
        
        
        // Sync with physics step before moving rigidbodies
        yield return new WaitForFixedUpdate();

        // Reset vehicle state
        vehicle.ResetMotionState();
        
        // Phase 2: move the vehicle (atomic operation)
        vehicle.transform.SetPositionAndRotation(position, rotation);
        
        // Re-enable physics and set Rigidbody properties
        if (rb != null)
        {
            rb.isKinematic = false;
            rb.position = position;
            rb.rotation = rotation;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Wait for physics to stabilize
        yield return new WaitForFixedUpdate();
        // Disable physics temporarily
        if (rb != null)
        {
            rb.isKinematic = true;
        }
        yield return new WaitForFixedUpdate();
        vehicle.transform.SetPositionAndRotation(position, rotation);
        yield return new WaitForFixedUpdate();
        vehicle.transform.SetPositionAndRotation(position, rotation);
        yield return new WaitForFixedUpdate();

        teleporting.Remove(vehicle.transform);
        if (rb != null)
        {
            rb.isKinematic = false;
        }
    }

    // Method to activate a specific scenario and deactivate all others
    public void ActivateScenario(int scenarioIndex)
    {
        Debug.Log($"Activate scenario called by value {scenarioIndex}");
        if (scenarioIndex >= 0 && scenarioIndex < scenarios.Count)
        {
            DeactivateAllScenarios();
            scenarios[scenarioIndex].gameObject.SetActive(true);
        }
        else
        {
            Debug.LogWarning("Scenario index out of range: " + scenarioIndex);
        }
    }

    // Method to deactivate all scenarios
    private void DeactivateAllScenarios()
    {
        foreach (Transform scenario in scenarios)
        {
            if (scenario != null)
                scenario.gameObject.SetActive(false);
        }
    }

    // Input handling
    
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            Teleport(targetTransform, startPoint1);
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            Teleport(targetTransform, startPoint2);
        }

        if (Input.GetKeyDown(KeyCode.F))
        {
            Teleport(targetTransform, startPoint3);
        }

        if (Input.GetKeyDown(KeyCode.X))
        {
            Teleport(targetTransform, startPoint4);
        }
    }

    public void ApplyDropDownData(TMP_Dropdown dropdown)
    {
        int idPeaked = dropdown.value;
        DeactivateAllScenarios();
        ActivateScenario(idPeaked);
    }
}
