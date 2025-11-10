using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using AWSIM;
using UnityEngine;

/// <summary>
/// Monitors and logs the positions and states of NPCs (vehicles and pedestrians) within a defined trigger area.
/// Writes log data to CSV files for later analysis.
/// </summary>
[RequireComponent(typeof(BoxCollider))]
public class LogArea : MonoBehaviour
{
    #region Serialized Fields
    
    [Header("Log File Paths")]
    [SerializeField] private string logPathHumans;
    [SerializeField] private string logPathCars;

    [Header("Tracking Settings")]
    [SerializeField] private bool checkHumans = true;
    [SerializeField] private bool checkCars = true;

    [Header("Performance Settings")]
    [Tooltip("Wait time between log saves in seconds (only used when ultraMode is false)")]
    [SerializeField] private float waitTime = 0.05f;
    
    [Tooltip("Number of frames to wait between log saves (only used when ultraMode is true)")]
    [SerializeField] private int waitForFrame = 2;
    
    [Tooltip("When enabled, uses frame-based timing instead of time-based")]
    [SerializeField] private bool ultraMode = true;

    [Header("Dependencies")]
    [SerializeField] private CheckpointJumper checkpointJumper;
    
    [Header("Layer Settings")]
    [Tooltip("Layer used for vehicle detection")]
    [SerializeField] private LayerMask vehicleLayer;
    
    #endregion

    #region Private Fields
    
    private ILogIndex _logIndexBus;
    private List<Transform> _humanTransforms = new List<Transform>();
    private List<Transform> _carTransforms = new List<Transform>();
    
    #endregion

    #region Unity Lifecycle Methods
    
    private void Start()
    {
        _logIndexBus = checkpointJumper;
        
        StartCoroutine(InitializeCsvFiles());
        StartCoroutine(LoggingLoop(waitTime, ultraMode));
    }

    private void OnTriggerEnter(Collider other)
    {
        if (checkCars && TryGetNPCVehicle(other, out Transform vehicleTransform))
        {
            AddUniqueTransform(_carTransforms, vehicleTransform);
        }
        else if (checkHumans && TryGetNPCPedestrian(other, out Transform pedestrianTransform))
        {
            AddUniqueTransform(_humanTransforms, pedestrianTransform);
        }
        else if (IsVehicleLayer(other))
        {
            // Handle special case for bus or other vehicles with Rigidbody
            Rigidbody rigidbody = FindRigidbodyInHierarchy(other.gameObject);
            if (rigidbody != null)
            {
                AddUniqueTransform(_carTransforms, rigidbody.transform);
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (TryGetNPCVehicle(other, out Transform vehicleTransform))
        {
            _carTransforms.Remove(vehicleTransform);
        }
        else if (TryGetNPCPedestrian(other, out Transform pedestrianTransform))
        {
            _humanTransforms.Remove(pedestrianTransform);
        }
        else if (IsVehicleLayer(other))
        {
            Rigidbody rigidbody = FindRigidbodyInHierarchy(other.gameObject);
            if (rigidbody != null)
            {
                _carTransforms.Remove(rigidbody.transform);
            }
        }
    }
    
    #endregion

    #region Collision Detection Helpers
    
    private bool TryGetNPCVehicle(Collider collider, out Transform vehicleTransform)
    {
        NPCVehicle vehicle = collider.gameObject.GetComponentInParent<NPCVehicle>();
        vehicleTransform = vehicle?.transform;
        return vehicle != null;
    }

    private bool TryGetNPCPedestrian(Collider collider, out Transform pedestrianTransform)
    {
        NPCPedestrian pedestrian = collider.gameObject.GetComponentInParent<NPCPedestrian>();
        pedestrianTransform = pedestrian?.transform;
        if (pedestrian == null)
        {
            WaypointFollower waypointFollower = collider.gameObject.GetComponentInParent<WaypointFollower>();
            pedestrianTransform = waypointFollower?.transform;
            return waypointFollower != null;
        }
        return false;
    }

    private bool IsVehicleLayer(Collider collider)
    {
        return ((1 << collider.gameObject.layer) & vehicleLayer) != 0;
    }

    private void AddUniqueTransform(List<Transform> list, Transform transform)
    {
        if (!list.Contains(transform))
        {
            list.Add(transform);
        }
    }
    
    #endregion

    #region Velocity Calculation
    
    /// <summary>
    /// Calculates velocity components for a transform.
    /// Returns velocity vector (x, y, z) and total speed amount.
    /// </summary>
    private (Vector3 velocity, float speedAmount) GetVelocity(Transform transform)
    {
        ISpeed speedComponent = transform.GetComponent<ISpeed>();
        
        if (speedComponent == null)
        {
            return (Vector3.zero, 0f);
        }

        float speedAmount = speedComponent.GetSpeed();
        
        // Calculate velocity components using the forward vector
        Vector3 forward = transform.forward;
        Vector3 velocity = forward * speedAmount;
        
        return (velocity, speedAmount);
    }
    
    #endregion

    #region Rigidbody Search
    
    /// <summary>
    /// Recursively searches up the GameObject hierarchy to find a Rigidbody component.
    /// </summary>
    /// <param name="childObject">The starting GameObject to search from</param>
    /// <returns>The first Rigidbody found in the hierarchy, or null if none exists</returns>
    private Rigidbody FindRigidbodyInHierarchy(GameObject childObject)
    {
        Rigidbody rigidbody = childObject.GetComponent<Rigidbody>();
        
        if (rigidbody != null)
        {
            return rigidbody;
        }

        if (childObject.transform.parent == null)
        {
            return null;
        }

        return FindRigidbodyInHierarchy(childObject.transform.parent.gameObject);
    }
    
    #endregion

    #region Logging System
    
    /// <summary>
    /// Main logging coroutine that periodically saves transform data to CSV files.
    /// </summary>
    private IEnumerator LoggingLoop(float waitingTime, bool useUltraMode)
    {
        // Wait for initial setup to complete
        yield return WaitForNFrames(UnityEngine.Random.Range(20, 30));

        while (true)
        {
            LogTransforms(_carTransforms, logPathCars);
            LogTransforms(_humanTransforms, logPathHumans);

            if (useUltraMode)
            {
                yield return WaitForNFrames(waitForFrame);
            }
            else
            {
                yield return new WaitForSeconds(waitingTime);
            }
        }
    }

    /// <summary>
    /// Logs all transforms in the provided list to the specified file path.
    /// Removes any null or destroyed transforms from the list.
    /// </summary>
    private void LogTransforms(List<Transform> transforms, string filePath)
    {
        for (int i = transforms.Count - 1; i >= 0; i--)
        {
            string dataRow = GenerateDataRow(transforms[i]);
            
            if (string.IsNullOrEmpty(dataRow))
            {
                Debug.Log($"Removing destroyed object at index {i}");
                transforms.RemoveAt(i);
                continue;
            }

            AppendToFile(filePath, dataRow);
        }
    }

    /// <summary>
    /// Generates a CSV data row for the given transform.
    /// </summary>
    /// <returns>A CSV-formatted string, or empty string if the transform is invalid</returns>
    private string GenerateDataRow(Transform transform)
    {
        if (!IsTransformValid(transform))
        {
            return string.Empty;
        }

        LineOfSight lineOfSight = transform.GetComponent<LineOfSight>();
        lineOfSight?.checkImmidiately();

        builtin_interfaces.msg.Time rosTime = SimulatorROS2Node.GetCurrentRosTime();
        Vector3 position = GetRosPosition(transform.position);
        Quaternion rotation = ROS2Utility.UnityToRosRotation(transform.rotation);
        int busState = _logIndexBus?.GetIndex() ?? 0;
        
        // Get velocity data
        var (velocity, speedAmount) = GetVelocity(transform);

        if (lineOfSight == null)
        {
            return FormatDataRow(
                transform.name,
                position,
                rotation,
                rosTime,
                boxState: LineOfSight.BoxState.Unknown,
                sensorData: string.Empty,
                busState,
                velocity,
                speedAmount
            );
        }

        string sensorData = BuildSensorData(lineOfSight);
        LineOfSight.BoxState boxState = lineOfSight.GetCarBoxState();

        return FormatDataRow(
            transform.name,
            position,
            rotation,
            rosTime,
            boxState,
            sensorData,
            busState,
            velocity,
            speedAmount
        );
    }

    private bool IsTransformValid(Transform transform)
    {
        try
        {
            if (transform == null)
            {
                Debug.LogWarning("Transform is null, cannot save log");
                return false;
            }
            return true;
        }
        catch (Exception e)
        {
            Debug.LogWarning("Object was destroyed: " + e.Message);
            return false;
        }
    }

    private Vector3 GetRosPosition(Vector3 unityPosition)
    {
        Vector3 rosPosition = ROS2Utility.UnityToRosPosition(unityPosition);
        return rosPosition + AWSIM.Environment.Instance.MgrsOffsetPosition;
    }

    private string BuildSensorData(LineOfSight lineOfSight)
    {
        List<MockDetectionSensor> sensors = lineOfSight.GetObservableSensors();
        List<string> sensorEntries = new List<string>();

        foreach (MockDetectionSensor sensor in sensors)
        {
            int detectionCount = lineOfSight.GetNumberOfDetectedPoint(sensor);
            sensorEntries.Add($"{sensor.getName()}*{detectionCount}*");
        }

        return string.Join("|", sensorEntries);
    }

    private string FormatDataRow(
        string name,
        Vector3 position,
        Quaternion rotation,
        builtin_interfaces.msg.Time rosTime,
        LineOfSight.BoxState boxState,
        string sensorData,
        int busState,
        Vector3 velocity,
        float speedAmount)
    {
        return $"{name}," +
               $"{position.x},{position.y},{position.z}," +
               $"{rotation.w},{rotation.x},{rotation.y},{rotation.z}," +
               $"{rosTime.Sec},{rosTime.Nanosec}," +
               $"{Time.frameCount}," +
               $"{boxState}," +
               $"{sensorData}," +
               $"{busState}," +
               $"{velocity.x},{velocity.y},{velocity.z},{speedAmount}\n";
    }
    
    #endregion

    #region File I/O
    
    private IEnumerator InitializeCsvFiles()
    {
        const string CsvHeader = "Name,X,Y,Z,W rotation,X rotation,Y rotation,Z rotation," +
                                 "Time_sec,Time_nano,Frame,Box_State,Sensor Names,Index," +
                                 "X Speed,Y Speed,Z Speed,Speed Amount\n";

        if (checkCars && !string.IsNullOrEmpty(logPathCars))
        {
            EnsureFileExistsWithHeader(logPathCars, CsvHeader);
        }

        if (checkHumans && !string.IsNullOrEmpty(logPathHumans))
        {
            EnsureFileExistsWithHeader(logPathHumans, CsvHeader);
        }

        yield return null;
    }

    private void EnsureFileExistsWithHeader(string filePath, string header)
    {
        string directory = Path.GetDirectoryName(filePath);
        
        if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }

        if (!File.Exists(filePath))
        {
            File.WriteAllText(filePath, header);
        }
    }

    private static void AppendToFile(string filePath, string content)
    {
        try
        {
            using (StreamWriter writer = new StreamWriter(filePath, true))
            {
                writer.Write(content);
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to write to file {filePath}: {e.Message}");
        }
    }
    
    #endregion

    #region Utility Methods
    
    private IEnumerator WaitForNFrames(int frameCount)
    {
        for (int i = 0; i < frameCount; i++)
        {
            yield return null;
        }
    }
    
    #endregion
}