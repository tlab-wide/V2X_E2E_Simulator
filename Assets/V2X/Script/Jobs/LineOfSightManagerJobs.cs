using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using System.Collections.Generic;
using AWSIM;

public class LineOfSightManagerJobs : SingletonMonoBehaviour<LineOfSightManagerJobs>
{
    
    [Header("Job Settings")]
    private bool isJobsRunning = true;
    
    [Header("Layer Settings")]
    public LayerMask mockLidarLayer;
    
    [Header("Update Frequency (seconds)")]
    public float updateFrequency = 0.5f;  // Frequency at which the check is performed.
    private float timeSinceLastUpdate = 0f;
    

    // Registered sensors and their per-sensor parameters.
    private List<Transform> sensorTransforms = new List<Transform>();
    private List<float> sensorMaxDistances = new List<float>();
    private List<float> sensorHfovs = new List<float>();
    private List<float> sensorVfovs = new List<float>();

    // Registered destination objects (cars, etc.).
    private List<Transform> destinationTransforms = new List<Transform>();

    /// <summary>
    /// Registers a sensor with its parameters.
    /// </summary>
    public void RegisterSensor(Transform sensor, float maxDistance, float hfov, float vfov)
    {
        sensorTransforms.Add(sensor);
        sensorMaxDistances.Add(maxDistance);
        sensorHfovs.Add(hfov);
        sensorVfovs.Add(vfov);
    }

    /// <summary>
    /// Registers a destination (car or game object) to be checked.
    /// </summary>
    public void RegisterDestination(Transform destination)
    {
        destinationTransforms.Add(destination);
    }

    public bool IsJobsRunning()
    {
        return isJobsRunning;
    }

    void Update()
    {
        timeSinceLastUpdate += Time.deltaTime;
        if (timeSinceLastUpdate >= updateFrequency)
        {
            // Evaluate line-of-sight for every sensor-destination pair.
            List<bool> results = EvaluateLineOfSight();

            // For demonstration, log the result for each sensor-destination pair.
            int sensorCount = sensorTransforms.Count;
            int destCount = destinationTransforms.Count;
            for (int i = 0; i < sensorCount; i++)
            {
                for (int j = 0; j < destCount; j++)
                {
                    int idx = i * destCount + j;
                    Debug.Log($"Sensor {i} -> Destination {j}: {results[idx]}");
                }
            }
            timeSinceLastUpdate = 0f;
        }
    }

    /// <summary>
    /// Evaluates line-of-sight for each sensor-destination pair.
    /// Returns a flat list of booleans where:
    ///     index = sensorIndex * destinationCount + destinationIndex.
    /// </summary>
    public List<bool> EvaluateLineOfSight()
    {
        int sensorCount = sensorTransforms.Count;
        int destCount = destinationTransforms.Count;
        int totalPairs = sensorCount * destCount;
        if (totalPairs == 0)
        {
            return new List<bool>();
        }

        // Allocate NativeArrays for each sensor-destination pair.
        NativeArray<Vector3> origins = new NativeArray<Vector3>(totalPairs, Allocator.TempJob);
        NativeArray<Vector3> targetPositions = new NativeArray<Vector3>(totalPairs, Allocator.TempJob);
        NativeArray<float> nativeMaxDistances = new NativeArray<float>(totalPairs, Allocator.TempJob);
        NativeArray<float> nativeHfovs = new NativeArray<float>(totalPairs, Allocator.TempJob);
        NativeArray<float> nativeVfovs = new NativeArray<float>(totalPairs, Allocator.TempJob);
        NativeArray<Vector3> forwards = new NativeArray<Vector3>(totalPairs, Allocator.TempJob);
        NativeArray<Vector3> rights = new NativeArray<Vector3>(totalPairs, Allocator.TempJob);
        NativeArray<Vector3> ups = new NativeArray<Vector3>(totalPairs, Allocator.TempJob);
        NativeArray<bool> viewResults = new NativeArray<bool>(totalPairs, Allocator.TempJob);

        // Build the arrays using nested loops (each sensor paired with each destination).
        for (int i = 0; i < sensorCount; i++)
        {
            for (int j = 0; j < destCount; j++)
            {
                int idx = i * destCount + j;
                origins[idx] = sensorTransforms[i].position;
                targetPositions[idx] = destinationTransforms[j].position;
                nativeMaxDistances[idx] = sensorMaxDistances[i];
                nativeHfovs[idx] = sensorHfovs[i];
                nativeVfovs[idx] = sensorVfovs[i];
                forwards[idx] = sensorTransforms[i].forward;
                rights[idx] = sensorTransforms[i].right;
                ups[idx] = sensorTransforms[i].up;
            }
        }

        // --- Step 1: Run the view check job for each sensor-destination pair ---
        SensorDestinationViewJob viewJob = new SensorDestinationViewJob
        {
            origins = origins,
            targetPositions = targetPositions,
            maxDistances = nativeMaxDistances,
            Hfovs = nativeHfovs,
            Vfovs = nativeVfovs,
            forwards = forwards,
            rights = rights,
            ups = ups,
            viewResults = viewResults
        };
        JobHandle viewJobHandle = viewJob.Schedule(totalPairs, 64);
        viewJobHandle.Complete();

        // --- Step 2: Prepare and schedule batch raycasts for pairs that passed the view check ---
        NativeList<RaycastCommand> commands = new NativeList<RaycastCommand>(Allocator.Temp);
        NativeList<int> commandIndices = new NativeList<int>(Allocator.Temp);

        for (int idx = 0; idx < totalPairs; idx++)
        {
            if (viewResults[idx])
            {
                Vector3 origin = origins[idx];
                Vector3 direction = (targetPositions[idx] - origin).normalized;
                float distance = nativeMaxDistances[idx];
                commands.Add(new RaycastCommand(origin, direction, distance, mockLidarLayer));
                commandIndices.Add(idx);
            }
        }

        NativeArray<RaycastHit> raycastHits = new NativeArray<RaycastHit>(commands.Length, Allocator.TempJob);
        JobHandle raycastHandle = RaycastCommand.ScheduleBatch(commands.AsArray(), raycastHits, 1);
        raycastHandle.Complete();

        // Prepare final line-of-sight results (default to false).
        NativeArray<bool> lineOfSightResults = new NativeArray<bool>(totalPairs, Allocator.TempJob);
        for (int i = 0; i < totalPairs; i++)
        {
            lineOfSightResults[i] = false;
        }
        // For each executed raycast, if a collider was hit then mark that pair as having line-of-sight.
        for (int k = 0; k < commandIndices.Length; k++)
        {
            int pairIndex = commandIndices[k];
            if (raycastHits[k].collider != null)
            {
                lineOfSightResults[pairIndex] = true;
            }
        }

        // Convert the results to a List<bool>.
        List<bool> results = new List<bool>(totalPairs);
        for (int i = 0; i < totalPairs; i++)
        {
            results.Add(lineOfSightResults[i]);
        }

        // Dispose of temporary NativeArrays.
        origins.Dispose();
        targetPositions.Dispose();
        nativeMaxDistances.Dispose();
        nativeHfovs.Dispose();
        nativeVfovs.Dispose();
        forwards.Dispose();
        rights.Dispose();
        ups.Dispose();
        viewResults.Dispose();
        commands.Dispose();
        commandIndices.Dispose();
        raycastHits.Dispose();
        lineOfSightResults.Dispose();

        return results;
    }
}

[BurstCompile]
public struct SensorDestinationViewJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> origins;
    [ReadOnly] public NativeArray<Vector3> targetPositions;
    [ReadOnly] public NativeArray<float> maxDistances;
    [ReadOnly] public NativeArray<float> Hfovs;
    [ReadOnly] public NativeArray<float> Vfovs;
    [ReadOnly] public NativeArray<Vector3> forwards;
    [ReadOnly] public NativeArray<Vector3> rights;
    [ReadOnly] public NativeArray<Vector3> ups;
    public NativeArray<bool> viewResults;

    public void Execute(int index)
    {
        Vector3 origin = origins[index];
        Vector3 targetPos = targetPositions[index];
        float maxDistance = maxDistances[index];
        float Hfov = Hfovs[index];
        float Vfov = Vfovs[index];
        Vector3 forward = forwards[index];
        Vector3 right = rights[index];
        Vector3 up = ups[index];

        Vector3 targetVector = targetPos - origin;

        // Early out if the destination is too far.
        if (targetVector.magnitude > maxDistance)
        {
            viewResults[index] = false;
            return;
        }

        // Check vertical field-of-view.
        Vector3 targetInXY = Vector3.ProjectOnPlane(targetVector, right);
        float angleY = Vector3.Angle(forward, targetInXY);
        if (angleY > Vfov)
        {
            viewResults[index] = false;
            return;
        }

        // Check horizontal field-of-view.
        Vector3 targetInXZ = Vector3.ProjectOnPlane(targetVector, up);
        float angleX = Vector3.Angle(forward, targetInXZ);
        if (angleX > Hfov)
        {
            viewResults[index] = false;
            return;
        }

        viewResults[index] = true;
    }
}
