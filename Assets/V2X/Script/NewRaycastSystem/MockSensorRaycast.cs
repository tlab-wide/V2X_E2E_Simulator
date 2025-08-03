using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// Multithreaded ray‑casting sensor (Burst for casts, tally on main thread).
/// </summary>
public class MockSensorRaycast : MonoBehaviour
{
    [Header("Sensor set‑up")] [SerializeField, Range(0, 180)]
    private float horizontalFov = 90f;

    [SerializeField, Range(0, 180)] private float verticalFov = 60f;
    [SerializeField] private float CustomFrequency = 0f;
    [SerializeField] private float maximumDistance = -1.0f;
    [SerializeField] private bool IsCamera = true;
    [SerializeField] private bool IsRsu = false;
    [SerializeField] LayerMask targetLayer = ~0;

    /* ─────────────────────────────────────────────────────────────── */

    private Dictionary<int, TrackableObject> trackedObjects = new();

    // private readonly List<TrackableObject> detected = new();
    private Dictionary<int, TrackableObject> detected = new();

    private float updateInterval;
    private float timeAcc;

    /* ─────────────────────────────────────────────────────────────── */


    public bool CheckIsDetected(int id)
    {
        return detected.ContainsKey(id);
    }


    private void Awake()
    {
        if (maximumDistance < 0)
            maximumDistance = RayCastManager.Instance.getMaxDistanceForSensorDetection();

        if (CustomFrequency <= 0)
        {
            updateInterval = 1f / RayCastManager.Instance.getDefaultFrequency();
        }
        else
        {
            updateInterval = 1f / CustomFrequency;
        }
    }

    private void Update()
    {
        timeAcc += Time.deltaTime;
        if (timeAcc < updateInterval || trackedObjects.Count == 0) return;

        timeAcc = 0f;
        PerformRayCasts();
    }

    /* ========================  Public API  ======================== */

    public bool GetIsCamera()
    {
        return IsCamera;
    }

    public bool GetIsRsu()
    {
        return IsRsu;
    }

    public void subscribeToSensor(int id, TrackableObject obj) => trackedObjects[id] = obj;
    public void unsubscribeFromSensor(int id) => trackedObjects.Remove(id);

    /* =====================  Core processing  ====================== */
    [SerializeField] private List<TrackableObject> trackables;

    private void PerformRayCasts()
    {
        // List<TrackableObject> trackables = new(trackedObjects.Values);
        trackables = new(trackedObjects.Values);

        // Build ray commands
        var cmdList = new List<RaycastCommand>(512);
        var ownerList = new List<int>(256);

        Vector3 sensorPos = transform.position;
        float halfH = horizontalFov * 0.5f;
        float halfV = verticalFov * 0.5f;
        float maxSq = maximumDistance * maximumDistance;

        for (int t = 0; t < trackables.Count; ++t)
        {
            foreach (Transform p in trackables[t].points)
            {
                Vector3 dir = p.position - sensorPos;
                if (!objectInView(p.transform))
                    continue;
                // float sq = dir.sqrMagnitude;
                // if (sq > maxSq) continue;
                //
                // // horizontal & vertical FOV checks
                // float horiz = math.degrees(math.atan2(dir.x, dir.z));
                // if (math.abs(horiz) > halfH) continue;
                //
                // float vert = math.degrees(math.asin(dir.y / math.sqrt(sq)));
                // if (math.abs(vert) > halfV) continue;


                // draw the ray for one frame (or set duration to updateInterval to persist until next cast)
                Debug.DrawRay(p.position, -dir.normalized * maximumDistance, Color.red, updateInterval);
                cmdList.Add(new RaycastCommand(p.position, -dir.normalized, maximumDistance));
                ownerList.Add(t);
            }
        }

        if (cmdList.Count == 0)
        {
            Debug.Log("nothing");
            detected.Clear();
            return;
        }

        // Native arrays for the batch
        using var cmds = new NativeArray<RaycastCommand>(cmdList.ToArray(), Allocator.TempJob);
        using var hits = new NativeArray<RaycastHit>(cmdList.Count, Allocator.TempJob);
        using var owners = new NativeArray<int>(ownerList.ToArray(), Allocator.TempJob);

        // Schedule and complete
        JobHandle handle = RaycastCommand.ScheduleBatch(cmds, hits, 32, default);
        handle.Complete();


        // //test situation
        // // 3) Draw both:  
        // //    - Burst result: green/red  
        // //    - Normal Physics.Raycast: blue/yellow  
        // float duration = updateInterval;
        // for (int i = 0; i < trackables[0].points.Count; i++)
        // {
        //     Vector3 o = trackables[0].points[i].position;
        //     Vector3 d = o - this.transform.position;
        //     float max = maximumDistance;
        //
        //     // --- Burst result ---
        //     bool cmdHit = hits[i].collider != null;
        //     float cmdDist = cmdHit ? hits[i].distance : max;
        //     // Color   cmdCol  = cmdHit ? Color.green : Color.red;
        //     if (cmdHit)
        //         Debug.DrawLine(o, o + d * cmdDist, Color.green, duration);
        //
        //     // --- Normal Raycast ---
        //     if (Physics.Raycast(o, d, out var normalHit, max, targetLayer))
        //     {
        //         // Debug.DrawLine(o, o + d * normalHit.distance, Color.blue, duration);
        //     }
        //     else
        //     {
        //         // Debug.DrawLine(o, o + d * max, Color.yellow, duration);
        //     }
        // }


        // ---------- tally on main thread ----------
        int[] hitCounts = new int[trackables.Count];


        for (int i = 0; i < hits.Length; ++i)
        {
            if (hits[i].collider != null && ((1 << hits[i].transform.gameObject.layer) & targetLayer.value) != 0)
                ++hitCounts[owners[i]];
        }

        Debug.Log($"hit count {hitCounts[0]}");

        // ---------- threshold tests ----------


        int globalThresh = IsCamera
            ? RayCastManager.Instance.getThresholdCameras()
            : RayCastManager.Instance.getThresholdLidars();

        for (int i = 0; i < trackables.Count; ++i)
        {
            TrackableObject tr = trackables[i];
            if (hitCounts[i] >= globalThresh &&
                hitCounts[i] >= tr.minimumNumberOfPointsVisible)
            {
                Debug.Log("finalizing");
                if (!detected.ContainsKey(tr.GetInstanceID()))
                {
                    detected.Add(tr.GetInstanceID(), tr);
                }
            }
            else
            {
                detected.Remove(tr.transform.GetInstanceID());
            }
        }
    }

    private bool objectInView(Transform targetPoint)
    {
        Vector3 targetVector = targetPoint.position - this.transform.position;

        //this if reduce amount of process and ignore far car from the max depth in calculation
        if (targetVector.magnitude > maximumDistance)
        {
            Debug.Log("distance out of range");
            return false;
        }

        Vector3 targetInXY = Vector3.ProjectOnPlane(targetVector, this.transform.right);
        float angleY = Vector3.Angle(this.transform.forward, targetInXY);


        if (angleY > verticalFov)
        {
            Debug.Log("Y problem");
            return false;
        }

        Vector3 targetInXZ = Vector3.ProjectOnPlane(targetVector, this.transform.up);
        float angleX = Vector3.Angle(this.transform.forward, targetInXZ);


        if (angleX > horizontalFov)
        {
            Debug.Log("X problem");
            return false;
        }
        else
        {
            return true;
        }
    }
}