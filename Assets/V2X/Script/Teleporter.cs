using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using AWSIM.TrafficSimulation;
using UnityEngine;

public class Teleporter : MonoBehaviour
{
    [Serializable]
    private class LaneSpawnSource
    {
        [Tooltip("Lane used to auto-generate spawn samples along its geometry.")]
        public TrafficLane lane;
    }

    [Serializable]
    private class RandomSpawnPoint
    {
        [Tooltip("Fixed spawn position used when random strategy is selected.")]
        public Transform point;
        [Tooltip("Optional yaw override in degrees. Leave blank to use point rotation.")]
        public bool useCustomRotation;
        public Vector3 customEulerRotation;

        public Quaternion GetRotation()
        {
            if (!useCustomRotation || point == null || customEulerRotation == Vector3.zero)
                return point != null ? point.rotation : Quaternion.identity;
            return Quaternion.Euler(customEulerRotation);
        }
    }

    private class LaneCache
    {
        public Vector3[] Points;
        public List<float> CumulativeDistances;
        public float Length;
    }

    private class SpawnCandidate
    {
        public TrafficLane Lane;
        public float DistanceFromStart;
        public Vector3 Position;
        public Quaternion Rotation;
        public int SampleIndex;
    }

    private class SpawnSelection
    {
        public SpawnCandidate Candidate;
        public float DistanceToMidpoint;
        public float EstimatedTime;
        public float TargetTime;
        public bool ReachedCruiseSpeed;
    }

    public enum SpawnStrategy
    {
        LaneMidpoint,
        RandomPoint
    }

    [Header("Teleport timing")]
    [SerializeField, Tooltip("Radius used to clear other vehicles before teleporting.")]
    private float clearanceRadius = 10f;
    [SerializeField, Tooltip("Fixed wait after clearing NPCs, before teleport.")]
    private float teleportDelay = 0.5f;
    [SerializeField, Tooltip("Additional random delay added before teleport. X = min, Y = max. Zero disables.")]
    private Vector2 spawnDelayRange = Vector2.zero;

    [Header("Strategy selection")]
    [SerializeField, Tooltip("Choose lane-based midpoint targeting or random spawn point.")]
    private SpawnStrategy spawnStrategy = SpawnStrategy.LaneMidpoint;

    [Header("Lane-based (midpoint targeting)")]
    [SerializeField, Tooltip("Lanes used to auto-generate spawn candidates.")]
    private List<LaneSpawnSource> laneSpawnSources = new List<LaneSpawnSource>();
    [SerializeField, Tooltip("Meters between auto-generated spawn points along each lane.")]
    private float spawnSubdivisionSpacing = 10f;
    [SerializeField, Tooltip("Include start/end of each lane as spawn candidates.")]
    private bool includeSegmentEndpoints = true;
    [SerializeField, Tooltip("Midpoint target segment (TrafficLane) vehicles should reach near t_target.")]
    private TrafficLane midpointSegment;
    [SerializeField, Range(0f, 1f), Tooltip("Normalized position along the midpoint segment (0=start, 1=end).")]
    private float midpointSegmentFraction = 0.5f;
    [SerializeField, Tooltip("Desired cruise speed used for ETA calculation (km/h).")]
    private float cruiseSpeedKmh = 30f;
    [SerializeField, Tooltip("Assumed constant acceleration in m/s^2 for the spawn ETA calculation.")]
    private float assumedAcceleration = 2f;
    [SerializeField, Tooltip("Random arrival time window (seconds) for midpoint targeting: X=min, Y=max.")]
    private Vector2 targetArrivalWindow = new Vector2(0f, 90f);

    [Header("Random spawn points")]
    [SerializeField, Tooltip("Fixed spawn points used when strategy is RandomPoint.")]
    private List<RandomSpawnPoint> randomSpawnPoints = new List<RandomSpawnPoint>();

    private List<TrafficManager> trafficManagers;
    private HashSet<Vehicle> teleportingVehicles = new HashSet<Vehicle>();
    private Dictionary<TrafficLane, LaneCache> laneCache = new Dictionary<TrafficLane, LaneCache>();

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

        SpawnSelection spawnSelection = SelectSpawnCandidate();
        Vector3 targetPosition;
        Quaternion targetRotation;

        if (spawnSelection != null)
        {
            targetPosition = spawnSelection.Candidate.Position;
            targetRotation = spawnSelection.Candidate.Rotation;
        }
        else
        {
            Debug.LogWarning("Teleporter: No spawn candidate available. Aborting teleport.");
            teleportingVehicles.Remove(vehicle);
            yield break;
        }

        // Phase 1: Remove vehicles around teleport destination
        foreach (TrafficManager trafficManager in trafficManagers)
        {
            trafficManager.RemoveVehiclesInRadius(targetPosition, clearanceRadius);
        }

        // Wait for removal to complete
        yield return new WaitForSeconds(teleportDelay);


        if (spawnDelayRange.y > 0f)
        {
            var min = Mathf.Max(0f, spawnDelayRange.x);
            var max = Mathf.Max(min, spawnDelayRange.y);
            var randomDelay = UnityEngine.Random.Range(min, max);
            if (randomDelay > 0f)
                yield return new WaitForSeconds(randomDelay);
        }


        // Wait for fixed update
        yield return new WaitForFixedUpdate();
        // Disable physics temporarily
        if (rb != null)
        {
            rb.isKinematic = true;
        }

        vehicle.transform.SetPositionAndRotation(targetPosition, targetRotation);
        yield return new WaitForFixedUpdate();
        // Reset vehicle state
        vehicle.transform.SetPositionAndRotation(targetPosition, targetRotation);
        yield return new WaitForFixedUpdate();

        // Phase 2: Teleport the vehicle (do ALL at once)
        vehicle.transform.SetPositionAndRotation(targetPosition, targetRotation);

        LogSpawnSelection(vehicle, spawnSelection, targetPosition);

        // Re-enable physics and reset velocities
        if (rb != null)
        {
            rb.isKinematic = false;
            rb.position = targetPosition;
            rb.rotation = targetRotation;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Wait for physics to stabilize
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // Remove from tracking set
        teleportingVehicles.Remove(vehicle);
    }

    private SpawnSelection SelectSpawnCandidate()
    {
        switch (spawnStrategy)
        {
            case SpawnStrategy.LaneMidpoint:
                return SelectLaneBasedSpawn();
            case SpawnStrategy.RandomPoint:
                return SelectRandomSpawn();
            default:
                return null;
        }
    }

    private SpawnSelection SelectLaneBasedSpawn()
    {
        var candidates = BuildLaneSpawnCandidates();
        if (candidates.Count == 0 || midpointSegment == null)
            return null;

        float minArrival = Mathf.Min(targetArrivalWindow.x, targetArrivalWindow.y);
        float maxArrival = Mathf.Max(targetArrivalWindow.x, targetArrivalWindow.y);
        float targetTime = UnityEngine.Random.Range(minArrival, maxArrival);
        float targetDistanceOnMidpoint = GetTargetDistanceOnMidpointLane();

        SpawnSelection best = null;
        foreach (var candidate in candidates)
        {
            float distanceToMidpoint = ComputeDistanceToMidpoint(candidate, targetDistanceOnMidpoint);
            if (!IsFinite(distanceToMidpoint))
                continue;

            bool reachedCruise;
            float estimatedTime = EstimateTime(distanceToMidpoint, out reachedCruise);
            if (!IsFinite(estimatedTime))
                continue;

            float diff = Mathf.Abs(estimatedTime - targetTime);
            if (best == null || diff < Mathf.Abs(best.EstimatedTime - best.TargetTime))
            {
                best = new SpawnSelection
                {
                    Candidate = candidate,
                    DistanceToMidpoint = distanceToMidpoint,
                    EstimatedTime = estimatedTime,
                    TargetTime = targetTime,
                    ReachedCruiseSpeed = reachedCruise
                };
            }
        }

        return best;
    }

    private SpawnSelection SelectRandomSpawn()
    {
        if (randomSpawnPoints == null || randomSpawnPoints.Count == 0)
            return null;

        int index = UnityEngine.Random.Range(0, randomSpawnPoints.Count);
        var random = randomSpawnPoints[index];
        if (random == null || random.point == null)
            return null;

        var candidate = new SpawnCandidate
        {
            Lane = null,
            DistanceFromStart = 0f,
            Position = random.point.position,
            Rotation = random.GetRotation(),
            SampleIndex = index
        };

        return new SpawnSelection
        {
            Candidate = candidate,
            DistanceToMidpoint = 0f,
            EstimatedTime = 0f,
            TargetTime = 0f,
            ReachedCruiseSpeed = false
        };
    }

    private List<SpawnCandidate> BuildLaneSpawnCandidates()
    {
        var candidates = new List<SpawnCandidate>();
        if (laneSpawnSources == null || laneSpawnSources.Count == 0)
            return candidates;

        float spacing = Mathf.Max(1f, spawnSubdivisionSpacing);
        foreach (var source in laneSpawnSources)
        {
            if (source == null)
                continue;

            if (source.lane == null)
                continue;

            var lane = source.lane;
            var cache = GetLaneCache(lane);
            if (cache == null || cache.Length <= 0.01f)
                continue;

            int sampleIndex = 0;
            float startDistance = includeSegmentEndpoints ? 0f : spacing;
            for (float d = startDistance; d <= cache.Length; d += spacing)
            {
                if (d > cache.Length && (cache.Length - d) > 0.1f)
                    break;

                float clampedD = Mathf.Min(d, cache.Length);
                var pos = GetPointAlongLane(cache, clampedD);
                var forward = GetDirectionAlongLane(cache, clampedD);
                var rotation = forward.sqrMagnitude > 0.001f ? Quaternion.LookRotation(forward, Vector3.up) : lane.transform.rotation;

                candidates.Add(new SpawnCandidate
                {
                    Lane = lane,
                    DistanceFromStart = clampedD,
                    Position = pos,
                    Rotation = rotation,
                    SampleIndex = sampleIndex++
                });

                if (Mathf.Approximately(clampedD, cache.Length))
                    break;
            }
        }

        return candidates;
    }

    private LaneCache GetLaneCache(TrafficLane lane)
    {
        if (lane == null)
            return null;

        if (laneCache.TryGetValue(lane, out var cached))
            return cached;

        var points = lane.Waypoints;
        if (points == null || points.Length < 2)
            return null;

        var cumulative = new List<float>(points.Length) { 0f };
        float total = 0f;
        for (int i = 1; i < points.Length; i++)
        {
            total += Vector3.Distance(points[i - 1], points[i]);
            cumulative.Add(total);
        }

        var entry = new LaneCache
        {
            Points = points,
            CumulativeDistances = cumulative,
            Length = total
        };
        laneCache[lane] = entry;
        return entry;
    }

    private Vector3 GetPointAlongLane(LaneCache cache, float distance)
    {
        if (cache == null || cache.Points == null || cache.Points.Length == 0)
            return Vector3.zero;

        float clamped = Mathf.Clamp(distance, 0f, cache.Length);
        for (int i = 1; i < cache.CumulativeDistances.Count; i++)
        {
            float prevDist = cache.CumulativeDistances[i - 1];
            float nextDist = cache.CumulativeDistances[i];
            if (clamped <= nextDist)
            {
                float t = nextDist > prevDist ? (clamped - prevDist) / (nextDist - prevDist) : 0f;
                return Vector3.Lerp(cache.Points[i - 1], cache.Points[i], t);
            }
        }
        return cache.Points[cache.Points.Length - 1];
    }

    private Vector3 GetDirectionAlongLane(LaneCache cache, float distance)
    {
        if (cache == null || cache.Points == null || cache.Points.Length < 2)
            return Vector3.forward;

        float clamped = Mathf.Clamp(distance, 0f, cache.Length);
        for (int i = 1; i < cache.CumulativeDistances.Count; i++)
        {
            float prevDist = cache.CumulativeDistances[i - 1];
            float nextDist = cache.CumulativeDistances[i];
            if (clamped <= nextDist)
            {
                var dir = cache.Points[i] - cache.Points[i - 1];
                if (dir.sqrMagnitude < 0.0001f)
                    return Vector3.forward;
                return dir.normalized;
            }
        }
        var tailDir = cache.Points[cache.Points.Length - 1] - cache.Points[cache.Points.Length - 2];
        return tailDir.sqrMagnitude > 0.0001f ? tailDir.normalized : Vector3.forward;
    }

    private float GetTargetDistanceOnMidpointLane()
    {
        var cache = GetLaneCache(midpointSegment);
        if (cache == null)
            return 0f;
        return Mathf.Clamp01(midpointSegmentFraction) * cache.Length;
    }

    private float ComputeDistanceToMidpoint(SpawnCandidate candidate, float targetDistanceOnMidpoint)
    {
        if (candidate == null || candidate.Lane == null || midpointSegment == null)
            return float.PositiveInfinity;

        var startCache = GetLaneCache(candidate.Lane);
        if (startCache == null)
            return float.PositiveInfinity;

        if (candidate.Lane == midpointSegment)
        {
            if (candidate.DistanceFromStart <= targetDistanceOnMidpoint)
                return targetDistanceOnMidpoint - candidate.DistanceFromStart;
            return float.PositiveInfinity;
        }

        var distances = new Dictionary<TrafficLane, float>();
        var queue = new List<KeyValuePair<TrafficLane, float>>();

        float initialCost = Mathf.Max(0f, startCache.Length - candidate.DistanceFromStart);
        distances[candidate.Lane] = initialCost;
        queue.Add(new KeyValuePair<TrafficLane, float>(candidate.Lane, initialCost));

        while (queue.Count > 0)
        {
            int bestIndex = 0;
            float bestCost = queue[0].Value;
            for (int i = 1; i < queue.Count; i++)
            {
                if (queue[i].Value < bestCost)
                {
                    bestCost = queue[i].Value;
                    bestIndex = i;
                }
            }

            var current = queue[bestIndex];
            queue.RemoveAt(bestIndex);

            var currentLane = current.Key;
            float costToLaneEnd = current.Value;

            if (currentLane == midpointSegment)
            {
                var midCache = GetLaneCache(midpointSegment);
                if (midCache == null)
                    return float.PositiveInfinity;

                float overshoot = Mathf.Max(0f, midCache.Length - targetDistanceOnMidpoint);
                return costToLaneEnd - overshoot;
            }

            foreach (var nextLane in currentLane.NextLanes)
            {
                if (nextLane == null)
                    continue;

                var nextCache = GetLaneCache(nextLane);
                if (nextCache == null)
                    continue;

                float nextCost = costToLaneEnd + nextCache.Length;
                if (!distances.TryGetValue(nextLane, out var existing) || nextCost < existing)
                {
                    distances[nextLane] = nextCost;
                    queue.Add(new KeyValuePair<TrafficLane, float>(nextLane, nextCost));
                }
            }
        }

        return float.PositiveInfinity;
    }

    private float EstimateTime(float distance, out bool reachedCruise)
    {
        reachedCruise = false;
        if (distance <= 0f)
            return 0f;

        float cruiseSpeed = Mathf.Max(0f, cruiseSpeedKmh) * (1000f / 3600f);
        float accel = Mathf.Max(0f, assumedAcceleration);

        if (cruiseSpeed <= 0f && accel <= 0f)
            return float.PositiveInfinity;

        if (accel <= 0f)
            return cruiseSpeed > 0f ? distance / cruiseSpeed : float.PositiveInfinity;

        float accelDistance = cruiseSpeed > 0f ? (cruiseSpeed * cruiseSpeed) / (2f * accel) : 0f;
        float accelTime = cruiseSpeed > 0f ? cruiseSpeed / accel : 0f;

        if (distance <= accelDistance || cruiseSpeed <= 0f)
        {
            reachedCruise = false;
            return Mathf.Sqrt(2f * distance / Mathf.Max(accel, 0.0001f));
        }

        reachedCruise = true;
        float remaining = distance - accelDistance;
        return accelTime + remaining / cruiseSpeed;
    }

    private void LogSpawnSelection(Vehicle vehicle, SpawnSelection selection, Vector3 targetPosition)
    {
        if (vehicle == null || selection == null)
            return;

        var logger = vehicle.GetComponent<LogAutonomous>();
        if (logger == null)
            return;

        string midpointName = midpointSegment != null ? midpointSegment.name : "None";
        string spawnLaneName = selection.Candidate.Lane != null ? selection.Candidate.Lane.name : "RandomPoint";

        logger.LogSpawnSelection(
            midpointName,
            spawnLaneName,
            selection.Candidate.SampleIndex,
            selection.DistanceToMidpoint,
            selection.EstimatedTime,
            selection.TargetTime,
            selection.ReachedCruiseSpeed,
            targetPosition);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
