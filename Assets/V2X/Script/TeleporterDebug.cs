using UnityEngine;
using AWSIM.TrafficSimulation;
using System.Collections;
using System.Reflection;
using System.Collections.Generic;

[ExecuteInEditMode]
public class TeleporterDebug : MonoBehaviour
{
    [SerializeField] private Teleporter teleporter;

    void OnEnable()
    {
        if (teleporter == null) teleporter = FindObjectOfType<Teleporter>();
        if (teleporter == null) { Debug.LogWarning("No Teleporter found."); return; }

        var t = teleporter.GetType();
        var sourcesField = t.GetField("laneSpawnSources", BindingFlags.NonPublic | BindingFlags.Instance);
        var windowField  = t.GetField("targetArrivalWindow", BindingFlags.NonPublic | BindingFlags.Instance);
        var cruiseField  = t.GetField("cruiseSpeedKmh", BindingFlags.NonPublic | BindingFlags.Instance);
        var accelField   = t.GetField("assumedAcceleration", BindingFlags.NonPublic | BindingFlags.Instance);
        var spacingField = t.GetField("spawnSubdivisionSpacing", BindingFlags.NonPublic | BindingFlags.Instance);
        var endpointsField = t.GetField("includeSegmentEndpoints", BindingFlags.NonPublic | BindingFlags.Instance);
        var midpointField = t.GetField("midpointSegment", BindingFlags.NonPublic | BindingFlags.Instance);
        var midpointFracField = t.GetField("midpointSegmentFraction", BindingFlags.NonPublic | BindingFlags.Instance);

        var sources = sourcesField?.GetValue(teleporter) as IEnumerable;
        if (sources == null) { Debug.LogWarning("laneSpawnSources not found."); return; }

        float total = 0f, minLen = float.PositiveInfinity, maxLen = 0f;
        int lanes = 0;
        FieldInfo laneField = null;

        foreach (var src in sources)
        {
            if (src == null) continue;
            laneField ??= src.GetType().GetField("lane", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            var lane = laneField?.GetValue(src) as TrafficLane;
            if (lane == null || lane.Waypoints == null || lane.Waypoints.Length < 2) continue;

            float len = 0f;
            var pts = lane.Waypoints;
            for (int i = 1; i < pts.Length; i++) len += Vector3.Distance(pts[i - 1], pts[i]);

            lanes++;
            total   += len;
            minLen   = Mathf.Min(minLen, len);
            maxLen   = Mathf.Max(maxLen, len);
            Debug.Log($"Lane '{lane.name}' length ≈ {len:F1} m");
        }

        if (lanes == 0) { Debug.LogWarning("No lane lengths calculated."); return; }

        Debug.Log($"Total source lanes: {lanes}, combined length ≈ {total:F1} m");

        var window = windowField != null ? (Vector2)windowField.GetValue(teleporter) : Vector2.zero;
        float cruiseKmh = cruiseField != null ? Mathf.Max(0f, (float)cruiseField.GetValue(teleporter)) : 0f;
        float accel     = accelField  != null ? Mathf.Max(0f, (float)accelField.GetValue(teleporter))  : 0f;
        Debug.Log($"Configured target arrival window: {window.x:F1}–{window.y:F1} s; cruise {cruiseKmh:F1} km/h accel {accel:F1} m/s²");

        float minEta   = EstimateTime(minLen, cruiseKmh, accel);
        float maxEta   = EstimateTime(maxLen, cruiseKmh, accel);
        float totalEta = EstimateTime(total,  cruiseKmh, accel);

        Debug.Log($"ETA range (shortest→longest lane): {minEta:F1}–{maxEta:F1} s (delta {(maxEta - minEta):F1} s)");
        Debug.Log($"ETA if you drove the sum of all lane lengths: {totalEta:F1} s");

        // --- Distances to target point (midpointSegment @ fraction) ---
        var midpoint = midpointField != null ? midpointField.GetValue(teleporter) as TrafficLane : null;
        float midpointFraction = midpointFracField != null ? Mathf.Clamp01((float)midpointFracField.GetValue(teleporter)) : 0.5f;
        float spacing = spacingField != null ? Mathf.Max(1f, (float)spacingField.GetValue(teleporter)) : 10f;
        bool includeEndpoints = endpointsField != null && (bool)endpointsField.GetValue(teleporter);

        var laneCache = new Dictionary<TrafficLane, LaneCache>();
        System.Func<TrafficLane, LaneCache> getCache = lane =>
        {
            if (lane == null) return null;
            if (laneCache.TryGetValue(lane, out var c)) return c;
            var ptsLocal = lane.Waypoints;
            if (ptsLocal == null || ptsLocal.Length < 2) return null;
            var cumulative = new List<float> { 0f };
            float lenSum = 0f;
            for (int i = 1; i < ptsLocal.Length; i++)
            {
                lenSum += Vector3.Distance(ptsLocal[i - 1], ptsLocal[i]);
                cumulative.Add(lenSum);
            }
            var cache = new LaneCache { Points = ptsLocal, Cumulative = cumulative, Length = lenSum };
            laneCache[lane] = cache;
            return cache;
        };

        float targetDistanceOnMidpoint = 0f;
        var midCache = getCache(midpoint);
        if (midCache != null)
            targetDistanceOnMidpoint = midpointFraction * midCache.Length;

        float minDistToTarget = float.PositiveInfinity;
        float maxDistToTarget = 0f;
        foreach (var src in sources)
        {
            if (src == null) continue;
            var lane = laneField?.GetValue(src) as TrafficLane;
            var cache = getCache(lane);
            if (cache == null || cache.Length <= 0.01f) continue;

            float start = includeEndpoints ? 0f : spacing;
            for (float d = start; d <= cache.Length + 0.01f; d += spacing)
            {
                float clamped = Mathf.Min(d, cache.Length);
                float distToMid = ComputeDistanceToMidpoint(lane, clamped, midpoint, targetDistanceOnMidpoint, getCache);
                if (float.IsInfinity(distToMid) || float.IsNaN(distToMid)) continue;
                minDistToTarget = Mathf.Min(minDistToTarget, distToMid);
                maxDistToTarget = Mathf.Max(maxDistToTarget, distToMid);
                if (Mathf.Approximately(clamped, cache.Length)) break;
            }
        }

        if (float.IsInfinity(minDistToTarget))
        {
            Debug.LogWarning("Could not compute distance to midpoint (check midpointSegment and lane graph).");
        }
        else
        {
            float minEtaToTarget = EstimateTime(minDistToTarget, cruiseKmh, accel);
            float maxEtaToTarget = EstimateTime(maxDistToTarget, cruiseKmh, accel);
            Debug.Log($"ETA to target point (closest→farthest spawn): {minEtaToTarget:F1}–{maxEtaToTarget:F1} s (delta {(maxEtaToTarget - minEtaToTarget):F1} s)");
        }
    }

    private static float EstimateTime(float distance, float cruiseKmh, float accel)
    {
        distance = Mathf.Max(0f, distance);
        float cruise = cruiseKmh * (1000f / 3600f);
        accel = Mathf.Max(0f, accel);

        if (distance <= 0f) return 0f;
        if (cruise <= 0f && accel <= 0f) return float.PositiveInfinity;
        if (accel <= 0f) return cruise > 0f ? distance / cruise : float.PositiveInfinity;

        float accelDist = cruise > 0f ? (cruise * cruise) / (2f * accel) : 0f;
        float accelTime = cruise > 0f ? cruise / accel : 0f;

        if (distance <= accelDist || cruise <= 0f)
            return Mathf.Sqrt(2f * distance / Mathf.Max(accel, 0.0001f));

        float remaining = distance - accelDist;
        return accelTime + remaining / cruise;
    }

    private class LaneCache
    {
        public Vector3[] Points;
        public List<float> Cumulative;
        public float Length;
    }

    private float ComputeDistanceToMidpoint(TrafficLane startLane, float startDist, TrafficLane midpoint, float targetDistOnMid, System.Func<TrafficLane, LaneCache> cacheGetter)
    {
        if (startLane == null || midpoint == null) return float.PositiveInfinity;
        var startCache = cacheGetter(startLane);
        var midCache = cacheGetter(midpoint);
        if (startCache == null || midCache == null) return float.PositiveInfinity;

        if (startLane == midpoint)
        {
            if (startDist <= targetDistOnMid)
                return targetDistOnMid - startDist;
            return float.PositiveInfinity;
        }

        var distances = new Dictionary<TrafficLane, float>();
        var queue = new List<KeyValuePair<TrafficLane, float>>();

        float initialCost = Mathf.Max(0f, startCache.Length - startDist);
        distances[startLane] = initialCost;
        queue.Add(new KeyValuePair<TrafficLane, float>(startLane, initialCost));

        while (queue.Count > 0)
        {
            int bestIdx = 0;
            float bestCost = queue[0].Value;
            for (int i = 1; i < queue.Count; i++)
            {
                if (queue[i].Value < bestCost)
                {
                    bestCost = queue[i].Value;
                    bestIdx = i;
                }
            }

            var current = queue[bestIdx];
            queue.RemoveAt(bestIdx);

            var lane = current.Key;
            float costToEnd = current.Value;

            if (lane == midpoint)
            {
                float overshoot = Mathf.Max(0f, midCache.Length - targetDistOnMid);
                return costToEnd - overshoot;
            }

            foreach (var next in lane.NextLanes)
            {
                if (next == null) continue;
                var nextCache = cacheGetter(next);
                if (nextCache == null) continue;

                float nextCost = costToEnd + nextCache.Length;
                if (!distances.TryGetValue(next, out var existing) || nextCost < existing)
                {
                    distances[next] = nextCost;
                    queue.Add(new KeyValuePair<TrafficLane, float>(next, nextCost));
                }
            }
        }

        return float.PositiveInfinity;
    }
}
