using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace V2X.Paths
{
    /// <summary>
    /// Spawns cyclist agents onto paths loaded from PathConfigLoader.
    /// </summary>
    public class CyclistPathSpawner : MonoBehaviour
    {
        [Header("Dependencies")]
        [SerializeField] private PathConfigLoader pathConfig;

        [Header("Prefabs & Pooling")]
        [Tooltip("Cyclist prefabs used for spawning (must include WaypointFollower).")]
        [SerializeField] private List<Transform> cyclistPrefabs = new List<Transform>();
        [Tooltip("Optional parent for spawned cyclists.")]
        [SerializeField] private Transform activeParent;
        [Tooltip("Max extra clones we create per spawn if pool runs out.")]
        [SerializeField] private int maxExtraInstancesPerSpawn = 3;

        [Header("Spawn Control")]
        [Tooltip("Maximum concurrently active cyclists. 0 = unlimited.")]
        [SerializeField] private int maxActive = 0;
        [Tooltip("Initial delay range before starting spawn loop (seconds).")]
        [SerializeField] private Vector2 initialSpawnDelayRange = new Vector2(0f, 2f);

        private readonly List<Transform> _pooled = new List<Transform>();
        private readonly List<Coroutine> _activeCoroutines = new List<Coroutine>();
        private int _spawnCount = 0;

        private void Awake()
        {
            if (pathConfig == null)
            {
                pathConfig = FindObjectOfType<PathConfigLoader>();
            }
        }

        private void Start()
        {
            if (pathConfig == null || !pathConfig.HasLoaded)
            {
                Debug.LogWarning("[CyclistPathSpawner] No path config loaded; nothing will spawn.");
                return;
            }

            foreach (var path in pathConfig.GetPaths(PathConfigLoader.PathUserType.cyclist))
            {
                StartPathRoutine(path);
            }
        }

        private void OnDestroy()
        {
            foreach (var co in _activeCoroutines)
            {
                if (co != null) StopCoroutine(co);
            }
            _activeCoroutines.Clear();
        }

        private void StartPathRoutine(PathConfigLoader.PathDefinition path)
        {
            if (path == null || path.spawnPerMinute <= 0f)
                return;

            var ws = pathConfig.GetWaypointSystem(path.id);
            if (ws == null)
            {
                Debug.LogWarning($"[CyclistPathSpawner] Waypoint system missing for path {path.id}");
                return;
            }

            float interval = Mathf.Max(0.1f, 60f / path.spawnPerMinute);
            var routine = StartCoroutine(SpawnLoop(ws, interval));
            _activeCoroutines.Add(routine);
        }

        private IEnumerator SpawnLoop(WaypointSystem path, float baseInterval)
        {
            float initDelay = Mathf.Clamp(Random.Range(initialSpawnDelayRange.x, initialSpawnDelayRange.y), 0f, 999f);
            if (initDelay <= 0f)
            {
                // Add a per-path phase offset up to one interval to further de-sync spawns.
                initDelay = Random.Range(0f, baseInterval);
            }
            if (initDelay > 0f)
                yield return new WaitForSeconds(initDelay);
            while (true)
            {
                if (maxActive > 0 && CountActive() >= maxActive)
                {
                    yield return new WaitForSeconds(baseInterval);
                    continue;
                }

                SpawnAgent(path);
                _spawnCount++;

                // Sample a near-mean interval (Gaussian-ish, clamped to 0.5x–1.5x mean) to keep timing close to target rate.
                float next = SampleNearMeanInterval(baseInterval);
                Debug.Log($"[CyclistPathSpawner] Spawn Cyclist #{_spawnCount} on path '{path.name}' at t={Time.time:F2}s (next wait {next:F2}s)");
                yield return new WaitForSeconds(next);
            }
        }

        private int CountActive()
        {
            int count = 0;
            for (int i = 0; i < _pooled.Count; i++)
            {
                if (_pooled[i] != null && _pooled[i].gameObject.activeSelf)
                    count++;
            }
            return count;
        }

        private void SpawnAgent(WaypointSystem path)
        {
            var agent = GetOrCreateAgent();
            if (agent == null)
            {
                Debug.LogWarning("[CyclistPathSpawner] Could not obtain an agent to spawn.");
                return;
            }

            var follower = agent.GetComponent<WaypointFollower>();
            if (follower == null)
            {
                Debug.LogWarning($"[CyclistPathSpawner] Missing WaypointFollower on {agent.name}");
                return;
            }

            follower.SetWaypointSystem(path);
            follower.ForceStartAtFirstWaypoint();

            if (activeParent != null)
                agent.SetParent(activeParent, true);

            agent.gameObject.SetActive(true);
        }

        private Transform GetOrCreateAgent()
        {
            for (int i = 0; i < _pooled.Count; i++)
            {
                var t = _pooled[i];
                if (t != null && !t.gameObject.activeSelf)
                    return t;
            }

            if (maxExtraInstancesPerSpawn <= 0 || cyclistPrefabs.Count == 0)
                return null;

            int allowed = maxExtraInstancesPerSpawn;
            while (allowed-- > 0)
            {
                var template = cyclistPrefabs[Random.Range(0, cyclistPrefabs.Count)];
                if (template == null) continue;

                var clone = Instantiate(template.gameObject).transform;
                clone.gameObject.SetActive(false);
                clone.name = template.name + "_clone";
                _pooled.Add(clone);
                return clone;
            }

            return null;
        }

        private string ExtractPathId(string pathName)
        {
            if (string.IsNullOrEmpty(pathName)) return string.Empty;
            return pathName.StartsWith("Path_") ? pathName.Substring("Path_".Length) : pathName;
        }

        private float SampleNearMeanInterval(float mean)
        {
            // Box-Muller to sample a normal-ish value; clamp to limit extremes but bias near the mean.
            float u1 = Mathf.Max(float.Epsilon, Random.value);
            float u2 = Random.value;
            float z0 = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
            float std = mean * 0.1f; // 10% std dev around mean interval
            float sample = mean + z0 * std;
            return Mathf.Clamp(sample, mean * 0.5f, mean * 1.5f);
        }
    }
}
