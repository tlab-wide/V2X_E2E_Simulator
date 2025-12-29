using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWSIM;

namespace V2X.Paths
{
    /// <summary>
    /// Spawns pedestrians and cyclists using a single spawner based on PathConfigLoader definitions.
    /// </summary>
    public class PedestrianCyclistSpawner : MonoBehaviour
    {
        [Header("Prefabs & Pooling")]
        [Tooltip("Pedestrian prefabs used for spawning (must include WaypointFollower).")]
        [SerializeField] private List<Transform> pedestrianPrefabs = new List<Transform>();
        [Tooltip("Cyclist prefabs used for spawning (must include WaypointFollower).")]
        [SerializeField] private List<Transform> cyclistPrefabs = new List<Transform>();
        [Tooltip("Optional parent for spawned agents.")]
        [SerializeField] private Transform activeParent;
        [Tooltip("Max extra clones we create per spawn if pool runs out.")]
        [SerializeField] private int maxExtraInstancesPerSpawn = 5;

        [Header("Spawn Control")]
        [Tooltip("Maximum concurrently active agents. 0 = unlimited.")]
        [SerializeField] private int maxActive = 0;
        [Tooltip("Initial delay range before starting spawn loop (seconds).")]
        [SerializeField] private Vector2 initialSpawnDelayRange = new Vector2(0f, 2f);
        [Header("Coordinate Conversion")]
        [Tooltip("Optional world anchor; coordinates in JSON are treated as world positions relative to this. If unset, uses world origin.")]
        [SerializeField] private Transform worldOrigin;
        [Tooltip("If true, use Environment.Instance.MgrsOffsetPosition as anchor for load/save.")]
        [SerializeField] private bool useMgrsOffset = true;
        [Tooltip("Optional manual offset (world space) applied when neither Environment offset nor worldOrigin is available.")]
        [SerializeField] private Vector3 manualOffset = Vector3.zero;
        [Tooltip("If true, snap generated waypoints to the ground using a downward raycast.")]
        [SerializeField] private bool snapWaypointsToGround = true;
        [Tooltip("Layer mask used when snapping waypoints to ground.")]
        [SerializeField] private LayerMask groundMask = ~0;
        [Tooltip("Vertical raycast height above the waypoint when snapping to ground.")]
        [SerializeField] private float groundRayHeight = 3f;

        private readonly List<Transform> _pooled = new List<Transform>();
        private readonly List<Coroutine> _activeCoroutines = new List<Coroutine>();
        private int _spawnCount = 0;
        [Header("NPC Paths (loaded from JSON)")]
        [SerializeField] private List<PathConfigLoader.PathDefinition> npcPaths = new List<PathConfigLoader.PathDefinition>();
        [SerializeField, Tooltip("JSON path for pedestrian/cyclist paths.")]
        private string pathsJsonPath = "Assets/Configs/pedestrian_spawn.json";

        private readonly Dictionary<string, WaypointSystem> _waypointsById = new Dictionary<string, WaypointSystem>();
        private Transform _runtimeParent;

        private void Start()
        {
            LoadNpcPaths();
            Debug.Log($"[PedestrianCyclistSpawner] Loaded {npcPaths.Count} paths from '{pathsJsonPath}'.");
        }

        private void OnDestroy()
        {
            foreach (var co in _activeCoroutines)
            {
                if (co != null) StopCoroutine(co);
            }
            _activeCoroutines.Clear();
        }

        [ContextMenu("Load NPC Paths From JSON")]
        public void LoadNpcPaths()
        {
            StopAllSpawns();
            LoadFromJson();
            BuildWaypointSystems();
            Debug.Log($"[PedestrianCyclistSpawner] Built {_waypointsById.Count} waypoint systems.");
            _spawnCount = 0;
            StartSpawns();
        }

        [ContextMenu("Save NPC Paths To JSON")]
        public void SaveNpcPaths()
        {
            SaveToJson();
            Debug.Log("[PedestrianCyclistSpawner] Saved NPC paths to JSON.");
        }

        private void StartSpawns()
        {
            if (npcPaths == null || npcPaths.Count == 0)
            {
                Debug.LogWarning("[PedestrianCyclistSpawner] No NPC paths loaded; nothing will spawn.");
                return;
            }

            foreach (var path in npcPaths)
            {
                if (path == null) continue;
                var type = path.type;
                if (string.IsNullOrEmpty(path.id))
                {
                    Debug.LogWarning("[PedestrianCyclistSpawner] Skipping path with empty id.");
                    continue;
                }
                if (path.spawnPerMinute <= 0f)
                {
                    Debug.LogWarning($"[PedestrianCyclistSpawner] Skipping path '{path.id}' due to spawnPerMinute <= 0.");
                    continue;
                }
                Debug.Log($"[PedestrianCyclistSpawner] Starting path '{path.id}' type {type} with {path.spawnPerMinute} spm.");
                StartPathRoutine(path, type);
            }
        }

        private void StopAllSpawns()
        {
            foreach (var co in _activeCoroutines)
            {
                if (co != null) StopCoroutine(co);
            }
            _activeCoroutines.Clear();
        }

        private void LoadFromJson()
        {
            npcPaths = new List<PathConfigLoader.PathDefinition>();
            if (string.IsNullOrEmpty(pathsJsonPath))
                return;
            string fullPath = ResolvePath(pathsJsonPath);
            if (!System.IO.File.Exists(fullPath))
            {
                Debug.LogWarning($"[PedestrianCyclistSpawner] Paths JSON not found at {fullPath}");
                return;
            }
            try
            {
                var json = System.IO.File.ReadAllText(fullPath);
                var cfg = JsonUtility.FromJson<PathConfigLoader.PathConfigFileString>(json) ?? new PathConfigLoader.PathConfigFileString();
                if (cfg.paths != null)
                {
                    foreach (var p in cfg.paths)
                    {
                        if (p == null) continue;
                        PathConfigLoader.PathUserType parsedType = PathConfigLoader.PathUserType.both;
                        if (!string.IsNullOrEmpty(p.type))
                            System.Enum.TryParse(p.type, true, out parsedType);
                        npcPaths.Add(new PathConfigLoader.PathDefinition
                        {
                            id = p.id,
                            type = parsedType,
                            spawnPerMinute = p.spawnPerMinute,
                            waypoints = p.waypoints ?? new List<PathConfigLoader.PathPoint>()
                        });
                    }
                    Debug.Log($"[PedestrianCyclistSpawner] Parsed {npcPaths.Count} paths from JSON.");
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[PedestrianCyclistSpawner] Failed to load paths: {ex.Message}");
            }
        }

        private void SaveToJson()
        {
            var cfg = new PathConfigLoader.PathConfigFileString { paths = new List<PathConfigLoader.PathDefinitionString>() };
            foreach (var p in npcPaths)
            {
                if (p == null) continue;
                cfg.paths.Add(new PathConfigLoader.PathDefinitionString
                {
                    id = p.id,
                    type = p.type.ToString(),
                    spawnPerMinute = p.spawnPerMinute,
                    waypoints = p.waypoints != null ? new List<PathConfigLoader.PathPoint>(p.waypoints) : new List<PathConfigLoader.PathPoint>()
                });
            }
            string fullPath = ResolvePath(pathsJsonPath);
            var dir = System.IO.Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(dir) && !System.IO.Directory.Exists(dir))
                System.IO.Directory.CreateDirectory(dir);
            var json = JsonUtility.ToJson(cfg, true);
            System.IO.File.WriteAllText(fullPath, json);
        }

        private void StartPathRoutine(PathConfigLoader.PathDefinition path, PathConfigLoader.PathUserType type)
        {
            if (path == null || path.spawnPerMinute <= 0f)
                return;

            var ws = GetWaypointSystem(path.id);
            if (ws == null)
            {
                Debug.LogWarning($"[PedestrianCyclistSpawner] Waypoint system missing for path {path.id}");
                return;
            }

            float interval = Mathf.Max(0.1f, 60f / path.spawnPerMinute);
            var routine = StartCoroutine(SpawnLoop(ws, interval, type));
            _activeCoroutines.Add(routine);
            Debug.Log($"[PedestrianCyclistSpawner] Spawn loop started for path '{path.id}' interval {interval:F2}s");
        }

        private IEnumerator SpawnLoop(WaypointSystem path, float baseInterval, PathConfigLoader.PathUserType type)
        {
            float initDelay = Mathf.Clamp(Random.Range(initialSpawnDelayRange.x, initialSpawnDelayRange.y), 0f, 999f);
            if (initDelay <= 0f)
            {
                initDelay = Random.Range(0f, baseInterval);
            }
            Debug.Log($"[PedestrianCyclistSpawner] Path '{path.name}' type {type} initial delay {initDelay:F2}s (timeScale {Time.timeScale}).");
            if (initDelay > 0f)
                yield return new WaitForSecondsRealtime(initDelay);
            Debug.Log($"[PedestrianCyclistSpawner] Path '{path.name}' type {type} entering loop after init delay.");

            while (true)
            {
                int activeCount = CountActive();
                Debug.Log($"[PedestrianCyclistSpawner] Loop path '{path.name}' type {type} active={activeCount} max={maxActive}.");

                if (maxActive > 0 && activeCount >= maxActive)
                {
                    Debug.Log($"[PedestrianCyclistSpawner] MaxActive reached ({maxActive}); waiting.");
                    yield return new WaitForSeconds(baseInterval);
                    continue;
                }

                SpawnAgent(path, type);
                _spawnCount++;

                float next = SampleNearMeanInterval(baseInterval);
                Debug.Log($"[PedestrianCyclistSpawner] Spawn #{_spawnCount} on path '{path.name}' type {type} at t={Time.time:F2}s (next wait {next:F2}s)");
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

        private void SpawnAgent(WaypointSystem path, PathConfigLoader.PathUserType type)
        {
            var agent = GetOrCreateAgent(type);
            if (agent == null)
            {
                Debug.LogWarning("[PedestrianCyclistSpawner] Could not obtain an agent to spawn (pool empty or no prefabs).");
                return;
            }

            var follower = agent.GetComponent<WaypointFollower>();
            if (follower == null)
            {
                Debug.LogWarning($"[PedestrianCyclistSpawner] Missing WaypointFollower on {agent.name}");
                return;
            }

            follower.SetWaypointSystem(path);
            follower.ForceStartAtFirstWaypoint();

            if (activeParent != null)
                agent.SetParent(activeParent, true);

            agent.gameObject.SetActive(true);
            Debug.Log($"[PedestrianCyclistSpawner] Activated agent '{agent.name}' for type {type} on path '{path.name}'.");
        }

        private Transform GetOrCreateAgent(PathConfigLoader.PathUserType type)
        {
            for (int i = 0; i < _pooled.Count; i++)
            {
                var t = _pooled[i];
                if (t != null && !t.gameObject.activeSelf)
                    return t;
            }

            var sourceList = BuildSourceList(type);
            if (sourceList.Count == 0)
            {
                Debug.LogWarning($"[PedestrianCyclistSpawner] No prefabs available for type {type}; cannot spawn.");
                return null;
            }
            if (maxExtraInstancesPerSpawn <= 0)
            {
                Debug.LogWarning($"[PedestrianCyclistSpawner] Pool empty and maxExtraInstancesPerSpawn <= 0; cannot spawn type {type}.");
                return null;
            }

            int allowed = maxExtraInstancesPerSpawn;
            while (allowed-- > 0)
            {
                var template = sourceList[Random.Range(0, sourceList.Count)];
                if (template == null) continue;

                var clone = Instantiate(template.gameObject).transform;
                clone.gameObject.SetActive(false);
                clone.name = template.name + "_clone";
                _pooled.Add(clone);
                return clone;
            }

            return null;
        }

        private List<Transform> BuildSourceList(PathConfigLoader.PathUserType type)
        {
            var list = new List<Transform>();
            if (type == PathConfigLoader.PathUserType.pedestrian || type == PathConfigLoader.PathUserType.both)
                list.AddRange(pedestrianPrefabs);
            if (type == PathConfigLoader.PathUserType.cyclist || type == PathConfigLoader.PathUserType.both)
                list.AddRange(cyclistPrefabs);
            // remove nulls
            for (int i = list.Count - 1; i >= 0; i--)
            {
                if (list[i] == null) list.RemoveAt(i);
            }
            return list;
        }

        private float SampleNearMeanInterval(float mean)
        {
            float u1 = Mathf.Max(float.Epsilon, Random.value);
            float u2 = Random.value;
            float z0 = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
            float std = mean * 0.1f;
            float sample = mean + z0 * std;
            return Mathf.Clamp(sample, mean * 0.5f, mean * 1.5f);
        }

        private void BuildWaypointSystems()
        {
            if (_runtimeParent != null)
            {
                DestroyImmediate(_runtimeParent.gameObject);
                _runtimeParent = null;
            }
            _waypointsById.Clear();

            if (npcPaths == null || npcPaths.Count == 0)
                return;

            _runtimeParent = new GameObject("RuntimePaths_PedCyclist").transform;
            _runtimeParent.hideFlags = HideFlags.HideAndDontSave | HideFlags.HideInHierarchy;
            foreach (var def in npcPaths)
            {
                if (def == null || def.waypoints == null || def.waypoints.Count == 0)
                    continue;

                var wsGO = new GameObject($"Path_{def.id ?? "unnamed"}");
                wsGO.transform.SetParent(_runtimeParent, worldPositionStays: false);
                wsGO.transform.localPosition = Vector3.zero;
                wsGO.transform.localRotation = Quaternion.identity;
                wsGO.transform.localScale = Vector3.one;
                wsGO.hideFlags = HideFlags.HideAndDontSave | HideFlags.HideInHierarchy;
                var ws = wsGO.AddComponent<WaypointSystem>();
                ws.loopStatus = false;
                var meta = wsGO.AddComponent<PathMetadata>();
                meta.type = def.type;
                meta.spawnPerMinute = def.spawnPerMinute;

                foreach (var pt in def.waypoints)
                {
                    var child = new GameObject("wp");
                    child.transform.SetParent(wsGO.transform, worldPositionStays: false);
                    child.transform.localPosition = ToUnityPosition(pt);
                    child.transform.localRotation = Quaternion.identity;
                    child.transform.localScale = Vector3.one;
                    child.hideFlags = HideFlags.HideAndDontSave | HideFlags.HideInHierarchy;

                    if (pt.checkLight && !string.IsNullOrEmpty(pt.trafficLightId))
                    {
                        var constraint = child.AddComponent<ConstraintPedestrianTrafficLight>();
                        var tl = FindTrafficLightById(pt.trafficLightId);
                        if (tl == null)
                        {
                            Debug.LogWarning($"[PedestrianCyclistSpawner] Traffic light '{pt.trafficLightId}' not found for path '{def.id}'.");
                        }
                        else
                        {
                            var field = typeof(ConstraintPedestrianTrafficLight).GetField("trafficLight", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                            if (field != null)
                                field.SetValue(constraint, tl);
                        }
                    }

                    var node = new WaypointSystem.WaypointNode { waypoint = child.transform };
                    ws.waypoints.Add(node);
                }
                _waypointsById[def.id] = ws;
            }
        }

        private WaypointSystem GetWaypointSystem(string id)
        {
            if (string.IsNullOrEmpty(id)) return null;
            _waypointsById.TryGetValue(id, out var ws);
            return ws;
        }

        private string ResolvePath(string path)
        {
            if (string.IsNullOrEmpty(path))
                return string.Empty;
            if (System.IO.Path.IsPathRooted(path))
                return path;
            return System.IO.Path.Combine(Application.dataPath, "..", path).Replace("\\", "/");
        }

        private TrafficLight FindTrafficLightById(string id)
        {
            if (string.IsNullOrEmpty(id)) return null;
#if UNITY_2020_1_OR_NEWER
            var lights = GameObject.FindObjectsOfType<TrafficLight>(includeInactive: true);
#else
            var lights = Resources.FindObjectsOfTypeAll<TrafficLight>();
#endif
            foreach (var tl in lights)
            {
                if (tl != null && tl.name.Equals(id, System.StringComparison.OrdinalIgnoreCase))
                    return tl;
            }
            return null;
        }

        public Vector3 ToUnityPosition(PathConfigLoader.PathPoint pt)
        {
            Vector3 rosWorld = new Vector3(pt.x, pt.y, pt.z);
            Vector3 offset;
            bool useOffset = TryGetOffset(out offset);
            Vector3 rosLocal = useOffset ? (rosWorld - offset) : rosWorld;
            Vector3 unityFrame = new Vector3(-rosLocal.y, rosLocal.z, rosLocal.x);
            Vector3 worldOriginPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
            Vector3 unityWorld = unityFrame + worldOriginPos;

            if (snapWaypointsToGround)
            {
                RaycastHit hit;
                Vector3 origin = unityWorld + Vector3.up * Mathf.Max(0.1f, groundRayHeight);
                float maxDist = Mathf.Max(0.2f, groundRayHeight * 2f);
                if (Physics.Raycast(origin, Vector3.down, out hit, maxDist, groundMask, QueryTriggerInteraction.Ignore))
                {
                    unityWorld.y = hit.point.y;
                }
            }

            return unityWorld;
        }

        public void UpdatePathPointFromUnity(ref PathConfigLoader.PathPoint pt, Vector3 unityWorld)
        {
            Vector3 worldOriginPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
            Vector3 unityFrame = unityWorld - worldOriginPos;
            Vector3 rosLocal = new Vector3(unityFrame.z, -unityFrame.x, unityFrame.y);
            Vector3 offset;
            bool useOffset = TryGetOffset(out offset);
            Vector3 rosWorld = useOffset ? rosLocal + offset : rosLocal;
            pt.x = rosWorld.x;
            pt.y = rosWorld.y;
            pt.z = rosWorld.z;
        }

        private bool TryGetOffset(out Vector3 offset)
        {
            offset = Vector3.zero;
            if (useMgrsOffset)
            {
                if (AWSIM.Environment.Instance != null)
                {
                    offset = AWSIM.Environment.Instance.MgrsOffsetPosition;
                    return true;
                }
                if (manualOffset != Vector3.zero)
                {
                    offset = manualOffset;
                    return true;
                }
            }
            if (worldOrigin != null)
            {
                offset = worldOrigin.position;
                return true;
            }
            return false;
        }
    }
}
