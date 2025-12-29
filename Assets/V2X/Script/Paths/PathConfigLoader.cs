using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using AWSIM;
using System.Reflection;
using System.Linq;

namespace V2X.Paths
{
    /// <summary>Metadata attached to generated path GameObjects to allow editing type/spawn rate.</summary>
    public class PathMetadata : MonoBehaviour
    {
        public PathConfigLoader.PathUserType type;
        public float spawnPerMinute = 1f;
    }

    /// <summary>
    /// Loads pedestrian/cyclist paths from a JSON file and builds runtime WaypointSystems.
    /// </summary>
    public class PathConfigLoader : MonoBehaviour
    {
        [Serializable]
        public class PathPoint
        {
            public float x;
            public float y;
            public float z;
            public bool checkLight;
            public string trafficLightId;
        }

        public enum PathUserType
        {
            pedestrian,
            cyclist,
            both
        }

        [Serializable]
        public class PathDefinition
        {
            public string id;
            public PathUserType type;
            public float spawnPerMinute = 0f;
            public List<PathPoint> waypoints = new List<PathPoint>();
        }

        [Serializable]
        private class PathConfigFile
        {
            public List<PathDefinition> paths = new List<PathDefinition>();
        }

        [Serializable]
        public class PathDefinitionString
        {
            public string id;
            public string type;
            public float spawnPerMinute = 0f;
            public List<PathPoint> waypoints = new List<PathPoint>();
        }

        [Serializable]
        public class PathConfigFileString
        {
            public List<PathDefinitionString> paths = new List<PathDefinitionString>();
        }

        [Header("Config")]
        [Tooltip("JSON file path (relative to project or absolute) containing path definitions.")]
        [SerializeField] private string jsonConfigPath = "Assets/Configs/pedestrian_spawn.json";

        [Header("Runtime generation")]
        [Tooltip("Parent for runtime-generated WaypointSystems.")]
        [SerializeField] private Transform runtimeParent;
        [Tooltip("Create editable path GameObjects in scene (instead of hidden runtime-only)?")]
        [SerializeField] private bool instantiateEditableInScene = false;
        [Tooltip("Optional world anchor; coordinates in JSON are treated as world positions relative to this. If unset, uses runtimeParent or world origin.")]
        [SerializeField] private Transform worldOrigin;
        [Tooltip("If true, use Environment.Instance.MgrsOffsetPosition as anchor for load/save.")]
        [SerializeField] private bool useMgrsOffset = true;
        [Tooltip("Optional manual offset (world space) applied when neither Environment offset nor worldOrigin is available.")]
        [SerializeField] private Vector3 manualOffset = Vector3.zero;
        [Header("Grounding")]
        [Tooltip("If true, snap generated waypoints to the ground using a downward raycast.")]
        [SerializeField] private bool snapWaypointsToGround = true;
        [Tooltip("Layer mask used when snapping waypoints to ground.")]
        [SerializeField] private LayerMask groundMask = ~0;
        [Tooltip("Vertical raycast height above the waypoint when snapping to ground.")]
        [SerializeField] private float groundRayHeight = 3f;

        private readonly Dictionary<string, WaypointSystem> _pathsById = new Dictionary<string, WaypointSystem>();
        private PathConfigFile _loadedConfig;

        public bool HasLoaded => _loadedConfig != null && _loadedConfig.paths != null && _loadedConfig.paths.Count > 0;

        public string JsonConfigPath => jsonConfigPath;
        public void SetJsonConfigPath(string path) => jsonConfigPath = path;
        public void SetPaths(List<PathDefinition> paths)
        {
            _pathsById.Clear();
            _loadedConfig = new PathConfigFile { paths = ClonePaths(paths) };
            BuildWaypointSystems();
        }

        public IEnumerable<PathDefinition> GetPaths(PathUserType typeFilter)
        {
            if (!HasLoaded) yield break;
            foreach (var def in _loadedConfig.paths)
            {
                if (def == null) continue;
                // Only return "both" when explicitly requested; otherwise require exact match.
                if ((typeFilter == PathUserType.both && def.type == PathUserType.both) ||
                    (typeFilter != PathUserType.both && def.type == typeFilter))
                {
                    yield return def;
                }
            }
        }

        public PathDefinition GetPathDefinition(string id)
        {
            if (!HasLoaded || string.IsNullOrEmpty(id))
                return null;
            return _loadedConfig.paths.Find(p => p != null && p.id == id);
        }
        public List<PathDefinition> GetAllPaths()
        {
            return HasLoaded ? new List<PathDefinition>(_loadedConfig.paths) : new List<PathDefinition>();
        }
        public WaypointSystem GetWaypointSystem(string id)
        {
            if (string.IsNullOrEmpty(id)) return null;
            _pathsById.TryGetValue(id, out var sys);
            return sys;
        }

        private void Awake()
        {
            LoadPaths();
        }

        [ContextMenu("Load Paths JSON")]
        public void ContextLoadPaths()
        {
            LoadPaths();
        }

        [ContextMenu("Save Paths JSON")]
        public void ContextSavePaths()
        {
            SavePaths();
        }

        public void LoadPaths()
        {
            _pathsById.Clear();
            _loadedConfig = null;

            string fullPath = ResolvePath(jsonConfigPath);
            if (!File.Exists(fullPath))
            {
                Debug.LogWarning($"PathConfigLoader: Config file not found at {fullPath}");
                return;
            }

            try
            {
                var json = File.ReadAllText(fullPath);
                var configString = JsonUtility.FromJson<PathConfigFileString>(json) ?? new PathConfigFileString();
                _loadedConfig = new PathConfigFile { paths = new List<PathDefinition>() };
                if (configString.paths != null)
                {
                    foreach (var p in configString.paths)
                    {
                        if (p == null) continue;
                        PathUserType parsedType = PathUserType.both;
                        if (!string.IsNullOrEmpty(p.type))
                            Enum.TryParse(p.type, true, out parsedType);
                        var def = new PathDefinition
                        {
                            id = p.id,
                            type = parsedType,
                            spawnPerMinute = p.spawnPerMinute,
                            waypoints = p.waypoints ?? new List<PathPoint>()
                        };
                        _loadedConfig.paths.Add(def);
                    }
                }
                BuildWaypointSystems();
            }
            catch (Exception ex)
            {
                Debug.LogError($"PathConfigLoader: Failed to read config: {ex.Message}");
            }
        }

        private void BuildWaypointSystems()
        {
            if (_loadedConfig == null || _loadedConfig.paths == null)
                return;
            _pathsById.Clear();

            Transform parent = runtimeParent;
            if (parent == null)
            {
                var go = new GameObject("RuntimePaths");
                parent = go.transform;
                go.transform.position = Vector3.zero;
                go.transform.rotation = Quaternion.identity;
                go.transform.localScale = Vector3.one;
            }

            Transform anchorTransform = parent;
            Vector3 offsetVector;
            bool useOffset = TryGetOffset(out offsetVector);
            Vector3 worldOriginPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
            foreach (var def in _loadedConfig.paths)
            {
                if (def == null || def.waypoints == null || def.waypoints.Count == 0)
                    continue;

                var wsGO = new GameObject($"Path_{def.id ?? "unnamed"}");
                wsGO.transform.SetParent(parent, worldPositionStays: false);
                wsGO.transform.localPosition = Vector3.zero;
                wsGO.transform.localRotation = Quaternion.identity;
                wsGO.transform.localScale = Vector3.one;
                var ws = wsGO.AddComponent<WaypointSystem>();
                ws.loopStatus = false;
                var meta = wsGO.AddComponent<PathMetadata>();
                meta.type = def.type;
                meta.spawnPerMinute = def.spawnPerMinute;

                foreach (var pt in def.waypoints)
                {
                    var child = new GameObject("wp");
                    child.transform.SetParent(wsGO.transform, worldPositionStays: false);

                    // Convert JSON (ROS-style) to Unity local.
                    // ROS->Unity: (x,y,z)_unity = (-rosY, rosZ, rosX)
                    Vector3 rosWorld = new Vector3(pt.x, pt.y, pt.z);
                    Vector3 rosLocal = useOffset ? (rosWorld - offsetVector) : rosWorld;
                    Vector3 unityFrame = new Vector3(-rosLocal.y, rosLocal.z, rosLocal.x); // Unity world with origin at (0,0,0)
                    Vector3 unityWorld = unityFrame + worldOriginPos; // apply worldOrigin positional offset if any

                    // Optionally align to ground so pedestrians don't float or get stuck on edges.
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
                    Vector3 localPos = anchorTransform != null ? anchorTransform.InverseTransformPoint(unityWorld) : unityWorld;

                    child.transform.localPosition = localPos;
                    child.transform.localRotation = Quaternion.identity;
                    child.transform.localScale = Vector3.one;

                    if (pt.checkLight && !string.IsNullOrEmpty(pt.trafficLightId))
                    {
                        var constraint = child.AddComponent<ConstraintPedestrianTrafficLight>();
                        var tl = FindTrafficLightById(pt.trafficLightId);
                        if (tl == null)
                        {
                            Debug.LogWarning($"PathConfigLoader: Traffic light '{pt.trafficLightId}' not found for path '{def.id}'.");
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
                _pathsById[def.id] = ws;

                if (!instantiateEditableInScene)
                {
                    wsGO.hideFlags = HideFlags.HideAndDontSave;
                }
            }
        }

        private List<PathDefinition> ClonePaths(List<PathDefinition> src)
        {
            if (src == null) return new List<PathDefinition>();
            var list = new List<PathDefinition>();
            foreach (var p in src)
            {
                if (p == null) continue;
                var clone = new PathDefinition
                {
                    id = p.id,
                    type = p.type,
                    spawnPerMinute = p.spawnPerMinute,
                    waypoints = p.waypoints != null ? p.waypoints.Select(w => new PathPoint
                    {
                        x = w.x,
                        y = w.y,
                        z = w.z,
                        checkLight = w.checkLight,
                        trafficLightId = w.trafficLightId
                    }).ToList() : new List<PathPoint>()
                };
                list.Add(clone);
            }
            return list;
        }

        public void SavePaths()
        {
            // Build fresh config from current generated paths (drop removed ones).
            var newPathList = new List<PathDefinition>();
            var systemsById = new Dictionary<string, WaypointSystem>(StringComparer.OrdinalIgnoreCase);

            // Start with paths we already know about.
            foreach (var kvp in _pathsById)
            {
                var id = kvp.Key;
                var ws = kvp.Value;
                if (ws == null)
                    continue;
                if (!systemsById.ContainsKey(id))
                    systemsById.Add(id, ws);
            }

            // Also pick up any waypoint systems that were manually duplicated in the scene (e.g., copy/paste).
#if UNITY_2020_1_OR_NEWER
            var sceneSystems = runtimeParent != null
                ? runtimeParent.GetComponentsInChildren<WaypointSystem>(includeInactive: true)
                : GameObject.FindObjectsOfType<WaypointSystem>(includeInactive: true);
#else
            var sceneSystems = runtimeParent != null
                ? runtimeParent.GetComponentsInChildren<WaypointSystem>(true)
                : Resources.FindObjectsOfTypeAll<WaypointSystem>();
#endif
            foreach (var ws in sceneSystems)
            {
                if (ws == null) continue;
                var id = ExtractPathId(ws.name);
                if (string.IsNullOrEmpty(id)) continue;
                if (!systemsById.ContainsKey(id))
                    systemsById.Add(id, ws);
            }

            foreach (var kvp in systemsById)
            {
                var id = kvp.Key;
                var ws = kvp.Value;
                if (ws == null)
                    continue;

                var def = _loadedConfig.paths.Find(p => p != null && p.id == id);
                if (def == null)
                {
                    def = new PathDefinition { id = id, type = PathUserType.both, spawnPerMinute = 0f, waypoints = new List<PathPoint>() };
                }

                // Preserve existing metadata (checkLight/trafficLightId) when possible by index.
                var previous = def.waypoints ?? new List<PathPoint>();
                def.waypoints = new List<PathPoint>();

                var meta = ws.GetComponent<PathMetadata>();
                if (meta != null)
                {
                    def.type = meta.type;
                    def.spawnPerMinute = meta.spawnPerMinute;
                }

                int childCount = ws.transform.childCount;
                for (int i = 0; i < childCount; i++)
                {
                    var child = ws.transform.GetChild(i);
                    if (child == null) continue;
                    // Convert Unity local back to ROS world for saving: ros = (unity.z, -unity.x, unity.y)
                    Vector3 offsetVector;
                    bool useOffset = TryGetOffset(out offsetVector);
                    Vector3 worldOriginPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
                    Transform anchorTransformSave = ws.transform.parent;
                    Vector3 unityWorld = anchorTransformSave != null ? anchorTransformSave.TransformPoint(child.localPosition) : child.localPosition;
                    Vector3 unityFrame = unityWorld - worldOriginPos;
                    Vector3 rosLocal = new Vector3(unityFrame.z, -unityFrame.x, unityFrame.y);
                    Vector3 rosWorld = useOffset ? rosLocal + offsetVector : rosLocal;
                    Vector3 worldPos = rosWorld;
                    bool checkLight = false;
                    string tlId = string.Empty;

                    // Prefer live constraint component if present
                    var constraint = child.GetComponent<ConstraintPedestrianTrafficLight>();
                    if (constraint != null)
                    {
                        var field = typeof(ConstraintPedestrianTrafficLight).GetField("trafficLight", BindingFlags.NonPublic | BindingFlags.Instance);
                        if (field != null)
                        {
                            var tl = field.GetValue(constraint) as TrafficLight;
                            if (tl != null)
                            {
                                checkLight = true;
                                tlId = tl.name;
                            }
                        }
                    }

                    // If no light resolved, clear flags (ignore cached values).
                    if (string.IsNullOrEmpty(tlId))
                        checkLight = false;

                    def.waypoints.Add(new PathPoint { x = worldPos.x, y = worldPos.y, z = worldPos.z, checkLight = checkLight, trafficLightId = tlId });
                }

                newPathList.Add(def);
            }

            _loadedConfig = new PathConfigFile { paths = newPathList };

            string fullPath = ResolvePath(jsonConfigPath);
            var directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }

            var cfgString = new PathConfigFileString { paths = new List<PathDefinitionString>() };
            if (_loadedConfig != null && _loadedConfig.paths != null)
            {
                foreach (var def in _loadedConfig.paths)
                {
                    if (def == null) continue;
                    cfgString.paths.Add(new PathDefinitionString
                    {
                        id = def.id,
                        type = def.type.ToString(),
                        spawnPerMinute = def.spawnPerMinute,
                        waypoints = def.waypoints != null ? new List<PathPoint>(def.waypoints) : new List<PathPoint>()
                    });
                }
            }

            var json = JsonUtility.ToJson(cfgString, true);
            File.WriteAllText(fullPath, json);
            Debug.Log($"PathConfigLoader: Saved paths to {fullPath}");
        }

        private string ResolvePath(string path)
        {
            if (string.IsNullOrEmpty(path))
                return string.Empty;

            if (Path.IsPathRooted(path))
                return path;

            return Path.Combine(Application.dataPath, "..", path).Replace("\\", "/");
        }

        private string ExtractPathId(string pathName)
        {
            if (string.IsNullOrEmpty(pathName)) return string.Empty;
            return pathName.StartsWith("Path_") ? pathName.Substring("Path_".Length) : pathName;
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
                if (tl != null && tl.name.Equals(id, StringComparison.OrdinalIgnoreCase))
                    return tl;
            }
            return null;
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
