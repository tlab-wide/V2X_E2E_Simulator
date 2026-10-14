using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Component for managing traffic simulators. Traffic manager collets all traffic simulators and manages the spawning process.
    /// - Reproducibility by Seed value
    /// </summary>
    public class TrafficManager : MonoBehaviour
    {
        [SerializeField, Tooltip("Seed value for random generator.")]
        public int seed;
        [Header("NPC Vehicle Settings")]
        [SerializeField] private NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();

        [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        private LayerMask vehicleLayerMask;

        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        private LayerMask groundLayerMask;

        [SerializeField, Tooltip("A maximum number of vehicles that can simultaneously live in the scene. Lowering this value results in less dense traffic but improves the simulator's performance.")]
        public int maxVehicleCount = 40;

        [SerializeField, Tooltip("A minimal distance between the EGO and the NPC to spawn")]
        private float spawnDistanceToEgo = 50.0f;

        [SerializeField, Tooltip("Ego vehicle handler. If not set, the manager creates a dummy ego. This reference is also set automatically when the Ego spawns via the traffic simulator.")]
        private GameObject _egoVehicle;

        [SerializeField, Tooltip("TrafficLanes parent GameObject. This should contain all the TrafficLane components in the scene.")]
        private GameObject trafficLanesObject;

        public GameObject egoVehicle
        {
            get
            {
                return _egoVehicle;
            }
            set
            {
                _egoVehicle = value;
                if (_egoVehicle != null)
                {
                    npcVehicleSimulator.RegisterEgo(value);
                }
                else
                {
                    npcVehicleSimulator.UnregisterEgo();
                    _egoVehicle = dummyEgo;
                }
            }
        }

        [Header("Debug")]
        [SerializeField] protected bool showGizmos = false;
        [SerializeField] protected bool showYieldingPhase = false;
        [SerializeField] protected bool showObstacleChecking = false;
        [SerializeField] protected bool showSpawnPoints = false;
        public RandomTrafficSimulatorConfiguration[] randomTrafficSims;
        public RouteTrafficSimulatorConfiguration[] routeTrafficSims;
        public NPCVehicleSimulator npcVehicleSimulator;
        private List<ITrafficSimulator> trafficSimulatorNodes;
        private Dictionary<NPCVehicleSpawnPoint, Dictionary<ITrafficSimulator, GameObject>> spawnLanes;
        private GameObject dummyEgo;
        [Header("Spawn Config (optional)")]
        [SerializeField, Tooltip("JSON path for saving/loading spawn config.")]
        private string spawnConfigPath = "Assets/Configs/traffic_spawn.json";
        private readonly Dictionary<string, float> lastSpawnTimeByLane = new Dictionary<string, float>(System.StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, float> spmByLane = new Dictionary<string, float>(System.StringComparer.OrdinalIgnoreCase);

        public string GetSpawnConfigPath() => spawnConfigPath;

        // ------------------------------------------------------------------------------ external (config file) setup
        /// <summary>Traffic set up from a config file instead of the inspector (see <see cref="ApplyExternalConfig"/>).</summary>
        public class ExternalConfig
        {
            public int seed;
            public int maxVehicleCount = 60;
            public float spawnDistanceToEgo = 50f;
            public float spawnClearance = 4f;
            public int maximumSpawns;                                     // 0 = unlimited
            public List<GameObject> prefabs = new List<GameObject>();
            public List<float> prefabWeights = new List<float>();
            public List<TrafficLane> spawnLanes = new List<TrafficLane>();
            public List<float> vehiclesPerMinute = new List<float>();
            public List<RandomTrafficSimulatorConfiguration.BranchWeightSet> branches = new List<RandomTrafficSimulatorConfiguration.BranchWeightSet>();
            public NPCTrafficBehaviour behaviour = NPCTrafficBehaviour.Default();
        }

        private class ScheduledSpawnPoint
        {
            public NPCVehicleSpawnPoint point;
            public string laneName;
            public float interval;
            public float nextTime;
            public GameObject pendingPrefab;
            public int spawned;
        }

        private ExternalConfig externalConfig;
        private List<ScheduledSpawnPoint> schedule;

        /// <summary>
        /// Replaces the inspector setup (call before Start): one random traffic simulator with the given spawn lanes,
        /// each spawning at its own rate, weighted vehicle types and branch ratios. The spawn JSON is not read.
        /// </summary>
        public void ApplyExternalConfig(ExternalConfig config)
        {
            externalConfig = config;
            seed = config.seed;
            maxVehicleCount = config.maxVehicleCount;
            spawnDistanceToEgo = config.spawnDistanceToEgo;
            spawnConfigPath = "";
            var lanes = new RandomTrafficSimulatorConfiguration.SpawnableLaneConfig[config.spawnLanes.Count];
            for (int i = 0; i < lanes.Length; i++)
                lanes[i] = new RandomTrafficSimulatorConfiguration.SpawnableLaneConfig { lane = config.spawnLanes[i], spawnsPerMinute = config.vehiclesPerMinute[i] };
            randomTrafficSims = new[]
            {
                new RandomTrafficSimulatorConfiguration
                {
                    npcPrefabs = config.prefabs.ToArray(), spawnableLanes = lanes, branchWeights = config.branches.ToArray(),
                    maximumSpawns = config.maximumSpawns, enabled = true,
                },
            };
            routeTrafficSims = new RouteTrafficSimulatorConfiguration[0];
        }

        private GameObject PickPrefab()
        {
            var c = externalConfig;
            float total = 0f;
            for (int i = 0; i < c.prefabs.Count; i++) total += Mathf.Max(0f, c.prefabWeights[i]);
            if (total <= 0f) return c.prefabs[UnityEngine.Random.Range(0, c.prefabs.Count)];
            float r = UnityEngine.Random.value * total;
            for (int i = 0; i < c.prefabs.Count; i++)
            {
                r -= Mathf.Max(0f, c.prefabWeights[i]);
                if (r <= 0f) return c.prefabs[i];
            }
            return c.prefabs[c.prefabs.Count - 1];
        }

        /// <summary>Each spawn point spawns at its own rate; a blocked spawn is retried on the next step.</summary>
        private void ScheduledSpawn()
        {
            var sim = trafficSimulatorNodes.OfType<RandomTrafficSimulator>().FirstOrDefault();
            if (sim == null || !sim.IsEnabled() || externalConfig.prefabs.Count == 0)
                return;
            foreach (var sp in schedule)
            {
                if (sp.interval <= 0f || Time.time < sp.nextTime)
                    continue;
                if (GeometryUtility.Distance2D(_egoVehicle.transform.position, sp.point.Position) < spawnDistanceToEgo)
                    continue;
                sp.pendingPrefab ??= PickPrefab();     // fixed until spawned, so big vehicles are not starved
                if (!NPCVehicleSpawner.IsSpawnable(sp.pendingPrefab.GetComponent<NPCVehicle>().Bounds, sp.point, externalConfig.spawnClearance))
                    continue;
                if (!sim.Spawn(sp.pendingPrefab, sp.point, out var vehicle))
                    continue;
                sp.pendingPrefab = null;
                sp.spawned++;
                sp.nextTime = Time.time + sp.interval * UnityEngine.Random.Range(0.75f, 1.25f);
                if (vehicle && vehicle.gameObject.tag == "Ego")
                    npcVehicleSimulator.EGOVehicle = vehicle.transform;
            }
        }

        [ContextMenu("Save Spawn Config JSON")]
        public void SaveSpawnConfigJson()
        {
            var data = BuildDataFromTrafficSims();
            if (data == null) return;

            string json = JsonUtility.ToJson(data, true);
            string fullPath = ResolveJsonPath();
            var dir = System.IO.Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(dir) && !System.IO.Directory.Exists(dir))
                System.IO.Directory.CreateDirectory(dir);
            System.IO.File.WriteAllText(fullPath, json);
            Debug.Log($"[TrafficManager] Saved spawn config to {fullPath}");
#if UNITY_EDITOR
            UnityEditor.AssetDatabase.Refresh();
#endif
        }

        [ContextMenu("Load Spawn Config JSON")]
        public void LoadSpawnConfigJson()
        {
            string fullPath = ResolveJsonPath();
            var data = TrafficSpawnConfig.ReadJson(fullPath);
            if (data == null)
            {
                Debug.LogWarning($"[TrafficManager] No data loaded from {fullPath}");
                return;
            }
            ApplyDataToTrafficSims(data);
            TrafficSpawnConfig.Load(data);
            Debug.Log($"[TrafficManager] Loaded spawn config from {fullPath}");
        }

        /// <summary>
        /// Adds traffic simulator to the manager
        /// </summary>
        public void AddTrafficSimulator(ITrafficSimulator simulator)
        {
            trafficSimulatorNodes.Add(simulator);
        }

        /// <summary>
        /// Clears all NPC vehicles running in simulation
        /// </summary>
        public void ClearAll()
        {
            trafficSimulatorNodes.Clear();
            npcVehicleSimulator?.ClearAll();
        }

        /// <summary>
        /// Remove all NPC vehicles within a specified radius from a point.
        /// Vehicles are marked for despawn and will be removed in the next FixedUpdate.
        /// </summary>
        /// <param name="center">The center position to check from</param>
        /// <param name="radius">The radius to check within</param>
        /// <returns>Number of vehicles marked for removal</returns>
        public int RemoveVehiclesInRadius(Vector3 center, float radius)
        {
            if (npcVehicleSimulator == null)
            {
                Debug.LogWarning("NPCVehicleSimulator is not initialized");
                return 0;
            }
            
            return npcVehicleSimulator.RemoveVehiclesInRadius(center, radius);
        }

        /// <summary>
        /// Remove all NPC vehicles within a specified radius from a transform's position.
        /// Vehicles are marked for despawn and will be removed in the next FixedUpdate.
        /// </summary>
        /// <param name="centerTransform">The transform whose position to check from</param>
        /// <param name="radius">The radius to check within</param>
        /// <returns>Number of vehicles marked for removal</returns>
        public int RemoveVehiclesInRadius(Transform centerTransform, float radius)
        {
            if (centerTransform == null)
            {
                Debug.LogWarning("RemoveVehiclesInRadius: centerTransform is null");
                return 0;
            }
            
            return RemoveVehiclesInRadius(centerTransform.position, radius);
        }

        void Initialize()
        {
            UnityEngine.Random.InitState(seed);

            spawnLanes = new Dictionary<NPCVehicleSpawnPoint, Dictionary<ITrafficSimulator, GameObject>>();
            var built = BuildDataFromTrafficSims();
            if (!string.IsNullOrEmpty(spawnConfigPath))
            {
                var data = TrafficSpawnConfig.ReadJson(ResolveJsonPath());
                if (data != null)
                {
                    ApplyDataToTrafficSims(data);
                    TrafficSpawnConfig.Load(data);
                    built = data;
                }
            }
            TrafficSpawnConfig.Load(built);
            dummyEgo = new GameObject("DummyEgo");
            if (_egoVehicle == null)
            {
                _egoVehicle = dummyEgo;
            }
            npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, maxVehicleCount, _egoVehicle);
            npcVehicleSimulator.SetDummyEgo(dummyEgo);
            if (externalConfig != null)
            {
                npcVehicleSimulator.Behaviour = externalConfig.behaviour;
                schedule = new List<ScheduledSpawnPoint>();
                for (int i = 0; i < externalConfig.spawnLanes.Count; i++)
                {
                    var vpm = externalConfig.vehiclesPerMinute[i];
                    schedule.Add(new ScheduledSpawnPoint
                    {
                        point = new NPCVehicleSpawnPoint(externalConfig.spawnLanes[i], 0),
                        laneName = externalConfig.spawnLanes[i].name,
                        interval = vpm > 0f ? 60f / vpm : 0f,
                        nextTime = vpm > 0f ? UnityEngine.Random.Range(0f, 60f / vpm) : float.MaxValue,
                    });
                }
            }

            verifyIntegrationEnvironmentElements();
            trafficSimulatorNodes = new List<ITrafficSimulator>();
            if (npcVehicleSimulator == null)
            {
                Debug.LogError("Traffic manager requires NPC Vehicle Simulator script.");
                return;
            }

            foreach (var randomTrafficConf in randomTrafficSims)
            {
                RandomTrafficSimulator randomTs = new RandomTrafficSimulator(
                    this.gameObject,
                    randomTrafficConf.npcPrefabs,
                    randomTrafficConf.spawnableLanes,
                    npcVehicleSimulator,
                    randomTrafficConf.maximumSpawns
                );
                randomTs.enabled = randomTrafficConf.enabled;
                trafficSimulatorNodes.Add(randomTs);
            }

            foreach (var routeTrafficSimConf in routeTrafficSims)
            {
                RouteTrafficSimulator routeTs = new RouteTrafficSimulator(
                    this.gameObject,
                    routeTrafficSimConf.npcPrefabs,
                    routeTrafficSimConf.route,
                    npcVehicleSimulator,
                    routeTrafficSimConf.maximumSpawns
                );
                routeTs.enabled = routeTrafficSimConf.enabled;
                trafficSimulatorNodes.Add(routeTs);
            }
        }

        void Dispose()
        {
            ClearAll();
            Despawn();
            npcVehicleSimulator?.Dispose();   // after Despawn, which still reads the vehicle states
            spawnLanes.Clear();
            if (dummyEgo)
            {
                Destroy(dummyEgo);
                _egoVehicle = null;
            }
        }

        void Start()
        {
            Initialize();
        }

        public void Restart(int newSeed = 0, int newMaxVehicleCount = 10)
        {
            Dispose();

            seed = newSeed;
            maxVehicleCount = newMaxVehicleCount;

            Initialize();
        }

        public void SetSpawnConfigPath(string path)
        {
            spawnConfigPath = path;
            LoadSpawnConfigJson();
        }

        private void verifyIntegrationEnvironmentElements()
        {
            if (trafficLanesObject == null)
            {
                trafficLanesObject = GameObject.Find("TrafficLanes");
                if (trafficLanesObject == null)
                {
                    Debug.LogError("VerifyIntegrationEnvironmentElements error: TrafficLanes GameObject is not assigned and could not be found in the scene. Please assign it in the Inspector.");
                    return;
                }
            }

            Transform[] children = trafficLanesObject.GetComponentsInChildren<Transform>();
            HashSet<string> uniqueNames = new HashSet<string>();
            bool isAnyIntersectionLane = false;
            bool isAnyTrafficScript = false;
            foreach (Transform child in children)
            {
                var trafficScript = child.gameObject.GetComponent<TrafficLane>();
                if (trafficScript)
                {
                    isAnyTrafficScript = true;
                    if (trafficScript.intersectionLane)
                    {
                        isAnyIntersectionLane = true;
                    }
                    if (!uniqueNames.Add(child.name))
                    {
                        Debug.LogError("VerifyIntegrationEnvironmentElements error: Found repeated child name in the 'TrafficLanes' object: " + child.name);
                    }
                }
            }
            if (!isAnyIntersectionLane)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Not found any TrafficLane with 'IntersectionLane' set to true.");
            }
            if (!isAnyTrafficScript)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Not found any TrafficLane with 'TrafficScript'.");
            }

        }

        private void FixedUpdate()
        {
            // Manage NPC spawning with the traffic simulators

            // Clear null elements in current list of traffic simulator
            if (trafficSimulatorNodes == null)
            {
                Debug.LogError("TrafficSimulatorNodes is null. TrafficManager may not have initialized correctly. Check that TrafficLanes GameObject is assigned.");
                return;
            }
            trafficSimulatorNodes.RemoveAll(item => item == null);

            if (schedule != null)
            {
                ScheduledSpawn();
                npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);
                Despawn();
                return;
            }

            // Find out which lanes are used by multiple spawners
            // We can easly spawn vehicle on a lane which only one spawner hooked to that lane.
            // If there are multiple spawners for one lane, we need to manage spawning. Without
            // managing, one spawner might overcome all others because of the vehicle size check.
            foreach (var trafficSimulator in trafficSimulatorNodes)
            {
                if (!trafficSimulator.IsEnabled()) continue;

                trafficSimulator.GetRandomSpawnInfo(out var spawnPoint, out var prefab);
                if (spawnLanes.ContainsKey(spawnPoint))
                {
                    spawnLanes[spawnPoint].Add(trafficSimulator, prefab);
                }
                else
                {
                    var tsims = new Dictionary<ITrafficSimulator, GameObject>();
                    tsims.Add(trafficSimulator, prefab);
                    spawnLanes.Add(spawnPoint, tsims);
                }
            }

            // For lane with single vehicle spawner - just spawn it.
            // For lane with multiple vehicle spawners - make priorities and spawn one by one.
            foreach (var spawnLoc in spawnLanes)
            {
                NPCVehicle spawnedVehicle;

                var distance2D = GeometryUtility.Distance2D(_egoVehicle.transform.position, spawnLoc.Key.Position);
                if (distance2D < spawnDistanceToEgo)
                {
                    continue;
                }

                // Optional spawn rate limiting per lane
                string laneName = spawnLoc.Key.Lane != null ? spawnLoc.Key.Lane.name : string.Empty;
                if (spmByLane.TryGetValue(laneName, out var spm) && spm > 0f)
                {
                    float interval = 60f / Mathf.Max(0.0001f, spm);
                    if (lastSpawnTimeByLane.TryGetValue(laneName, out var lastTime))
                    {
                        if (Time.time - lastTime < interval)
                            continue;
                    }
                }

                if (spawnLoc.Value.Count == 1)
                {
                    var tsimAndPrefab = spawnLoc.Value.First();
                    var trafficSim = tsimAndPrefab.Key;
                    var prefab = tsimAndPrefab.Value;
                    if (!NPCVehicleSpawner.IsSpawnable(prefab.GetComponent<NPCVehicle>().Bounds, spawnLoc.Key))
                        continue;
                    var spawned = trafficSim.Spawn(prefab, spawnLoc.Key, out spawnedVehicle);
                }
                else
                {
                    var priorityTrafficSimList = spawnLoc.Value.OrderByDescending(x => x.Key.GetCurrentPriority());
                    var priorityTrafficSimGo = priorityTrafficSimList.First();
                    var prefab = priorityTrafficSimGo.Value;
                    if (!NPCVehicleSpawner.IsSpawnable(prefab.GetComponent<NPCVehicle>().Bounds, spawnLoc.Key))
                    {
                        continue;
                    }
                    bool spawned = priorityTrafficSimGo.Key.Spawn(prefab, spawnLoc.Key, out spawnedVehicle);
                    if (spawned)
                    {
                        foreach (var rest in priorityTrafficSimList)
                        {
                            rest.Key.IncreasePriority(1);
                        }
                        priorityTrafficSimGo.Key.ResetPriority();
                    }
                }

                if (spawnedVehicle != null && !string.IsNullOrEmpty(laneName))
                {
                    lastSpawnTimeByLane[laneName] = Time.time;
                }

                if (spawnedVehicle && spawnedVehicle.gameObject.tag == "Ego")
                {
                    npcVehicleSimulator.EGOVehicle = spawnedVehicle.transform;
                }
            }

            spawnLanes.Clear();
            npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);

            Despawn();
        }
        private void Despawn()
        {
            foreach (var state in npcVehicleSimulator.VehicleStates)
            {
                if (state.ShouldDespawn)
                {
                    UnityEngine.Object.DestroyImmediate(state.Vehicle.gameObject);
                }
            }
            npcVehicleSimulator.RemoveInvalidVehicles();
        }

        /// <summary>Vehicles spawned per spawn lane (config mode), and the stuck / right-of-way counters.</summary>
        public string SpawnSummary()
        {
            if (schedule == null) return "inspector mode";
            var t = Mathf.Max(1f, Time.time);
            return string.Join(", ", schedule.Select(s => $"{s.laneName}: {s.spawned} ({s.spawned * 60f / t:F1}/min)")) +
                   $"; removed stuck {npcVehicleSimulator?.StuckDespawnCount}, right-of-way given {npcVehicleSimulator?.ForcedPassCount}";
        }

        private void OnDestroy()
        {
            if (schedule != null)
                Debug.Log("[Traffic] " + SpawnSummary());
            // Dispose first: a failing export must not leak the simulator's native arrays.
            npcVehicleSimulator?.Dispose();
            try
            {
                TrafficSpawnConfig.ExportBranchDecisionCsv();
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[TrafficManager] Could not export branch decisions: {e.Message}");
            }
        }

        private TrafficSpawnConfig.ConfigData BuildDataFromTrafficSims()
        {
            var data = new TrafficSpawnConfig.ConfigData();
            spmByLane.Clear();

            foreach (var sim in randomTrafficSims)
            {
                if (sim.spawnableLanes == null) continue;
                foreach (var cfg in sim.spawnableLanes)
                {
                    if (cfg.lane == null) continue;
                    data.spawnRates.Add(new TrafficSpawnConfig.SpawnRateEntry
                    {
                        lane = cfg.lane.name,
                        spawnsPerMinute = cfg.spawnsPerMinute
                    });
                    spmByLane[cfg.lane.name] = cfg.spawnsPerMinute;
                }
                if (sim.branchWeights != null)
                {
                    foreach (var bwSet in sim.branchWeights)
                    {
                        if (bwSet.fromLane == null || bwSet.next == null) continue;
                        var entry = new TrafficSpawnConfig.BranchEntry
                        {
                            fromLane = bwSet.fromLane.name,
                            next = new List<TrafficSpawnConfig.BranchWeight>()
                        };
                        foreach (var bw in bwSet.next)
                        {
                            if (bw.nextLane == null || bw.weight <= 0f) continue;
                            entry.next.Add(new TrafficSpawnConfig.BranchWeight
                            {
                                lane = bw.nextLane.name,
                                weight = bw.weight
                            });
                        }
                        if (entry.next.Count > 0)
                            data.branchWeights.Add(entry);
                    }
                }
            }

            return data;
        }

        private void ApplyDataToTrafficSims(TrafficSpawnConfig.ConfigData data)
        {
            spmByLane.Clear();

            var laneLookup = new Dictionary<string, TrafficLane>(System.StringComparer.OrdinalIgnoreCase);
            foreach (var lane in FindObjectsOfType<TrafficLane>(includeInactive: true))
            {
                if (lane != null && !laneLookup.ContainsKey(lane.name))
                    laneLookup.Add(lane.name, lane);
            }

            if (data.spawnRates != null)
            {
                for (int si = 0; si < randomTrafficSims.Length; si++)
                {
                    var sim = randomTrafficSims[si];
                    if (sim.spawnableLanes != null)
                    {
                        for (int i = 0; i < sim.spawnableLanes.Length; i++)
                        {
                            var cfg = sim.spawnableLanes[i];
                            if (cfg.lane == null) continue;
                            var match = data.spawnRates.Find(sr => sr != null && sr.lane.Equals(cfg.lane.name, StringComparison.OrdinalIgnoreCase));
                            if (match != null)
                            {
                                cfg.spawnsPerMinute = match.spawnsPerMinute;
                                sim.spawnableLanes[i] = cfg;
                                spmByLane[cfg.lane.name] = cfg.spawnsPerMinute;
                            }
                        }
                    }
                    randomTrafficSims[si] = sim;
                }
            }

            if (data.branchWeights != null)
            {
                for (int si = 0; si < randomTrafficSims.Length; si++)
                {
                    var sim = randomTrafficSims[si];
                    var branchList = new List<RandomTrafficSimulatorConfiguration.BranchWeightSet>();
                    foreach (var bw in data.branchWeights)
                    {
                        if (bw == null || string.IsNullOrEmpty(bw.fromLane) || bw.next == null) continue;
                        if (!laneLookup.TryGetValue(bw.fromLane, out var fromLane)) continue;
                        var set = new RandomTrafficSimulatorConfiguration.BranchWeightSet
                        {
                            fromLane = fromLane,
                            next = bw.next.Select(n =>
                            {
                                if (n == null || string.IsNullOrEmpty(n.lane) || n.weight <= 0f) return default;
                                return new RandomTrafficSimulatorConfiguration.BranchWeight
                                {
                                    nextLane = laneLookup.TryGetValue(n.lane, out var nextLane) ? nextLane : null,
                                    weight = n.weight
                                };
                            }).Where(x => x.nextLane != null && x.weight > 0f).ToArray()
                        };
                        if (set.next != null && set.next.Length > 0)
                            branchList.Add(set);
                    }
                    sim.branchWeights = branchList.ToArray();
                    randomTrafficSims[si] = sim;
                }
            }
        }

        private string ResolveJsonPath()
        {
            if (string.IsNullOrEmpty(spawnConfigPath))
                return System.IO.Path.Combine(Application.dataPath, "..", "Assets/Configs/traffic_spawn.json");
            if (System.IO.Path.IsPathRooted(spawnConfigPath))
                return spawnConfigPath;
            return System.IO.Path.Combine(Application.dataPath, "..", spawnConfigPath);
        }

        private void DrawSpawnPoints()
        {
            Gizmos.color = Color.cyan;
            foreach (var randomTrafficConf in randomTrafficSims)
            {
                foreach (var lane in randomTrafficConf.spawnableLanes)
                {
                    if (lane.lane == null || lane.lane.Waypoints.Length == 0) continue;
                    Gizmos.DrawCube(lane.lane.Waypoints[0], new Vector3(2.5f, 0.2f, 2.5f));
                }
            }

            Gizmos.color = Color.magenta;
            foreach (var routeTrafficSimConf in routeTrafficSims)
            {
                if (routeTrafficSimConf.route.Length > 0)
                {
                    Gizmos.DrawCube(routeTrafficSimConf.route[0].Waypoints[0], new Vector3(2.5f, 0.2f, 2.5f));
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (!showGizmos)
                return;

            var defaultColor = Gizmos.color;
            npcVehicleSimulator?.ShowGizmos(showYieldingPhase, showObstacleChecking);
            if (showSpawnPoints)
                DrawSpawnPoints();

            Gizmos.color = defaultColor;
        }
    }
}
