// Builds the simulation from config/ when play starts (runs before every other script of the scene).
//
// With a valid config it replaces the hand-placed RSU setup of the scene: the objects listed in
// "Scene Objects Replaced By Config" are switched off, and the RSUs, pseudo sensors and object-info publishers
// of the scenario are created from the sensor-type prefabs. If the config has errors, they are logged and the
// scene objects are kept as they are.
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace V2X.Config
{
    [DefaultExecutionOrder(-10000)]
    public class ConfigBootstrap : MonoBehaviour
    {
        [Tooltip("Scenario folder under config/scenarios. Empty = config/default_scenario.txt. --scenario on the command line wins.")]
        [SerializeField] string scenario = "";

        [Tooltip("Switched off when the config defines RSUs (the scene's own RSU sensors and object-info publishers).")]
        [SerializeField] List<GameObject> sceneObjectsReplacedByConfig = new List<GameObject>();

        [Header("Pseudo-sensor prefabs (per sensor kind)")]
        [SerializeField] GameObject lidarPrefab;
        [SerializeField] GameObject cameraPrefab;

        public static LoadedConfig Current { get; private set; }

        void Awake()
        {
            var config = ConfigLoader.Load(scenario);
            Current = config;
            foreach (var w in config.Warnings) Debug.LogWarning("[Config] " + w);
            if (config.Errors.Count > 0)
            {
                Debug.LogError($"[Config] {config.Errors.Count} error(s) in {config.Root}; the scene's own setup is used:\n  " +
                               string.Join("\n  ", config.Errors));
                return;
            }
            Debug.Log($"[Config] scenario '{config.ScenarioName}' from {config.ScenarioDir}");

            ApplyNoise(config);
            ApplyNetwork(config);
            if (config.Rsus != null)
                BuildRsus(config);
            if (config.Vehicles != null)
                SetupVehicles(config);
            if (config.Pedestrians != null)
                SetupPedestrians(config);
            if (config.Teleports != null)
                SetupTeleports(config);
        }

        /// <summary>The lanes (by name) under the object at <paramref name="path"/>; with several such objects, the one holding most of <paramref name="wanted"/>.</summary>
        static Dictionary<string, AWSIM.TrafficSimulation.TrafficLane> FindLanes(string path, HashSet<string> wanted)
        {
            var root = FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Where(t => ScenePath(t) == path)
                .OrderByDescending(t => t.GetComponentsInChildren<AWSIM.TrafficSimulation.TrafficLane>(true).Count(l => wanted.Contains(l.name)))
                .FirstOrDefault();
            if (root == null) return null;
            var lanes = new Dictionary<string, AWSIM.TrafficSimulation.TrafficLane>();
            foreach (var lane in root.GetComponentsInChildren<AWSIM.TrafficSimulation.TrafficLane>(true))
                lanes[lane.name] = lane;
            return lanes;
        }

        /// <summary>One PedestrianCyclistSpawner walks the paths of pedestrians.yaml; the scene's other spawners are switched off.</summary>
        static void SetupPedestrians(LoadedConfig c)
        {
            var spawners = FindObjectsByType<V2X.Paths.PedestrianCyclistSpawner>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .OrderBy(s => ScenePath(s.transform)).ToList();
            var template = spawners.FirstOrDefault(s => s.isActiveAndEnabled) ?? spawners.FirstOrDefault(s => s.enabled) ?? spawners.FirstOrDefault();
            if (template == null)
            {
                Debug.LogError("[Config] pedestrians: no PedestrianCyclistSpawner in the scene (it holds the pedestrian / cyclist prefabs)");
                return;
            }
            foreach (var s in spawners)
                if (s != template) s.enabled = false;
            foreach (var legacy in FindObjectsByType<V2X.Paths.PathConfigLoader>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                legacy.enabled = false;
            template.transform.SetParent(null, true);
            template.gameObject.SetActive(true);
            template.enabled = true;

            var paths = new List<V2X.Paths.PathConfigLoader.PathDefinition>();
            foreach (var p in c.Pedestrians.Paths.Where(p => p.Enabled))
            {
                paths.Add(new V2X.Paths.PathConfigLoader.PathDefinition
                {
                    id = p.Id, spawnPerMinute = p.SpawnsPerMinute,
                    type = p.CyclistRatio <= 0f ? V2X.Paths.PathConfigLoader.PathUserType.pedestrian
                         : p.CyclistRatio >= 1f ? V2X.Paths.PathConfigLoader.PathUserType.cyclist
                         : V2X.Paths.PathConfigLoader.PathUserType.both,
                    cyclistRatio = p.CyclistRatio,
                    waypoints = p.Waypoints.Select(w => new V2X.Paths.PathConfigLoader.PathPoint
                    {
                        x = (float)w.X, y = (float)w.Y, z = (float)w.Z,
                        checkLight = !string.IsNullOrEmpty(w.TrafficLight), trafficLightId = w.TrafficLight ?? "",
                    }).ToList(),
                });
            }
            template.ApplyExternalConfig(paths, c.Pedestrians.MaxActive, c.Pedestrians.SnapToGround);
            Debug.Log($"[Config] pedestrians: {paths.Count} paths, max {c.Pedestrians.MaxActive} active ({template.name})");
        }

        /// <summary>Teleport gates of teleports.yaml; the scene's own Teleporters are switched off.</summary>
        static void SetupTeleports(LoadedConfig c)
        {
            foreach (var t in FindObjectsByType<Teleporter>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                t.enabled = false;
            var lanesRoot = !string.IsNullOrEmpty(c.Teleports.LanesRoot) ? c.Teleports.LanesRoot : c.Vehicles?.LanesRoot;
            var wanted = new HashSet<string>(c.Teleports.Teleports.Where(t => t.LaneMidpoint != null)
                .SelectMany(t => t.LaneMidpoint.Lanelets.Append(t.LaneMidpoint.MidpointLanelet)).Select(id => "lanelet" + id));
            var lanes = string.IsNullOrEmpty(lanesRoot) ? null : FindLanes(lanesRoot, wanted);

            var root = new GameObject("Teleports (config)");
            int n = 0;
            foreach (var t in c.Teleports.Teleports.Where(t => t.Enabled))
            {
                var go = new GameObject(t.Name);
                go.SetActive(false);
                go.transform.SetParent(root.transform, false);
                ConfigGeometry.ApplyPose(go.transform, t.Trigger);
                var box = go.AddComponent<BoxCollider>();
                box.isTrigger = true;
                box.size = new Vector3((float)t.TriggerSize.X, (float)t.TriggerSize.Y, (float)t.TriggerSize.Z);

                var laneList = new List<AWSIM.TrafficSimulation.TrafficLane>();
                AWSIM.TrafficSimulation.TrafficLane midpoint = null;
                if (t.LaneMidpoint != null && lanes != null)
                {
                    foreach (var id in t.LaneMidpoint.Lanelets)
                        if (lanes.TryGetValue("lanelet" + id, out var lane)) laneList.Add(lane);
                        else Debug.LogError($"[Config] teleport {t.Name}: lanelet {id} not found under {lanesRoot}");
                    lanes.TryGetValue("lanelet" + t.LaneMidpoint.MidpointLanelet, out midpoint);
                    if (midpoint == null && t.Strategy == "lane_midpoint")
                        Debug.LogError($"[Config] teleport {t.Name}: midpoint_lanelet {t.LaneMidpoint.MidpointLanelet} not found");
                }
                var points = new List<Transform>();
                foreach (var p in t.RandomPoints)
                {
                    var pt = new GameObject("point");
                    pt.transform.SetParent(go.transform, false);
                    ConfigGeometry.ApplyPose(pt.transform, p);
                    points.Add(pt.transform);
                }
                var lm = t.LaneMidpoint ?? new LaneMidpointConfig();
                Vector2 Range2(List<float> v, Vector2 dflt) => v != null && v.Count >= 2 ? new Vector2(v[0], v[1]) : dflt;
                go.AddComponent<Teleporter>().Configure(
                    t.Strategy == "random_point" ? Teleporter.SpawnStrategy.RandomPoint : Teleporter.SpawnStrategy.LaneMidpoint,
                    t.ClearanceRadiusM, t.DelayS, Range2(t.RandomDelayS, Vector2.zero), laneList, lm.SpacingM, lm.IncludeEndpoints,
                    midpoint, lm.MidpointFraction, lm.CruiseSpeedKmh, lm.AccelerationMps2, Range2(lm.ArrivalWindowS, new Vector2(0f, 90f)), points);
                go.SetActive(true);
                n++;
            }
            Debug.Log($"[Config] teleports: {n} gates");
        }

        static string ScenePath(Transform t) => t.parent ? ScenePath(t.parent) + "/" + t.name : t.name;

        /// <summary>
        /// One TrafficManager drives the traffic from vehicles.yaml: the active one in the scene is kept as the template
        /// (prefabs, ego, layers, vehicle dynamics); every other TrafficManager is switched off.
        /// </summary>
        static void SetupVehicles(LoadedConfig c)
        {
            var v = c.Vehicles;
            var managers = FindObjectsByType<AWSIM.TrafficSimulation.TrafficManager>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            // Any TrafficManager works as the template (they share prefabs, ego, layers); prefer an enabled one.
            var template = managers.FirstOrDefault(m => !string.IsNullOrEmpty(v.TemplateTrafficManager) && ScenePath(m.transform) == v.TemplateTrafficManager)
                           ?? managers.OrderBy(m => ScenePath(m.transform)).FirstOrDefault(m => m.enabled && m.randomTrafficSims != null && m.randomTrafficSims.Length > 0)
                           ?? managers.FirstOrDefault();
            if (!string.IsNullOrEmpty(v.TemplateTrafficManager) && ScenePath(template.transform) != v.TemplateTrafficManager)
                Debug.LogWarning($"[Config] vehicles: template_traffic_manager '{v.TemplateTrafficManager}' not found; using {ScenePath(template.transform)}");
            if (template == null)
            {
                Debug.LogError("[Config] vehicles: no TrafficManager in the scene");
                return;
            }
            foreach (var m in managers)
                if (m != template) m.enabled = false;
            // Independent of which scenario object the selector switches on
            template.transform.SetParent(null, true);
            template.gameObject.SetActive(true);
            template.enabled = true;

            // Prefabs known to any traffic manager of the scene, by name
            var prefabs = new Dictionary<string, GameObject>();
            foreach (var m in managers)
                foreach (var sim in m.randomTrafficSims ?? new AWSIM.TrafficSimulation.RandomTrafficSimulatorConfiguration[0])
                    foreach (var p in sim.npcPrefabs ?? new GameObject[0])
                        if (p != null && !prefabs.ContainsKey(p.name)) prefabs[p.name] = p;

            // Several objects can share the path; use the one that holds the most of the configured lanelets.
            var wanted = new HashSet<string>(v.SpawnPoints.Select(sp => "lanelet" + sp.Lanelet)
                .Concat(v.Branches.SelectMany(b => b.To.Select(t => "lanelet" + t.Lanelet).Append("lanelet" + b.FromLanelet))));
            var root = FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Where(t => ScenePath(t) == v.LanesRoot)
                .OrderByDescending(t => t.GetComponentsInChildren<AWSIM.TrafficSimulation.TrafficLane>(true).Count(l => wanted.Contains(l.name)))
                .FirstOrDefault();
            if (root == null)
            {
                Debug.LogError($"[Config] vehicles: lanes_root '{v.LanesRoot}' not found; the scene's traffic setup is used");
                foreach (var m in managers) m.enabled = m == template;
                return;
            }
            var lanes = new Dictionary<string, AWSIM.TrafficSimulation.TrafficLane>();
            foreach (var lane in root.GetComponentsInChildren<AWSIM.TrafficSimulation.TrafficLane>(true))
                lanes[lane.name] = lane;
            AWSIM.TrafficSimulation.TrafficLane Lane(long id, string where)
            {
                if (lanes.TryGetValue("lanelet" + id, out var lane)) return lane;
                Debug.LogError($"[Config] vehicles: {where}: lanelet {id} has no lane under {v.LanesRoot}");
                return null;
            }

            var ext = new AWSIM.TrafficSimulation.TrafficManager.ExternalConfig
            {
                seed = v.Seed, maxVehicleCount = v.MaxVehicles, spawnDistanceToEgo = v.SpawnDistanceToEgoM,
                spawnClearance = v.SpawnClearanceM, maximumSpawns = v.MaximumSpawns,
                behaviour = new AWSIM.TrafficSimulation.NPCTrafficBehaviour
                {
                    deadlockTimeout = v.Behaviour.DeadlockTimeoutS, forcedPassDuration = v.Behaviour.ForcedPassS,
                    forcedPassRadius = v.Behaviour.ForcedPassRadiusM, stuckDespawnTimeout = v.Behaviour.StuckDespawnS,
                    standstillSpeed = v.Behaviour.StandstillSpeedMps,
                },
            };
            foreach (var t in v.VehicleTypes)
            {
                if (prefabs.TryGetValue(t.Prefab, out var p)) { ext.prefabs.Add(p); ext.prefabWeights.Add(t.Weight); }
                else Debug.LogError($"[Config] vehicles: unknown vehicle prefab '{t.Prefab}' (known: {string.Join(", ", prefabs.Keys)})");
            }
            foreach (var s in v.SpawnPoints)
            {
                var lane = Lane(s.Lanelet, $"spawn point {s.Name}");
                if (lane == null) continue;
                ext.spawnLanes.Add(lane);
                ext.vehiclesPerMinute.Add(s.VehiclesPerMinute);
            }
            foreach (var b in v.Branches)
            {
                var from = Lane(b.FromLanelet, "branch");
                if (from == null) continue;
                var next = new List<AWSIM.TrafficSimulation.RandomTrafficSimulatorConfiguration.BranchWeight>();
                foreach (var t in b.To)
                {
                    var to = Lane(t.Lanelet, $"branch from {b.FromLanelet}");
                    if (to == null) continue;
                    if (!from.NextLanes.Contains(to))
                        Debug.LogWarning($"[Config] vehicles: lanelet {t.Lanelet} does not follow lanelet {b.FromLanelet}; the ratio is ignored");
                    next.Add(new AWSIM.TrafficSimulation.RandomTrafficSimulatorConfiguration.BranchWeight { nextLane = to, weight = t.Ratio });
                }
                ext.branches.Add(new AWSIM.TrafficSimulation.RandomTrafficSimulatorConfiguration.BranchWeightSet { fromLane = from, next = next.ToArray() });
            }
            template.ApplyExternalConfig(ext);
            Debug.Log($"[Config] vehicles: {ext.spawnLanes.Count} spawn points, {ext.branches.Count} branches, " +
                      $"{ext.prefabs.Count} vehicle types, max {ext.maxVehicleCount} ({template.name})");
        }

        static void ApplyNoise(LoadedConfig c)
        {
            if (c.Noise == null || NoiseSetting.Instance == null) return;
            NoiseSetting.Instance.SetProfiles(c.Noise.NoiseProfiles.Select(n => new NoiseSetting.Noise(
                n.Name, n.Distribution != "gaussian", n.XM, n.YM, n.ZM, n.GaussianMean, n.GaussianStdDev)));
        }

        static MessageDelayConfig ToDelay(NetworkProfile n) => new MessageDelayConfig(
            n.DelayMs, n.PacketLoss, n.TopicSuffix, n.GroundTruth, n.DelayVarianceMs, n.PacketLossVariance);

        static void ApplyNetwork(LoadedConfig c)
        {
            if (c.Network == null || NetworkSimulator.Instance == null) return;
            NetworkSimulator.Instance.SetGeneralDelayMessagesConfigs(c.Network.NetworkProfiles.Select(ToDelay));
        }

        void BuildRsus(LoadedConfig c)
        {
            foreach (var go in sceneObjectsReplacedByConfig)
                if (go != null) go.SetActive(false);

            var scenarioLists = BlindScenarioManager.Instance != null ? BlindScenarioManager.Instance.getCurrentScenario() : null;
            if (scenarioLists != null)
            {
                // the line-of-sight check only looks at sensors listed in the scenario
                scenarioLists.getRSUsensors().Clear();
                scenarioLists.getRsuCameras().Clear();
            }

            // Everything is created under an inactive root and configured before any Awake / OnEnable runs.
            var root = new GameObject("RSUs (config)");
            root.SetActive(false);
            var sensorsByRsu = new Dictionary<RsuConfig, List<DetectionSensor>>();
            int sensorCount = 0;

            foreach (var rsu in c.Rsus.Rsus.Where(r => r.Enabled))
            {
                var rsuGo = new GameObject(rsu.Name);
                rsuGo.transform.SetParent(root.transform, false);
                var list = sensorsByRsu[rsu] = new List<DetectionSensor>();
                foreach (var s in rsu.Sensors.Where(s => s.Enabled))
                {
                    var type = c.FindSensorType(s.Type);
                    bool camera = type.Kind == "camera";
                    var prefab = camera ? cameraPrefab : lidarPrefab;
                    if (prefab == null)
                    {
                        Debug.LogError($"[Config] no {type.Kind} prefab set on ConfigBootstrap; sensor {rsu.Name}/{s.Name} skipped");
                        continue;
                    }
                    var go = Instantiate(prefab, rsuGo.transform);
                    go.name = s.Name;
                    ConfigGeometry.ApplyPose(go.transform, s.Pose);
                    var mock = go.GetComponentInChildren<MockDetectionSensor>(true);
                    mock.Configure(s.Name,
                        camera ? MockDetectionSensor.MockSensorType.RSU_CAMERA : MockDetectionSensor.MockSensorType.RSU_Lidar,
                        s.HFovDeg ?? type.HFovDeg, s.VFovDeg ?? type.VFovDeg, s.RangeM ?? type.RangeM, (byte)s.SensorId);
                    list.Add(mock);
                    if (scenarioLists != null)
                        (camera ? scenarioLists.getRsuCameras() : scenarioLists.getRSUsensors()).Add(mock);
                    sensorCount++;

                    if (s.ObjectInfo != null && !string.IsNullOrEmpty(s.ObjectInfo.Topic))
                    {
                        var p = c.Rsus.PerSensorObjectInfo;
                        var pub = new GameObject($"object_info {rsu.Name}/{s.Name}");
                        pub.transform.SetParent(rsuGo.transform, false);
                        pub.AddComponent<ObjectInfo>().Configure(rsu.RsuId, new List<DetectionSensor> { mock }, null, p.Hz,
                            s.ObjectInfo.Topic, s.ObjectInfo.GroundTruthTopic,
                            p.Noise.Position, p.Noise.Rotation, p.Noise.Dimension, p.Noise.Probability, null);
                    }
                }
            }

            foreach (var p in c.Rsus.ObjectInfoPublishers)
            {
                var rsus = p.Rsus.Contains("all") ? sensorsByRsu.Keys.ToList()
                    : sensorsByRsu.Keys.Where(r => p.Rsus.Contains(r.Name)).ToList();
                var groups = rsus.Select(r => new ObjectInfo.RsuSensors { rsuId = r.RsuId, sensors = sensorsByRsu[r] }).ToList();
                var go = new GameObject($"object_info {p.Name}");
                go.transform.SetParent(root.transform, false);
                go.AddComponent<ObjectInfo>().Configure(rsus.Count > 0 ? rsus[0].RsuId : "0x0", null, groups, p.Hz,
                    p.Topic, p.GroundTruthTopic, p.Noise.Position, p.Noise.Rotation, p.Noise.Dimension, p.Noise.Probability,
                    p.Network.Select(n => ToDelay(c.FindNetwork(n))).ToList());
            }

            foreach (var p in c.Rsus.AutowareObjectPublishers)
            {
                var sensors = (p.Rsus.Contains("all") ? sensorsByRsu.Keys : sensorsByRsu.Keys.Where(r => p.Rsus.Contains(r.Name)))
                    .SelectMany(r => sensorsByRsu[r]).ToList();
                var go = new GameObject($"predicted_objects {p.Name}");
                go.transform.SetParent(root.transform, false);
                go.AddComponent<PredictedObjectsAutoware>().Configure(sensors, p.Hz, (ulong)p.StationId, p.Topic,
                    p.Noise.Position, p.Noise.Rotation, p.Noise.Dimension, p.Noise.Probability,
                    p.Network.Select(n => ToDelay(c.FindNetwork(n))).ToList());
            }

            var intersections = FindObjectsByType<AWSIM.TrafficSimulation.TrafficIntersection>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            foreach (var p in c.Rsus.TrafficSignalPublishers)
            {
                var lights = new List<AWSIM.TrafficLight>();
                foreach (var name in p.Intersections)
                {
                    var match = intersections.Where(i => i.name == name).ToList();
                    if (match.Count == 0) Debug.LogError($"[Config] traffic-signal publisher {p.Name}: intersection '{name}' not found");
                    // only lights linked to Lanelet2 (vehicle lights) end up in the message
                    foreach (var light in match.SelectMany(i => i.GetComponentsInChildren<AWSIM.TrafficLight>(true)))
                        if (!lights.Contains(light)) lights.Add(light);
                }
                var go = new GameObject($"traffic_signals {p.Name}");
                go.transform.SetParent(root.transform, false);
                go.AddComponent<AWSIM.CustomV2I>().Configure(p.StationId, p.Hz, lights.ToArray());
                foreach (var topic in p.Topics)
                    go.AddComponent<AWSIM.CustomV2IPublisher>().Configure(topic,
                        p.SignalId == "way" ? AWSIM.CustomV2IPublisher.TrafficSignalID.WayID : AWSIM.CustomV2IPublisher.TrafficSignalID.RelationID);
            }

            root.SetActive(true);
            Debug.Log($"[Config] {sensorsByRsu.Count} RSUs, {sensorCount} pseudo sensors, {c.Rsus.ObjectInfoPublishers.Count} merged object-info, " +
                      $"{c.Rsus.AutowareObjectPublishers.Count} Autoware-object and {c.Rsus.TrafficSignalPublishers.Count} traffic-signal publishers created");
        }
    }
}
