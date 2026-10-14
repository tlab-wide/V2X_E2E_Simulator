// Writes the RSU / pseudo-sensor / object-info setup of the open scene to config/ (defaults + one scenario).
// Menu: V2X > Config > Export Scene To Config. Batch mode: -executeMethod V2X.Config.ConfigExporter.ExportBatch
//   (optional -exportScenario <name>, default kashiwa_cool4).
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace V2X.Config
{
    public static class ConfigExporter
    {
        const string DefaultScenario = "kashiwa_cool4";
        const string PseudoSensorsRoot = "Pseudo Sensors setup";

        [MenuItem("V2X/Config/Export Scene To Config")]
        public static void ExportMenu()
        {
            var report = Export(DefaultScenario);
            EditorUtility.DisplayDialog("Export scene to config", report, "OK");
        }

        public static void ExportBatch()
        {
            var scenario = ConfigLoader.GetArg("-exportScenario") ?? DefaultScenario;
            var scene = ConfigLoader.GetArg("-exportScene");
            if (!string.IsNullOrEmpty(scene))
                EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            Debug.Log("[ConfigExporter] " + Export(scenario).Replace("\n", "\n[ConfigExporter] "));
            EditorApplication.Exit(0);
        }

        static IEnumerable<T> All<T>() where T : Object
            => Object.FindObjectsByType(typeof(T), FindObjectsInactive.Include, FindObjectsSortMode.None).Cast<T>()
                .Where(o => !EditorUtility.IsPersistent(o));

        static string Path_(Transform t) => t.parent ? Path_(t.parent) + "/" + t.name : t.name;

        public static string Export(string scenario)
        {
            var log = new List<string>();
            var root = ConfigLoader.ConfigRoot();

            // ------------------------------------------------------------------ noise profiles
            var noise = new NoiseConfig();
            var noiseSetting = All<NoiseSetting>().FirstOrDefault();
            if (noiseSetting != null)
            {
                var list = new SerializedObject(noiseSetting).FindProperty("noises");
                for (int i = 0; i < list.arraySize; i++)
                {
                    var n = list.GetArrayElementAtIndex(i);
                    noise.NoiseProfiles.Add(new NoiseProfile
                    {
                        Name = n.FindPropertyRelative("name").stringValue,
                        Distribution = n.FindPropertyRelative("IsUniform").boolValue ? "uniform" : "gaussian",
                        XM = n.FindPropertyRelative("xNoise").floatValue,
                        YM = n.FindPropertyRelative("yNoise").floatValue,
                        ZM = n.FindPropertyRelative("zNoise").floatValue,
                        GaussianMean = n.FindPropertyRelative("gaussianMean").floatValue,
                        GaussianStdDev = n.FindPropertyRelative("gaussianStdDev").floatValue,
                    });
                }
            }
            if (noise.NoiseProfiles.Count == 0) noise.NoiseProfiles.Add(new NoiseProfile { Name = "Default" });
            string NoiseName(string scene) => noise.NoiseProfiles.Any(p => p.Name == scene) ? scene : noise.NoiseProfiles[0].Name;

            // ------------------------------------------------------------------ network profiles
            var network = new NetworkConfig();
            var netSim = All<NetworkSimulator>().FirstOrDefault();
            if (netSim != null)
            {
                var list = new SerializedObject(netSim).FindProperty("generalDelayMessagesConfigs");
                for (int i = 0; i < list.arraySize; i++)
                {
                    var d = list.GetArrayElementAtIndex(i).FindPropertyRelative("delayConfig");
                    var suffix = d.FindPropertyRelative("topicName").stringValue;
                    network.NetworkProfiles.Add(new NetworkProfile
                    {
                        Name = suffix.Trim('/'),
                        TopicSuffix = suffix,
                        DelayMs = d.FindPropertyRelative("delayMilliseconds").floatValue,
                        DelayVarianceMs = d.FindPropertyRelative("delayVariance").floatValue,
                        PacketLoss = d.FindPropertyRelative("packetLossProbability").floatValue,
                        PacketLossVariance = d.FindPropertyRelative("packetLossVariance").floatValue,
                        GroundTruth = d.FindPropertyRelative("isGroundTruth").boolValue,
                    });
                }
            }

            // ------------------------------------------------------------------ which RSU ID each sensor reports under
            var publishers = All<ObjectInfo>().ToList();
            var rsuOfSensor = new Dictionary<MockDetectionSensor, string>();
            var perSensor = new Dictionary<MockDetectionSensor, ObjectInfo>();
            ObjectInfo combined = null;
            foreach (var p in publishers)
            {
                var so = new SerializedObject(p);
                var own = Refs<MockDetectionSensor>(so.FindProperty("sensors"));
                foreach (var s in own) rsuOfSensor[s] = p.rsuId;
                var groups = so.FindProperty("rsuSensors");
                for (int i = 0; i < groups.arraySize; i++)
                {
                    var g = groups.GetArrayElementAtIndex(i);
                    foreach (var s in Refs<MockDetectionSensor>(g.FindPropertyRelative("sensors")))
                        rsuOfSensor[s] = g.FindPropertyRelative("rsuId").stringValue;
                }
                bool isCombined = so.FindProperty("sources").arraySize > 0 || groups.arraySize > 0;
                if (isCombined && p.gameObject.activeInHierarchy && p.enabled) combined = combined ?? p;
                else if (own.Count == 1 && p.gameObject.activeInHierarchy && p.enabled) perSensor[own[0]] = p;
            }

            // ------------------------------------------------------------------ RSUs and their sensors
            var types = new List<PseudoSensorType>();
            var rsus = new RsusConfig();
            var setup = All<Transform>().FirstOrDefault(t => t.name == PseudoSensorsRoot);
            var skipped = new List<string>();
            if (setup != null)
            {
                foreach (Transform group in setup)
                {
                    var sensors = group.GetComponentsInChildren<MockDetectionSensor>(true)
                        .Where(s => s.transform.parent == group).ToList();
                    if (sensors.Count == 0) continue;
                    var baseName = System.Text.RegularExpressions.Regex.Replace(group.name, @"^RSU\d+_|_CooL4$", "");
                    var ids = sensors.Where(rsuOfSensor.ContainsKey).Select(s => rsuOfSensor[s]).Distinct()
                        .OrderBy(i => i).ToList();
                    foreach (var s in sensors.Where(s => !rsuOfSensor.ContainsKey(s)))
                        skipped.Add($"{group.name}/{s.name} (not used by any object-info publisher{(s.gameObject.activeInHierarchy ? "" : ", inactive")})");
                    for (int k = 0; k < ids.Count; k++)
                    {
                        var rsu = new RsuConfig
                        {
                            Name = ids.Count > 1 ? $"{baseName}_{(char)('A' + k)}" : baseName,
                            RsuId = ids[k], Location = baseName, Enabled = group.gameObject.activeInHierarchy,
                        };
                        foreach (var s in sensors.Where(s => rsuOfSensor.TryGetValue(s, out var id) && id == ids[k])
                                     .OrderBy(s => s.GetSensorId()))
                        {
                            var so = new SerializedObject(s);
                            var kind = s.GetMockSensorType() == MockDetectionSensor.MockSensorType.RSU_CAMERA ? "camera" : "lidar";
                            // MockSensor compares |angle| with Hfov / Vfov, so the full field of view is twice the value.
                            float h = 2f * so.FindProperty("Hfov").floatValue, v = 2f * so.FindProperty("Vfov").floatValue;
                            float range = so.FindProperty("maxDistance").floatValue;
                            var type = types.FirstOrDefault(t => t.Kind == kind && t.HFovDeg == h && t.VFovDeg == v && t.RangeM == range);
                            if (type == null)
                            {
                                type = new PseudoSensorType
                                {
                                    Name = $"{kind}_{h:0.#}x{v:0.#}deg_{range:0.#}m", Kind = kind, HFovDeg = h, VFovDeg = v, RangeM = range,
                                    Description = $"exported from {group.name}/{s.name}",
                                };
                                types.Add(type);
                            }
                            var sensor = new PseudoSensorConfig
                            {
                                Name = s.name, Type = type.Name, SensorId = s.GetSensorId(),
                                Enabled = s.gameObject.activeInHierarchy && s.enabled, Pose = ConfigGeometry.ToPose(s.transform),
                            };
                            if (perSensor.TryGetValue(s, out var ps))
                            {
                                sensor.ObjectInfo = new PerSensorTopics { Topic = ps.topic, GroundTruthTopic = ps.topicGroundTruth };
                                var pso = new SerializedObject(ps);
                                rsus.PerSensorObjectInfo = new PerSensorPublisherSettings
                                {
                                    Hz = pso.FindProperty("Hz").floatValue, Noise = Noise(pso, NoiseName),
                                };
                            }
                            rsu.Sensors.Add(sensor);
                        }
                        rsus.Rsus.Add(rsu);
                        log.Add($"RSU {rsu.Name} ({rsu.RsuId}): {rsu.Sensors.Count} sensors");
                    }
                }
            }
            if (combined != null)
            {
                var so = new SerializedObject(combined);
                rsus.ObjectInfoPublishers.Add(new ObjectInfoPublisherConfig
                {
                    Name = "all_rsus", Rsus = new List<string> { "all" },
                    Topic = combined.topic, GroundTruthTopic = combined.topicGroundTruth,
                    Hz = so.FindProperty("Hz").floatValue, Noise = Noise(so, NoiseName),
                    Network = so.FindProperty("enableDelayedMessages").boolValue
                        ? network.NetworkProfiles.Where(n => !n.GroundTruth).Select(n => n.Name).ToList() : new List<string>(),
                });
                log.Add($"object-info publisher all_rsus: {combined.topic}");
            }
            foreach (var s in skipped) log.Add("skipped sensor " + s);

            // ------------------------------------------------------------------ write
            var defaults = Path.Combine(root, ConfigLoader.DefaultsDir);
            ConfigLoader.Write(Path.Combine(defaults, "sensor_types.yaml"), new SensorTypesConfig { PseudoSensorTypes = types },
                "# Pseudo-sensor (MockSensor) types: field of view (full angles, degrees) and range (m).\n" +
                "# kind: lidar | camera. RSU sensors in config/scenarios/*/rsus.yaml refer to a type by name and may override any value.");
            ConfigLoader.Write(Path.Combine(defaults, "noise.yaml"), noise,
                "# Noise profiles, referred to by name from the object-info publishers.\n" +
                "# uniform: a value in [-amplitude, +amplitude] per axis (x_m, y_m, z_m, metres).\n" +
                "# gaussian: gaussian_mean + gaussian_std_dev × amplitude × N(0, 1) per axis.");
            ConfigLoader.Write(Path.Combine(defaults, "network.yaml"), network,
                "# Network profiles: delayed / lossy copies of a topic, published as <topic><topic_suffix>.\n" +
                "# delay_ms ± delay_variance_ms; packet_loss is a probability (0-1). ground_truth: copy the ground-truth topic.");
            var dir = Path.Combine(root, ConfigLoader.ScenariosDir, scenario);
            ConfigLoader.Write(Path.Combine(dir, "scenario.yaml"),
                new ScenarioConfig { Name = scenario, Description = $"exported from {EditorSceneManager.GetActiveScene().name}" },
                "# Scenario: the files below (relative to this folder) are loaded when the simulation starts.");
            ConfigLoader.Write(Path.Combine(dir, "rsus.yaml"), rsus,
                "# RSUs, their pseudo sensors and the object-info publishers.\n" +
                "# pose: map frame (MGRS-local x/y/z, m), heading clockwise from north, pitch > 0 looks up (degrees).\n" +
                "# type: a name from config/defaults/sensor_types.yaml; h_fov_deg / v_fov_deg / range_m override it.\n" +
                "# object_info on a sensor: its own topics (no network copies). object_info_publishers: merged topics.");
            var def = Path.Combine(root, "default_scenario.txt");
            if (!File.Exists(def)) File.WriteAllText(def, scenario + "\n");

            log.Insert(0, $"wrote {root} (scenario {scenario}): {types.Count} sensor types, {noise.NoiseProfiles.Count} noise profiles, " +
                          $"{network.NetworkProfiles.Count} network profiles, {rsus.Rsus.Count} RSUs");
            return string.Join("\n", log);
        }

        [MenuItem("V2X/Config/Export Vehicle Traffic To Config")]
        public static void ExportVehiclesMenu()
        {
            EditorUtility.DisplayDialog("Export vehicle traffic to config", ExportVehicles(ConfigLoader.ResolveScenarioName(ConfigLoader.ConfigRoot(), null) ?? DefaultScenario), "OK");
        }

        public static void ExportVehiclesBatch()
        {
            var scene = ConfigLoader.GetArg("-exportScene");
            if (!string.IsNullOrEmpty(scene))
                EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            Debug.Log("[ConfigExporter] " + ExportVehicles(ConfigLoader.GetArg("-exportScenario") ?? DefaultScenario));
            EditorApplication.Exit(0);
        }

        static long LaneletId(string laneName)
            => laneName != null && laneName.StartsWith("lanelet") && long.TryParse(laneName.Substring(7), out var id) ? id : -1;

        /// <summary>Writes the active TrafficManager (and its spawn JSON) to vehicles.yaml and links it from scenario.yaml.</summary>
        public static string ExportVehicles(string scenario)
        {
            var managers = All<AWSIM.TrafficSimulation.TrafficManager>().ToList();
            var tm = managers.FirstOrDefault(m => m.isActiveAndEnabled) ?? managers.FirstOrDefault(m => m.gameObject.activeInHierarchy && m.enabled);
            if (tm == null) return "no active TrafficManager in the scene";
            var so = new SerializedObject(tm);
            var sim = tm.randomTrafficSims.FirstOrDefault(r => r.enabled && r.spawnableLanes != null && r.spawnableLanes.Length > 0);
            var lanes = sim.spawnableLanes.Where(l => l.lane != null).ToList();
            if (lanes.Count == 0) return $"{tm.name}: no spawn lanes";
            var root = lanes[0].lane.transform.parent;
            var v = new VehiclesConfig
            {
                LanesRoot = Path_(root), TemplateTrafficManager = Path_(tm.transform), Seed = tm.seed, MaxVehicles = tm.maxVehicleCount,
                SpawnDistanceToEgoM = so.FindProperty("spawnDistanceToEgo").floatValue, MaximumSpawns = sim.maximumSpawns,
            };
            foreach (var p in sim.npcPrefabs.Where(p => p != null))
                v.VehicleTypes.Add(new VehicleTypeConfig { Prefab = p.name, Weight = 1f });

            // spawn rates and branch weights: the JSON wins over the inspector, as at runtime
            var rates = lanes.ToDictionary(l => l.lane.name, l => l.spawnsPerMinute);
            var branches = (sim.branchWeights ?? new AWSIM.TrafficSimulation.RandomTrafficSimulatorConfiguration.BranchWeightSet[0])
                .Where(b => b.fromLane != null).Select(b => (from: b.fromLane.name,
                    to: (b.next ?? new AWSIM.TrafficSimulation.RandomTrafficSimulatorConfiguration.BranchWeight[0])
                        .Where(n => n.nextLane != null).Select(n => (n.nextLane.name, n.weight)).ToList())).ToList();
            var jsonPath = tm.GetSpawnConfigPath();
            if (!string.IsNullOrEmpty(jsonPath))
            {
                var full = System.IO.Path.IsPathRooted(jsonPath) ? jsonPath : System.IO.Path.Combine(Application.dataPath, "..", jsonPath);
                var data = AWSIM.TrafficSimulation.TrafficSpawnConfig.ReadJson(full);
                if (data != null)
                {
                    foreach (var r in data.spawnRates ?? new List<AWSIM.TrafficSimulation.TrafficSpawnConfig.SpawnRateEntry>())
                        if (rates.ContainsKey(r.lane)) rates[r.lane] = r.spawnsPerMinute;
                    if (data.branchWeights != null && data.branchWeights.Count > 0)
                        branches = data.branchWeights.Select(b => (from: b.fromLane, to: b.next.Select(n => (n.lane, n.weight)).ToList())).ToList();
                }
            }
            var skipped = new List<string>();
            foreach (var l in lanes)
            {
                var id = LaneletId(l.lane.name);
                if (id < 0) { skipped.Add(l.lane.name); continue; }
                var m = ConfigGeometry.UnityToMap(l.lane.Waypoints[0]);
                v.SpawnPoints.Add(new SpawnPointConfig
                {
                    Name = l.lane.name, Lanelet = id, VehiclesPerMinute = (float)System.Math.Round(rates[l.lane.name], 3),
                    Position = new MapPoint { X = System.Math.Round(m.x, 2), Y = System.Math.Round(m.y, 2), Z = System.Math.Round(m.z, 2) },
                });
            }
            foreach (var b in branches)
            {
                if (LaneletId(b.from) < 0) { skipped.Add(b.from); continue; }
                v.Branches.Add(new BranchConfig
                {
                    FromLanelet = LaneletId(b.from),
                    To = b.to.Where(t => LaneletId(t.Item1) >= 0).Select(t => new BranchTarget { Lanelet = LaneletId(t.Item1), Ratio = (float)System.Math.Round(t.Item2, 3) }).ToList(),
                });
            }

            var dir = Path.Combine(ConfigLoader.ConfigRoot(), ConfigLoader.ScenariosDir, scenario);
            ConfigLoader.Write(Path.Combine(dir, "vehicles.yaml"), v,
                "# NPC vehicle traffic.\n" +
                "# lanes_root: scene path of the TrafficLanes object; lanes are named lanelet<Lanelet2 ID>.\n" +
                "# template_traffic_manager: scene TrafficManager whose vehicle dynamics, layers and ego are used.\n" +
                "# spawn_points: vehicles start at the first point of the lanelet, vehicles_per_minute each (±25 % jitter).\n" +
                "# branches: where a lanelet splits, the ratio of vehicles taking each next lanelet (normalised; others get 0).\n" +
                "# behaviour: deadlock_timeout_s = standing still while yielding this long gives the right of way for forced_pass_s;\n" +
                "#            stuck_despawn_s = standing still this long for any reason removes the vehicle (0 = off).\n" +
                "# vehicle_types: NPC prefab names (from the scene's traffic managers) with relative weights.");
            var scenarioFile = Path.Combine(dir, "scenario.yaml");
            var text = File.Exists(scenarioFile) ? File.ReadAllText(scenarioFile) : "";
            if (!System.Text.RegularExpressions.Regex.IsMatch(text, @"^vehicles:", System.Text.RegularExpressions.RegexOptions.Multiline))
                File.WriteAllText(scenarioFile, text.TrimEnd() + "\nvehicles: vehicles.yaml\n");
            return $"{tm.name}: {v.SpawnPoints.Count} spawn points, {v.Branches.Count} branches, {v.VehicleTypes.Count} vehicle types, " +
                   $"lanes {v.LanesRoot}" + (skipped.Count > 0 ? $"; skipped (not lanelet-named): {string.Join(", ", skipped)}" : "");
        }

        [MenuItem("V2X/Config/Export Traffic Lanes For The Web Viewer")]
        public static void ExportLanesMenu()
        {
            EditorUtility.DisplayDialog("Export traffic lanes", ExportTrafficLanes(ConfigLoader.ResolveScenarioName(ConfigLoader.ConfigRoot(), null) ?? DefaultScenario), "OK");
        }

        public static void ExportLanesBatch()
        {
            var scene = ConfigLoader.GetArg("-exportScene");
            if (!string.IsNullOrEmpty(scene))
                EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            Debug.Log("[ConfigExporter] " + ExportTrafficLanes(ConfigLoader.GetArg("-exportScenario") ?? DefaultScenario));
            EditorApplication.Exit(0);
        }

        /// <summary>
        /// Writes the TrafficLanes under lanes_root of vehicles.yaml (centre lines in the map frame, next / previous lanes)
        /// to config/maps/traffic_lanes.json. The web viewer draws them in Settings so spawn points, branches and teleport
        /// lanes are picked from the lanes the simulator really uses (their numbers are not the Lanelet2 IDs of the map file).
        /// </summary>
        public static string ExportTrafficLanes(string scenario)
        {
            var dir = Path.Combine(ConfigLoader.ConfigRoot(), ConfigLoader.ScenariosDir, scenario);
            var vehicles = Path.Combine(dir, "vehicles.yaml");
            var lanesRoot = File.Exists(vehicles) ? ConfigLoader.Read<VehiclesConfig>(vehicles, new LoadedConfig())?.LanesRoot : null;
            var roots = All<Transform>().Where(t => lanesRoot == null ? t.name == "TrafficLanes" : Path_(t) == lanesRoot).ToList();
            var root = roots.OrderByDescending(t => t.GetComponentsInChildren<AWSIM.TrafficSimulation.TrafficLane>(true).Length).FirstOrDefault();
            if (root == null) return $"lanes root '{lanesRoot}' not found";
            var sb = new System.Text.StringBuilder();
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            string Ids(IEnumerable<AWSIM.TrafficSimulation.TrafficLane> l)
                => string.Join(",", (l ?? Enumerable.Empty<AWSIM.TrafficSimulation.TrafficLane>()).Where(x => x != null).Select(x => LaneletId(x.name)).Where(id => id >= 0));
            var lanes = root.GetComponentsInChildren<AWSIM.TrafficSimulation.TrafficLane>(true).Where(l => LaneletId(l.name) >= 0 && l.Waypoints != null && l.Waypoints.Length > 1).ToList();
            sb.Append("{\"lanes_root\":\"").Append(Path_(root).Replace("\"", "\\\"")).Append("\",\"lanes\":[");
            for (int i = 0; i < lanes.Count; i++)
            {
                var l = lanes[i];
                if (i > 0) sb.Append(",");
                sb.Append("\n{\"id\":").Append(LaneletId(l.name)).Append(",\"turn\":\"").Append(l.TurnDirection).Append("\"")
                  .Append(",\"intersection\":").Append(l.intersectionLane ? "true" : "false")
                  .Append(",\"speed_limit\":").Append(System.Math.Round(l.SpeedLimit, 2).ToString(inv))
                  .Append(",\"next\":[").Append(Ids(l.NextLanes)).Append("],\"prev\":[").Append(Ids(l.PrevLanes)).Append("],\"points\":[");
                for (int k = 0; k < l.Waypoints.Length; k++)
                {
                    var m = ConfigGeometry.UnityToMap(l.Waypoints[k]);
                    if (k > 0) sb.Append(",");
                    sb.Append("[").Append(System.Math.Round(m.x, 2).ToString(inv)).Append(",").Append(System.Math.Round(m.y, 2).ToString(inv))
                      .Append(",").Append(System.Math.Round(m.z, 2).ToString(inv)).Append("]");
                }
                sb.Append("]}");
            }
            sb.Append("\n]}\n");
            var outDir = Path.Combine(ConfigLoader.ConfigRoot(), "maps");
            Directory.CreateDirectory(outDir);
            File.WriteAllText(Path.Combine(outDir, "traffic_lanes.json"), sb.ToString());
            return $"{lanes.Count} traffic lanes of {Path_(root)} written to config/maps/traffic_lanes.json";
        }

        [MenuItem("V2X/Config/Export Pedestrians And Teleports To Config")]
        public static void ExportHumansMenu()
        {
            var scenario = ConfigLoader.ResolveScenarioName(ConfigLoader.ConfigRoot(), null) ?? DefaultScenario;
            EditorUtility.DisplayDialog("Export pedestrians and teleports", ExportPedestrians(scenario) + "\n" + ExportTeleports(scenario), "OK");
        }

        public static void ExportHumansBatch()
        {
            var scene = ConfigLoader.GetArg("-exportScene");
            if (!string.IsNullOrEmpty(scene))
                EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            var scenario = ConfigLoader.GetArg("-exportScenario") ?? DefaultScenario;
            Debug.Log("[ConfigExporter] " + ExportPedestrians(scenario));
            Debug.Log("[ConfigExporter] " + ExportTeleports(scenario));
            EditorApplication.Exit(0);
        }

        static void LinkFromScenario(string dir, string key, string file)
        {
            var scenarioFile = Path.Combine(dir, "scenario.yaml");
            var text = File.Exists(scenarioFile) ? File.ReadAllText(scenarioFile) : "";
            if (!System.Text.RegularExpressions.Regex.IsMatch(text, "^" + key + ":", System.Text.RegularExpressions.RegexOptions.Multiline))
                File.WriteAllText(scenarioFile, text.TrimEnd() + $"\n{key}: {file}\n");
        }

        /// <summary>The active PedestrianCyclistSpawner (its JSON paths) to pedestrians.yaml.</summary>
        public static string ExportPedestrians(string scenario)
        {
            var spawners = All<V2X.Paths.PedestrianCyclistSpawner>().ToList();
            var sp = spawners.FirstOrDefault(s => s.gameObject.activeInHierarchy && s.enabled) ?? spawners.FirstOrDefault();
            if (sp == null) return "pedestrians: no PedestrianCyclistSpawner";
            var so = new SerializedObject(sp);
            int peds = Refs<Transform>(so.FindProperty("pedestrianPrefabs")).Count, cyc = Refs<Transform>(so.FindProperty("cyclistPrefabs")).Count;
            float bothRatio = peds + cyc > 0 ? (float)System.Math.Round((float)cyc / (peds + cyc), 2) : 0f;
            var jsonPath = so.FindProperty("pathsJsonPath").stringValue;
            var full = Path.IsPathRooted(jsonPath) ? jsonPath : Path.Combine(Application.dataPath, "..", jsonPath);
            if (!File.Exists(full)) return $"pedestrians: {jsonPath} not found";
            var file = JsonUtility.FromJson<V2X.Paths.PathConfigLoader.PathConfigFileString>(File.ReadAllText(full));
            var cfg = new PedestriansConfig
            {
                MaxActive = so.FindProperty("maxActive").intValue, SnapToGround = so.FindProperty("snapWaypointsToGround").boolValue,
            };
            foreach (var p in file.paths ?? new List<V2X.Paths.PathConfigLoader.PathDefinitionString>())
            {
                var type = (p.type ?? "both").ToLowerInvariant();
                cfg.Paths.Add(new HumanPathConfig
                {
                    Id = p.id, SpawnsPerMinute = p.spawnPerMinute,
                    CyclistRatio = type == "pedestrian" ? 0f : type == "cyclist" ? 1f : bothRatio,
                    Waypoints = (p.waypoints ?? new List<V2X.Paths.PathConfigLoader.PathPoint>()).Select(w => new HumanWaypoint
                    {
                        X = System.Math.Round(w.x, 3), Y = System.Math.Round(w.y, 3), Z = System.Math.Round(w.z, 3),
                        TrafficLight = w.checkLight && !string.IsNullOrEmpty(w.trafficLightId) ? w.trafficLightId : null,
                    }).ToList(),
                });
            }
            var dir = Path.Combine(ConfigLoader.ConfigRoot(), ConfigLoader.ScenariosDir, scenario);
            ConfigLoader.Write(Path.Combine(dir, "pedestrians.yaml"), cfg,
                "# Pedestrians and cyclists walking / riding predefined paths.\n" +
                "# spawns_per_minute per path (±10 % jitter); cyclist_ratio: share of the spawns that are cyclists (0 = pedestrians only, 1 = cyclists only).\n" +
                "# waypoints: map frame (MGRS-local x/y/z, m). traffic_light: pedestrian light to wait for (green) before going on.\n" +
                "# max_active: agents on the paths at the same time (0 = unlimited).");
            LinkFromScenario(dir, "pedestrians", "pedestrians.yaml");
            return $"pedestrians: {cfg.Paths.Count} paths from {jsonPath} (both = cyclist ratio {bothRatio}), max {cfg.MaxActive} active";
        }

        /// <summary>The scene's Teleporters to teleports.yaml.</summary>
        public static string ExportTeleports(string scenario)
        {
            var cfg = new TeleportsConfig();
            var skippedGates = new List<string>();
            foreach (var tp in All<Teleporter>().OrderBy(t => Path_(t.transform)))
            {
                var so = new SerializedObject(tp);
                var box = tp.GetComponent<BoxCollider>();
                var size = box != null ? Vector3.Scale(box.size, tp.transform.lossyScale) : new Vector3(10, 3, 4);
                var trigger = ConfigGeometry.ToPose(tp.transform);
                if (box != null)
                {
                    var m = ConfigGeometry.UnityToMap(tp.transform.TransformPoint(box.center));
                    trigger.X = System.Math.Round(m.x, 3); trigger.Y = System.Math.Round(m.y, 3); trigger.Z = System.Math.Round(m.z, 3);
                }
                var strategy = so.FindProperty("spawnStrategy").enumValueIndex == 1 ? "random_point" : "lane_midpoint";
                var lanes = new List<long>();
                var src = so.FindProperty("laneSpawnSources");
                for (int i = 0; i < src.arraySize; i++)
                    if (src.GetArrayElementAtIndex(i).FindPropertyRelative("lane").objectReferenceValue is AWSIM.TrafficSimulation.TrafficLane l && LaneletId(l.name) >= 0)
                        lanes.Add(LaneletId(l.name));
                var mid = so.FindProperty("midpointSegment").objectReferenceValue as AWSIM.TrafficSimulation.TrafficLane;
                var window = so.FindProperty("targetArrivalWindow").vector2Value;
                var rnd = so.FindProperty("spawnDelayRange").vector2Value;
                var t = new TeleportConfig
                {
                    Name = tp.name, Enabled = tp.gameObject.activeInHierarchy && tp.enabled, Trigger = trigger,
                    TriggerSize = new MapPoint { X = System.Math.Round(size.x, 2), Y = System.Math.Round(size.y, 2), Z = System.Math.Round(size.z, 2) },
                    Strategy = strategy, ClearanceRadiusM = so.FindProperty("clearanceRadius").floatValue,
                    DelayS = so.FindProperty("teleportDelay").floatValue, RandomDelayS = new List<float> { rnd.x, rnd.y },
                    LaneMidpoint = lanes.Count == 0 && mid == null ? null : new LaneMidpointConfig
                    {
                        Lanelets = lanes, SpacingM = so.FindProperty("spawnSubdivisionSpacing").floatValue,
                        IncludeEndpoints = so.FindProperty("includeSegmentEndpoints").boolValue,
                        MidpointLanelet = mid != null ? LaneletId(mid.name) : 0, MidpointFraction = so.FindProperty("midpointSegmentFraction").floatValue,
                        CruiseSpeedKmh = so.FindProperty("cruiseSpeedKmh").floatValue, AccelerationMps2 = so.FindProperty("assumedAcceleration").floatValue,
                        ArrivalWindowS = new List<float> { window.x, window.y },
                    },
                };
                var pts = so.FindProperty("randomSpawnPoints");
                for (int i = 0; i < pts.arraySize; i++)
                    if (pts.GetArrayElementAtIndex(i).FindPropertyRelative("point").objectReferenceValue is Transform pt)
                        t.RandomPoints.Add(ConfigGeometry.ToPose(pt));
                if ((t.LaneMidpoint == null || t.LaneMidpoint.Lanelets.Count == 0) && t.RandomPoints.Count == 0)
                {
                    skippedGates.Add(Path_(tp.transform));
                    continue;   // no target configured
                }
                cfg.Teleports.Add(t);
            }
            var dir = Path.Combine(ConfigLoader.ConfigRoot(), ConfigLoader.ScenariosDir, scenario);
            ConfigLoader.Write(Path.Combine(dir, "teleports.yaml"), cfg,
                "# Teleport gates: when the ego vehicle enters the trigger box it is moved to a target, after the NPC vehicles\n" +
                "# within clearance_radius_m of the target are removed.\n" +
                "# trigger: centre and heading of the box (map frame); trigger_size: x = width across, y = height, z = length along the heading (m).\n" +
                "# lane_midpoint: target on the listed lanelets, chosen so the ego reaches midpoint_lanelet within arrival_window_s.\n" +
                "# random_point: one of random_points. lanes_root: empty = the one of vehicles.yaml.");
            LinkFromScenario(dir, "teleports", "teleports.yaml");
            return $"teleports: {cfg.Teleports.Count} gates ({cfg.Teleports.Count(t => t.Enabled)} enabled)" +
                   (skippedGates.Count > 0 ? $"; skipped (no target): {string.Join(", ", skippedGates)}" : "");
        }

        public const string LidarPrefabDir = "Assets/AWSIM/Prefabs/Sensors/RobotecGPULidars";
        public static readonly string[] CameraPrefabs =
        {
            "Assets/AWSIM/Prefabs/Sensors/CameraSensor.prefab", "Assets/AWSIM/Prefabs/Sensors/CameraC1-198.prefab",
            "Assets/AWSIM/Prefabs/Sensors/CameraC2-176.prefab",
        };

        /// <summary>LiDAR and camera prefabs that can be created from config (raw_sensor_types).</summary>
        public static List<GameObject> RawSensorPrefabs()
        {
            var list = AssetDatabase.FindAssets("t:Prefab", new[] { LidarPrefabDir })
                .Select(g => AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(g)))
                .Where(p => p != null && p.GetComponentInChildren<RGLUnityPlugin.LidarSensor>(true) != null).ToList();
            list.AddRange(CameraPrefabs.Select(AssetDatabase.LoadAssetAtPath<GameObject>).Where(p => p != null));
            return list;
        }

        [MenuItem("V2X/Config/Export Raw Sensors To Config")]
        public static void ExportRawMenu()
        {
            EditorUtility.DisplayDialog("Export raw sensors", ExportRawSensors(ConfigLoader.ResolveScenarioName(ConfigLoader.ConfigRoot(), null) ?? DefaultScenario), "OK");
        }

        public static void ExportRawBatch()
        {
            var scene = ConfigLoader.GetArg("-exportScene");
            if (!string.IsNullOrEmpty(scene))
                EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            Debug.Log("[ConfigExporter] " + ExportRawSensors(ConfigLoader.GetArg("-exportScenario") ?? DefaultScenario));
            EditorApplication.Exit(0);
        }

        /// <summary>The scene's RGL LiDARs and cameras (on the ego vehicles and elsewhere) to raw_sensors.yaml; raw sensor types to sensor_types.yaml.</summary>
        public static string ExportRawSensors(string scenario)
        {
            var cfg = new RawSensorsConfig();
            string Mount(Component c, RawSensorConfig r)
            {
                var vehicle = c.GetComponentInParent<AWSIM.Vehicle>(true);
                if (vehicle != null) { r.Vehicle = vehicle.name; r.Object = ConfigGeometry.RelativeKey(vehicle.transform, c.transform); return "ego"; }
                r.Object = Path_(c.transform);
                return "scene";
            }
            foreach (var l in All<RGLUnityPlugin.LidarSensor>().OrderBy(x => Path_(x.transform)))
            {
                var r = new RawSensorConfig { Name = l.name, Kind = "lidar", Enabled = l.gameObject.activeSelf && l.enabled, Hz = l.AutomaticCaptureHz };
                r.Mount = Mount(l, r);
                var pub = l.GetComponent<AWSIM.RglLidarPublisher>();
                if (pub != null)
                {
                    r.FrameId = pub.frameId;
                    r.PointCloudTopics = pub.pointCloud2Publishers.Select(p => new PointCloudTopic { Topic = p.topic, Format = p.fieldsPreset.ToString(), Publish = p.publish }).ToList();
                }
                cfg.RawSensors.Add(r);
            }
            foreach (var cam in All<AWSIM.CameraSensor>().OrderBy(x => Path_(x.transform)))
            {
                var r = new RawSensorConfig { Name = cam.name, Kind = "camera", Enabled = cam.gameObject.activeSelf && cam.enabled, Width = cam.ImageWidth, Height = cam.ImageHeight };
                r.Mount = Mount(cam, r);
                var holder = cam.GetComponentInParent<AWSIM.CameraSensorHolder>(true);
                if (holder != null) r.Hz = (int)holder.PublishHz;
                var pub = cam.GetComponent<AWSIM.CameraRos2Publisher>();
                if (pub != null) { r.FrameId = pub.frameId; r.ImageTopic = pub.imageTopic; r.CameraInfoTopic = pub.cameraInfoTopic; }
                cfg.RawSensors.Add(r);
            }
            // names must be unique (the web UI uses them as keys)
            foreach (var r in cfg.RawSensors.Where(r => r.Mount == "ego"))
                r.Name = r.Vehicle + "/" + r.Object;
            foreach (var g in cfg.RawSensors.GroupBy(r => r.Name).Where(g => g.Count() > 1))
            {
                int i = 0;
                foreach (var r in g) r.Name += $" ({++i})";
            }

            var dir = Path.Combine(ConfigLoader.ConfigRoot(), ConfigLoader.ScenariosDir, scenario);
            ConfigLoader.Write(Path.Combine(dir, "raw_sensors.yaml"), cfg,
                "# Raw sensors (RGL LiDARs publishing point clouds, cameras publishing images).\n" +
                "# mount: ego   = a sensor of an ego vehicle prefab, matched by vehicle + object (path below the vehicle; name[i] when names repeat)\n" +
                "#        scene = a sensor placed in the scene, matched by its scene path in object\n" +
                "#        world = a new sensor created from a raw_sensor_types entry (config/defaults/sensor_types.yaml) at pose\n" +
                "# enabled: false switches a sensor off. Raw sensors that are not listed stay as they are in the scene.\n" +
                "# lidar: hz, frame_id, point_cloud_topics (format: Pcl24, PointXYZIRCAEDT, ...). camera: image_topic, camera_info_topic,\n" +
                "# frame_id, width × height (a multiple of 4); hz is the rate of the camera holder (shared by its cameras).");
            LinkFromScenario(dir, "raw_sensors", "raw_sensors.yaml");

            // raw sensor types, keeping the pseudo-sensor types
            var typesFile = Path.Combine(ConfigLoader.ConfigRoot(), ConfigLoader.DefaultsDir, "sensor_types.yaml");
            var tmp = new LoadedConfig();
            var types = ConfigLoader.Read<SensorTypesConfig>(typesFile, tmp) ?? new SensorTypesConfig();
            types.RawSensorTypes = RawSensorPrefabs().Select(p => new RawSensorType
            {
                Name = p.name.Replace(" ", "_"), Kind = p.GetComponentInChildren<RGLUnityPlugin.LidarSensor>(true) != null ? "lidar" : "camera",
                Prefab = p.name, Description = AssetDatabase.GetAssetPath(p),
            }).ToList();
            ConfigLoader.Write(typesFile, types,
                "# Pseudo-sensor (MockSensor) types: field of view (full angles, degrees) and range (m).\n" +
                "# kind: lidar | camera. RSU sensors in config/scenarios/*/rsus.yaml refer to a type by name and may override any value.\n" +
                "# Raw sensor types: prefabs that raw_sensors.yaml can create (mount: world).");
            return $"raw sensors: {cfg.RawSensors.Count} ({cfg.RawSensors.Count(r => r.Mount == "ego")} on ego vehicles, " +
                   $"{cfg.RawSensors.Count(r => r.Mount == "scene")} in the scene, {cfg.RawSensors.Count(r => r.Enabled)} enabled); {types.RawSensorTypes.Count} raw sensor types";
        }

        static NoiseSelection Noise(SerializedObject so, System.Func<string, string> map) => new NoiseSelection
        {
            Position = map(so.FindProperty("positionNoiseName").stringValue),
            Rotation = map(so.FindProperty("rotationNoiseName").stringValue),
            Dimension = map(so.FindProperty("dimensionNoiseName").stringValue),
            Probability = map(so.FindProperty("probabilityNoiseName").stringValue),
        };

        static List<T> Refs<T>(SerializedProperty list) where T : Object
        {
            var res = new List<T>();
            if (list == null) return res;
            for (int i = 0; i < list.arraySize; i++)
                if (list.GetArrayElementAtIndex(i).objectReferenceValue is T t) res.Add(t);
            return res;
        }
    }
}
