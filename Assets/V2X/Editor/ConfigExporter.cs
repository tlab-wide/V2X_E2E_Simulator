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
