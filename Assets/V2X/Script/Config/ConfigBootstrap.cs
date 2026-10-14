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
