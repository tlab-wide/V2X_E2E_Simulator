// Data model of the simulator configuration (config/ folder, YAML, snake_case keys).
//
// config/defaults/      shared defaults: sensor types, noise profiles, network profiles
// config/scenarios/<s>/ one folder per scenario: scenario.yaml + the files it lists
//
// Positions are in the map frame (MGRS-local x/y/z in metres, the frame of Autoware and the Lanelet2 map).
// Headings are clockwise from north in degrees; pitch > 0 looks up. Fields of view are full angles.
using System.Collections.Generic;

namespace V2X.Config
{
    // ------------------------------------------------------------------------------------------------- scenario
    public class ScenarioConfig
    {
        public string Name { get; set; } = "";
        public string Description { get; set; } = "";
        // Files of this scenario folder, relative to it. Empty or missing = not configured (the scene objects stay).
        public string Rsus { get; set; } = "rsus.yaml";
        public string Vehicles { get; set; } = "";
        public string Pedestrians { get; set; } = "";
        public string Teleports { get; set; } = "";
    }

    // ------------------------------------------------------------------------------------------------- pedestrians / cyclists
    public class PedestriansConfig
    {
        public int MaxActive { get; set; } = 30;                // agents walking/riding at the same time (0 = unlimited)
        public bool SnapToGround { get; set; } = true;
        public List<HumanPathConfig> Paths { get; set; } = new List<HumanPathConfig>();
    }

    public class HumanPathConfig
    {
        public string Id { get; set; } = "";
        public float SpawnsPerMinute { get; set; }
        public float CyclistRatio { get; set; }                 // share of spawns that are cyclists: 0 = pedestrians only, 1 = cyclists only
        public bool Enabled { get; set; } = true;
        public List<HumanWaypoint> Waypoints { get; set; } = new List<HumanWaypoint>();
    }

    public class HumanWaypoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public string TrafficLight { get; set; }                // pedestrian light to wait for (green) before going on, e.g. Kakeiken_P_0x30_1
    }

    // ------------------------------------------------------------------------------------------------- teleports
    public class TeleportsConfig
    {
        public string LanesRoot { get; set; } = "";             // empty = lanes_root of vehicles.yaml
        public List<TeleportConfig> Teleports { get; set; } = new List<TeleportConfig>();
    }

    public class TeleportConfig
    {
        public string Name { get; set; } = "";
        public bool Enabled { get; set; } = true;
        public Pose Trigger { get; set; } = new Pose();          // centre and heading of the trigger box
        public MapPoint TriggerSize { get; set; } = new MapPoint { X = 10, Y = 3, Z = 4 };   // width (across), height, length (along heading), m
        public string Strategy { get; set; } = "lane_midpoint";  // lane_midpoint | random_point
        public float ClearanceRadiusM { get; set; } = 10f;       // NPC vehicles within this radius of the target are removed
        public float DelayS { get; set; } = 0.5f;
        public List<float> RandomDelayS { get; set; } = new List<float> { 0f, 0f };
        public LaneMidpointConfig LaneMidpoint { get; set; }
        public List<Pose> RandomPoints { get; set; } = new List<Pose>();
    }

    public class LaneMidpointConfig
    {
        public List<long> Lanelets { get; set; } = new List<long>();    // candidate lanes for the target
        public float SpacingM { get; set; } = 10f;
        public bool IncludeEndpoints { get; set; } = true;
        public long MidpointLanelet { get; set; }
        public float MidpointFraction { get; set; } = 0.5f;
        public float CruiseSpeedKmh { get; set; } = 30f;
        public float AccelerationMps2 { get; set; } = 2f;
        public List<float> ArrivalWindowS { get; set; } = new List<float> { 0f, 90f };
    }

    // ------------------------------------------------------------------------------------------------- vehicles
    public class VehiclesConfig
    {
        // Scene path of the TrafficLanes object whose lanes (named lanelet<ID>) are used
        public string LanesRoot { get; set; } = "";
        // Scene path of the TrafficManager whose vehicle dynamics, layers and ego are used (empty = the first enabled one)
        public string TemplateTrafficManager { get; set; } = "";
        public int Seed { get; set; }
        public int MaxVehicles { get; set; } = 60;
        public float SpawnDistanceToEgoM { get; set; } = 50f;   // no spawning closer than this to the ego vehicle
        public float SpawnClearanceM { get; set; } = 4f;        // free space needed in front of / behind a new vehicle
        public int MaximumSpawns { get; set; }                  // 0 = unlimited
        public TrafficBehaviourConfig Behaviour { get; set; } = new TrafficBehaviourConfig();
        public List<VehicleTypeConfig> VehicleTypes { get; set; } = new List<VehicleTypeConfig>();
        public List<SpawnPointConfig> SpawnPoints { get; set; } = new List<SpawnPointConfig>();
        public List<BranchConfig> Branches { get; set; } = new List<BranchConfig>();
    }

    public class TrafficBehaviourConfig
    {
        public float DeadlockTimeoutS { get; set; } = 8f;       // standing still while yielding this long -> right of way
        public float ForcedPassS { get; set; } = 6f;
        public float ForcedPassRadiusM { get; set; } = 70f;
        public float StuckDespawnS { get; set; } = 180f;        // standing still this long for any reason -> removed
        public float StandstillSpeedMps { get; set; } = 0.3f;
    }

    public class VehicleTypeConfig
    {
        public string Prefab { get; set; } = "";               // NPC vehicle prefab name (from the scene's traffic managers)
        public float Weight { get; set; } = 1f;
    }

    public class SpawnPointConfig
    {
        public string Name { get; set; } = "";
        public long Lanelet { get; set; }                       // Lanelet2 lanelet ID; vehicles start at its first point
        public float VehiclesPerMinute { get; set; }
        public MapPoint Position { get; set; }                  // informational (map frame), e.g. where it was picked
    }

    public class MapPoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }

    public class BranchConfig
    {
        public long FromLanelet { get; set; }
        public List<BranchTarget> To { get; set; } = new List<BranchTarget>();
    }

    public class BranchTarget
    {
        public long Lanelet { get; set; }
        public float Ratio { get; set; }
    }

    // ------------------------------------------------------------------------------------------------- defaults
    public class SensorTypesConfig
    {
        public List<PseudoSensorType> PseudoSensorTypes { get; set; } = new List<PseudoSensorType>();
    }

    public class PseudoSensorType
    {
        public string Name { get; set; } = "";
        public string Description { get; set; } = "";
        public string Kind { get; set; } = "lidar";            // lidar | camera
        public float HFovDeg { get; set; } = 90f;
        public float VFovDeg { get; set; } = 60f;
        public float RangeM { get; set; } = 50f;
    }

    public class NoiseConfig
    {
        public List<NoiseProfile> NoiseProfiles { get; set; } = new List<NoiseProfile>();
    }

    public class NoiseProfile
    {
        public string Name { get; set; } = "";
        public string Distribution { get; set; } = "uniform";  // uniform | gaussian
        // uniform: ± amplitude per axis (m). gaussian: the standard deviation is gaussian_std_dev × amplitude.
        public float XM { get; set; }
        public float YM { get; set; }
        public float ZM { get; set; }
        public float GaussianMean { get; set; }
        public float GaussianStdDev { get; set; } = 0.25f;
    }

    public class NetworkConfig
    {
        public List<NetworkProfile> NetworkProfiles { get; set; } = new List<NetworkProfile>();
    }

    public class NetworkProfile
    {
        public string Name { get; set; } = "";
        public string TopicSuffix { get; set; } = "";
        public float DelayMs { get; set; }
        public float DelayVarianceMs { get; set; }
        public float PacketLoss { get; set; }                 // 0 … 1
        public float PacketLossVariance { get; set; }
        public bool GroundTruth { get; set; }                 // copy the ground-truth topic instead of the noisy one
    }

    // ------------------------------------------------------------------------------------------------- RSUs
    public class RsusConfig
    {
        public List<RsuConfig> Rsus { get; set; } = new List<RsuConfig>();
        public List<ObjectInfoPublisherConfig> ObjectInfoPublishers { get; set; } = new List<ObjectInfoPublisherConfig>();
        // Rate and noise of the per-sensor topics (sensors with object_info set)
        public PerSensorPublisherSettings PerSensorObjectInfo { get; set; } = new PerSensorPublisherSettings();
        // Autoware PredictedObjects of the objects seen by the RSU sensors
        public List<AutowareObjectPublisherConfig> AutowareObjectPublishers { get; set; } = new List<AutowareObjectPublisherConfig>();
        // v2x_msgs/V2XSignals with the state of the vehicle traffic lights of intersections
        public List<TrafficSignalPublisherConfig> TrafficSignalPublishers { get; set; } = new List<TrafficSignalPublisherConfig>();
    }

    public class AutowareObjectPublisherConfig
    {
        public string Name { get; set; } = "";
        public List<string> Rsus { get; set; } = new List<string>();
        public string Topic { get; set; } = "";
        public int StationId { get; set; } = 1;
        public float Hz { get; set; } = 10f;
        public NoiseSelection Noise { get; set; } = new NoiseSelection();
        public List<string> Network { get; set; } = new List<string>();
    }

    public class TrafficSignalPublisherConfig
    {
        public string Name { get; set; } = "";
        public int StationId { get; set; } = 1;
        public List<string> Intersections { get; set; } = new List<string>();   // TrafficIntersection GameObject names
        public List<string> Topics { get; set; } = new List<string>();
        public int Hz { get; set; } = 10;
        public string SignalId { get; set; } = "relation";                       // relation | way (Lanelet2 IDs in the message)
    }

    public class PerSensorPublisherSettings
    {
        public float Hz { get; set; } = 10f;
        public NoiseSelection Noise { get; set; } = new NoiseSelection();
    }

    public class RsuConfig
    {
        public string Name { get; set; } = "";
        public string RsuId { get; set; } = "0x00000000";      // 32-bit device ID, hex
        public string Location { get; set; } = "";             // free text, e.g. the intersection
        public bool Enabled { get; set; } = true;
        public List<PseudoSensorConfig> Sensors { get; set; } = new List<PseudoSensorConfig>();
    }

    public class PseudoSensorConfig
    {
        public string Name { get; set; } = "";
        public string Type { get; set; } = "";                 // a pseudo_sensor_types name
        public int SensorId { get; set; } = 1;                 // unique within the RSU
        public bool Enabled { get; set; } = true;
        public Pose Pose { get; set; } = new Pose();
        // Optional overrides of the type
        public float? HFovDeg { get; set; }
        public float? VFovDeg { get; set; }
        public float? RangeM { get; set; }
        // Optional per-sensor object-info topics (no network copies)
        public PerSensorTopics ObjectInfo { get; set; }
    }

    public class PerSensorTopics
    {
        public string Topic { get; set; } = "";
        public string GroundTruthTopic { get; set; } = "";
    }

    public class Pose
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public float HeadingDeg { get; set; }
        public float PitchDeg { get; set; }
        public float RollDeg { get; set; }
    }

    public class ObjectInfoPublisherConfig
    {
        public string Name { get; set; } = "";
        public List<string> Rsus { get; set; } = new List<string>();   // RSU names; ["all"] = every enabled RSU
        public string Topic { get; set; } = "";
        public string GroundTruthTopic { get; set; } = "";
        public float Hz { get; set; } = 10f;
        public NoiseSelection Noise { get; set; } = new NoiseSelection();
        public List<string> Network { get; set; } = new List<string>();  // network profile names
    }

    public class NoiseSelection
    {
        public string Position { get; set; } = "Default";
        public string Rotation { get; set; } = "Default";
        public string Dimension { get; set; } = "Default";
        public string Probability { get; set; } = "Default";
    }
}
