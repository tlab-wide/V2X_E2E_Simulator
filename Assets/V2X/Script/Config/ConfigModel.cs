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
