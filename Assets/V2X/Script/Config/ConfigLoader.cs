// Finds and reads the simulator configuration (YAML).
//
// Config root: --config-root <dir> on the command line, else <project>/config in the editor and <build folder>/config
// in a player build. Scenario: --scenario <name> on the command line, else the name given by the caller
// (the inspector of ConfigBootstrap), else the one in config/default_scenario.txt.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace V2X.Config
{
    public class LoadedConfig
    {
        public string Root;
        public string ScenarioName;
        public string ScenarioDir;
        public ScenarioConfig Scenario;
        public SensorTypesConfig SensorTypes;
        public NoiseConfig Noise;
        public NetworkConfig Network;
        public RsusConfig Rsus;                        // null when the scenario does not configure RSUs
        public VehiclesConfig Vehicles;                // null when the scenario does not configure vehicle traffic
        public PedestriansConfig Pedestrians;
        public TeleportsConfig Teleports;
        public RawSensorsConfig RawSensors;
        public SimulationConfig Simulation;
        public SignalsConfig Signals;
        public RawSensorType FindRawSensorType(string name) => SensorTypes?.RawSensorTypes.FirstOrDefault(t => t.Name == name);
        public readonly List<string> Errors = new List<string>();
        public readonly List<string> Warnings = new List<string>();

        public PseudoSensorType FindSensorType(string name) => SensorTypes?.PseudoSensorTypes.FirstOrDefault(t => t.Name == name);
        public NetworkProfile FindNetwork(string name) => Network?.NetworkProfiles.FirstOrDefault(n => n.Name == name);
        public bool HasNoise(string name) => Noise?.NoiseProfiles.Any(n => n.Name == name) ?? false;
    }

    public static class ConfigLoader
    {
        public const string DefaultsDir = "defaults";
        public const string ScenariosDir = "scenarios";

        static IDeserializer deserializer;
        static ISerializer serializer;

        static IDeserializer Deserializer => deserializer ??= new DeserializerBuilder()
            .WithNamingConvention(UnderscoredNamingConvention.Instance)
            .Build();

        public static ISerializer Serializer => serializer ??= new SerializerBuilder()
            .WithNamingConvention(UnderscoredNamingConvention.Instance)
            .ConfigureDefaultValuesHandling(DefaultValuesHandling.OmitNull)
            .Build();

        public static string GetArg(string name)
        {
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i] == name)
                    return args[i + 1];
            return null;
        }

        public static string ConfigRoot()
        {
            var arg = GetArg("--config-root");
            if (!string.IsNullOrEmpty(arg))
                return Path.GetFullPath(arg);
            // Application.dataPath = <project>/Assets in the editor, <build>/<name>_Data in a player
            return Path.GetFullPath(Path.Combine(Application.dataPath, "..", "config"));
        }

        public static string ResolveScenarioName(string root, string requested)
        {
            var arg = GetArg("--scenario");
            if (!string.IsNullOrEmpty(arg)) return arg;
            if (!string.IsNullOrEmpty(requested)) return requested;
            var file = Path.Combine(root, "default_scenario.txt");
            return File.Exists(file) ? File.ReadAllText(file).Trim() : null;
        }

        /// <summary>"HH:MM" or "HH:MM:SS" -> hours since midnight.</summary>
        public static bool TryParseTimeOfDay(string text, out double hours)
        {
            hours = 0;
            var parts = (text ?? "").Split(':');
            if (parts.Length < 2 || parts.Length > 3) return false;
            if (!int.TryParse(parts[0], out var h) || !int.TryParse(parts[1], out var m) || h < 0 || h > 23 || m < 0 || m > 59) return false;
            int sec = 0;
            if (parts.Length == 3 && (!int.TryParse(parts[2], out sec) || sec < 0 || sec > 59)) return false;
            hours = h + m / 60.0 + sec / 3600.0;
            return true;
        }

        public static List<string> ListScenarios(string root)
        {
            var dir = Path.Combine(root, ScenariosDir);
            if (!Directory.Exists(dir)) return new List<string>();
            return Directory.GetDirectories(dir).Where(d => File.Exists(Path.Combine(d, "scenario.yaml")))
                .Select(Path.GetFileName).OrderBy(n => n).ToList();
        }

        public static T Read<T>(string path, LoadedConfig into) where T : class, new()
        {
            if (!File.Exists(path))
            {
                into.Errors.Add($"missing file: {path}");
                return null;
            }
            try
            {
                return Deserializer.Deserialize<T>(File.ReadAllText(path)) ?? new T();
            }
            catch (Exception e)
            {
                into.Errors.Add($"{path}: {e.Message}{(e.InnerException != null ? " (" + e.InnerException.Message + ")" : "")}");
                return null;
            }
        }

        public static void Write<T>(string path, T value, string header = null)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var text = Serializer.Serialize(value);
            File.WriteAllText(path, (header != null ? header.TrimEnd() + "\n\n" : "") + text);
        }

        /// <summary>Reads the defaults and the scenario. Errors and warnings are collected, never thrown.</summary>
        public static LoadedConfig Load(string requestedScenario = null)
        {
            var c = new LoadedConfig { Root = ConfigRoot() };
            if (!Directory.Exists(c.Root))
            {
                c.Errors.Add($"config folder not found: {c.Root}");
                return c;
            }
            var defaults = Path.Combine(c.Root, DefaultsDir);
            c.SensorTypes = Read<SensorTypesConfig>(Path.Combine(defaults, "sensor_types.yaml"), c);
            c.Noise = Read<NoiseConfig>(Path.Combine(defaults, "noise.yaml"), c);
            c.Network = Read<NetworkConfig>(Path.Combine(defaults, "network.yaml"), c);

            c.ScenarioName = ResolveScenarioName(c.Root, requestedScenario);
            if (string.IsNullOrEmpty(c.ScenarioName))
            {
                c.Errors.Add("no scenario selected (--scenario, ConfigBootstrap.scenario or config/default_scenario.txt)");
                return c;
            }
            c.ScenarioDir = Path.Combine(c.Root, ScenariosDir, c.ScenarioName);
            c.Scenario = Read<ScenarioConfig>(Path.Combine(c.ScenarioDir, "scenario.yaml"), c);
            if (c.Scenario == null)
                return c;
            if (!string.IsNullOrEmpty(c.Scenario.Rsus))
                c.Rsus = Read<RsusConfig>(Path.Combine(c.ScenarioDir, c.Scenario.Rsus), c);
            if (!string.IsNullOrEmpty(c.Scenario.Vehicles))
                c.Vehicles = Read<VehiclesConfig>(Path.Combine(c.ScenarioDir, c.Scenario.Vehicles), c);
            if (!string.IsNullOrEmpty(c.Scenario.Pedestrians))
                c.Pedestrians = Read<PedestriansConfig>(Path.Combine(c.ScenarioDir, c.Scenario.Pedestrians), c);
            if (!string.IsNullOrEmpty(c.Scenario.Teleports))
                c.Teleports = Read<TeleportsConfig>(Path.Combine(c.ScenarioDir, c.Scenario.Teleports), c);
            if (!string.IsNullOrEmpty(c.Scenario.RawSensors))
                c.RawSensors = Read<RawSensorsConfig>(Path.Combine(c.ScenarioDir, c.Scenario.RawSensors), c);
            if (!string.IsNullOrEmpty(c.Scenario.Simulation))
                c.Simulation = Read<SimulationConfig>(Path.Combine(c.ScenarioDir, c.Scenario.Simulation), c);
            if (!string.IsNullOrEmpty(c.Scenario.Signals))
                c.Signals = Read<SignalsConfig>(Path.Combine(c.ScenarioDir, c.Scenario.Signals), c);

            Validate(c);
            return c;
        }

        static void Validate(LoadedConfig c)
        {
            if (c.Simulation != null)
            {
                var sim = c.Simulation;
                if (sim.TimeScale <= 0f || sim.TimeScale > 100f) c.Errors.Add($"simulation: time_scale {sim.TimeScale} must be > 0 and <= 100");
                var l = sim.Lighting;
                if (l != null)
                {
                    if (!string.IsNullOrEmpty(l.TimeOfDay) && !TryParseTimeOfDay(l.TimeOfDay, out _))
                        c.Errors.Add($"simulation: lighting.time_of_day '{l.TimeOfDay}' is not HH:MM");
                    if (!string.IsNullOrEmpty(l.TimeOfDay) && !System.DateTime.TryParse(l.Date, System.Globalization.CultureInfo.InvariantCulture,
                            System.Globalization.DateTimeStyles.None, out _))
                        c.Errors.Add($"simulation: lighting.date '{l.Date}' is not YYYY-MM-DD");
                    if (l.SunElevationDeg.HasValue != l.SunAzimuthDeg.HasValue)
                        c.Errors.Add("simulation: set sun_elevation_deg and sun_azimuth_deg together");
                }
                if (sim.Weather != null && sim.Weather.FogVisibilityM < 0f) c.Errors.Add("simulation: weather.fog_visibility_m < 0");
            }
            if (c.Signals != null)
            {
                var names = new HashSet<string>();
                foreach (var i in c.Signals.Intersections)
                {
                    if (string.IsNullOrEmpty(i.Name) || !names.Add(i.Name)) c.Errors.Add($"signals: intersection '{i.Name}' is empty or listed twice");
                    if (i.Steps.Count == 0 || i.Steps.Any(st => st.DurationS <= 0f))
                        c.Errors.Add($"signals: {i.Name}: needs steps, each with duration_s > 0");
                }
            }
            if (c.Pedestrians != null)
            {
                var ids = new HashSet<string>();
                foreach (var p in c.Pedestrians.Paths)
                {
                    if (string.IsNullOrEmpty(p.Id) || !ids.Add(p.Id)) c.Errors.Add($"pedestrians: path id '{p.Id}' is empty or used twice");
                    if (!p.Enabled) continue;
                    if (p.CyclistRatio < 0f || p.CyclistRatio > 1f) c.Errors.Add($"pedestrians: path {p.Id}: cyclist_ratio must be 0-1");
                    if (p.SpawnsPerMinute < 0f) c.Errors.Add($"pedestrians: path {p.Id}: spawns_per_minute < 0");
                    if (p.Waypoints.Count < 2) c.Errors.Add($"pedestrians: path {p.Id}: needs at least 2 waypoints");
                }
            }
            if (c.RawSensors != null)
            {
                foreach (var s in c.RawSensors.RawSensors)
                {
                    if (s.Mount != "ego" && s.Mount != "scene" && s.Mount != "world") c.Errors.Add($"raw sensor {s.Name}: mount must be ego, scene or world");
                    if (s.Kind != "lidar" && s.Kind != "camera") c.Errors.Add($"raw sensor {s.Name}: kind must be lidar or camera");
                    if (s.Mount != "world" && string.IsNullOrEmpty(s.Object)) c.Errors.Add($"raw sensor {s.Name}: {s.Mount} sensors need object (name in the vehicle / scene path)");
                    if (s.Mount == "world" && s.Enabled && (c.FindRawSensorType(s.Type) == null || s.Pose == null))
                        c.Errors.Add($"raw sensor {s.Name}: world sensors need a known type and a pose");
                    if (s.Width.HasValue != s.Height.HasValue || (s.Width.HasValue && (s.Width * s.Height) % 4 != 0))
                        c.Errors.Add($"raw sensor {s.Name}: set width and height together (width × height a multiple of 4)");
                }
            }
            if (c.Teleports != null)
            {
                foreach (var t in c.Teleports.Teleports.Where(t => t.Enabled))
                {
                    if (t.Strategy != "lane_midpoint" && t.Strategy != "random_point") c.Errors.Add($"teleport {t.Name}: strategy must be lane_midpoint or random_point");
                    if (t.Strategy == "lane_midpoint" && (t.LaneMidpoint == null || t.LaneMidpoint.Lanelets.Count == 0)) c.Errors.Add($"teleport {t.Name}: lane_midpoint needs lanelets");
                    if (t.Strategy == "random_point" && t.RandomPoints.Count == 0) c.Errors.Add($"teleport {t.Name}: random_point needs random_points");
                }
                if (string.IsNullOrEmpty(c.Teleports.LanesRoot) && c.Vehicles == null && c.Teleports.Teleports.Any(t => t.Strategy == "lane_midpoint"))
                    c.Errors.Add("teleports: lanes_root is empty and there is no vehicles.yaml to take it from");
            }
            if (c.Vehicles != null)
            {
                var v = c.Vehicles;
                if (string.IsNullOrEmpty(v.LanesRoot)) c.Errors.Add("vehicles: lanes_root is empty");
                if (v.VehicleTypes.Count == 0) c.Errors.Add("vehicles: no vehicle_types");
                if (v.MaxVehicles < 0) c.Errors.Add("vehicles: max_vehicles < 0");
                foreach (var s in v.SpawnPoints)
                    if (s.VehiclesPerMinute < 0) c.Errors.Add($"vehicles: spawn point lanelet {s.Lanelet}: vehicles_per_minute < 0");
                foreach (var b in v.Branches)
                    if (b.To.Count == 0 || b.To.Any(t => t.Ratio < 0)) c.Errors.Add($"vehicles: branch from lanelet {b.FromLanelet}: needs targets with ratio >= 0");
            }
            if (c.Rsus == null) return;
            var rsuNames = new HashSet<string>();
            var rsuIds = new HashSet<uint>();
            foreach (var rsu in c.Rsus.Rsus)
            {
                if (!rsuNames.Add(rsu.Name)) c.Errors.Add($"RSU name '{rsu.Name}' is used twice");
                if (!TryParseHex(rsu.RsuId, out var id)) c.Errors.Add($"RSU {rsu.Name}: rsu_id '{rsu.RsuId}' is not a 32-bit hex number");
                else if (!rsuIds.Add(id)) c.Errors.Add($"RSU {rsu.Name}: rsu_id {rsu.RsuId} is used by another RSU");
                var sensorIds = new HashSet<int>();
                foreach (var s in rsu.Sensors)
                {
                    if (c.FindSensorType(s.Type) == null) c.Errors.Add($"RSU {rsu.Name} / {s.Name}: unknown sensor type '{s.Type}'");
                    if (s.SensorId < 0 || s.SensorId > 255) c.Errors.Add($"RSU {rsu.Name} / {s.Name}: sensor_id {s.SensorId} is not 0-255");
                    else if (s.Enabled && !sensorIds.Add(s.SensorId)) c.Errors.Add($"RSU {rsu.Name} / {s.Name}: sensor_id {s.SensorId} is used twice in this RSU");
                }
            }
            void CheckNoise(string where, NoiseSelection n)
            {
                foreach (var name in new[] { n.Position, n.Rotation, n.Dimension, n.Probability })
                    if (!c.HasNoise(name)) c.Errors.Add($"{where}: unknown noise profile '{name}'");
            }
            foreach (var p in c.Rsus.ObjectInfoPublishers)
            {
                foreach (var r in p.Rsus)
                    if (r != "all" && !rsuNames.Contains(r)) c.Errors.Add($"object-info publisher {p.Name}: unknown RSU '{r}'");
                foreach (var n in p.Network)
                    if (c.FindNetwork(n) == null) c.Errors.Add($"object-info publisher {p.Name}: unknown network profile '{n}'");
                if (string.IsNullOrEmpty(p.Topic)) c.Errors.Add($"object-info publisher {p.Name}: topic is empty");
                CheckNoise($"object-info publisher {p.Name}", p.Noise);
            }
            CheckNoise("per_sensor_object_info", c.Rsus.PerSensorObjectInfo.Noise);
            foreach (var p in c.Rsus.AutowareObjectPublishers)
            {
                foreach (var r in p.Rsus)
                    if (r != "all" && !rsuNames.Contains(r)) c.Errors.Add($"autoware-object publisher {p.Name}: unknown RSU '{r}'");
                foreach (var n in p.Network)
                    if (c.FindNetwork(n) == null) c.Errors.Add($"autoware-object publisher {p.Name}: unknown network profile '{n}'");
                if (string.IsNullOrEmpty(p.Topic)) c.Errors.Add($"autoware-object publisher {p.Name}: topic is empty");
                CheckNoise($"autoware-object publisher {p.Name}", p.Noise);
            }
            foreach (var p in c.Rsus.TrafficSignalPublishers)
            {
                if (p.Topics.Count == 0) c.Errors.Add($"traffic-signal publisher {p.Name}: no topics");
                if (p.SignalId != "relation" && p.SignalId != "way") c.Errors.Add($"traffic-signal publisher {p.Name}: signal_id must be relation or way");
                if (p.Hz < 1 || p.Hz > 100) c.Errors.Add($"traffic-signal publisher {p.Name}: hz must be 1-100");
            }
        }

        public static bool TryParseHex(string text, out uint value)
        {
            value = 0;
            if (string.IsNullOrEmpty(text)) return false;
            var t = text.StartsWith("0x", StringComparison.OrdinalIgnoreCase) ? text.Substring(2) : text;
            return uint.TryParse(t, System.Globalization.NumberStyles.HexNumber, null, out value);
        }
    }
}
