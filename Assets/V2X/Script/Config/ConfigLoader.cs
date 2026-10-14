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

            Validate(c);
            return c;
        }

        static void Validate(LoadedConfig c)
        {
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
