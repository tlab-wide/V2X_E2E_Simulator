using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Loads optional spawn rates (vehicles/min) and branch weights from a JSON file or component data.
    /// </summary>
    public static class TrafficSpawnConfig
    {
        /// <summary>Log every weighted branch decision (very verbose: two lines per decision).</summary>
        public static bool VerboseBranchLogging = false;

        [Serializable]
        public class SpawnRateEntry
        {
            public string lane;
            public float spawnsPerMinute = 0f;
        }

        [Serializable]
        public class BranchWeight
        {
            public string lane;
            public float weight = 0f;
        }

        [Serializable]
        public class BranchEntry
        {
            public string fromLane;
            public List<BranchWeight> next = new List<BranchWeight>();
        }

        [Serializable]
        public class ConfigData
        {
            public List<SpawnRateEntry> spawnRates = new List<SpawnRateEntry>();
            public List<BranchEntry> branchWeights = new List<BranchEntry>();
        }

        private static readonly Dictionary<string, float> _spawnPerMinute = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);
        private static readonly Dictionary<string, List<BranchWeight>> _branchWeights = new Dictionary<string, List<BranchWeight>>(StringComparer.OrdinalIgnoreCase);
        private static readonly Dictionary<BranchDecisionKey, BranchDecisionStat> _branchDecisionStats = new Dictionary<BranchDecisionKey, BranchDecisionStat>();
        private const string DefaultBranchDecisionCsvPath = "Assets/Configs/traffic_branch_decisions.csv";
        private static bool _loaded = false;

        private struct BranchDecisionKey : IEquatable<BranchDecisionKey>
        {
            public string FromLane;
            public string ToLane;
            public string SelectionMode;

            public BranchDecisionKey(string fromLane, string toLane, string selectionMode)
            {
                FromLane = fromLane ?? string.Empty;
                ToLane = toLane ?? string.Empty;
                SelectionMode = selectionMode ?? string.Empty;
            }

            public bool Equals(BranchDecisionKey other)
            {
                return string.Equals(FromLane, other.FromLane, StringComparison.OrdinalIgnoreCase)
                    && string.Equals(ToLane, other.ToLane, StringComparison.OrdinalIgnoreCase)
                    && string.Equals(SelectionMode, other.SelectionMode, StringComparison.OrdinalIgnoreCase);
            }

            public override bool Equals(object obj)
            {
                return obj is BranchDecisionKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = StringComparer.OrdinalIgnoreCase.GetHashCode(FromLane);
                    hash = (hash * 397) ^ StringComparer.OrdinalIgnoreCase.GetHashCode(ToLane);
                    hash = (hash * 397) ^ StringComparer.OrdinalIgnoreCase.GetHashCode(SelectionMode);
                    return hash;
                }
            }
        }

        private sealed class BranchDecisionStat
        {
            public int Count;
            public float ConfiguredWeightSum;
            public float EffectiveProbabilitySum;
        }

        public static ConfigData ReadJson(string jsonPath)
        {
            if (string.IsNullOrEmpty(jsonPath))
                return null;

            string fullPath = jsonPath;
            if (!Path.IsPathRooted(fullPath))
                fullPath = Path.Combine(Application.dataPath, "..", jsonPath);
            if (!File.Exists(fullPath))
            {
                Debug.LogWarning($"[TrafficSpawnConfig] Config file not found at {fullPath}; using defaults.");
                return null;
            }

            try
            {
                var json = File.ReadAllText(fullPath);
                var data = JsonUtility.FromJson<ConfigData>(json) ?? new ConfigData();
                return data;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[TrafficSpawnConfig] Failed to read config: {ex.Message}");
                return null;
            }
        }

        public static void Load(string jsonPath)
        {
            Clear();
            var data = ReadJson(jsonPath);
            if (data == null) return;
            FillFromData(data);
            Debug.Log($"[TrafficSpawnConfig] Loaded config from {jsonPath}");
        }

        private static void Clear()
        {
            _spawnPerMinute.Clear();
            _branchWeights.Clear();
            _branchDecisionStats.Clear();
            _loaded = false;
        }

        public static bool TryGetSpawnRate(string laneName, out float spawnsPerMinute)
        {
            if (!_loaded || string.IsNullOrEmpty(laneName))
            {
                spawnsPerMinute = 0f;
                return false;
            }
            return _spawnPerMinute.TryGetValue(laneName, out spawnsPerMinute);
        }

        public static TrafficLane ChooseNextLane(TrafficLane fromLane, IList<TrafficLane> options)
        {
            if (fromLane == null || options == null || options.Count == 0)
                return null;

            if (!_loaded)
                return ChooseUniformLane(fromLane, options, "uniform_not_loaded");

            if (!_branchWeights.TryGetValue(fromLane.name, out var weights) || weights == null || weights.Count == 0)
                return ChooseUniformLane(fromLane, options, "uniform_no_config");

            // Filter weights to available options.
            float sum = 0f;
            var filtered = new List<BranchWeight>();
            foreach (var w in weights)
            {
                if (w == null || w.weight <= 0f) continue;
                for (int i = 0; i < options.Count; i++)
                {
                    if (options[i] != null && options[i].name.Equals(w.lane, StringComparison.OrdinalIgnoreCase))
                    {
                        filtered.Add(w);
                        sum += w.weight;
                        break;
                    }
                }
            }

            if (sum <= 0f || filtered.Count == 0)
                return ChooseUniformLane(fromLane, options, "uniform_no_matching_weights");

            float r = UnityEngine.Random.value * sum;
            float accum = 0f;
            // Debug the branch selection to verify weights/options.
            if (VerboseBranchLogging)
                Debug.Log($"[TrafficSpawnConfig] Choosing next lane from '{fromLane.name}' (r={r:F3}, sum={sum:F3}, options={string.Join(",", options.Where(o=>o!=null).Select(o=>o.name))})");
            foreach (var w in filtered)
            {
                accum += w.weight;
                if (r <= accum)
                {
                    foreach (var opt in options)
                    {
                        if (opt != null && opt.name.Equals(w.lane, StringComparison.OrdinalIgnoreCase))
                        {
                            RecordBranchDecision(fromLane.name, opt.name, "weighted", w.weight, w.weight / sum);
                            if (VerboseBranchLogging)
                                Debug.Log($"[TrafficSpawnConfig] Selected '{opt.name}' with weight {w.weight:F3} (accum {accum:F3})");
                            return opt;
                        }
                    }
                }
            }

            return ChooseUniformLane(fromLane, options, "uniform_weighted_fallback");
        }

        public static string ExportBranchDecisionCsv(string csvPath = null)
        {
            string fullPath = ResolveOutputPath(string.IsNullOrEmpty(csvPath) ? DefaultBranchDecisionCsvPath : csvPath);
            string directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);

            var totalsByLane = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            var totalsByLaneAndMode = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            foreach (var entry in _branchDecisionStats)
            {
                string fromLane = entry.Key.FromLane;
                string laneModeKey = BuildLaneModeKey(entry.Key.FromLane, entry.Key.SelectionMode);

                if (!totalsByLane.ContainsKey(fromLane))
                    totalsByLane[fromLane] = 0;
                totalsByLane[fromLane] += entry.Value.Count;

                if (!totalsByLaneAndMode.ContainsKey(laneModeKey))
                    totalsByLaneAndMode[laneModeKey] = 0;
                totalsByLaneAndMode[laneModeKey] += entry.Value.Count;
            }

            var orderedEntries = _branchDecisionStats
                .OrderBy(x => x.Key.FromLane, StringComparer.OrdinalIgnoreCase)
                .ThenBy(x => x.Key.SelectionMode, StringComparer.OrdinalIgnoreCase)
                .ThenBy(x => x.Key.ToLane, StringComparer.OrdinalIgnoreCase)
                .ToList();

            var csv = new StringBuilder();
            csv.AppendLine("from_lane,to_lane,selection_mode,decision_count,total_from_lane,total_from_lane_mode,observed_ratio_from_lane,observed_ratio_from_lane_mode,avg_configured_weight,avg_effective_probability");
            foreach (var entry in orderedEntries)
            {
                BranchDecisionKey key = entry.Key;
                BranchDecisionStat stat = entry.Value;
                int totalFromLane = totalsByLane.TryGetValue(key.FromLane, out int laneTotal) ? laneTotal : 0;
                int totalFromLaneMode = totalsByLaneAndMode.TryGetValue(BuildLaneModeKey(key.FromLane, key.SelectionMode), out int laneModeTotal) ? laneModeTotal : 0;

                float observedRatioFromLane = totalFromLane > 0 ? (float)stat.Count / totalFromLane : 0f;
                float observedRatioFromLaneMode = totalFromLaneMode > 0 ? (float)stat.Count / totalFromLaneMode : 0f;
                float avgConfiguredWeight = stat.Count > 0 ? stat.ConfiguredWeightSum / stat.Count : 0f;
                float avgEffectiveProbability = stat.Count > 0 ? stat.EffectiveProbabilitySum / stat.Count : 0f;

                csv.Append(EscapeCsv(key.FromLane)).Append(',')
                    .Append(EscapeCsv(key.ToLane)).Append(',')
                    .Append(EscapeCsv(key.SelectionMode)).Append(',')
                    .Append(stat.Count).Append(',')
                    .Append(totalFromLane).Append(',')
                    .Append(totalFromLaneMode).Append(',')
                    .Append(observedRatioFromLane.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(observedRatioFromLaneMode.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(avgConfiguredWeight.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(avgEffectiveProbability.ToString("F6", CultureInfo.InvariantCulture))
                    .AppendLine();
            }

            File.WriteAllText(fullPath, csv.ToString());
            Debug.Log($"[TrafficSpawnConfig] Exported branch decision stats to {fullPath}");
            return fullPath;
        }

        public static void Load(ConfigData data)
        {
            Clear();
            FillFromData(data);
            Debug.Log("[TrafficSpawnConfig] Loaded config from component data.");
        }

        private static void FillFromData(ConfigData data)
        {
            if (data == null) return;

            if (data.spawnRates != null)
            {
                foreach (var sr in data.spawnRates)
                {
                    if (sr == null || string.IsNullOrEmpty(sr.lane)) continue;
                    _spawnPerMinute[sr.lane] = Mathf.Max(0f, sr.spawnsPerMinute);
                }
            }

            if (data.branchWeights != null)
            {
                foreach (var bw in data.branchWeights)
                {
                    if (bw == null || string.IsNullOrEmpty(bw.fromLane) || bw.next == null) continue;
                    float sum = 0f;
                    foreach (var n in bw.next)
                    {
                        if (n == null || string.IsNullOrEmpty(n.lane)) continue;
                        if (n.weight > 0f) sum += n.weight;
                    }
                    if (sum <= 0f) continue;

                    var normalized = new List<BranchWeight>();
                    foreach (var n in bw.next)
                    {
                        if (n == null || string.IsNullOrEmpty(n.lane) || n.weight <= 0f) continue;
                        normalized.Add(new BranchWeight { lane = n.lane, weight = n.weight / sum });
                    }
                    if (normalized.Count > 0)
                        _branchWeights[bw.fromLane] = normalized;
                }
            }

            _loaded = true;
        }

        private static TrafficLane ChooseUniformLane(TrafficLane fromLane, IList<TrafficLane> options, string selectionMode)
        {
            TrafficLane selected = RandomTrafficUtils.GetRandomElement(options);
            if (fromLane != null && selected != null)
            {
                float effectiveProbability = options.Count > 0 ? 1f / options.Count : 0f;
                RecordBranchDecision(fromLane.name, selected.name, selectionMode, 0f, effectiveProbability);
            }
            return selected;
        }

        private static void RecordBranchDecision(string fromLane, string toLane, string selectionMode, float configuredWeight, float effectiveProbability)
        {
            if (string.IsNullOrEmpty(fromLane) || string.IsNullOrEmpty(toLane))
                return;

            BranchDecisionKey key = new BranchDecisionKey(fromLane, toLane, selectionMode);
            if (!_branchDecisionStats.TryGetValue(key, out BranchDecisionStat stat))
            {
                stat = new BranchDecisionStat();
                _branchDecisionStats[key] = stat;
            }

            stat.Count++;
            stat.ConfiguredWeightSum += configuredWeight;
            stat.EffectiveProbabilitySum += effectiveProbability;
        }

        private static string ResolveOutputPath(string csvPath)
        {
            string fullPath = csvPath;
            if (!Path.IsPathRooted(fullPath))
                fullPath = Path.Combine(Application.dataPath, "..", csvPath);
            return fullPath;
        }

        private static string BuildLaneModeKey(string fromLane, string selectionMode)
        {
            return $"{fromLane}::{selectionMode}";
        }

        private static string EscapeCsv(string value)
        {
            if (string.IsNullOrEmpty(value))
                return string.Empty;

            if (value.Contains(",") || value.Contains("\"") || value.Contains("\n") || value.Contains("\r"))
                return $"\"{value.Replace("\"", "\"\"")}\"";

            return value;
        }
    }
}
