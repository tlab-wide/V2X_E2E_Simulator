using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Loads optional spawn rates (vehicles/min) and branch weights from a JSON file or component data.
    /// </summary>
    public static class TrafficSpawnConfig
    {
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
        private static bool _loaded = false;

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
            if (fromLane == null || options == null || options.Count == 0 || !_loaded)
                return RandomTrafficUtils.GetRandomElement(options);

            if (!_branchWeights.TryGetValue(fromLane.name, out var weights) || weights == null || weights.Count == 0)
                return RandomTrafficUtils.GetRandomElement(options);

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
                return RandomTrafficUtils.GetRandomElement(options);

            float r = UnityEngine.Random.value * sum;
            float accum = 0f;
            // Debug the branch selection to verify weights/options.
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
                            Debug.Log($"[TrafficSpawnConfig] Selected '{opt.name}' with weight {w.weight:F3} (accum {accum:F3})");
                            return opt;
                        }
                    }
                }
            }

            return RandomTrafficUtils.GetRandomElement(options);
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
    }
}
