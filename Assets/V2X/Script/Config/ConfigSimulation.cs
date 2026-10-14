// Applies simulation.yaml (time scale, seed, ego vehicle, sun and fog, loggers) and signals.yaml (signal step durations)
// to the scene. Called by ConfigBootstrap.Awake before anything else is set up, so the ego and the signal timing are
// in place before the traffic managers, intersections and loggers start.
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace V2X.Config
{
    public static class ConfigSimulation
    {
        const BindingFlags Any = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

        // The chosen ego stays inactive while the rest of the config is applied (its sensors are configured first),
        // and Finish() activates it at the end of ConfigBootstrap.Awake.
        static GameObject pendingEgo;

        static void SetField(object target, string name, object value)
        {
            var f = target.GetType().GetField(name, Any);
            if (f == null) Debug.LogWarning($"[Config] {target.GetType().Name}.{name} not found");
            else f.SetValue(target, value);
        }

        static object GetField(object target, string name) => target.GetType().GetField(name, Any)?.GetValue(target);

        // ------------------------------------------------------------------------------------------------ simulation
        public static void Apply(SimulationConfig sim)
        {
            Time.timeScale = sim.TimeScale;
            AWSIM.TimeScaleProvider.DoUpdate();       // the ROS clock thread reads the cached value
            if (sim.Seed.HasValue)
            {
                int seed = sim.Seed.Value;
                Random.InitState(seed);
                LineOfSight.random = new System.Random(seed + 1);
                TrackableObject.random = new System.Random(seed + 2);
                ObjectInfo.SeedObjectNumbers(seed + 3);
            }
            GameObject ego = sim.Ego != null ? SetupEgo(sim.Ego) : null;
            if (sim.Lighting != null) ApplyLighting(sim.Lighting);
            if (sim.Weather != null) ApplyWeather(sim.Weather, ego);
            if (sim.Loggers != null) ApplyLoggers(sim.Loggers, ego ?? ActiveEgo());
            Debug.Log($"[Config] simulation: time scale {sim.TimeScale}, seed {(sim.Seed.HasValue ? sim.Seed.ToString() : "random")}" +
                      (ego != null ? $", ego {ego.name}" : ""));
        }

        /// <summary>Top-level objects tagged Ego (vehicles that Autoware drives), active or not.</summary>
        public static List<GameObject> EgoVehicles()
            => Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Where(t => t.CompareTag("Ego") && (t.parent == null || !t.parent.CompareTag("Ego")) && t.GetComponent<AWSIM.Vehicle>() != null)
                .Select(t => t.gameObject).ToList();

        static GameObject ActiveEgo() => EgoVehicles().FirstOrDefault(g => g.activeInHierarchy);

        /// <summary>Activates the chosen ego vehicle once everything else is configured.</summary>
        public static void Finish()
        {
            if (pendingEgo != null) pendingEgo.SetActive(true);
            pendingEgo = null;
        }

        static GameObject SetupEgo(EgoConfig cfg)
        {
            var egos = EgoVehicles();
            var ego = string.IsNullOrEmpty(cfg.Vehicle) ? egos.FirstOrDefault(g => g.activeInHierarchy) : egos.FirstOrDefault(g => g.name == cfg.Vehicle);
            if (ego == null)
            {
                Debug.LogError($"[Config] simulation: ego vehicle '{cfg.Vehicle}' not found; ego vehicles in the scene: " +
                               string.Join(", ", egos.Select(g => g.name)));
                return ActiveEgo();
            }
            foreach (var other in egos) other.SetActive(false);
            pendingEgo = ego;
            if (cfg.Pose != null)
            {
                ConfigGeometry.ApplyPose(ego.transform, cfg.Pose);
                Debug.Log($"[Config] simulation: ego {ego.name} placed at {ego.transform.position} (map {cfg.Pose.X}, {cfg.Pose.Y})");
            }

            // traffic managers that pointed at another ego follow the chosen one
            foreach (var tm in Object.FindObjectsByType<AWSIM.TrafficSimulation.TrafficManager>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (GetField(tm, "_egoVehicle") is GameObject current && current != null && egos.Contains(current))
                    SetField(tm, "_egoVehicle", ego);

            // the "Selected Car" dropdown shows the chosen ego
            foreach (var list in Object.FindObjectsByType<SelectFromTheList>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                var objects = GetField(list, "objects") as List<GameObject>;
                int index = objects?.IndexOf(ego) ?? -1;
                if (index < 0) continue;
                foreach (var dd in Object.FindObjectsByType<TMPro.TMP_Dropdown>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                    for (int i = 0; i < dd.onValueChanged.GetPersistentEventCount(); i++)
                        if (dd.onValueChanged.GetPersistentTarget(i) == list) dd.SetValueWithoutNotify(index);
            }
            return ego;
        }

        // ------------------------------------------------------------------------------------------------ sun and fog
        /// <summary>Sun elevation and azimuth (degrees, azimuth clockwise from north) for a local date and time (NOAA formulas).</summary>
        public static void SunPosition(System.DateTime date, double hours, double latDeg, double lonDeg, double utcOffsetH,
                                       out double elevationDeg, out double azimuthDeg)
        {
            double g = 2 * System.Math.PI / 365.0 * (date.DayOfYear - 1 + (hours - 12) / 24.0);
            double eqTime = 229.18 * (0.000075 + 0.001868 * System.Math.Cos(g) - 0.032077 * System.Math.Sin(g)
                                      - 0.014615 * System.Math.Cos(2 * g) - 0.040849 * System.Math.Sin(2 * g));
            double decl = 0.006918 - 0.399912 * System.Math.Cos(g) + 0.070257 * System.Math.Sin(g) - 0.006758 * System.Math.Cos(2 * g)
                          + 0.000907 * System.Math.Sin(2 * g) - 0.002697 * System.Math.Cos(3 * g) + 0.00148 * System.Math.Sin(3 * g);
            double trueSolarMin = hours * 60 + eqTime + 4 * lonDeg - 60 * utcOffsetH;
            double ha = (trueSolarMin / 4 - 180) * Mathf.Deg2Rad;
            double lat = latDeg * Mathf.Deg2Rad;
            double cosZen = System.Math.Sin(lat) * System.Math.Sin(decl) + System.Math.Cos(lat) * System.Math.Cos(decl) * System.Math.Cos(ha);
            elevationDeg = 90 - System.Math.Acos(System.Math.Max(-1, System.Math.Min(1, cosZen))) * Mathf.Rad2Deg;
            azimuthDeg = (System.Math.Atan2(System.Math.Sin(ha), System.Math.Cos(ha) * System.Math.Sin(lat) - System.Math.Tan(decl) * System.Math.Cos(lat))
                          * Mathf.Rad2Deg + 180 + 360) % 360;
        }

        static Light Sun()
            => Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude, FindObjectsSortMode.None)
                .Where(l => l.type == LightType.Directional).OrderByDescending(l => l.intensity).FirstOrDefault();

        static void ApplyLighting(LightingConfig l)
        {
            double elevation, azimuth;
            if (l.SunElevationDeg.HasValue && l.SunAzimuthDeg.HasValue)
            {
                elevation = l.SunElevationDeg.Value;
                azimuth = l.SunAzimuthDeg.Value;
            }
            else if (!string.IsNullOrEmpty(l.TimeOfDay) && ConfigLoader.TryParseTimeOfDay(l.TimeOfDay, out var hours) &&
                     System.DateTime.TryParse(l.Date, System.Globalization.CultureInfo.InvariantCulture, System.Globalization.DateTimeStyles.None, out var date))
                SunPosition(date, hours, l.LatitudeDeg, l.LongitudeDeg, l.UtcOffsetH, out elevation, out azimuth);
            else
            {
                ApplyLightValues(Sun(), l, false);
                return;
            }
            var sun = Sun();
            if (sun == null) { Debug.LogError("[Config] simulation: no directional light in the scene"); return; }
            bool night = elevation < 0;
            // below the horizon: a dim "moon" opposite the sun, so the scene stays readable
            double e = (night ? 35 : elevation) * Mathf.Deg2Rad, a = (night ? azimuth + 180 : azimuth) * Mathf.Deg2Rad;
            // direction towards the light in Unity: east = +z, north = -x, up = +y
            var toLight = new Vector3((float)(-System.Math.Cos(a) * System.Math.Cos(e)), (float)System.Math.Sin(e), (float)(System.Math.Sin(a) * System.Math.Cos(e)));
            sun.transform.rotation = Quaternion.LookRotation(-toLight);
            ApplyLightValues(sun, l, night);
            Debug.Log($"[Config] simulation: sun elevation {elevation:F1}°, azimuth {azimuth:F1}°{(night ? " (night: moonlight)" : "")}");
        }

        static void ApplyLightValues(Light sun, LightingConfig l, bool night)
        {
            if (sun == null) return;
            if (night)
            {
                sun.intensity = l.NightIntensityLux;
                sun.useColorTemperature = true;
                sun.colorTemperature = 4100f;
                return;
            }
            if (l.SunIntensityLux.HasValue) sun.intensity = l.SunIntensityLux.Value;
            if (l.ColorTemperatureK.HasValue)
            {
                sun.useColorTemperature = true;
                sun.colorTemperature = l.ColorTemperatureK.Value;
            }
        }

        static void ApplyWeather(WeatherConfig w, GameObject ego)
        {
            if (w.FogVisibilityM <= 0f) return;
            var volume = Object.FindObjectsByType<Volume>(FindObjectsInactive.Exclude, FindObjectsSortMode.None)
                .Where(v => v.isGlobal && v.enabled).OrderByDescending(v => v.priority).FirstOrDefault();
            if (volume == null) { Debug.LogError("[Config] simulation: no global Volume for the fog"); return; }
            var profile = volume.profile;                      // a runtime copy; the asset is not changed
            if (!profile.TryGet<Fog>(out var fog)) fog = profile.Add<Fog>(true);
            float ground = ego != null ? ego.transform.position.y : 0f;
            fog.active = true;
            fog.enabled.Override(true);
            fog.meanFreePath.Override(w.FogVisibilityM / 3f);  // visibility (5 % contrast) ≈ 3 × mean free path
            fog.baseHeight.Override(ground);
            fog.maximumHeight.Override(ground + w.FogHeightM);
            Debug.Log($"[Config] simulation: fog, visibility {w.FogVisibilityM} m");
        }

        // ------------------------------------------------------------------------------------------------ loggers
        static void ApplyLoggers(LoggersConfig cfg, GameObject ego)
        {
            string P(string file) => string.IsNullOrEmpty(cfg.OutputDir) ? file : System.IO.Path.Combine(cfg.OutputDir, file);
            if (cfg.NpcArea != null)
                foreach (var area in Object.FindObjectsByType<LogArea>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                {
                    area.enabled = cfg.NpcArea.Enabled;
                    SetField(area, "logPathHumans", P(cfg.NpcArea.HumansCsv));
                    SetField(area, "logPathCars", P(cfg.NpcArea.CarsCsv));
                    SetField(area, "ultraMode", cfg.NpcArea.EveryNFrames > 0);
                    SetField(area, "waitForFrame", Mathf.Max(1, cfg.NpcArea.EveryNFrames));
                    SetField(area, "waitTime", cfg.NpcArea.IntervalS);
                }
            if (cfg.Ego != null)
                foreach (var e in EgoVehicles())
                {
                    if (cfg.Ego.Enabled && e == ego && e.GetComponentInChildren<LogAutonomous>(true) == null)
                    {
                        if (e.activeSelf) Debug.LogWarning($"[Config] simulation: {e.name} is active; the ego logger is added without its settings");
                        e.AddComponent<LogAutonomous>();         // inactive here, so its Awake runs after the settings below
                    }
                    foreach (var log in e.GetComponentsInChildren<LogAutonomous>(true))
                    {
                        if (!cfg.Ego.Enabled || e != ego)
                        {
                            Object.DestroyImmediate(log);   // its Awake would start logging even when the component is disabled
                            continue;
                        }
                        log.enabled = true;
                        SetField(log, "logPathCars", P(cfg.Ego.Csv));
                        SetField(log, "spawnSelectionLogPath", P(cfg.Ego.SpawnValidationCsv));
                        SetField(log, "logFilePrefix", cfg.Ego.Prefix ?? "");
                        log.waitTime = cfg.Ego.IntervalS;
                    }
                }
        }

        // ------------------------------------------------------------------------------------------------ signals
        public static void ApplySignals(SignalsConfig cfg)
        {
            var all = Object.FindObjectsByType<AWSIM.TrafficSimulation.TrafficIntersection>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            int done = 0;
            foreach (var i in cfg.Intersections.Where(i => i.Enabled))
            {
                var matches = all.Where(t => t.name == i.Name).ToList();
                var ti = matches.FirstOrDefault(t => t.enabled && t.gameObject.activeInHierarchy) ?? matches.FirstOrDefault();
                if (ti == null) { Debug.LogError($"[Config] signals: intersection '{i.Name}' not found"); continue; }
                if (!(GetField(ti, "lightingSequences") is System.Array seq)) continue;
                if (seq.Length != i.Steps.Count)
                {
                    Debug.LogError($"[Config] signals: {i.Name} has {seq.Length} steps in the scene, the config lists {i.Steps.Count}; timing not changed");
                    continue;
                }
                for (int k = 0; k < seq.Length; k++)
                    SetField(seq.GetValue(k), "intervalSec", i.Steps[k].DurationS);
                done++;
            }
            Debug.Log($"[Config] signals: timing of {done} intersections set");
        }

        /// <summary>Readable description of the lights of every step of an intersection (for the exporter).</summary>
        public static List<(float duration, string lights)> DescribeSteps(AWSIM.TrafficSimulation.TrafficIntersection ti)
        {
            var result = new List<(float, string)>();
            if (!(GetField(ti, "lightingSequences") is System.Array seq)) return result;
            foreach (var s in seq)
            {
                var parts = new List<string>();
                if (GetField(s, "groupLightingOrders") is System.Array orders)
                    foreach (var o in orders)
                    {
                        var group = GetField(o, "group")?.ToString() ?? "";
                        var name = group.Replace("VehicleTrafficLightGroup", "V").Replace("PedestrianTrafficLightGroup", "P");
                        var bulbs = (GetField(o, "bulbData") as AWSIM.TrafficLight.BulbData[] ?? new AWSIM.TrafficLight.BulbData[0])
                            .Where(b => b.Status != AWSIM.TrafficLight.BulbStatus.SOLID_OFF)
                            .Select(b => b.Color.ToString().ToLower() + (b.Type == AWSIM.TrafficLight.BulbType.ANY_CIRCLE_BULB ||
                                                                         b.Type.ToString().StartsWith(b.Color.ToString()) ? "" : " " + b.Type.ToString().ToLower().Replace("_bulb", "")) +
                                         (b.Status == AWSIM.TrafficLight.BulbStatus.FLASHING ? " flashing" : ""));
                        parts.Add($"{name} {string.Join("+", bulbs.DefaultIfEmpty("off"))}");
                    }
                result.Add(((float)GetField(s, "intervalSec"), string.Join(", ", parts)));
            }
            return result;
        }
    }
}
