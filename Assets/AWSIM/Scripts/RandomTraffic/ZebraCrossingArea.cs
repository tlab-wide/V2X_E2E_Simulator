using System.Collections.Generic;
using AWSIM;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Tracks pedestrians/cyclists inside a zebra crossing trigger and exposes stop points for NPC vehicles.
    /// Attach this to a trigger collider that covers the crosswalk.
    /// </summary>
    [RequireComponent(typeof(Collider))]
    public class ZebraCrossingArea : MonoBehaviour
    {
        [SerializeField, Tooltip("Point where NPC vehicles should stop before the crossing. Defaults to this object's position.")]
        private Transform stopPoint;
        [SerializeField, Tooltip("Max distance (m) within which vehicles will consider this crossing ahead.")]
        private float maxDetectionDistance = 40f;
        [SerializeField, Tooltip("Allowed angle (deg) between vehicle forward and crossing to still stop.")]
        private float forwardAngleTolerance = 80f;
        [SerializeField, Tooltip("Extra buffer (m) to keep vehicles from rolling onto the zebra.")]
        private float stopBuffer = 1.5f;
        [SerializeField, Tooltip("Log occupancy changes and vehicle stop decisions for this zebra area.")]
        private bool enableDebugLogs = false;

        private static readonly HashSet<ZebraCrossingArea> ActiveAreas = new HashSet<ZebraCrossingArea>();

        private readonly HashSet<Transform> occupantRoots = new HashSet<Transform>();
        private ZebraCrossingConfig parentConfig;

        public bool IsOccupied => occupantRoots.Count > 0;
        public bool EnableDebugLogs => enableDebugLogs;
        public IReadOnlyCollection<Transform> Occupants => occupantRoots;

        private Vector3 StopPointPosition => stopPoint != null ? stopPoint.position : transform.position;
        private float ForwardDotThreshold => Mathf.Cos(Mathf.Clamp(forwardAngleTolerance, 0f, 180f) * Mathf.Deg2Rad);

        private void Reset()
        {
            var col = GetComponent<Collider>();
            col.isTrigger = true;
        }

        private void OnEnable()
        {
            ActiveAreas.Add(this);
            parentConfig = GetComponentInParent<ZebraCrossingConfig>();
        }

        private void OnDisable()
        {
            ActiveAreas.Remove(this);
        }

        private void OnTriggerEnter(Collider other)
        {
            if (TryGetOccupantRoot(other, out var root) && IsAllowed(root))
            {
                if (occupantRoots.Add(root) && enableDebugLogs)
                {
                    Debug.Log($"[ZebraCrossingArea] Occupant entered '{name}': {root.name} (count={occupantRoots.Count})");
                }
            }
            else if (enableDebugLogs)
            {
                var otherName = other != null ? other.name : "null";
                Debug.Log($"[ZebraCrossingArea] Ignored enter on '{name}' for {otherName} (no matching occupant or filtered).");
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (TryGetOccupantRoot(other, out var root) && occupantRoots.Remove(root))
            {
                if (enableDebugLogs)
                {
                    Debug.Log($"[ZebraCrossingArea] Occupant exited '{name}': {root.name} (count={occupantRoots.Count})");
                }
            }
        }

        private bool TryGetOccupantRoot(Collider other, out Transform root)
        {
            root = null;
            if (other == null)
                return false;

            var ped = other.GetComponentInParent<NPCPedestrian>();
            if (ped != null)
            {
                root = ped.transform.root;
                return root != null;
            }

            if (HasTypeInParents(other, "SimpleCyclistMovement", out root)) return true;
            if (HasTypeInParents(other, "BicycleMoveRotate", out root)) return true;
            if (HasTypeInParents(other, "V2X.Paths.ClearWaypointFollower", out root)) return true;
            if (HasTypeInParents(other, "WaypointFollower", out root)) return true; // global namespace followers

            if (enableDebugLogs)
            {
                var comps = other.GetComponentsInParent<MonoBehaviour>(true);
                var names = new List<string>();
                foreach (var c in comps)
                {
                    if (c == null) continue;
                    names.Add(c.GetType().FullName);
                }
                Debug.Log($"[ZebraCrossingArea] No matching occupant found for collider '{other.name}' on '{name}'. Parents: {string.Join(";", names)}");
            }

            return false;
        }

        private static bool HasTypeInParents(Collider other, string typeFullName, out Transform root)
        {
            root = null;
            var behaviours = other.GetComponentsInParent<MonoBehaviour>(true);
            foreach (var b in behaviours)
            {
                if (b == null) continue;
                if (b.GetType().FullName == typeFullName)
                {
                    root = b.transform.root;
                    return true;
                }
            }
            return false;
        }

        private bool IsAllowed(Transform root)
        {
            if (root == null)
                return false;

            var prefabs = GetAllowedPrefabs();
            var nameTokens = GetAllowedNameTokens();

            bool hasWhitelist = (prefabs != null && prefabs.Count > 0) || (nameTokens != null && nameTokens.Count > 0);
            if (!hasWhitelist)
                return true;

            if (prefabs != null)
            {
                var rootName = NormalizeName(root.name);
                foreach (var prefab in prefabs)
                {
                    if (prefab == null) continue;
                    if (NormalizeName(prefab.name) == rootName)
                        return true;
                }
            }

            if (nameTokens != null)
            {
                foreach (var token in nameTokens)
                {
                    if (string.IsNullOrEmpty(token)) continue;
                    if (NormalizeName(root.name).IndexOf(token, System.StringComparison.OrdinalIgnoreCase) >= 0)
                        return true;
                }
            }

            if (enableDebugLogs)
            {
                var prefabList = prefabs != null ? string.Join(",", System.Array.ConvertAll(new List<GameObject>(prefabs).ToArray(), p => p != null ? p.name : "null")) : "none";
                var tokenList = nameTokens != null ? string.Join(",", nameTokens) : "none";
                Debug.Log($"[ZebraCrossingArea] Root '{root.name}' rejected by whitelist on '{name}'. Prefabs=[{prefabList}] Tokens=[{tokenList}]");
            }

            return false;
        }

        private IReadOnlyList<string> GetAllowedNameTokens()
        {
            if (parentConfig != null && parentConfig.AllowedRootNameSubstrings != null && parentConfig.AllowedRootNameSubstrings.Count > 0)
                return parentConfig.AllowedRootNameSubstrings;
            return null;
        }

        private IReadOnlyList<GameObject> GetAllowedPrefabs()
        {
            if (parentConfig != null && parentConfig.AllowedPrefabs != null && parentConfig.AllowedPrefabs.Count > 0)
                return parentConfig.AllowedPrefabs;
            return null;
        }

        private static string NormalizeName(string name)
        {
            if (string.IsNullOrEmpty(name))
                return string.Empty;
            var n = name.Replace("(Clone)", string.Empty).Replace("(clone)", string.Empty);
            n = n.Replace("_clone", string.Empty).Replace("_Clone", string.Empty);
            return n.Trim();
        }

        /// <summary>
        /// Returns the closest occupied zebra crossing stop distance in front of the vehicle, if any.
        /// </summary>
        public static bool TryGetBlockedStop(Vector3 vehiclePosition, Vector3 vehicleForward, out float distance, out ZebraCrossingArea matchedArea)
        {
            distance = float.MaxValue;
            matchedArea = null;
            var found = false;
            var flatForward = new Vector3(vehicleForward.x, 0f, vehicleForward.z);
            if (flatForward.sqrMagnitude > 0.0001f)
                flatForward.Normalize();

            foreach (var area in ActiveAreas)
            {
                if (area == null || !area.isActiveAndEnabled || !area.IsOccupied)
                    continue;

                var toStop = area.StopPointPosition - vehiclePosition;
                toStop.y = 0f;

                var sqrMag = toStop.sqrMagnitude;
                if (sqrMag <= 0.0001f)
                    continue;

                if (sqrMag > area.maxDetectionDistance * area.maxDetectionDistance)
                    continue;

                var dirToStop = toStop / Mathf.Sqrt(sqrMag);
                if (flatForward.sqrMagnitude > 0.0001f && Vector3.Dot(flatForward, dirToStop) < area.ForwardDotThreshold)
                    continue;

                var candidate = Mathf.Sqrt(sqrMag) - area.stopBuffer;
                if (candidate < distance)
                {
                    distance = Mathf.Max(0f, candidate);
                    matchedArea = area;
                    found = true;
                    if (area.enableDebugLogs)
                    {
                        var occupantNames = string.Join(",", System.Array.ConvertAll(new List<Transform>(area.occupantRoots).ToArray(), t => t != null ? t.name : "null"));
                        Debug.Log($"[ZebraCrossingArea] Blocked '{area.name}' occupants={area.occupantRoots.Count} [{occupantNames}] distance={distance:F1}m");
                    }
                }
            }

            return found;
        }

        private void OnDrawGizmosSelected()
        {
            var col = GetComponent<Collider>();
            var bounds = col != null ? col.bounds : new Bounds(transform.position, Vector3.one);
            Gizmos.color = IsOccupied ? new Color(1f, 0.2f, 0.2f, 0.6f) : new Color(0.2f, 0.8f, 0.2f, 0.6f);
            Gizmos.DrawWireCube(bounds.center, bounds.size);

            var stop = StopPointPosition;
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(stop, 0.3f);
        }
    }
}
