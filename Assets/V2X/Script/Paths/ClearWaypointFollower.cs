using System;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

namespace V2X.Paths
{
    /// <summary>
    /// Lightweight waypoint follower for pedestrians/cyclists.
    /// Supports manual waypoints (Transforms) or programmatic paths (Vector3).
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [DisallowMultipleComponent]
    public class ClearWaypointFollower : MonoBehaviour
    {
        public enum PathSource
        {
            ManualWaypoints,
            PathLoader
        }

        [Serializable]
        public enum LoopMode
        {
            NoLoop,
            Loop,
            PingPong
        }

        [Serializable]
        public class Waypoint
        {
            public Vector3 position;
            [Tooltip("Optional stop duration at this waypoint (seconds).")]
            public float waitSeconds = 0f;
            [Tooltip("Optional per-segment speed (m/s). If <=0, uses baseSpeed.")]
            public float speedOverride = 0f;
            [Tooltip("If true, check pedestrian light before leaving this waypoint.")]
            public bool checkLight = false;
            [Tooltip("Pedestrian light id to use at this waypoint (matches PathConfigLoader ids).")]
            public string trafficLightId = "";
        }

        [Header("Path Source")]
        [Tooltip("Choose where to get path data from.")]
        [SerializeField] private PathSource pathSource = PathSource.ManualWaypoints;
        [Tooltip("If set, waypoints will be read from these transforms on Start (in order).")]
        [SerializeField] private List<Transform> manualWaypoints = new List<Transform>();
        [Tooltip("Optional path injected at runtime (Vector3 list). Leave empty to use manual transforms.")]
        [SerializeField] private List<Waypoint> runtimePath = new List<Waypoint>();
        [Tooltip("If using PathLoader, the path id to load from PathConfigLoader.")]
        [SerializeField] private string pathId = "";
        [Tooltip("Optional explicit PathConfigLoader. If null, will find one in scene.")]
        [SerializeField] private PathConfigLoader pathLoader;

        [Header("Movement")]
        [Tooltip("Default travel speed (m/s).")]
        [SerializeField] private float baseSpeed = 1.5f;
        [Tooltip("Acceleration (m/s^2).")]
        [SerializeField] private float acceleration = 2.0f;
        [Tooltip("Deceleration (m/s^2).")]
        [SerializeField] private float deceleration = 3.0f;
        [Tooltip("Max turn rate (deg/s).")]
        [SerializeField] private float maxTurnRate = 360f;
        [Tooltip("Distance to consider waypoint reached (m).")]
        [SerializeField] private float arrivalRadius = 0.35f;
        [Tooltip("Look-ahead distance along segment for steering (m).")]
        [SerializeField] private float lookAheadDistance = 1.0f;
        [Tooltip("Path looping mode.")]
        [SerializeField] private LoopMode loopMode = LoopMode.Loop;
        [Tooltip("Random +/- seconds applied to waypoint waits.")]
        [SerializeField] private float waitJitterSeconds = 0.5f;

        [Header("Grounding")]
        [SerializeField] private LayerMask groundMask = ~0;
        [SerializeField, Tooltip("Max slope angle in degrees.")] private float maxSlope = 45f;
        [SerializeField, Tooltip("Force to keep agent grounded.")] private float stickToGroundForce = 5f;

        [Header("Obstacle Stop (simple)")]
        [SerializeField] private bool enableObstacleStop = false;
        [SerializeField] private LayerMask stopMask = 0;
        [SerializeField, Tooltip("Forward check distance (m).")] private float stopCheckDistance = 1.5f;
        [SerializeField, Tooltip("Half extents for BoxCast stop check.")] private Vector3 stopBoxHalfExtents = new Vector3(0.35f, 0.9f, 0.15f);
        [SerializeField, Tooltip("Vertical offset of BoxCast center.")] private float stopBoxVerticalOffset = 0.9f;

        [Header("Animation (optional)")]
        [SerializeField] private Animator animator;
        [SerializeField] private string moveSpeedParam = "moveSpeed";
        [SerializeField] private string rotateSpeedParam = "rotateSpeed";

        [Header("Pedestrian Traffic Light")]
        [Tooltip("If set, follower will respect pedestrian traffic light states.")]
        [SerializeField] private bool respectPedestrianLight = false;
        [SerializeField, Tooltip("Pedestrian traffic light to query (bulb colors).")]
        private TrafficLight pedestrianLight;
        [SerializeField, Tooltip("Speed multiplier when flashing green (run).")]
        private float flashingGreenSpeedMultiplier = 1.5f;

        private readonly List<Waypoint> _path = new List<Waypoint>();
        private int _index;
        private int _direction = 1; // 1 forward, -1 backward for pingpong
        private bool _waiting;
        private float _currentSpeed;
        private Rigidbody _rb;
        private bool _initialized;
        private Vector3 _lastLookDir = Vector3.forward;
        private Dictionary<string, TrafficLight> _lightsById = new Dictionary<string, TrafficLight>(StringComparer.OrdinalIgnoreCase);

        private void Awake()
        {
            _rb = GetComponent<Rigidbody>();
            if (animator == null) animator = GetComponentInChildren<Animator>();
            _rb.interpolation = RigidbodyInterpolation.Interpolate;
            _rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            _rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

            CacheTrafficLights();
        }

        private void Start()
        {
            if (!TryBuildPathFromLoader())
            {
                BuildPath();
            }
            SnapToStart();
            _initialized = true;
        }

        private void FixedUpdate()
        {
            if (!_initialized || _path.Count == 0) return;

            if (_waiting)
                return;

            var pedSignal = EvaluatePedLight();

            if (enableObstacleStop && IsBlocked())
            {
                ApplyVelocity(Vector3.zero);
                UpdateAnimator(0f, 0f);
                return;
            }

            var target = _path[_index];
            float targetSpeed = target.speedOverride > 0f ? target.speedOverride : baseSpeed;
            if (respectPedestrianLight)
            {
                switch (pedSignal)
                {
                    case PedSignalState.Red:
                        ApplyVelocity(Vector3.zero);
                        _currentSpeed = 0f;
                        UpdateAnimator(0f, 0f);
                        return;
                    case PedSignalState.FlashingGreen:
                        targetSpeed *= Mathf.Max(1f, flashingGreenSpeedMultiplier);
                        break;
                    case PedSignalState.Green:
                    default:
                        break;
                }
            }
            float desiredSpeed = Mathf.MoveTowards(_currentSpeed, targetSpeed, acceleration * Time.fixedDeltaTime);

            Vector3 toTarget = target.position - _rb.position;
            float distance = toTarget.magnitude;
            if (distance < arrivalRadius)
            {
                StartCoroutine(HandleWaypointReached(target));
                return;
            }

            Vector3 direction = toTarget.normalized;
            Vector3 lookAheadTarget = target.position;
            if (distance > lookAheadDistance)
            {
                lookAheadTarget = _rb.position + direction * lookAheadDistance;
            }

            Vector3 lookDir = lookAheadTarget - _rb.position;
            lookDir.y = 0f;
            if (lookDir.sqrMagnitude > 0.0001f)
                _lastLookDir = lookDir.normalized;

            if (_lastLookDir.sqrMagnitude > 0.0001f)
            {
                Quaternion desiredRot = Quaternion.LookRotation(_lastLookDir, Vector3.up);
                _rb.MoveRotation(Quaternion.RotateTowards(_rb.rotation, desiredRot, maxTurnRate * Time.fixedDeltaTime));
            }

            Vector3 velocity = _rb.rotation * Vector3.forward * desiredSpeed;
            _currentSpeed = desiredSpeed;
            ApplyVelocity(velocity);

            UpdateAnimator(desiredSpeed, maxTurnRate);
        }

        private IEnumerator<WaitForSeconds> HandleWaypointReached(Waypoint wp)
        {
            _waiting = true;
            ApplyVelocity(Vector3.zero);
            _currentSpeed = 0f;
            UpdateAnimator(0f, 0f);

            float wait = Mathf.Max(0f, wp.waitSeconds);
            if (wait > 0f && waitJitterSeconds > 0f)
            {
                float jitter = UnityEngine.Random.Range(-waitJitterSeconds, waitJitterSeconds);
                wait = Mathf.Max(0f, wait + jitter);
            }
            if (wait > 0f)
                yield return new WaitForSeconds(wait);

            AdvanceIndex();
            _waiting = false;
        }

        private void AdvanceIndex()
        {
            switch (loopMode)
            {
                case LoopMode.NoLoop:
                    _index = Mathf.Min(_index + 1, _path.Count - 1);
                    break;
                case LoopMode.Loop:
                    _index = (_index + 1) % _path.Count;
                    break;
                case LoopMode.PingPong:
                    if (_index == _path.Count - 1) _direction = -1;
                    if (_index == 0) _direction = 1;
                    _index = Mathf.Clamp(_index + _direction, 0, _path.Count - 1);
                    break;
            }
        }

        private void BuildPath()
        {
            _path.Clear();
            if (runtimePath != null && runtimePath.Count > 0)
            {
                _path.AddRange(runtimePath);
            }
            else
            {
                foreach (var t in manualWaypoints)
                {
                    if (t == null) continue;
                    _path.Add(new Waypoint { position = t.position });
                }
            }
        }

        private bool TryBuildPathFromLoader()
        {
            if (pathSource != PathSource.PathLoader)
                return false;

            if (pathLoader == null)
                pathLoader = FindObjectOfType<PathConfigLoader>();

            if (pathLoader == null || string.IsNullOrEmpty(pathId))
                return false;

            var def = pathLoader.GetPathDefinition(pathId);
            if (def != null && def.waypoints != null && def.waypoints.Count > 0)
            {
                runtimePath = new List<Waypoint>();
                foreach (var pt in def.waypoints)
                {
                    runtimePath.Add(new Waypoint
                    {
                        position = new Vector3(pt.x, pt.y, pt.z),
                        waitSeconds = 0f,
                        speedOverride = 0f,
                        checkLight = pt.checkLight,
                        trafficLightId = pt.trafficLightId
                    });
                }
                BuildPath();
                return _path.Count > 0;
            }

            var ws = pathLoader.GetWaypointSystem(pathId);
            if (ws == null)
            {
                Debug.LogWarning($"[ClearWaypointFollower] Path id '{pathId}' not found in PathConfigLoader.");
                return false;
            }

            var points = new List<Waypoint>();
            for (int i = 0; i < ws.transform.childCount; i++)
            {
                var child = ws.transform.GetChild(i);
                if (child == null) continue;
                points.Add(new Waypoint { position = child.position });
            }

            if (points.Count == 0)
            {
                Debug.LogWarning($"[ClearWaypointFollower] Path '{pathId}' has no waypoints.");
                return false;
            }

            runtimePath = points;
            BuildPath();
            return _path.Count > 0;
        }

        private Vector3 GetNextDirection()
        {
            if (_path.Count == 0) return Vector3.forward;
            int nextIndex = Mathf.Min(_path.Count - 1, 1);
            if (loopMode != LoopMode.NoLoop && _path.Count > 1)
                nextIndex = 1;
            var dir = (_path[nextIndex].position - _path[0].position);
            dir.y = 0f;
            return dir.normalized;
        }

        private enum PedSignalState
        {
            Unknown,
            Red,
            Green,
            FlashingGreen
        }

        private PedSignalState EvaluatePedLight()
        {
            if (!respectPedestrianLight)
                return PedSignalState.Green;

            TrafficLight activeLight = null;

            if (_path.Count > 0 && _index < _path.Count)
            {
                var wp = _path[_index];
                if (wp.checkLight && !string.IsNullOrEmpty(wp.trafficLightId))
                {
                    _lightsById.TryGetValue(wp.trafficLightId, out activeLight);
                }
            }

            if (activeLight == null)
                activeLight = pedestrianLight;

            if (activeLight == null || !activeLight.gameObject.activeInHierarchy)
                return PedSignalState.Green;

            var bulbs = activeLight.GetBulbData();
            bool anyGreenSolid = false;
            bool anyGreenFlashing = false;
            bool anyRed = false;
            foreach (var b in bulbs)
            {
                if (b.Color == TrafficLight.BulbColor.GREEN)
                {
                    if (b.Status == TrafficLight.BulbStatus.FLASHING)
                        anyGreenFlashing = true;
                    else if (b.Status == TrafficLight.BulbStatus.SOLID_ON)
                        anyGreenSolid = true;
                }
                if (b.Color == TrafficLight.BulbColor.RED && b.Status == TrafficLight.BulbStatus.SOLID_ON)
                    anyRed = true;
            }

            if (anyGreenFlashing) return PedSignalState.FlashingGreen;
            if (anyGreenSolid) return PedSignalState.Green;
            if (anyRed) return PedSignalState.Red;
            return PedSignalState.Unknown;
        }

        private void CacheTrafficLights()
        {
            _lightsById.Clear();
            var all = FindObjectsOfType<TrafficLight>(includeInactive: true);
            foreach (var tl in all)
            {
                if (tl == null) continue;
                var key = tl.name;
                if (!_lightsById.ContainsKey(key))
                    _lightsById.Add(key, tl);
            }
        }

        private void SnapToStart()
        {
            if (_path.Count == 0) return;
            _index = 0;
            _direction = 1;
            _currentSpeed = 0f;
            _waiting = false;
            Vector3 pos = _path[0].position;
            _rb.position = pos;
            var lookDir = GetNextDirection();
            if (lookDir.sqrMagnitude < 0.0001f)
                lookDir = _lastLookDir;
            _lastLookDir = lookDir.sqrMagnitude > 0.0001f ? lookDir : Vector3.forward;
            _rb.rotation = Quaternion.LookRotation(_lastLookDir, Vector3.up);
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        private bool IsBlocked()
        {
            if (stopMask == 0) return false;

            Vector3 center = _rb.position + Vector3.up * stopBoxVerticalOffset;
            Quaternion rot = _rb.rotation;
            RaycastHit hit;
            return Physics.BoxCast(center, stopBoxHalfExtents, _rb.rotation * Vector3.forward, out hit, rot, stopCheckDistance, stopMask, QueryTriggerInteraction.Ignore);
        }

        private void ApplyVelocity(Vector3 velocity)
        {
            _rb.MovePosition(_rb.position + velocity * Time.fixedDeltaTime);
        }

        private void UpdateAnimator(float moveSpeed, float rotateSpeed)
        {
            if (animator == null) return;
            if (!string.IsNullOrEmpty(moveSpeedParam)) animator.SetFloat(moveSpeedParam, moveSpeed);
            if (!string.IsNullOrEmpty(rotateSpeedParam)) animator.SetFloat(rotateSpeedParam, rotateSpeed);
        }

        // --- Public API ---
        public void SetPathFromVectors(List<Vector3> points, bool snapToStart = true)
        {
            runtimePath = new List<Waypoint>();
            if (points != null)
            {
                foreach (var p in points)
                    runtimePath.Add(new Waypoint { position = p });
            }
            BuildPath();
            if (snapToStart) SnapToStart();
        }

        public void SetPathFromLoader(PathConfigLoader loader, string id, bool snapToStart = true)
        {
            pathSource = PathSource.PathLoader;
            pathLoader = loader;
            pathId = id;
            if (!TryBuildPathFromLoader())
            {
                Debug.LogWarning($"[ClearWaypointFollower] Failed to load path '{id}' from loader.");
            }
            if (snapToStart) SnapToStart();
        }

        public void SetPathFromWaypoints(List<Waypoint> waypoints, bool snapToStart = true)
        {
            runtimePath = waypoints != null ? new List<Waypoint>(waypoints) : new List<Waypoint>();
            BuildPath();
            if (snapToStart) SnapToStart();
        }

        public void Pause() => _waiting = true;
        public void Resume() => _waiting = false;
        public void SetSpeedMultiplier(float multiplier)
        {
            baseSpeed = Mathf.Max(0f, baseSpeed * multiplier);
        }
    }
}
