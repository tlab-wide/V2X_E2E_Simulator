using System;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(CapsuleCollider))]
public class WaypointFollower : MonoBehaviour, ISpeed
{
    [Header("Path")]
    [Tooltip("If left empty, the script will try to find the closest WaypointSystem in the scene at runtime.")]
    public WaypointSystem waypointSystem;

    [Min(0.01f)] public float speed = 5f;
    [Min(0.01f)] public float turnSpeed = 5f;
    [Min(0f)]   public float stoppingDistance = 0.5f;

    [Header("Grounding & Slopes")]
    [Tooltip("Layers considered 'ground'. Defaults to Everything if unset.")]
    [SerializeField] private LayerMask groundMask = ~0;
    [Range(0f, 80f)] [SerializeField] private float maxSlope = 45f;
    [SerializeField] private float stickToGroundForce = 5f;

    [Header("Local Avoidance (simple)")]
    [Tooltip("If unset, uses this object's layer automatically.")]
    [SerializeField] private LayerMask characterMask = 0;
    [SerializeField] private float avoidRadius = 0.9f;
    [Range(0f, 2f)]  [SerializeField] private float avoidWeight = 0.6f;

    [Header("Step Assist")]
    [SerializeField] private float stepMaxHeight = 0.35f;    // maximum curb/step height to climb
    [SerializeField] private float stepCheckDistance = 0.3f; // how far ahead to probe for a step
    [SerializeField] private float stepClimbSpeed = 2.5f;    // gentle upward bias when stepping

    [Header("Capsule Dimensions (auto read on Start)")]
    [SerializeField] private float agentRadius = 0.35f; // will read from CapsuleCollider at runtime
    [SerializeField] private float agentHeight = 1.8f;  // will read from CapsuleCollider at runtime

    [Header("Animation (optional)")]
    [SerializeField] private string moveSpeedProperty = "moveSpeed";
    [SerializeField] private string rotateSpeedProperty = "rotateSpeed";

    [Header("Advanced")]
    [Tooltip("Default physics constraints when moving (usually FreezeRotation X/Z).")]
    [SerializeField] private RigidbodyConstraints initialConstraints =
        RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

    // Internal state
    [SerializeField] private WaypointSystem.WaypointNode currentWaypoint;
    [SerializeField] private int currentWaypointIndex;
    private IConstraintWayPoint[] selfConstraints;
    private Rigidbody rb;
    private CapsuleCollider capsule;
    private Animator animator;
    private int moveSpeedHash;
    private int rotateSpeedHash;

    void Reset()
    {
        // Sane defaults if user just adds the component
        speed = 5f;
        turnSpeed = 5f;
        stoppingDistance = 0.5f;
        maxSlope = 45f;
        avoidRadius = 0.9f;
        avoidWeight = 0.6f;
        stickToGroundForce = 5f;
        stepMaxHeight = 0.35f;
        stepCheckDistance = 0.3f;
        stepClimbSpeed = 2.5f;
        initialConstraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        groundMask = ~0; // Everything
        characterMask = 0; // Auto-pick our layer at runtime
        moveSpeedProperty = "moveSpeed";
        rotateSpeedProperty = "rotateSpeed";
    }

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        capsule = GetComponent<CapsuleCollider>();
        animator = GetComponent<Animator>();
        if (animator == null) animator = GetComponentInChildren<Animator>();

        moveSpeedHash = Animator.StringToHash(moveSpeedProperty);
        rotateSpeedHash = Animator.StringToHash(rotateSpeedProperty);

        // Defensive Rigidbody setup (override bad inspector states)
        rb.useGravity = true;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        if (initialConstraints == RigidbodyConstraints.None)
            initialConstraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.constraints = initialConstraints;

        // Read collider size for grounding/avoidance if present
        if (capsule != null)
        {
            agentRadius = Mathf.Max(agentRadius, capsule.radius);
            agentHeight = Mathf.Max(agentHeight, capsule.height);
        }

        // If characterMask not set, use our own layer so we avoid similar agents
        if (characterMask == 0)
        {
            int layer = gameObject.layer;
            characterMask = 1 << layer;
        }
    }

    void Start()
    {
        selfConstraints = GetComponents<IConstraintWayPoint>();

        // Auto-find a waypoint system if not assigned
        if (waypointSystem == null)
        {
#if UNITY_2022_3_OR_NEWER
            waypointSystem = FindFirstObjectByType<WaypointSystem>(FindObjectsInactive.Exclude);
#else
            waypointSystem = FindObjectOfType<WaypointSystem>();
#endif
            if (waypointSystem == null)
                Debug.LogWarning($"[{name}] No WaypointSystem found. Add one to the scene or assign it explicitly.");
        }

        // Find the closest waypoint at start
        if (waypointSystem != null && waypointSystem.GetPointsCount() > 0)
        {
            currentWaypointIndex = waypointSystem.GetClosestWaypointIndex(transform.position);
            currentWaypoint = waypointSystem.GetWaypointByIndex(currentWaypointIndex);
        }
        else
        {
            currentWaypoint = null;
        }
    }

    void FixedUpdate()
    {
        if (!isActiveAndEnabled || waypointSystem == null || currentWaypoint == null) return;
        MoveToWaypoint();
    }

    void Update()
    {
        CheckDie();

        if (animator == null) return;
        var v = rb.linearVelocity;
        float speed2D = new Vector2(v.x, v.z).magnitude;
        animator.SetFloat(moveSpeedHash, speed2D);
        animator.SetFloat(rotateSpeedHash, rb.angularVelocity.magnitude);
    }

    // --- Core movement with slope following, local avoidance, and step assist ---
    void MoveToWaypoint()
    {
        // Self constraints can freeze us
        foreach (IConstraintWayPoint constraint in selfConstraints)
        {
            if (!constraint.CheckState())
            {
                rb.constraints = RigidbodyConstraints.FreezeAll;
                rb.linearVelocity = Vector3.zero;
                return;
            }
        }

        // Direction to target
        Vector3 toTarget = currentWaypoint.waypoint.position - transform.position;
        float planarDist = new Vector2(toTarget.x, toTarget.z).magnitude;

        // Ground probing (ray from capsule center downward)
        Vector3 probeOrigin = transform.position + Vector3.up * Mathf.Max(0.5f, agentHeight * 0.5f);
        bool onGround = Physics.Raycast(
            probeOrigin,
            Vector3.down,
            out RaycastHit groundHit,
            Mathf.Max(agentHeight, 1.0f),
            groundMask,
            QueryTriggerInteraction.Ignore
        );

        // Desired direction projected to ground (slope following)
        Vector3 desiredDir = toTarget.normalized;
        if (onGround)
        {
            float slopeAngle = Vector3.Angle(groundHit.normal, Vector3.up);
            if (slopeAngle > maxSlope)
            {
                desiredDir = Vector3.zero; // too steep
            }
            else
            {
                desiredDir = Vector3.ProjectOnPlane(desiredDir, groundHit.normal).normalized;
            }
        }
        else
        {
            desiredDir.y = 0f;
            desiredDir.Normalize();
        }

        // Avoid neighbors on the same 'characterMask' layer, only other WaypointFollowers
        Vector3 avoid = Vector3.zero;
        int count = 0;

        Vector3 capP0 = transform.position + Vector3.up * 0.1f;
        Vector3 capP1 = transform.position + Vector3.up * (Mathf.Max(0.2f, agentHeight - 0.1f));
        Collider[] neighbors = Physics.OverlapCapsule(
            capP0, capP1, avoidRadius, characterMask, QueryTriggerInteraction.Ignore
        );

        foreach (var col in neighbors)
        {
            if (!col || col.attachedRigidbody == null) continue;
            if (col.attachedRigidbody == rb) continue;
            if (!col.GetComponent<WaypointFollower>()) continue; // only avoid our own agents

            Vector3 delta = transform.position - col.transform.position;
            delta.y = 0f;
            float d = delta.magnitude;
            if (d > 0.001f)
            {
                avoid += delta / (d * d); // stronger when closer
                count++;
            }
        }
        if (count > 0) avoid /= count;

        Vector3 moveDir = desiredDir;
        if (avoidWeight > 0f && avoid.sqrMagnitude > 0f)
            moveDir = (desiredDir + avoidWeight * avoid).normalized;

        // Waypoint arrival
        if (planarDist <= stoppingDistance)
        {
            rb.linearVelocity = new Vector3(0f, rb.linearVelocity.y, 0f);

            if (!SelectNextWaypoint(waypointSystem.GetLoopStatus()))
            {
                rb.constraints = RigidbodyConstraints.FreezeAll;
                rb.linearVelocity = Vector3.zero;
                return;
            }
        }
        else
        {
            // Final horizontal velocity
            Vector3 horizVel = moveDir * speed;

            // Keep feet stuck to ground a bit so we don't ride other colliders / small bumps
            float yVel = rb.linearVelocity.y;
            if (onGround && yVel <= 0f)
                horizVel += Vector3.down * stickToGroundForce * Time.fixedDeltaTime;

            // Step assist: if a small vertical face is ahead but the upper space is clear,
            // add a gentle upward bias so we "walk up" curbs instead of getting stuck.
            if (onGround && StepOffsetAssist(moveDir, out float yBias))
            {
                yVel = Mathf.Max(yVel, yBias);
            }

            rb.constraints = initialConstraints;
            rb.linearVelocity = new Vector3(horizVel.x, yVel, horizVel.z);
        }

        // Face movement direction for nicer steering
        Vector3 faceDir = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        if (faceDir.sqrMagnitude > 0.0001f)
        {
            Quaternion targetRot = Quaternion.LookRotation(faceDir.normalized, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, turnSpeed * Time.deltaTime);
        }
    }

    bool SelectNextWaypoint(bool loop = true)
    {
        // Check constraints attached to the waypoint (e.g., traffic lights)
        if (currentWaypoint != null)
        {
            IConstraintWayPoint[] constraintTraffics =
                currentWaypoint.waypoint.transform.GetComponentsInChildren<IConstraintWayPoint>();
            foreach (IConstraintWayPoint constraint in constraintTraffics)
            {
                if (!constraint.CheckState())
                {
                    rb.constraints = RigidbodyConstraints.FreezeAll;
                    rb.linearVelocity = Vector3.zero;
                    return false; // WAIT
                }
            }
        }

        if (waypointSystem == null || waypointSystem.GetPointsCount() == 0) return false;

        // End of path on non-looping route -> return to pool (deactivate)
        if (!loop && waypointSystem.EndPathCheck(currentWaypointIndex))
        {
            DeactivateForPooling();
            return false; // END
        }

        currentWaypointIndex = waypointSystem.GetNextWaypointIndex(currentWaypointIndex);
        currentWaypoint = waypointSystem.GetWaypointByIndex(currentWaypointIndex);
        return currentWaypoint != null;
    }

    public float GetSpeed() => rb ? rb.linearVelocity.magnitude : 0f;

    public void SetWaypointSystem(WaypointSystem waypoint)
    {
        waypointSystem = waypoint;
        // Snap to closest valid waypoint immediately if we can
        if (waypointSystem != null && waypointSystem.GetPointsCount() > 0)
        {
            currentWaypointIndex = waypointSystem.GetClosestWaypointIndex(transform.position);
            currentWaypoint = waypointSystem.GetWaypointByIndex(currentWaypointIndex);
        }
        else
        {
            currentWaypoint = null;
        }
    }

    public void ForceStartAtFirstWaypoint()
    {
        if (waypointSystem == null || waypointSystem.GetPointsCount() == 0)
        {
            Debug.LogWarning($"[{name}] No WaypointSystem or zero points. Cannot start at first waypoint.");
            return;
        }

        currentWaypointIndex = 0;
        currentWaypoint = waypointSystem.GetWaypointByIndex(0);

        Transform firstWaypoint = waypointSystem.GetFirstWaypoint();
        if (firstWaypoint != null)
            transform.SetPositionAndRotation(firstWaypoint.position, firstWaypoint.rotation);

        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public WaypointSystem GetWaypointSystem() => waypointSystem;

    private void CheckDie()
    {
        if (transform.position.y < -200f)
            DeactivateForPooling();
    }

    /// <summary>Cleanly disables this follower so the spawner can reuse it from the pool.</summary>
    private void DeactivateForPooling()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.constraints = RigidbodyConstraints.FreezeAll;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.Sleep();
        }

        if (animator == null) animator = GetComponentInChildren<Animator>();
        if (animator != null)
        {
            animator.SetFloat(moveSpeedHash, 0f);
            animator.SetFloat(rotateSpeedHash, 0f);
        }

        currentWaypointIndex = 0;
        currentWaypoint = null;
        gameObject.SetActive(false);
    }

    /// <summary>
    /// Helps the rigidbody climb a small step ("curb") in front of it.
    /// Two spherecasts: lower detects the face; upper checks free space above.
    /// If climbable, returns an upward Y velocity bias (no teleport).
    /// </summary>
    private bool StepOffsetAssist(Vector3 moveDir, out float yBias)
    {
        yBias = 0f;

        if (capsule == null) return false;
        if (moveDir.sqrMagnitude < 1e-4f) return false;

        Vector3 fwd = new Vector3(moveDir.x, 0f, moveDir.z).normalized;
        if (fwd.sqrMagnitude < 1e-4f) return false;

        float feetHeight = Mathf.Max(0.1f, capsule.radius * 0.5f);
        Vector3 lowerOrigin = transform.position + Vector3.up * feetHeight;
        Vector3 upperOrigin = lowerOrigin + Vector3.up * Mathf.Max(0.05f, stepMaxHeight);

        float castRadius = Mathf.Max(0.05f, capsule.radius * 0.9f);
        int mask = groundMask; // ignore characters; only consider world/ground

        bool lowerHit = Physics.SphereCast(
            lowerOrigin, castRadius, fwd,
            out RaycastHit hitLower,
            stepCheckDistance, mask,
            QueryTriggerInteraction.Ignore
        );
        if (!lowerHit) return false;

        // If normal is too slanted upward, it's not a vertical "step face"
        if (hitLower.normal.y > 0.1f) return false;

        bool upperHit = Physics.SphereCast(
            upperOrigin, castRadius, fwd,
            out _, stepCheckDistance, mask,
            QueryTriggerInteraction.Ignore
        );
        if (upperHit) return false; // blocked above: can't step

        yBias = stepClimbSpeed; // gentle upward push (used as max with current yVel)
        return true;
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        speed = Mathf.Max(0.01f, speed);
        turnSpeed = Mathf.Max(0.01f, turnSpeed);
        stoppingDistance = Mathf.Max(0f, stoppingDistance);
        avoidRadius = Mathf.Max(0.01f, avoidRadius);
        agentRadius = Mathf.Max(0.01f, agentRadius);
        agentHeight = Mathf.Max(0.5f, agentHeight);
        stepMaxHeight = Mathf.Max(0.01f, stepMaxHeight);
        stepCheckDistance = Mathf.Max(0.05f, stepCheckDistance);
        stepClimbSpeed = Mathf.Max(0.01f, stepClimbSpeed);

        // Keep upright by default
        if ((initialConstraints & (RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ)) == 0)
            initialConstraints |= RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // Refresh animator hashes if names changed in inspector
        moveSpeedHash = Animator.StringToHash(moveSpeedProperty);
        rotateSpeedHash = Animator.StringToHash(rotateSpeedProperty);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position + Vector3.up * 0.1f, avoidRadius);

        // Visualize step check
        Vector3 fwd = transform.forward;
        Vector3 lowerOrigin = transform.position + Vector3.up * Mathf.Max(0.1f, agentRadius * 0.5f);
        Vector3 upperOrigin = lowerOrigin + Vector3.up * Mathf.Max(0.05f, stepMaxHeight);
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(lowerOrigin, lowerOrigin + fwd * stepCheckDistance);
        Gizmos.DrawLine(upperOrigin, upperOrigin + fwd * stepCheckDistance);
    }
#endif
}
