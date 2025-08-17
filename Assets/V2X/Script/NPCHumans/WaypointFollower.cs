using System;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class WaypointFollower : MonoBehaviour, ISpeed
{
    public WaypointSystem waypointSystem;
    public float speed = 5f;
    public float turnSpeed = 5f;
    public float stoppingDistance = 0.5f;

    [SerializeField] private WaypointSystem.WaypointNode currentWaypoint;
    [SerializeField] private int currentWaypointIndex;
    private IConstraintWayPoint[] selfConstraints;
    private Rigidbody rb;
    [SerializeField]private RigidbodyConstraints initialConstraints = RigidbodyConstraints.FreezeRotation;

    private const string moveSpeedProperty = "moveSpeed";
    private const string rotateSpeedProperty = "rotateSpeed";
    private Animator animator;
    

    void Start()
    {
        selfConstraints = GetComponents<IConstraintWayPoint>();
        rb = GetComponent<Rigidbody>();

        animator = GetComponent<Animator>();
        if (animator == null) animator = GetComponentInChildren<Animator>();

        // Find the closest waypoint at the start
        if (gameObject.activeInHierarchy && waypointSystem != null)
        {
            currentWaypointIndex = waypointSystem.GetClosestWaypointIndex(transform.position);
            currentWaypoint = waypointSystem.GetWaypointByIndex(currentWaypointIndex);
        }
    }

    void FixedUpdate()
    {
        if (gameObject.activeInHierarchy && currentWaypoint != null && waypointSystem != null)
        {
            MoveToWaypoint();
        }
    }

    private static readonly Vector3 PlaneXZ = new Vector3(1f, 0f, 1f);

    void MoveToWaypoint()
    {
        if (!gameObject.activeInHierarchy || waypointSystem == null) return;

        foreach (IConstraintWayPoint constraint in selfConstraints)
        {
            if (!constraint.CheckState())
            {
                // blocked by self-constraint: freeze & wait (stay active)
                rb.constraints = RigidbodyConstraints.FreezeAll;
                rb.linearVelocity = Vector3.zero;
                return;
            }
        }

        // move toward the target
        Vector3 direction = (currentWaypoint.waypoint.position - transform.position).normalized;
        Vector3 distanceVector = currentWaypoint.waypoint.position - transform.position;
        distanceVector.y = 0;
        float distance = Vector3.Magnitude(distanceVector); // ignore y distance

        rb.constraints = initialConstraints;

        if (distance > stoppingDistance)
        {
            rb.linearVelocity = distanceVector.normalized * speed + rb.linearVelocity.y * Vector3.up;
        }
        else
        {
            rb.linearVelocity = Vector3.zero;

            // Try to advance; false means either WAIT or END. Only end-of-path deactivates (inside SelectNextWaypoint).
            if (!SelectNextWaypoint(waypointSystem.GetLoopStatus()))
            {
                // WAIT case: stay frozen & active; END case already handled (deactivated) inside SelectNextWaypoint.
                rb.constraints = RigidbodyConstraints.FreezeAll;
                rb.linearVelocity = Vector3.zero;
                return;
            }
        }

        // Rotate towards the waypoint
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        float currentRotation = transform.rotation.eulerAngles.y;
        float yTargetRotation = targetRotation.eulerAngles.y;
        if (yTargetRotation - currentRotation > 180) yTargetRotation -= 360;
        else if (yTargetRotation - currentRotation < -180) yTargetRotation += 360;

        float new_rotation = Mathf.Lerp(currentRotation, yTargetRotation, turnSpeed * Time.deltaTime);
        transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, new_rotation, transform.rotation.eulerAngles.z);
    }

    bool SelectNextWaypoint(bool loop = true)
    {
        // check constraints on the waypoint itself
        IConstraintWayPoint[] constraintTraffics =
            currentWaypoint.waypoint.transform.GetComponentsInChildren<IConstraintWayPoint>();
        foreach (IConstraintWayPoint constraint in constraintTraffics)
        {
            if (!constraint.CheckState())
            {
                // blocked by traffic constraint: freeze & WAIT (stay active)
                rb.constraints = RigidbodyConstraints.FreezeAll;
                rb.linearVelocity = Vector3.zero;
                return false; // WAIT, not end-of-path
            }
        }

        // End of path on non-looping route -> return to pool (deactivate)
        if (!waypointSystem.GetLoopStatus() && waypointSystem.EndPathCheck(currentWaypointIndex))
        {
            DeactivateForPooling(); // <-- only here we return to pool
            return false; // END
        }

        // advance to next waypoint (forward only)
        currentWaypointIndex = waypointSystem.GetNextWaypointIndex(currentWaypointIndex);
        currentWaypoint = waypointSystem.GetWaypointByIndex(currentWaypointIndex);
        return true;
    }

    // Animation update and falling check 
    private void Update()
    {
        CheckDie();

        if (animator == null) return; // objects without animators

        // Switch animation based on movement speed (m/s).
        var speed2D = new Vector2(rb.linearVelocity.x, rb.linearVelocity.z).magnitude;

        if (speed2D < 0.3f)
        {
            animator.SetFloat(moveSpeedProperty, 0);
            animator.SetFloat(rotateSpeedProperty, 0);
        }
        else
        {
            animator.SetFloat(moveSpeedProperty, speed2D);
            animator.SetFloat(rotateSpeedProperty, rb.angularVelocity.magnitude);
        }
    }

    public float GetSpeed()
    {
        return rb.linearVelocity.magnitude;
    }

    public void SetWaypointSystem(WaypointSystem waypoint)
    {
        this.waypointSystem = waypoint;
    }

    public void ForceStartAtFirstWaypoint()
    {
        if (waypointSystem == null)
        {
            Debug.LogWarning($"[{name}] No WaypointSystem assigned. Cannot start at first waypoint.");
            return;
        }
        if (waypointSystem.GetPointsCount() == 0)
        {
            Debug.LogWarning($"[{name}] WaypointSystem '{waypointSystem.name}' has no waypoints.");
            return;
        }

        // Set waypoint index to 0
        currentWaypointIndex = 0;
        currentWaypoint = waypointSystem.GetWaypointByIndex(0);

        // Snap to first waypoint's position and rotation
        Transform firstWaypoint = waypointSystem.GetFirstWaypoint();
        if (firstWaypoint != null)
        {
            transform.SetPositionAndRotation(firstWaypoint.position, firstWaypoint.rotation);
        }

        // Reset Rigidbody motion
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    public WaypointSystem GetWaypointSystem()
    {
        return waypointSystem;
    }

    private void CheckDie()
    {
        if (transform.position.y < -200)
        {
            // fell off the map — return to pool rather than destroy
            DeactivateForPooling();
        }
    }

    /// <summary>
    /// Cleanly disables this follower so the spawner can reuse it from the pool.
    /// </summary>
    private void DeactivateForPooling()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.constraints = RigidbodyConstraints.FreezeAll;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        if (animator == null) animator = GetComponentInChildren<Animator>();
        if (animator != null)
        {
            animator.SetFloat(moveSpeedProperty, 0f);
            animator.SetFloat(rotateSpeedProperty, 0f);
        }

        // reset internal state; spawner will ForceStartAtFirstWaypoint() on reuse
        currentWaypointIndex = 0;
        currentWaypoint = null;

        gameObject.SetActive(false);
    }
}
