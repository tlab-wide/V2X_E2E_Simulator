using System;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class WaypointFollower : MonoBehaviour,ISpeed
{
    public WaypointSystem waypointSystem;
    public float speed = 5f;
    public float turnSpeed = 5f;
    public float stoppingDistance = 0.5f;
    public bool loopPath = true; // Toggle for looping
    // public bool allowBidirectional = true; // Toggle for bidirectional movement

    [SerializeField] private WaypointSystem.WaypointNode currentWaypoint;
    [SerializeField] private int currentWaypointIndex;
    private IConstraintWayPoint[] selfConstraints;
    private Rigidbody rb;
    private RigidbodyConstraints initialConstraints;

    void Start()
    {
        selfConstraints = this.GetComponents<IConstraintWayPoint>();
        rb = GetComponent<Rigidbody>();
        initialConstraints = rb.constraints;
        
        animator = GetComponent<Animator>();
        if (animator == null)
        {
            animator = GetComponentInChildren<Animator>();
        }

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
        if (!gameObject.activeInHierarchy || waypointSystem == null)
        {
            return;
        }
        
        foreach (IConstraintWayPoint constraint in selfConstraints)
        {
            if (!constraint.CheckState())
            {
                //you are not allow to move
                rb.constraints = RigidbodyConstraints.FreezeAll;
                return;
            }
        }

        //move toward the target

        Vector3 direction = (currentWaypoint.waypoint.position - transform.position).normalized;
        Vector3 distanceVector = currentWaypoint.waypoint.position - transform.position;
        distanceVector.y = 0;
        float distance = Vector3.Magnitude(distanceVector); // we ignore y distance


        rb.constraints = initialConstraints;


        // Move forward if not close enough
        if (distance > stoppingDistance)
        {
            rb.linearVelocity = distanceVector.normalized * speed +   rb.linearVelocity.y * Vector3.up;
        }
        else
        {
            rb.linearVelocity = Vector3.zero;
            if (!SelectNextWaypoint(loopPath))
            {
                return;
            }
        }


        // Rotate towards the waypoint
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        float currentRotation = transform.rotation.eulerAngles.y;
        float yTargetRotation = targetRotation.eulerAngles.y;
        if (yTargetRotation - currentRotation > 180)
        {
            yTargetRotation -= 360;
        }
        else if (yTargetRotation - currentRotation < -180)
        {
            yTargetRotation += 360;
        }

        float new_rotation = Mathf.Lerp(currentRotation, yTargetRotation, turnSpeed * Time.deltaTime);

        this.transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, new_rotation,
            transform.rotation.eulerAngles.z);

        // rb.rotation = Quaternion.Slerp(rb.rotation, targetRotation, turnSpeed * Time.fixedDeltaTime);
    }

    bool SelectNextWaypoint(bool loop = true)
    {
        //check constraints
        IConstraintWayPoint[] constraintTraffics =
            currentWaypoint.waypoint.transform.GetComponentsInChildren<IConstraintWayPoint>();
        foreach (IConstraintWayPoint constraint in constraintTraffics)
        {
            if (!constraint.CheckState())
            {
                //you are not allow to move
                rb.constraints = RigidbodyConstraints.FreezeAll;
                return false;
            }
        }

        if (!loopPath && waypointSystem.EndPathCheck(currentWaypointIndex))
        {
            // Now the object is at the end of the path
            // rb.constraints = RigidbodyConstraints.FreezeAll;
            // return false;
            
            Destroy(this.gameObject);
            return false;
        }
        
        

        WaypointSystem.WaypointNode nextWaypoint = null;

        // Move only forward
        // nextWaypoint = currentWaypoint.nextWaypoint;
        currentWaypointIndex = waypointSystem.GetNextWaypointIndex(currentWaypointIndex);
        currentWaypoint = waypointSystem.GetWaypointByIndex(currentWaypointIndex);
        return true;
    }

    private const string moveSpeedProperty = "moveSpeed";
    private const string rotateSpeedProperty = "rotateSpeed";

    private Animator animator;

    //Animation update and falling check 
    private void Update()
    {
        CheckDie();
        if (animator==null)
        {
            // Cars don't have animators
            return;
        }
        
        // Switch animation based on movement speed (m/s).
        var speed2D = new Vector2(rb.linearVelocity.x, rb.linearVelocity.z).magnitude;
        // Debug.Log(speed2D);

        if (speed2D < 0.3f)
        {
            animator.SetFloat(moveSpeedProperty, 0);

            // Switch animation based on rotation speed (rad/s).
            animator.SetFloat(rotateSpeedProperty, 0);
        }
        else
        {
            animator.SetFloat(moveSpeedProperty, speed2D);

            // Switch animation based on rotation speed (rad/s).
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
    
    private void CheckDie()
    {
        if (transform.position.y < -200)
        {
            Destroy(this.gameObject);
        }
    }
}