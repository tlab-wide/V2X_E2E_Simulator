using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConstraintSaftyDistance : MonoBehaviour,IConstraintWayPoint
{
    [SerializeField] Transform startPoint;
    [SerializeField] float frequency;
    [SerializeField] float depth;
    
    public const int VEHICLE_LAYER = 6; // Vehicle layer index
    private float lastCheckTime = -Mathf.Infinity; // Stores the last time CheckState was called
    private bool lastCheckResult = false; // Stores the last raycast result
    public bool CheckState()
    {
        // If the required time hasn't passed since the last check, return the cached result
        if (Time.time - lastCheckTime < 1f / frequency)
        {
            return lastCheckResult;
        }

        // Update the last check time
        lastCheckTime = Time.time;

        // Perform the raycast
        RaycastHit hit;
        
        
        if (Physics.Raycast(startPoint.position, startPoint.forward, out hit, depth))
        {
            Debug.DrawLine(startPoint.position, hit.point, Color.red, 0.1f);
            if (hit.collider.gameObject.layer == VEHICLE_LAYER)
            {
                Debug.Log($"Ray hit: {hit.collider.name} on Vehicle layer");
                lastCheckResult = false;
                return false;
            }
        }

        lastCheckResult = true;
        return true;
    }

   
}
