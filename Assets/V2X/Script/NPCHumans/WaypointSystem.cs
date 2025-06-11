using System;
using System.Collections.Generic;
using UnityEngine;

public class WaypointSystem : MonoBehaviour
{
    [Serializable]
    public class WaypointNode
    {
        public Transform waypoint;
    }

    public List<WaypointNode> waypoints = new List<WaypointNode>();

    public WaypointNode GetClosestWaypoint(Vector3 position)
    {
        WaypointNode closest = null;
        float minDistance = Mathf.Infinity;

        foreach (WaypointNode node in waypoints)
        {
            float distance = Vector3.Distance(position, node.waypoint.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = node;
            }
        }

        return closest;
    }

    public int GetClosestWaypointIndex(Vector3 position)
    {
        WaypointNode closest = null;
        float minDistance = Mathf.Infinity;

        int idMinDist = 0;
        for (int i = 0; i < waypoints.Count; i++)
        {
            float distance = Vector3.Distance(position, waypoints[i].waypoint.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = waypoints[i];
                idMinDist = i;
            }
        }

        return idMinDist;
    }

    public WaypointNode GetWaypointByIndex(int index)
    {
        return waypoints[index];
    }

    public int GetNextWaypointIndex(int index)
    {
        return (index + 1 )% waypoints.Count;
    }

    public void GenerateWaypointsFromChildren()
    {
        waypoints.Clear();
        Transform[] children = GetComponentsInChildren<Transform>();

        foreach (Transform child in children)
        {
            if (child != transform) // Exclude parent itself
            {
                WaypointNode newNode = new WaypointNode
                {
                    waypoint = child
                };
                waypoints.Add(newNode);
            }
        }

        Debug.Log($"Generated {waypoints.Count} waypoints from children.");
    }


    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Count == 0) return;


        Gizmos.color = Color.cyan; // Sphere color for waypoints


        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            if (waypoints[i].waypoint != null)
            {
                Gizmos.DrawSphere(waypoints[i].waypoint.position, 0.3f);

                // Draw line to next waypoint

                Gizmos.color = Color.green;
                Gizmos.DrawLine(waypoints[i].waypoint.position, waypoints[i+1].waypoint.position);
            }
        }

        if (waypoints.Count > 1)
        {
            Gizmos.DrawLine(waypoints[waypoints.Count-1].waypoint.position, waypoints[0].waypoint.position);
        }
    }

    public int GetPointsCount()
    {
        return waypoints.Count;
    }

    public bool EndPathCheck(int currentWaypointIndex)
    {
        if (currentWaypointIndex == waypoints.Count-1)
        {
            return true;
        }

        return false;
    }
}