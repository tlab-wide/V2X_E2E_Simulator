using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(WaypointSystem))]
public class WaypointEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        WaypointSystem waypointSystem = (WaypointSystem)target;

        // Button to generate waypoints from children
        if (GUILayout.Button("Generate Waypoints from Children"))
        {
            waypointSystem.GenerateWaypointsFromChildren();
            // EditorUtility.SetDirty(waypointSystem);  // Mark object as modified
            Debug.Log("Waypoints generated from children.");
        }

        // Button to clear waypoints
        if (GUILayout.Button("Clear Waypoints"))
        {
            waypointSystem.waypoints.Clear();
            EditorUtility.SetDirty(waypointSystem);
            Debug.Log("Waypoints cleared.");
        }
    }
}
