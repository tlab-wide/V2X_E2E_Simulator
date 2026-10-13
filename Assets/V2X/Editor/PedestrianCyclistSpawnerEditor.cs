using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using V2X.Paths;

[CustomEditor(typeof(PedestrianCyclistSpawner))]
public class PedestrianCyclistSpawnerEditor : Editor
{
    private int selectedPathIndex = 0;
    private float handleSize = 0.6f;
    private bool showLabels = true;

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var spawner = (PedestrianCyclistSpawner)target;
        var paths = GetPaths(spawner);
        if (paths == null || paths.Count == 0)
            return;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Path Editing", EditorStyles.boldLabel);
        string[] names = BuildPathNames(paths);
        selectedPathIndex = Mathf.Clamp(selectedPathIndex, 0, names.Length - 1);
        selectedPathIndex = EditorGUILayout.Popup("Selected Path", selectedPathIndex, names);
        handleSize = EditorGUILayout.Slider("Handle Size", handleSize, 0.1f, 2f);
        showLabels = EditorGUILayout.Toggle("Show Labels", showLabels);
    }

    private void OnSceneGUI()
    {
        var spawner = (PedestrianCyclistSpawner)target;
        if (spawner == null) return;

        var paths = GetPaths(spawner);
        if (paths == null || paths.Count == 0) return;

        selectedPathIndex = Mathf.Clamp(selectedPathIndex, 0, paths.Count - 1);
        var path = paths[selectedPathIndex];
        if (path == null || path.waypoints == null || path.waypoints.Count == 0)
            return;

        Handles.color = GetColor(path.type);
        Vector3 prev = Vector3.zero;
        bool hasPrev = false;

        for (int i = 0; i < path.waypoints.Count; i++)
        {
            var wp = path.waypoints[i];
            Vector3 pos = spawner.ToUnityPosition(wp);

            if (hasPrev)
                Handles.DrawLine(prev, pos);

            if (showLabels)
                Handles.Label(pos + Vector3.up * 0.2f, $"{path.id}:{i}");

            EditorGUI.BeginChangeCheck();
            var fmh_60_58_639015162384816031 = Quaternion.identity; Vector3 newPos = Handles.FreeMoveHandle(pos, handleSize, Vector3.zero, Handles.SphereHandleCap);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(spawner, "Move NPC Path Waypoint");
                spawner.UpdatePathPointFromUnity(ref wp, newPos);
                path.waypoints[i] = wp;
                EditorUtility.SetDirty(spawner);
            }

            prev = newPos;
            hasPrev = true;
        }
    }

    private List<PathConfigLoader.PathDefinition> GetPaths(PedestrianCyclistSpawner spawner)
    {
        var type = spawner.GetType();
        var field = type.GetField("npcPaths", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        return field?.GetValue(spawner) as List<PathConfigLoader.PathDefinition>;
    }

    private string[] BuildPathNames(List<PathConfigLoader.PathDefinition> paths)
    {
        var names = new List<string>();
        for (int i = 0; i < paths.Count; i++)
        {
            var p = paths[i];
            if (p == null)
                names.Add($"<null {i}>");
            else
                names.Add($"{i}: {p.id} ({p.type})");
        }
        return names.ToArray();
    }

    private Color GetColor(PathConfigLoader.PathUserType type)
    {
        switch (type)
        {
            case PathConfigLoader.PathUserType.pedestrian: return Color.green;
            case PathConfigLoader.PathUserType.cyclist: return Color.cyan;
            case PathConfigLoader.PathUserType.both: return Color.yellow;
            default: return Color.white;
        }
    }

}
