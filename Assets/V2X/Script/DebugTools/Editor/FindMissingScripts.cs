using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneDiagnostics : EditorWindow
{
    // =========================
    // MENU
    // =========================
    [MenuItem("Tools/Diagnostics/Find Missing Scripts In Open Scenes")]
    static void FindMissingScriptsInScene() {
        int count = 0;
        foreach (var go in GetAllSceneObjects()) {
            var comps = go.GetComponents<Component>();
            for (int i = 0; i < comps.Length; i++) {
                if (comps[i] == null) {
                    Debug.LogWarning(
                        $"[MissingScript] path='{GetHierarchyPath(go)}' scene='{go.scene.name}'",
                        go
                    );
                    count++;
                }
            }
        }
        Debug.Log($"[MissingScript] Done. Missing components found: {count}");
    }

    [MenuItem("Tools/Diagnostics/Find Empty MeshRenderers In Open Scenes")]
    static void FindEmptyMeshRenderersInScene() {
        int issues = 0;

        foreach (var go in GetAllSceneObjects()) {
            // --- MeshRenderer + MeshFilter ---
            var mr = go.GetComponent<MeshRenderer>();
            var mf = go.GetComponent<MeshFilter>();
            if (mr) {
                if (mf == null || mf.sharedMesh == null) {
                    LogMeshProblem("MeshRenderer", go, null, mr.sharedMaterials?.Length ?? 0, mr.enabled, "No MeshFilter or Mesh is null");
                    issues++;
                } else {
                    if (IsEmptyMeshByFacesOrEdges(mf.sharedMesh, out var faceCount, out var edgeCount)) {
                        LogEmptyMesh("MeshRenderer", go, mf.sharedMesh, mr.sharedMaterials?.Length ?? 0, mr.enabled, faceCount, edgeCount);
                        issues++;
                    }
                }
            }

            // --- SkinnedMeshRenderer ---
            var smr = go.GetComponent<SkinnedMeshRenderer>();
            if (smr) {
                if (smr.sharedMesh == null) {
                    LogMeshProblem("SkinnedMeshRenderer", go, null, smr.sharedMaterials?.Length ?? 0, smr.enabled, "Mesh is null");
                    issues++;
                } else if (IsEmptyMeshByFacesOrEdges(smr.sharedMesh, out var faceCount, out var edgeCount)) {
                    LogEmptyMesh("SkinnedMeshRenderer", go, smr.sharedMesh, smr.sharedMaterials?.Length ?? 0, smr.enabled, faceCount, edgeCount);
                    issues++;
                }
            }
        }

        Debug.Log($"[EmptyMesh] Done. Problematic/empty renderers found: {issues}");
    }

    [MenuItem("Tools/Diagnostics/Scan Scene Issues (Missing Scripts + Empty Meshes)")]
    static void ScanAll() {
        FindMissingScriptsInScene();
        FindEmptyMeshRenderersInScene();
    }

    // =========================
    // HELPERS
    // =========================

    static IEnumerable<GameObject> GetAllSceneObjects() {
        var roots = new List<GameObject>();
        for (int i = 0; i < SceneManager.sceneCount; i++) {
            var scene = SceneManager.GetSceneAt(i);
            if (!scene.isLoaded) continue;
            scene.GetRootGameObjects(roots);
            foreach (var root in roots) {
                foreach (var t in root.GetComponentsInChildren<Transform>(true)) {
                    yield return t.gameObject;
                }
            }
        }
    }

    static bool IsEmptyMeshByFacesOrEdges(Mesh mesh, out int faceCount, out int edgeCount) {
        faceCount = 0;
        edgeCount = 0;
        if (mesh == null || mesh.vertexCount == 0) return true;

        var edges = new HashSet<ulong>();
        int subCount = Mathf.Max(mesh.subMeshCount, 1);
        for (int s = 0; s < subCount; s++) {
            var topology = mesh.GetTopology(s);
            int[] idx;
            try { idx = mesh.GetIndices(s); }
            catch { continue; }
            if (idx == null || idx.Length == 0) continue;

            switch (topology) {
                case MeshTopology.Triangles:
                    faceCount += idx.Length / 3;
                    for (int i = 0; i + 2 < idx.Length; i += 3)
                        AddTriEdges(edges, idx[i], idx[i + 1], idx[i + 2]);
                    break;
#if UNITY_2020_2_OR_NEWER
                case MeshTopology.Quads:
                    faceCount += (idx.Length / 4) * 2;
                    for (int i = 0; i + 3 < idx.Length; i += 4) {
                        AddTriEdges(edges, idx[i], idx[i + 1], idx[i + 2]);
                        AddTriEdges(edges, idx[i], idx[i + 2], idx[i + 3]);
                    }
                    break;
#endif
                case MeshTopology.Lines:
                    for (int i = 0; i + 1 < idx.Length; i += 2)
                        AddEdge(edges, idx[i], idx[i + 1]);
                    break;
            }
        }
        edgeCount = edges.Count;
        return faceCount == 0 || edgeCount == 0;
    }

    static void LogEmptyMesh(string type, GameObject go, Mesh mesh, int mats, bool enabled, int faces, int edges) {
        string msg =
            $"[EmptyMesh] type='{type}' path='{GetHierarchyPath(go)}' scene='{go.scene.name}' " +
            $"mesh='{mesh?.name ?? "(null)"}' vertices={mesh?.vertexCount ?? 0} faces={faces} edges={edges} " +
            $"materials={mats} rendererEnabled={enabled}";
        Debug.LogWarning(msg, go);
    }

    static void LogMeshProblem(string type, GameObject go, Mesh mesh, int mats, bool enabled, string reason) {
        string msg =
            $"[MeshProblem] type='{type}' path='{GetHierarchyPath(go)}' scene='{go.scene.name}' " +
            $"reason='{reason}' materials={mats} rendererEnabled={enabled}";
        Debug.LogWarning(msg, go);
    }

    static string GetHierarchyPath(GameObject obj) {
        string path = obj.name;
        var t = obj.transform;
        while (t.parent != null) { t = t.parent; path = t.name + "/" + path; }
        return path;
    }

    // Edge helpers
    static void AddTriEdges(HashSet<ulong> set, int a, int b, int c) {
        AddEdge(set, a, b); AddEdge(set, b, c); AddEdge(set, c, a);
    }
    static void AddEdge(HashSet<ulong> set, int a, int b) {
        if (a == b) return;
        if (a > b) { var t = a; a = b; b = t; }
        ulong key = (((ulong)(uint)a) << 32) | (uint)b;
        set.Add(key);
    }
}
