using System.Collections.Generic;
using System.Diagnostics; // for [Conditional]
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(BoxCollider))]
public sealed class GroundTruthArea : MonoBehaviour
{
    [Tooltip("Only GameObjects on these layers will be tracked.")]
    public LayerMask layerMask;

    // Fast membership set used by gameplay/runtime logic.
    private readonly HashSet<GameObject> _inside = new HashSet<GameObject>();
    private BoxCollider _box;

    /// <summary>Snapshot of objects currently inside the area (layer-filtered).</summary>
    public IReadOnlyCollection<GameObject> Objects => _inside;

    #if UNITY_EDITOR
    [Header("Inspector Debug (Editor Only)")]
    [Tooltip("Update the list only when this GameObject is selected in the Inspector.")]
    [SerializeField] private bool onlyWhenSelected = true;

    [SerializeField, Tooltip("Runtime view of objects currently inside (editor only).")]
    private List<GameObject> _insideDebug = new List<GameObject>();
    #endif

    private void Awake()
    {
        _box = GetComponent<BoxCollider>();
        _box.isTrigger = true; // ensure trigger so OnTrigger callbacks fire
    }

    private void Start()
    {
        // Seed with anything already inside at start.
        var center = _box.bounds.center;
        var halfExtents = _box.bounds.extents;
        var hits = Physics.OverlapBox(center, halfExtents, transform.rotation, layerMask, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < hits.Length; i++)
        {
            var go = hits[i].attachedRigidbody ? hits[i].attachedRigidbody.gameObject : hits[i].gameObject;
            if (go && PassesMask(go)) _inside.Add(go);
        }
        SyncDebugListEditorOnly();
    }

    private bool PassesMask(GameObject go)
    {
        int bit = 1 << go.layer;
        return (layerMask.value & bit) != 0;
    }

    private void OnTriggerEnter(Collider other)
    {
        var go = other.attachedRigidbody ? other.attachedRigidbody.gameObject : other.gameObject;
        if (go && PassesMask(go))
        {
            _inside.Add(go);
            SyncDebugListEditorOnly();
        }
    }

    private void OnTriggerExit(Collider other)
    {
        var go = other.attachedRigidbody ? other.attachedRigidbody.gameObject : other.gameObject;
        if (go)
        {
            _inside.Remove(go);
            SyncDebugListEditorOnly();
        }
    }

    private void OnDisable()
    {
        _inside.Clear();
        ClearDebugListEditorOnly();
    }

    // --- Editor-only helpers (no cost in builds) ---

    [Conditional("UNITY_EDITOR")]
    private void SyncDebugListEditorOnly()
    {
        #if UNITY_EDITOR
        if (onlyWhenSelected && UnityEditor.Selection.activeGameObject != gameObject)
            return;

        _insideDebug.Clear();
        foreach (var go in _inside)
            if (go) _insideDebug.Add(go);

        // Sort for readability
        _insideDebug.Sort((a, b) =>
        {
            if (!a && !b) return 0;
            if (!a) return 1;
            if (!b) return -1;
            return string.Compare(a.name, b.name, System.StringComparison.Ordinal);
        });
        #endif
    }

    [Conditional("UNITY_EDITOR")]
    private void ClearDebugListEditorOnly()
    {
        #if UNITY_EDITOR
        _insideDebug?.Clear();
        #endif
    }

    #if UNITY_EDITOR
    // Handy button in the component's context menu while in Editor
    [ContextMenu("Refresh Debug List (Editor Only)")]
    private void RefreshDebugListContextMenu() => SyncDebugListEditorOnly();
    #endif
}
