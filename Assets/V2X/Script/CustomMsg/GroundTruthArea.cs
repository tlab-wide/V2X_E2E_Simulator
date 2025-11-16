using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(BoxCollider))]
public sealed class GroundTruthArea : DetectionSensor
{
    [Tooltip("Only GameObjects on these layers will be tracked.")]
    public LayerMask layerMask;

    [Tooltip("List of objects currently inside the area.")]
    [SerializeField] private List<GameObject> _inside = new List<GameObject>();

    private BoxCollider _box;

    private void Awake()
    {
        _box = GetComponent<BoxCollider>();
        _box.isTrigger = true;
    }

    private void Start()
    {
        // Find any objects already inside at the start
        var bounds = _box.bounds;
        var hits = Physics.OverlapBox(bounds.center, bounds.extents, transform.rotation, layerMask, QueryTriggerInteraction.Ignore);

        foreach (var hit in hits)
        {
            var go = hit.attachedRigidbody ? hit.attachedRigidbody.gameObject : hit.gameObject;
            if (go && IsInLayerMask(go) && !_inside.Contains(go))
            {
                _inside.Add(go);
            }
        }
    }

    private bool IsInLayerMask(GameObject go)
    {
        return (layerMask.value & (1 << go.layer)) != 0;
    }

    private void OnTriggerEnter(Collider other)
    {
        var go = other.attachedRigidbody ? other.attachedRigidbody.gameObject : other.gameObject;
        if (go && IsInLayerMask(go) && !_inside.Contains(go))
        {
            _inside.Add(go);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        var go = other.attachedRigidbody ? other.attachedRigidbody.gameObject : other.gameObject;
        if (go && _inside.Contains(go))
        {
            _inside.Remove(go);
        }
    }

    private void OnDisable()
    {
        _inside.Clear();
    }

    public override List<Transform> GetSeenObjects()
    {
        
        // Remove destroyed references (but keep inactive ones for visibility)
        _inside.RemoveAll(go => go == null);
        
        // Build a list of active ones only
        var result = new List<Transform>();
        for (int i = 0; i < _inside.Count; i++)
        {
            var go = _inside[i];
            if (go != null && go.activeInHierarchy &&  !go.tag.Equals("Ego"))
            {
                result.Add(go.transform);
            }
        }


        Debug.Log($"{this.gameObject.name} Ground Truth size message{result.Count}");
        return result;
    }
}
