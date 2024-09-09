using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GizmoPCD : MonoBehaviour
{
    public Color gizmoColor;
    public float radius = 0.3f;

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        Gizmos.color = gizmoColor;
        Gizmos.DrawSphere(transform.position, radius);
    }
#endif
}
