// LiDARPointCloudRenderer.cs  – Unity 6 compatible
// Renders the point cloud produced by RaycastLiDARSensor as instanced meshes.

using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using UnitySensors.Sensor.LiDAR;
using UnitySensors.DataType.Sensor.PointCloud;

[RequireComponent(typeof(RaycastLiDARSensor))]
public class LiDARPointCloudRenderer : MonoBehaviour
{
    /* ─────────────────────────── Inspector ──────────────────────────── */

    [Header("Visual")]
    [SerializeField] private Mesh     pointMesh   = null;    // tiny cube / sphere
    [SerializeField] private Material pointMat    = null;    // unlit, vertex-colour shader
    [SerializeField, Min(0.001f)] private float pointSize = 0.02f;

    [Header("Intensity mapping")]
    [SerializeField] private Gradient intensityGradient;    // default is set in Reset()

    /* ─────────────────────────── Internals ──────────────────────────── */

    private RaycastLiDARSensor _sensor;
    private Matrix4x4[] _matrices;
    private Vector4[]   _colours;
    private const int   _batchSize = 1023;                   // DrawMeshInstanced hard limit

    /* ─────────────────────────── Unity flow ─────────────────────────── */

    /// <summary> Editor-time defaults. Called when the component is added or reset. </summary>
    private void Reset()
    {
        // Create a black→white gradient if none assigned yet.
        if (intensityGradient == null || intensityGradient.colorKeys.Length == 0)
        {
            intensityGradient = new Gradient();
            intensityGradient.SetKeys(
                new[] {
                    new GradientColorKey(Color.black, 0f),
                    new GradientColorKey(Color.white, 1f)
                },
                new[] {
                    new GradientAlphaKey(1f, 0f),
                    new GradientAlphaKey(1f, 1f)
                });
        }
    }

    private void Awake()
    {
        _sensor = GetComponent<RaycastLiDARSensor>();

        // Fallback assets if user forgot to assign them in the Inspector.
        if (pointMesh == null)  pointMesh = CreatePrimitiveCube();
        if (pointMat  == null)  pointMat  = Resources.Load<Material>("UnlitPointCloud");

        int maxPoints = _sensor.pointsNum;
        _matrices = new Matrix4x4[maxPoints];
        _colours  = new Vector4  [maxPoints];

        _sensor.onSensorUpdated += HandleSensorUpdated;
    }

    private void OnDestroy()
    {
        if (_sensor != null)
            _sensor.onSensorUpdated -= HandleSensorUpdated;
    }

    /* ────────────────────── Point-cloud rendering ────────────────────── */

    private void HandleSensorUpdated()
    {
        NativeArray<PointXYZI> pts = _sensor.pointCloud.points;
        if (!pts.IsCreated || pts.Length == 0) return;

        int       count      = pts.Length;
        int       emitCount  = 0;
        Vector3   origin     = _sensor.transform.position;
        Quaternion rot       = _sensor.transform.rotation;

        // Build instance data
        for (int i = 0; i < count; ++i)
        {
            PointXYZI p = pts[i];             // read-only copy

            if (p.intensity <= 0f) continue;  // skip “no-hit” slots

            Vector3 worldPos = origin + rot * p.position;
            _matrices[emitCount] = Matrix4x4.TRS(
                worldPos,
                Quaternion.identity,
                Vector3.one * pointSize
            );

            Color c = intensityGradient.Evaluate(p.intensity / _sensor.maxIntensity);
            _colours[emitCount] = new Vector4(c.r, c.g, c.b, c.a);

            ++emitCount;
        }

        if (emitCount == 0) return;

        // Push per-instance colours and draw in batches
        pointMat.SetVectorArray("_BaseColor", _colours);

        int drawn = 0;
        while (drawn < emitCount)
        {
            int slice = Mathf.Min(_batchSize, emitCount - drawn);
            Graphics.DrawMeshInstanced(
                pointMesh, 0, pointMat,
                _matrices, slice, null,
                UnityEngine.Rendering.ShadowCastingMode.Off,
                receiveShadows: false, layer: 0,
                camera: null, LightProbeUsage.Off
            );
            drawn += slice;
        }
    }

    /* ────────────────────────── Utilities ────────────────────────────── */

    /// <summary>Creates a 1-unit cube mesh at runtime if none supplied.</summary>
    private static Mesh CreatePrimitiveCube()
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Mesh mesh = go.GetComponent<MeshFilter>().sharedMesh;
        Object.DestroyImmediate(go);
        return mesh;
    }
}
