using System;
using System.Collections.Generic;
using RGLUnityPlugin;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;

[System.Serializable]
public class VisualizerPCD1 : MonoBehaviour
{
    public enum PointShape
    {
        FlatSquare = 0,
        Box = 1,
        Pyramid = 2
    }

    private static readonly List<Color> rainbowColors = new List<Color>
    {
        Color.red,
        new Color(1, 0.5f, 0, 1), // orange
        Color.yellow,
        Color.green,
        Color.blue,
        new Color(0.5f, 0, 1, 1) // violet
    };

    [SerializeField] private PointShape pointShape = PointShape.Box;

    [SerializeField] [Range(0.005f, 0.5f)] private float pointSize = 0.05f;

    [SerializeField] private List<Color> colors = new List<Color>(rainbowColors);

    [SerializeField] private bool autoComputeColoringHeights = false;

    [SerializeField] private float minColoringHeight = 0f;

    [SerializeField] private float maxColoringHeight = 20f;

    private Material material = null;

    private static readonly int visualizationLayerID = 11;

    private Mesh mesh;

    private Vector3[] onlyHits = Array.Empty<Vector3>();
    private int pointCount = 0;
    private int[] indices = Array.Empty<int>();

    private RGLNodeSequence rglSubgraphVisualizationOutput;

    // private MonoBehaviour sensor;

    private RaycastLiDARSensor lidar;

  

    public void Start()
    {
        // Check if LiDAR is attached
        lidar = GetComponent<RaycastLiDARSensor>();
        lidar.scanCallBackEvent.AddListener(OnNewLidarData);
        

      

        mesh = new Mesh();
        if (!material)
        {
            material = Instantiate<Material>(Resources.Load("PointCloudMaterial", typeof(Material)) as Material);

            // Colors in material need to be initialized with maximum length of the array (6 in this case)
            material.SetColorArray("_Colors", rainbowColors);
            material.SetInt("_ColorsNum", rainbowColors.Count);
        }

        OnValidate();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
    }

    public void OnValidate()
    {
        if (!material)
        {
            return;
        }

        material.SetFloat("_PointSize", pointSize);
        material.SetInt("_PointShape", (int)pointShape);

        if (!autoComputeColoringHeights)
        {
            material.SetFloat("_MinColoringHeight", minColoringHeight);
            material.SetFloat("_MaxColoringHeight", maxColoringHeight);
        }

        material.SetColorArray("_Colors", colors);
        material.SetInt("_ColorsNum", colors.Count);
    }

    public void SetPoints(Vector3[] points)
    {
        //todo 
        //update the mesh with these points and render it 
        
        if (points == null || points.Length == 0)
        {
            mesh.Clear();
            return;
        }

        /* 1. Re-allocate index buffer only when size changes */
        if (indices.Length != points.Length)
        {
            indices = new int[points.Length];
            for (int i = 0; i < points.Length; ++i) indices[i] = i;
        }

        /* 2. Determine height range */
        float hMin = minColoringHeight;
        float hMax = maxColoringHeight;

        if (autoComputeColoringHeights)
        {
            hMin = float.MaxValue;
            hMax = float.MinValue;
            foreach (var p in points)
            {
                if (p.y < hMin) hMin = p.y;
                if (p.y > hMax) hMax = p.y;
            }
            // avoid zero span
            if (Mathf.Approximately(hMin, hMax)) hMax = hMin + 0.001f;

            material.SetFloat("_MinColoringHeight", hMin);
            material.SetFloat("_MaxColoringHeight", hMax);
        }

        /* 3. Generate vertex colours */
        int nCol = colors.Count;
        Color[] vCols = new Color[points.Length];

        for (int i = 0; i < points.Length; ++i)
        {
            float t = Mathf.InverseLerp(hMin, hMax, points[i].y);   // 0‒1
            float scaled = t * (nCol - 1);
            int   idx    = Mathf.FloorToInt(scaled);
            int   next   = Mathf.Clamp(idx + 1, 0, nCol - 1);
            float localT = scaled - idx;
            vCols[i] = Color.Lerp(colors[idx], colors[next], localT);
        }

        /* 4. Commit to mesh */
        mesh.Clear();
        mesh.vertices = points;
        mesh.colors   = vCols;
        mesh.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh.RecalculateBounds();
    }

    public void Update()
    {
        if (!lidar.enabled)
        {
            mesh.Clear();
        }

        Graphics.DrawMesh(mesh, Vector3.zero, Quaternion.identity, material, visualizationLayerID);
    }

    public void OnNewLidarData()
    {
        if (!enabled)
        {
            return;
        }
        //
        // pointCount = rglSubgraphVisualizationOutput.GetResultData<Vector3>(ref onlyHits);
        
        SetPoints(lidar.hitPositions);
    }
}