using System;
using System.Collections.Generic;
using RGLUnityPlugin;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;

[System.Serializable]
public class VisualizerPCD : MonoBehaviour
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
    private const string visualizationOutputNodeId = "OUT_VISUALIZATION";

    // private MonoBehaviour sensor;

    private RaycastLiDARSensor lidar;

    public void Awake()
    {
        rglSubgraphVisualizationOutput = new RGLNodeSequence()
            .AddNodePointsYield(visualizationOutputNodeId, RGLField.XYZ_VEC3_F32);

        rglSubgraphVisualizationOutput.SetPriority(visualizationOutputNodeId, 1);
    }

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
        if (indices.Length < points.Length)
        {
            indices = new int[points.Length];
            for (int i = 0; i < points.Length; ++i)
            {
                indices[i] = i;
            }
        }

        mesh.Clear();
        mesh.vertices = points;
        mesh.SetIndices(indices, 0, pointCount, MeshTopology.Points, 0);

        if (autoComputeColoringHeights)
        {
            minColoringHeight = mesh.bounds.min.y;
            maxColoringHeight = mesh.bounds.max.y;

            material.SetFloat("_MinColoringHeight", minColoringHeight);
            material.SetFloat("_MaxColoringHeight", maxColoringHeight);
        }
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
        
        Debug.Log("It is called");
        Debug.Log(lidar.hitPositions.Length);
        Debug.Log(lidar.hitPositions[0]);
        SetPoints(lidar.hitPositions);
    }
}