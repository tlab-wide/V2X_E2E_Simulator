using UnityEngine;
using System.Collections.Generic;
using System.IO; // Required for directory and path operations
using RGLUnityPlugin; // Required for RGL and LidarSensor classes
using AWSIM.PointCloudMapping; // The original namespace for RGLScanAdapter

namespace AWSIM.Scanning
{
    /// <summary>
    /// A self-contained, central controller to manage, trigger, and save scans from multiple sensors.
    /// This class incorporates the RGL graph management logic previously found in RGLScanAdapter,
    /// removing the need for an adapter component on each sensor.
    /// </summary>
    public class MultiSensorStepScanner : MonoBehaviour
    {
        [Header("Sensor Configuration")]
        [SerializeField]
        [Tooltip("A list of all sensor GameObjects to be controlled. Each must have a LidarSensor component.")]
        private List<GameObject> sensorGameObjects;

        [Header("RGL Graph Configuration")]
        [SerializeField]
        [Tooltip("World origin in ROS coordinate systems, will be added to every point's coordinates.")]
        private Vector3 worldOriginROS;

        [SerializeField]
        [Tooltip("Enable/disable point cloud data downsampling.")]
        private bool enableDownsampling = true;

        [SerializeField]
        [Tooltip("Resolution for point cloud data downsampling. Smaller values mean higher density.")]
        [Min(0.000001f)]
        private float leafSize = 0.1f;

        [Header("Output Configuration")]
        [SerializeField]
        [Tooltip("The directory within the Assets folder where PCD files will be saved.")]
        private string outputDirectory = "PCD_Scans";

        [SerializeField]
        [Tooltip("The base name for the output PCD files. The sensor name will be appended to it.")]
        private string baseFileName = "scan_output";

        // Internal dictionary to manage a separate RGL subgraph for each sensor.
        private Dictionary<GameObject, RGLNodeSequence> sensorSubgraphs;

        private const string TransformNodeId = "ROS_WORLD_TF";
        private const string DownsampleNodeId = "DOWNSAMPLE";
        private const string TemporalMergeNodeId = "TEMPORAL_MERGE";

        void Start()
        {
            if (sensorGameObjects == null || sensorGameObjects.Count == 0)
            {
                Debug.LogError("MultiSensorStepScanner: No sensor GameObjects have been assigned. Disabling component.");
                enabled = false;
                return;
            }

            sensorSubgraphs = new Dictionary<GameObject, RGLNodeSequence>();

            foreach (var sensorGO in sensorGameObjects)
            {
                if (sensorGO != null && sensorGO.TryGetComponent<LidarSensor>(out var lidarSensor))
                {
                    // Disable automatic capturing to allow for manual, stepped control.
                    lidarSensor.AutomaticCaptureHz = 0;

                    // Initialize and connect an RGL subgraph for this specific sensor.
                    InitializeSubgraphForSensor(lidarSensor);
                }
                else
                {
                    Debug.LogWarning($"GameObject '{sensorGO?.name}' does not have a LidarSensor component and will be ignored.");
                }
            }
        }

        /// <summary>
        /// Creates, configures, and connects an RGL subgraph for a given LidarSensor.
        /// This logic is adapted from the RGLScanAdapter.Initialize method.
        /// </summary>
        private void InitializeSubgraphForSensor(LidarSensor lidarSensor)
        {
            // Calculate the coordinate system transformation matrix.
            var worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
            worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginROS);

            // Create the RGL node sequence for processing the point cloud.
            var subgraph = new RGLNodeSequence()
                .AddNodePointsTransform(TransformNodeId, worldTransform)
                .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize))
                .AddNodePointsTemporalMerge(TemporalMergeNodeId, new[] { RGLField.XYZ_VEC3_F32 });

            subgraph.SetActive(DownsampleNodeId, enableDownsampling);

            // Connect the newly created graph to the sensor's output.
            lidarSensor.ConnectToWorldFrame(subgraph);

            // Store the subgraph so we can access it later for saving.
            sensorSubgraphs[lidarSensor.gameObject] = subgraph;
            Debug.Log($"Initialized RGL subgraph for sensor: {lidarSensor.gameObject.name}");
        }

        /// <summary>
        /// Triggers a capture on all assigned sensors.
        /// This now includes recreating the RGL graph to ensure data from each scan is isolated,
        /// mimicking the behavior of RGLScanAdapter.CaptureStepByStep.
        /// </summary>
        public void TriggerScan()
        {
            if (!enabled) return;

            Debug.Log($"Triggering capture on {sensorSubgraphs.Count} sensor(s).");
            foreach (var sensorGO in sensorGameObjects)
            {
                if (sensorGO != null && sensorGO.TryGetComponent<LidarSensor>(out var lidarSensor))
                {
                    // To ensure data is not accumulated across captures, we clear the old graph
                    // and create a fresh one for each scan step.
                    if (sensorSubgraphs.TryGetValue(sensorGO, out var oldSubgraph))
                    {
                        oldSubgraph.Clear();
                    }
                    InitializeSubgraphForSensor(lidarSensor);

                    // Trigger the actual sensor capture.
                    lidarSensor.Capture();
                }
            }
        }

        /// <summary>
        /// Saves a separate .pcd file for each sensor using its managed subgraph.
        /// </summary>
        public void SaveIndividualScans()
        {
            string fullDirectoryPath = Path.Combine(Application.dataPath, outputDirectory);
            if (!Directory.Exists(fullDirectoryPath))
            {
                Directory.CreateDirectory(fullDirectoryPath);
            }

            Debug.Log($"Saving scans to directory: {fullDirectoryPath}");

            foreach (var entry in sensorSubgraphs)
            {
                var sensorGO = entry.Key;
                var subgraph = entry.Value;

                string sensorName = sensorGO.name;
                string fileName = $"{baseFileName}_{sensorName}.pcd";
                string fullFilePath = Path.Combine(fullDirectoryPath, fileName);

                // Use the subgraph directly to save the file.
                subgraph.SavePcdFile(fullFilePath);
                Debug.Log($"Saved PCD for {sensorName} to {fullFilePath}");
            }
        }

        /// <summary>
        /// Called by Unity when a value is changed in the Inspector.
        /// Updates the downsampling parameters on all managed RGL graphs.
        /// </summary>
        public void OnValidate()
        {
            if (sensorSubgraphs == null) return;

            foreach (var subgraph in sensorSubgraphs.Values)
            {
                subgraph.UpdateNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
                subgraph.SetActive(DownsampleNodeId, enableDownsampling);
            }
        }

        /// <summary>
        /// Example usage: Press 'Space' to trigger a scan, and 'S' to save the results.
        /// </summary>
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                TriggerScan();
            }

            if (Input.GetKeyDown(KeyCode.S))
            {
                SaveIndividualScans();
            }
        }
    }
}