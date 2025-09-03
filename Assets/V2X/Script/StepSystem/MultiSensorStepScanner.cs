using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO; // Required for directory and path operations
using RGLUnityPlugin; // Required for RGL and LidarSensor classes
using AWSIM.PointCloudMapping; // The original namespace for RGLScanAdapter

namespace AWSIM.Scanning
{
    /// <summary>
    /// Central controller to manage, trigger, and save scans from multiple sensors (LiDAR + Cameras).
    /// - Pure standalone LiDAR scans each step (no temporal integration).
    /// - Camera screenshots per step.
    /// - One trigger that captures and writes files with pattern: sensorName#step.ext
    /// </summary>
    public class MultiSensorStepScanner : MonoBehaviour
    {
        [Header("LiDAR Sensor Configuration")]
        [SerializeField]
        [Tooltip("All LiDAR GameObjects to be controlled. Each must have a LidarSensor component.")]
        private List<GameObject> sensorGameObjects;

        [Header("Camera Configuration")]
        [SerializeField]
        [Tooltip("Unity Camera components to capture images from each step.")]
        private List<Camera> cameras;

        [SerializeField]
        [Tooltip("Captured image width in pixels.")]
        private int imageWidth = 1920;

        [SerializeField]
        [Tooltip("Captured image height in pixels.")]
        private int imageHeight = 1080;

        public enum ImageFormat { PNG, JPG, EXR }
        [SerializeField]
        [Tooltip("Image format used when saving camera captures.")]
        private ImageFormat imageFormat = ImageFormat.PNG;

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
        [Tooltip("Directory within Assets where outputs are saved. Subfolders 'Lidar' and 'Images' are created inside.")]
        private string outputDirectoryRoot = "SensorSteps";

        // Internal dictionary to manage a separate RGL subgraph for each sensor.
        private Dictionary<GameObject, RGLNodeSequence> sensorSubgraphs;

        // Step index increased every time we capture & save a step.
        [SerializeField, Tooltip("Current step index (zero-padded in filenames).")]
        private int stepIndex = 0;

        private const string TransformNodeId = "ROS_WORLD_TF";
        private const string DownsampleNodeId = "DOWNSAMPLE";
        // NOTE: No temporal merge node => standalone scan every time.

        void Start()
        {
            if ((sensorGameObjects == null || sensorGameObjects.Count == 0) &&
                (cameras == null || cameras.Count == 0))
            {
                Debug.LogError("MultiSensorStepScanner: No sensors (LiDAR or Camera) assigned. Disabling component.");
                enabled = false;
                return;
            }

            sensorSubgraphs = new Dictionary<GameObject, RGLNodeSequence>();

            // Initialize LiDARs
            if (sensorGameObjects != null)
            {
                foreach (var sensorGO in sensorGameObjects)
                {
                    if (sensorGO != null && sensorGO.TryGetComponent<LidarSensor>(out var lidarSensor))
                    {
                        // Disable automatic capturing to allow for manual, stepped control.
                        lidarSensor.AutomaticCaptureHz = 0;

                        // Create a fresh RGL subgraph for this specific sensor (standalone per step).
                        InitializeSubgraphForSensor(lidarSensor);
                    }
                    else if (sensorGO != null)
                    {
                        Debug.LogWarning($"GameObject '{sensorGO.name}' does not have a LidarSensor component and will be ignored.");
                    }
                }
            }

            // Validate cameras list but no heavy init needed.
            if (cameras == null) cameras = new List<Camera>();
        }

        /// <summary>
        /// Creates, configures, and connects an RGL subgraph for a given LidarSensor.
        /// Standalone pipeline: Transform (+ optional downsample). No temporal accumulation.
        /// </summary>
        private void InitializeSubgraphForSensor(LidarSensor lidarSensor)
        {
            // Calculate the coordinate system transformation matrix.
            var worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
            worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginROS);

            // Create the RGL node sequence for processing the point cloud.
            var subgraph = new RGLNodeSequence()
                .AddNodePointsTransform(TransformNodeId, worldTransform);

            // Add downsampling node (toggle via SetActive).
            subgraph.AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
            subgraph.SetActive(DownsampleNodeId, enableDownsampling);

            // Connect the newly created graph to the sensor's output.
            lidarSensor.ConnectToWorldFrame(subgraph);

            // Store the subgraph so we can access it later for saving.
            sensorSubgraphs[lidarSensor.gameObject] = subgraph;
            Debug.Log($"Initialized RGL subgraph for sensor: {lidarSensor.gameObject.name}");
        }

        /// <summary>
        /// Triggers a LiDAR capture on all assigned sensors with fresh graphs (no accumulation).
        /// </summary>
        private void TriggerLidarScans()
        {
            if (!enabled) return;

            if (sensorGameObjects == null || sensorGameObjects.Count == 0) return;

            Debug.Log($"Triggering LiDAR capture on {sensorSubgraphs.Count} sensor(s).");
            foreach (var sensorGO in sensorGameObjects)
            {
                if (sensorGO != null && sensorGO.TryGetComponent<LidarSensor>(out var lidarSensor))
                {
                    // Ensure data is not accumulated across captures: clear and recreate graph for this step
                    if (sensorSubgraphs.TryGetValue(sensorGO, out var oldSubgraph))
                    {
                        oldSubgraph.Clear();
                    }
                    InitializeSubgraphForSensor(lidarSensor);

                    // Trigger the actual sensor capture (expected to be per-frame/step).
                    lidarSensor.Capture();
                }
            }
        }

        /// <summary>
        /// Saves a separate .pcd file for each LiDAR using its managed subgraph.
        /// Filename: sensorName#step.pcd
        /// </summary>
        private void SaveLidarScans()
        {
            if (sensorSubgraphs == null || sensorSubgraphs.Count == 0) return;

            string lidarDir = Path.Combine(Application.dataPath, outputDirectoryRoot, "Lidar");
            Directory.CreateDirectory(lidarDir);

            foreach (var entry in sensorSubgraphs)
            {
                var sensorGO = entry.Key;
                var subgraph = entry.Value;

                string sensorName = SanitizeName(sensorGO.name);
                string fileName = $"{sensorName}#{stepIndex:D4}.pcd";
                string fullFilePath = Path.Combine(lidarDir, fileName);

                subgraph.SavePcdFile(fullFilePath);
                Debug.Log($"Saved PCD for {sensorName} to {fullFilePath}");
            }
        }

        /// <summary>
        /// Captures and saves an image for each configured camera.
        /// Filename: cameraName#step.(png|jpg|exr)
        /// </summary>
        private void SaveCameraShots()
        {
            if (cameras == null || cameras.Count == 0) return;

            string imgDir = Path.Combine(Application.dataPath, outputDirectoryRoot, "Images");
            Directory.CreateDirectory(imgDir);

            foreach (var cam in cameras)
            {
                if (cam == null) continue;

                string camName = SanitizeName(cam.name);
                string filePath = Path.Combine(imgDir, $"{camName}#{stepIndex:D4}.{GetImageExtension()}");
                CaptureCameraToFile(cam, imageWidth, imageHeight, imageFormat, filePath);
                Debug.Log($"Saved camera image for {camName} to {filePath}");
            }
        }

        /// <summary>
        /// Main entry point: triggers a new step capture and writes files for LiDARs and Cameras.
        /// Call this to perform one "step".
        /// </summary>
        public void TriggerAndSaveStep()
        {
            // 1) Trigger LiDARs (fresh subgraphs => standalone capture)
            TriggerLidarScans();

            // 2) Save LiDAR scans for this step
            SaveLidarScans();

            // 3) Capture and save camera images
            SaveCameraShots();

            // 4) Advance the step counter
            stepIndex++;
        }

        /// <summary>
        /// Optional coroutine version if you prefer to ensure a frame passes.
        /// Useful if your pipeline requires the next frame for rendering side-effects.
        /// </summary>
        public IEnumerator TriggerAndSaveStepCoroutine()
        {
            TriggerAndSaveStep();
            yield return null;
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

            imageWidth = Mathf.Max(1, imageWidth);
            imageHeight = Mathf.Max(1, imageHeight);
        }

        /// <summary>
        /// Example usage: Press 'Space' to capture & save a full step (LiDAR + Cameras).
        /// </summary>
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                TriggerAndSaveStep();
            }
        }

        // --------------------------
        // Helpers
        // --------------------------

        private static string SanitizeName(string name)
        {
            // Remove/replace characters that could upset filesystems. '#' is allowed and required by spec.
            foreach (char c in Path.GetInvalidFileNameChars())
                name = name.Replace(c.ToString(), "_");
            return name;
        }

        private static string GetImageExtension()
        {
            return imageExtStatic switch
            {
                ImageFormat.JPG => "jpg",
                ImageFormat.EXR => "exr",
                _ => "png",
            };
        }

        // Static cache for enum inside static method
        private static ImageFormat imageExtStatic = ImageFormat.PNG;

        private void CaptureCameraToFile(Camera cam, int width, int height, ImageFormat fmt, string filePath)
        {
            // Allow GetImageExtension() to know current format
            imageExtStatic = fmt;

            var rt = new RenderTexture(width, height, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default);
            var prevTarget = cam.targetTexture;
            var prevActive = RenderTexture.active;

            try
            {
                cam.targetTexture = rt;
                RenderTexture.active = rt;
                cam.Render();

                var tex = new Texture2D(width, height, fmt == ImageFormat.EXR ? TextureFormat.RGBAFloat : TextureFormat.RGB24, false, fmt == ImageFormat.EXR);
                tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
                tex.Apply();

                byte[] bytes;
                switch (fmt)
                {
                    case ImageFormat.JPG:
                        bytes = tex.EncodeToJPG(95);
                        break;
                    case ImageFormat.EXR:
                        bytes = tex.EncodeToEXR(Texture2D.EXRFlags.OutputAsFloat | Texture2D.EXRFlags.CompressZIP);
                        break;
                    default:
                        bytes = tex.EncodeToPNG();
                        break;
                }

                File.WriteAllBytes(filePath, bytes);
                Destroy(tex);
            }
            finally
            {
                cam.targetTexture = prevTarget;
                RenderTexture.active = prevActive;
                rt.Release();
                Destroy(rt);
            }
        }
    }
}
