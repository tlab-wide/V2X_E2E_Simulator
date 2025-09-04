using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO; // Directory and path ops
using RGLUnityPlugin; // LidarSensor + RGLNodeSequence
using AWSIM.PointCloudMapping; // Keep if your project already references it (for ROS2.Transformations)

// NOTE: Requires ROS2.Transformations.Unity2RosMatrix4x4() in your project when global mode is used.

namespace AWSIM.Scanning
{
    /// <summary>
    /// Central controller to manage, trigger, and save scans from multiple sensors (LiDAR + Cameras).
    /// - Standalone LiDAR scans each step (no temporal accumulation).
    /// - Toggle between global (ROS) or local LiDAR coordinates without modifying LidarSensor.
    /// - Camera screenshots per step.
    /// - Filenames follow: sensorName#step.ext
    /// </summary>
    public class MultiSensorStepScanner : MonoBehaviour
    {
        [Header("LiDAR Sensor Configuration")]
        [SerializeField]
        [Tooltip("All LiDAR GameObjects to be controlled. Each must have a LidarSensor component.")]
        private List<GameObject> sensorGameObjects = new List<GameObject>();

        [Header("Coordinate Frame")]
        [SerializeField]
        [Tooltip("If true, LiDAR outputs are transformed to ROS world using worldOriginROS.\nIf false, LiDAR outputs are saved in LiDAR-local coordinates.")]
        private bool useGlobalCoordinatesForLidar = false;

        [SerializeField]
        [Tooltip("World origin in ROS coordinate system; used only when 'useGlobalCoordinatesForLidar' is true.")]
        private Vector3 worldOriginROS;

        [Header("Downsampling (applies to saved clouds)")]
        [SerializeField]
        [Tooltip("Enable/disable point cloud downsampling for saved outputs.")]
        private bool enableDownsampling = true;

        [SerializeField]
        [Tooltip("Voxel size for downsampling. Smaller values mean higher density.")]
        [Min(0.000001f)]
        private float leafSize = 0.1f;

        [Header("Cameras")]
        [SerializeField]
        [Tooltip("Unity Camera components to capture images from each step.")]
        private List<Camera> cameras = new List<Camera>();

        [SerializeField, Tooltip("Optional: populate 'cameras' from all enabled Cameras on Start().")]
        private bool autoPopulateCameras = false;

        [SerializeField, Min(1)] private int imageWidth = 1920;
        [SerializeField, Min(1)] private int imageHeight = 1080;

        public enum ImageFormat { PNG, JPG, EXR }
        [SerializeField] private ImageFormat imageFormat = ImageFormat.PNG;

        [Header("Output")]
        [SerializeField]
        [Tooltip("Root folder (inside Assets) where outputs are saved. Subfolders 'Lidar' and 'Images' are created.")]
        private string outputDirectoryRoot = "SensorSteps";

        [SerializeField, Tooltip("Zero-based step index; auto-increments after each step.")]
        private int stepIndex = 0;

        // Per-sensor subgraphs we own (one fresh subgraph per step for standalone output)
        private Dictionary<GameObject, RGLNodeSequence> sensorSubgraphs;

        // Node IDs inside our subgraphs
        private const string TransformNodeId = "ROS_WORLD_TF";
        private const string DownsampleNodeId = "DOWNSAMPLE";

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

            // Initialize LiDARs (no temporal nodes; standalone)
            if (sensorGameObjects != null)
            {
                foreach (var sensorGO in sensorGameObjects)
                {
                    if (sensorGO == null) continue;

                    if (sensorGO.TryGetComponent<LidarSensor>(out var lidarSensor))
                    {
                        // Manual stepped control
                        lidarSensor.AutomaticCaptureHz = 0;

                        // Build a per-sensor subgraph according to the chosen frame mode
                        InitializeSubgraphForSensor(lidarSensor);
                    }
                    else
                    {
                        Debug.LogWarning($"'{sensorGO.name}' has no LidarSensor component and will be ignored.");
                    }
                }
            }

            if (autoPopulateCameras)
            {
                cameras = new List<Camera>(FindObjectsOfType<Camera>(includeInactive: false));
            }
            if (cameras == null) cameras = new List<Camera>();
        }

        /// <summary>
        /// Build and connect a per-sensor subgraph:
        /// - Global (ROS): Transform (Unity->ROS + origin) -> [Downsample] -> connect to WORLD frame
        /// - Local: [Downsample] -> connect to LIDAR frame
        /// </summary>
        private void InitializeSubgraphForSensor(LidarSensor lidarSensor)
        {
            var subgraph = new RGLNodeSequence();

            if (useGlobalCoordinatesForLidar)
            {
                // Unity->ROS transform + origin offset
                var worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
                worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginROS);

                subgraph
                    .AddNodePointsTransform(TransformNodeId, worldTransform)
                    .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));

                subgraph.SetActive(DownsampleNodeId, enableDownsampling);

                // WORLD frame (Unity world points come from LidarSensor world branch)
                lidarSensor.ConnectToWorldFrame(subgraph);
            }
            else
            {
                // LOCAL frame: no transform node at all
                subgraph
                    .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));

                subgraph.SetActive(DownsampleNodeId, enableDownsampling);

                // Connect to LIDAR-local points (provided by LidarSensor without edits to that class)
                lidarSensor.ConnectToLidarFrame(subgraph);
            }

            sensorSubgraphs[lidarSensor.gameObject] = subgraph;
            Debug.Log($"Initialized subgraph for {lidarSensor.gameObject.name} ({(useGlobalCoordinatesForLidar ? "GLOBAL/ROS" : "LOCAL")})");
        }

        /// <summary>
        /// Trigger LiDAR capture on all sensors with fresh subgraphs (no accumulation).
        /// </summary>
        private void TriggerLidarScans()
        {
            if (!enabled) return;
            if (sensorGameObjects == null || sensorGameObjects.Count == 0) return;

            Debug.Log($"Triggering LiDAR capture on {sensorSubgraphs.Count} sensor(s).");
            foreach (var sensorGO in sensorGameObjects)
            {
                if (sensorGO == null) continue;

                if (sensorGO.TryGetComponent<LidarSensor>(out var lidarSensor))
                {
                    // Rebuild subgraph to ensure standalone output for this step
                    if (sensorSubgraphs.TryGetValue(sensorGO, out var old))
                    {
                        old.Clear();
                    }
                    InitializeSubgraphForSensor(lidarSensor);

                    // Per-step capture
                    lidarSensor.Capture();
                }
            }
        }

        /// <summary>
        /// Save one .pcd per LiDAR through our subgraphs.
        /// Files: <LidarName>#<step>.pcd
        /// - GLOBAL mode => ROS world coordinates
        /// - LOCAL mode  => LiDAR-local coordinates
        /// </summary>
        private void SaveLidarScans()
        {
            if (sensorSubgraphs == null || sensorSubgraphs.Count == 0) return;

            string lidarDir = Path.Combine(Application.dataPath, outputDirectoryRoot, "Lidar");
            Directory.CreateDirectory(lidarDir);

            foreach (var kv in sensorSubgraphs)
            {
                var sensorGO = kv.Key;
                var subgraph = kv.Value;

                string sensorName = Sanitize(sensorGO.name);
                string file = Path.Combine(lidarDir, $"{sensorName}#{stepIndex:D4}.pcd");

                subgraph.SavePcdFile(file);
                Debug.Log($"Saved PCD [{(useGlobalCoordinatesForLidar ? "GLOBAL/ROS" : "LOCAL")}] -> {file}");
            }
        }

        /// <summary>
        /// Save an image for each camera.
        /// Files: <CameraName>#<step>.(png|jpg|exr)
        /// </summary>
        private void SaveCameraShots()
        {
            if (cameras == null || cameras.Count == 0) return;

            string imgDir = Path.Combine(Application.dataPath, outputDirectoryRoot, "Images");
            Directory.CreateDirectory(imgDir);

            foreach (var cam in cameras)
            {
                if (cam == null) continue;

                string camName = Sanitize(cam.name);
                string path = Path.Combine(imgDir, $"{camName}#{stepIndex:D4}.{GetImageExtension(imageFormat)}");
                CaptureCameraToFile(cam, imageWidth, imageHeight, imageFormat, path);
                Debug.Log($"Saved image -> {path}");
            }
        }

        /// <summary>
        /// One call to capture and write everything for this step (LiDAR + Cameras), then increment stepIndex.
        /// </summary>
        public void TriggerAndSaveStep()
        {
            TriggerLidarScans(); // fresh graphs => standalone scans
            SaveLidarScans();
            SaveCameraShots();
            stepIndex++;
        }

        /// <summary>
        /// Optional coroutine variant if your pipeline needs a frame boundary.
        /// </summary>
        public IEnumerator TriggerAndSaveStepCoroutine()
        {
            TriggerAndSaveStep();
            yield return null;
        }

        public void OnValidate()
        {
            // Keep live subgraphs synced with inspector changes
            if (sensorSubgraphs != null)
            {
                foreach (var sg in sensorSubgraphs.Values)
                {
                    if (sg.HasNode(DownsampleNodeId))
                    {
                        sg.UpdateNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
                        sg.SetActive(DownsampleNodeId, enableDownsampling);
                    }
                }
            }

            imageWidth  = Mathf.Max(1, imageWidth);
            imageHeight = Mathf.Max(1, imageHeight);
        }

        void Update()
        {
            // Example hotkey
            if (Input.GetKeyDown(KeyCode.Space))
            {
                TriggerAndSaveStep();
            }
        }

        // --------------------------
        // Helpers
        // --------------------------

        private static string Sanitize(string name)
        {
            foreach (char c in Path.GetInvalidFileNameChars())
                name = name.Replace(c.ToString(), "_");
            return name;
        }

        private static string GetImageExtension(ImageFormat fmt)
        {
            switch (fmt)
            {
                case ImageFormat.JPG: return "jpg";
                case ImageFormat.EXR: return "exr";
                default: return "png";
            }
        }

        private void CaptureCameraToFile(Camera cam, int width, int height, ImageFormat fmt, string filePath)
        {
            var rt = new RenderTexture(width, height, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default);
            var prevTarget = cam.targetTexture;
            var prevActive = RenderTexture.active;

            try
            {
                cam.targetTexture = rt;
                RenderTexture.active = rt;
                cam.Render();

                var tex = new Texture2D(width, height,
                    fmt == ImageFormat.EXR ? TextureFormat.RGBAFloat : TextureFormat.RGB24,
                    false, fmt == ImageFormat.EXR);

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
