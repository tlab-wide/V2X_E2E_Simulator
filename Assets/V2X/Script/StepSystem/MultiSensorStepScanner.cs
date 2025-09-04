using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO; // Directory and path ops
using RGLUnityPlugin; // LidarSensor + RGLNodeSequence
using AWSIM.PointCloudMapping; // Keep if your project already references it (for ROS2.Transformations)

// NOTE: Requires ROS2.Transformations.Unity2RosMatrix4x4() when global mode is used.

namespace AWSIM.Scanning
{
    /// <summary>
    /// Central controller to manage, trigger, and save scans from multiple sensors (LiDAR + Cameras).
    /// - Standalone LiDAR scans each step (no temporal accumulation).
    /// - Toggle GLOBAL (ROS world + origin) or LOCAL (LiDAR origin) outputs.
    /// - LOCAL mode rotates Unity local points into ROS axes (x fwd, y left, z up) with no translation.
    /// - Camera screenshots per step with post-processing/TAA/dynamic-resolution safely disabled during capture.
    /// - Filenames: sensorName#step.ext
    /// </summary>
    public class MultiSensorStepScanner : MonoBehaviour
    {
        [Header("LiDAR Sensor Configuration")]
        [SerializeField]
        [Tooltip("All LiDAR GameObjects to be controlled. Each must have a LidarSensor component.")]
        private List<GameObject> sensorGameObjects = new List<GameObject>();

        [Header("Coordinate Frame")]
        [SerializeField]
        [Tooltip("If true, LiDAR outputs are transformed to ROS world using worldOriginROS.\nIf false, outputs are at LiDAR origin but rotated into ROS axes (no translation).")]
        private bool useGlobalCoordinatesForLidar = false;

        [SerializeField]
        [Tooltip("World origin in ROS coordinate system; used only when 'useGlobalCoordinatesForLidar' is true.")]
        private Vector3 worldOriginROS;

        [Header("LiDAR Downsampling (applies to saved clouds)")]
        [SerializeField]
        private bool enableDownsampling = true;

        [SerializeField, Min(0.000001f)]
        private float leafSize = 0.1f;

        [Header("Cameras")]
        [SerializeField]
        private List<Camera> cameras = new List<Camera>();

        [SerializeField]
        private bool autoPopulateCameras = false;

        [SerializeField, Min(1)] private int imageWidth = 1920;
        [SerializeField, Min(1)] private int imageHeight = 1080;

        public enum ImageFormat { PNG, JPG, EXR }
        [SerializeField] private ImageFormat imageFormat = ImageFormat.PNG;

        [Header("Capture Quality (Images)")]
        [SerializeField, Tooltip("Temporarily disable all camera post-processing (DOF, Motion Blur, etc.) while capturing.")]
        private bool disablePostProcessingForCapture = true;

        [SerializeField, Tooltip("Try to disable Temporal AA on URP/HDRP cameras while capturing.")]
        private bool disableTemporalAAForCapture = true;

        [SerializeField, Tooltip("Temporarily disable Dynamic Resolution while capturing.")]
        private bool disableDynamicResolutionForCapture = true;

        [SerializeField, Tooltip("MSAA samples for the offscreen RenderTexture (1,2,4,8).")]
        private int captureMsaaSamples = 1;

        [Header("Output")]
        [SerializeField]
        [Tooltip("Root folder (inside Assets) where outputs are saved. Subfolders 'Lidar' and 'Images' are created.")]
        private string outputDirectoryRoot = "SensorSteps";

        [SerializeField, Tooltip("Zero-based step index; auto-increments after each step.")]
        private int stepIndex = 0;

        // Per-sensor subgraphs we own (one fresh subgraph per step for standalone output)
        private Dictionary<GameObject, RGLNodeSequence> sensorSubgraphs;

        // Node IDs inside our subgraphs
        private const string TransformNodeId = "ROS_WORLD_TF";          // global rotation+translation
        private const string LocalAxesToRosNodeId = "LOCAL_TO_ROS_AXES"; // local rotation only (no translation)
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

            // Clamp MSAA to sane values (1,2,4,8)
            if (captureMsaaSamples != 1 && captureMsaaSamples != 2 && captureMsaaSamples != 4 && captureMsaaSamples != 8)
                captureMsaaSamples = 1;
        }

        /// <summary>
        /// Build and connect a per-sensor subgraph:
        /// - GLOBAL (ROS): Transform (Unity->ROS + origin) -> [Downsample] -> connect to WORLD frame
        /// - LOCAL: (Unity->ROS rotation ONLY, zero translation) -> [Downsample] -> connect to LIDAR frame
        /// </summary>
        private void InitializeSubgraphForSensor(LidarSensor lidarSensor)
        {
            var subgraph = new RGLNodeSequence();

            if (useGlobalCoordinatesForLidar)
            {
                // Unity->ROS transform + origin offset (full transform)
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
                // LOCAL mode: rotate Unity local points into ROS axes, but **no translation** (stay at LiDAR origin)
                var localToRosAxes = UnityLocalToRosAxesMatrix();

                subgraph
                    .AddNodePointsTransform(LocalAxesToRosNodeId, localToRosAxes)
                    .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));

                subgraph.SetActive(DownsampleNodeId, enableDownsampling);

                // Connect to LIDAR-local points (provided by LidarSensor)
                lidarSensor.ConnectToLidarFrame(subgraph);
            }

            sensorSubgraphs[lidarSensor.gameObject] = subgraph;
            Debug.Log($"Initialized subgraph for {lidarSensor.gameObject.name} ({(useGlobalCoordinatesForLidar ? "GLOBAL/ROS world" : "LOCAL @ LiDAR origin (ROS axes)" )})");
        }

        /// <summary>
        /// Rotation-only matrix that maps Unity axes (left-handed: x right, y up, z fwd)
        /// to ROS axes (right-handed: x fwd, y left, z up), with zero translation.
        /// Mapping: x_ros =  z_unity,  y_ros = -x_unity,  z_ros = y_unity.
        /// </summary>
        private static Matrix4x4 UnityLocalToRosAxesMatrix()
        {
            var m = Matrix4x4.identity;
            m.m00 = 0;  m.m01 = 0;  m.m02 = 1;  m.m03 = 0;  // x_ros
            m.m10 = -1; m.m11 = 0;  m.m12 = 0;  m.m13 = 0;  // y_ros
            m.m20 = 0;  m.m21 = 1;  m.m22 = 0;  m.m23 = 0;  // z_ros
            m.m30 = 0;  m.m31 = 0;  m.m32 = 0;  m.m33 = 1;  // homogeneous
            return m;
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
        /// - GLOBAL mode => ROS world coordinates (origin applied)
        /// - LOCAL mode  => LiDAR origin, but in ROS axes (no translation)
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
                Debug.Log($"Saved PCD [{(useGlobalCoordinatesForLidar ? "GLOBAL/ROS world" : "LOCAL (ROS axes)" )}] -> {file}");
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
                CaptureCameraToFile(cam, imageWidth, imageHeight, imageFormat, captureMsaaSamples, path);
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
            if (captureMsaaSamples != 1 && captureMsaaSamples != 2 && captureMsaaSamples != 4 && captureMsaaSamples != 8)
                captureMsaaSamples = 1;
        }

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

        /// <summary>
        /// Captures a crisp image by temporarily disabling PostFX, TAA, and Dynamic Resolution
        /// via reflection (works across Built-in, URP, HDRP without hard dependencies).
        /// </summary>
        private void CaptureCameraToFile(Camera cam, int width, int height, ImageFormat fmt, int msaaSamples, string filePath)
        {
            // Create the offscreen RT
            var rt = new RenderTexture(width, height, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default)
            {
                antiAliasing = Mathf.Max(1, msaaSamples),
                useMipMap = false,
                autoGenerateMips = false,
                anisoLevel = 0
            };

            // Save current state
            var prevTarget = cam.targetTexture;
            var prevActive = RenderTexture.active;
            var prevAllowDynRes = cam.allowDynamicResolution;

            // --- Try to disable post-processing / TAA across pipelines (non-throwing reflection) ---
            // Built-in (PostProcessing Stack v2)
            var ppLayerType = Type.GetType("UnityEngine.Rendering.PostProcessing.PostProcessLayer, Unity.Postprocessing.Runtime");
            Component ppLayer = ppLayerType != null ? cam.GetComponent(ppLayerType) : null;
            bool? prevPpEnabled = null;

            // URP
            var urpType = Type.GetType("UnityEngine.Rendering.Universal.UniversalAdditionalCameraData, Unity.RenderPipelines.Universal.Runtime");
            Component urpData = urpType != null ? cam.GetComponent(urpType) : null;
            object prevUrpPost = null;
            object prevUrpAA = null;

            // HDRP
            var hdrpType = Type.GetType("UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData, Unity.RenderPipelines.HighDefinition.Runtime");
            Component hdrpData = hdrpType != null ? cam.GetComponent(hdrpType) : null;
            object prevHdrpAA = null;

            try
            {
                // Set target and render
                cam.targetTexture = rt;
                RenderTexture.active = rt;

                if (disableDynamicResolutionForCapture)
                    cam.allowDynamicResolution = false;

                // Built-in PPSv2 off
                if (disablePostProcessingForCapture && ppLayer != null)
                {
                    var enabledProp = ppLayerType.GetProperty("enabled");
                    prevPpEnabled = (bool)enabledProp.GetValue(ppLayer, null);
                    enabledProp.SetValue(ppLayer, false, null);
                }

                // URP toggles
                if (urpData != null)
                {
                    if (disablePostProcessingForCapture)
                    {
                        var postProp = urpType.GetProperty("renderPostProcessing");
                        if (postProp != null)
                        {
                            prevUrpPost = postProp.GetValue(urpData, null);
                            postProp.SetValue(urpData, false, null);
                        }
                    }
                    if (disableTemporalAAForCapture)
                    {
                        // Set antialiasing to None (if available)
                        var aaProp = urpType.GetProperty("antialiasing");
                        if (aaProp != null)
                        {
                            prevUrpAA = aaProp.GetValue(urpData, null);
                            var enumType = aaProp.PropertyType;
                            var noneVal = Enum.Parse(enumType, "None", ignoreCase: true);
                            aaProp.SetValue(urpData, noneVal, null);
                        }
                    }
                }

                // HDRP toggles
                if (hdrpData != null && disableTemporalAAForCapture)
                {
                    // Set antialiasing to None
                    var aaProp = hdrpType.GetProperty("antialiasing");
                    if (aaProp != null)
                    {
                        prevHdrpAA = aaProp.GetValue(hdrpData, null);
                        var enumType = aaProp.PropertyType;
                        var noneVal = Enum.Parse(enumType, "None", ignoreCase: true);
                        aaProp.SetValue(hdrpData, noneVal, null);
                    }
                    // Note: HDRP post-processing is volume-driven; turning off TAA removes most blur.
                    // If you still see DOF, disable/adjust your Volume or use a capture-only camera layer.
                }

                // Render
                cam.Render();

                // Read back
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
                // Restore per-camera settings
                if (ppLayer != null && prevPpEnabled.HasValue)
                {
                    var enabledProp = ppLayerType.GetProperty("enabled");
                    enabledProp.SetValue(ppLayer, prevPpEnabled.Value, null);
                }

                if (urpData != null)
                {
                    if (prevUrpPost != null)
                    {
                        var postProp = urpType.GetProperty("renderPostProcessing");
                        postProp?.SetValue(urpData, prevUrpPost, null);
                    }
                    if (prevUrpAA != null)
                    {
                        var aaProp = urpType.GetProperty("antialiasing");
                        aaProp?.SetValue(urpData, prevUrpAA, null);
                    }
                }

                if (hdrpData != null && prevHdrpAA != null)
                {
                    var aaProp = hdrpType.GetProperty("antialiasing");
                    aaProp?.SetValue(hdrpData, prevHdrpAA, null);
                }

                cam.allowDynamicResolution = prevAllowDynRes;

                cam.targetTexture = prevTarget;
                RenderTexture.active = prevActive;
                rt.Release();
                Destroy(rt);
            }
        }
    }
}
