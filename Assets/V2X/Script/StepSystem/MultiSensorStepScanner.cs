using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using RGLUnityPlugin;
using AWSIM.PointCloudMapping;

namespace AWSIM.Scanning
{
    /// <summary>
    /// Multi-sensor step scanner (LiDAR + Cameras), optimized for HDRP/URP/Built-in.
    /// - Reuses camera buffers to minimize GC/stalls.
    /// - Caches reflection metadata for URP/HDRP/Built-in PP toggles.
    /// - Masks out HDRP Volumes during capture (or uses a dedicated capture mask).
    /// - Warm-up render(s) to clear temporal history.
    /// - Optional freeze of the scene so moving objects stay sharp.
    /// - Standalone LiDAR scans (no temporal accumulation).
    /// </summary>
    public class MultiSensorStepScanner : MonoBehaviour
    {
        // --------------------------
        // LiDAR Configuration
        // --------------------------
        [Header("LiDAR Sensor Configuration")]
        [SerializeField, Tooltip("All LiDAR GameObjects to be controlled. Each must have a LidarSensor component.")]
        private List<GameObject> sensorGameObjects = new List<GameObject>();

        [Header("Coordinate Frame")]
        [SerializeField, Tooltip("If true, LiDAR outputs are transformed to ROS world using worldOriginROS.")]
        private bool useGlobalCoordinatesForLidar = false;

        [SerializeField, Tooltip("World origin in ROS coordinate system; used only when global mode is true.")]
        private Vector3 worldOriginROS;

        [Header("LiDAR Downsampling (applies to saved clouds)")]
        [SerializeField] private bool enableDownsampling = true;
        [SerializeField, Min(0.000001f)] private float leafSize = 0.1f;

        // --------------------------
        // Camera Configuration
        // --------------------------
        [Header("Cameras")]
        [SerializeField] private List<Camera> cameras = new List<Camera>();
        [SerializeField] private bool autoPopulateCameras = false;

        [SerializeField, Min(1)] private int imageWidth = 1920;
        [SerializeField, Min(1)] private int imageHeight = 1080;

        public enum ImageFormat { PNG, JPG, EXR }
        [SerializeField] private ImageFormat imageFormat = ImageFormat.PNG;

        [Header("Capture Quality (Images)")]
        [SerializeField, Tooltip("Temporarily disable all camera post-processing (DoF, Motion Blur, etc.) while capturing.")]
        private bool disablePostProcessingForCapture = true;

        [SerializeField, Tooltip("Try to disable Temporal AA on URP/HDRP cameras while capturing.")]
        private bool disableTemporalAAForCapture = true;

        [SerializeField, Tooltip("Temporarily disable Dynamic Resolution while capturing.")]
        private bool disableDynamicResolutionForCapture = true;

        [SerializeField, Tooltip("MSAA samples for the offscreen RenderTexture (1,2,4,8). Ignored by HDRP deferred + post.")]
        private int captureMsaaSamples = 1;

        [Header("HDRP Capture Controls")]
        [SerializeField, Tooltip("Warm-up renders after toggles to flush temporal history (TAA/clouds). 1 is usually enough.")]
        private int warmupRenders = 1;

        [SerializeField, Tooltip("If non-zero, use this mask for capture-only HDRP volumes (neutral). If zero, disable all volumes.")]
        private LayerMask hdrpCaptureVolumeMaskOverride = 0;

        [Header("Freeze During Capture (to keep moving cars sharp)")]
        [SerializeField, Tooltip("Freeze time during capture so moving objects don't advance while histories flush.")]
        private bool freezeSceneDuringCapture = true;

        [SerializeField, Tooltip("Also freeze animators that run on UnscaledTime (rare). Slight perf cost when true.")]
        private bool freezeUnscaledAnimators = false;

        [Header("Performance")]
        [SerializeField, Tooltip("Reuse GPU/CPU buffers per camera to avoid allocations.")]
        private bool reuseBuffers = true;

        [SerializeField, Tooltip("Use GetTemporary for RTs (faster). If false, persistent RTs are created/destroyed.")]
        private bool useTemporaryRT = true;

        [SerializeField, Range(1, 100), Tooltip("JPEG quality if JPG is selected.")]
        private int jpgQuality = 95;

        // --------------------------
        // Output
        // --------------------------
        [Header("Output")]
        [SerializeField, Tooltip("Root folder (inside Assets) where outputs are saved. Subfolders 'Lidar' and 'Images' are created.")]
        private string outputDirectoryRoot = "SensorSteps";

        [SerializeField, Tooltip("Zero-based step index; auto-increments after each step.")]
        private int stepIndex = 0;

        // --------------------------
        // Internals
        // --------------------------
        private Dictionary<GameObject, RGLNodeSequence> sensorSubgraphs;
        private const string TransformNodeId = "ROS_WORLD_TF";
        private const string LocalAxesToRosNodeId = "LOCAL_TO_ROS_AXES";
        private const string DownsampleNodeId = "DOWNSAMPLE";

        private string _lidarDirPath;
        private string _imageDirPath;

        private readonly Dictionary<UnityEngine.Object, string> _sanitizedNameCache = new();
        private readonly Dictionary<Camera, CaptureBuffers> _buffersByCamera = new();

        // Animator freeze cache (only used if freezeUnscaledAnimators = true)
        private readonly List<(Animator anim, float speed)> _frozenAnimators = new();

        // -------- Reflection Cache (URP/HDRP/Built-in) --------
        private static class R
        {
            public static readonly Type PPLayerType =
                Type.GetType("UnityEngine.Rendering.PostProcessing.PostProcessLayer, Unity.Postprocessing.Runtime");

            public static readonly PropertyInfo PPLayer_enabled =
                PPLayerType?.GetProperty("enabled");

            public static readonly Type URPType =
                Type.GetType("UnityEngine.Rendering.Universal.UniversalAdditionalCameraData, Unity.RenderPipelines.Universal.Runtime");

            public static readonly PropertyInfo URP_renderPostProcessing =
                URPType?.GetProperty("renderPostProcessing");

            public static readonly PropertyInfo URP_antialiasing =
                URPType?.GetProperty("antialiasing");

            public static readonly Type HDRPType =
                Type.GetType("UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData, Unity.RenderPipelines.HighDefinition.Runtime");

            public static readonly PropertyInfo HDRP_antialiasing =
                HDRPType?.GetProperty("antialiasing");

            public static readonly PropertyInfo HDRP_volumeLayerMask =
                HDRPType?.GetProperty("volumeLayerMask");
        }

        private void Start()
        {
            if ((sensorGameObjects == null || sensorGameObjects.Count == 0) &&
                (cameras == null || cameras.Count == 0))
            {
                Debug.LogError("MultiSensorStepScanner: No sensors (LiDAR or Camera) assigned. Disabling component.");
                enabled = false;
                return;
            }

            if (autoPopulateCameras)
                cameras = new List<Camera>(FindObjectsOfType<Camera>(includeInactive: false));
            if (cameras == null) cameras = new List<Camera>();

            string root = Path.Combine(Application.dataPath, outputDirectoryRoot);
            _lidarDirPath = Path.Combine(root, "Lidar");
            _imageDirPath = Path.Combine(root, "Images");
            Directory.CreateDirectory(_lidarDirPath);
            Directory.CreateDirectory(_imageDirPath);

            if (captureMsaaSamples != 1 && captureMsaaSamples != 2 && captureMsaaSamples != 4 && captureMsaaSamples != 8)
                captureMsaaSamples = 1;

            sensorSubgraphs = new Dictionary<GameObject, RGLNodeSequence>();
            if (sensorGameObjects != null)
            {
                foreach (var sensorGO in sensorGameObjects)
                {
                    if (sensorGO && sensorGO.TryGetComponent<LidarSensor>(out var lidar))
                    {
                        lidar.AutomaticCaptureHz = 0;
                        InitializeOrRebuildSubgraph(lidar);
                    }
                }
            }
        }

        private void OnDestroy()
        {
            foreach (var kv in _buffersByCamera)
                kv.Value?.Release(kv.Value.IsTempRT);
            _buffersByCamera.Clear();

            if (sensorSubgraphs != null)
            {
                foreach (var sg in sensorSubgraphs.Values)
                    sg?.Clear();
                sensorSubgraphs.Clear();
            }
        }

        public void OnValidate()
        {
            imageWidth = Mathf.Max(1, imageWidth);
            imageHeight = Mathf.Max(1, imageHeight);
            if (captureMsaaSamples != 1 && captureMsaaSamples != 2 && captureMsaaSamples != 4 && captureMsaaSamples != 8)
                captureMsaaSamples = 1;

            if (sensorSubgraphs != null)
            {
                foreach (var sg in sensorSubgraphs.Values)
                {
                    if (sg != null && sg.HasNode(DownsampleNodeId))
                    {
                        sg.UpdateNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
                        sg.SetActive(DownsampleNodeId, enableDownsampling);
                    }
                }
            }
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
                TriggerAndSaveStep();
        }

        // --------------------------
        // Public API
        // --------------------------
        public void TriggerAndSaveStep()
        {
            TriggerLidarScans();
            SaveLidarScans();
            SaveCameraShots();
            stepIndex++;
        }

        public IEnumerator TriggerAndSaveStepCoroutine()
        {
            TriggerAndSaveStep();
            yield return null;
        }

        // --------------------------
        // LiDAR
        // --------------------------
        private void InitializeOrRebuildSubgraph(LidarSensor lidar)
        {
            if (!sensorSubgraphs.TryGetValue(lidar.gameObject, out var subgraph) || subgraph == null)
            {
                subgraph = new RGLNodeSequence();
                sensorSubgraphs[lidar.gameObject] = subgraph;
            }
            else
            {
                subgraph.Clear();
            }

            if (useGlobalCoordinatesForLidar)
            {
                var worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
                worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginROS);

                subgraph
                    .AddNodePointsTransform(TransformNodeId, worldTransform)
                    .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));

                subgraph.SetActive(DownsampleNodeId, enableDownsampling);
                lidar.ConnectToWorldFrame(subgraph);
            }
            else
            {
                var localToRosAxes = UnityLocalToRosAxesMatrix();
                subgraph
                    .AddNodePointsTransform(LocalAxesToRosNodeId, localToRosAxes) // <-- fixed: use constant, no missing method
                    .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));

                subgraph.SetActive(DownsampleNodeId, enableDownsampling);
                lidar.ConnectToLidarFrame(subgraph);
            }
        }

        private static Matrix4x4 UnityLocalToRosAxesMatrix()
        {
            var m = Matrix4x4.identity;
            m.m00 = 0;  m.m01 = 0;  m.m02 = 1;  m.m03 = 0;  // x_ros
            m.m10 = -1; m.m11 = 0;  m.m12 = 0;  m.m13 = 0;  // y_ros
            m.m20 = 0;  m.m21 = 1;  m.m22 = 0;  m.m23 = 0;  // z_ros
            m.m30 = 0;  m.m31 = 0;  m.m32 = 0;  m.m33 = 1;
            return m;
        }

        private void TriggerLidarScans()
        {
            if (!enabled || sensorGameObjects == null || sensorGameObjects.Count == 0) return;

            foreach (var sensorGO in sensorGameObjects)
            {
                if (!sensorGO) continue;
                if (sensorGO.TryGetComponent<LidarSensor>(out var lidar))
                {
                    InitializeOrRebuildSubgraph(lidar);
                    lidar.Capture();
                }
            }
        }

        private void SaveLidarScans()
        {
            if (sensorSubgraphs == null || sensorSubgraphs.Count == 0) return;

            foreach (var kv in sensorSubgraphs)
            {
                var sensorGO = kv.Key;
                var subgraph = kv.Value;
                if (!sensorGO || subgraph == null) continue;

                string sensorName = GetSanitized(sensorGO);
                string file = Path.Combine(_lidarDirPath, $"{sensorName}#{stepIndex:D4}.pcd");
                subgraph.SavePcdFile(file);
#if UNITY_EDITOR
                Debug.Log($"Saved PCD [{(useGlobalCoordinatesForLidar ? "GLOBAL/ROS world" : "LOCAL (ROS axes)")}]-> {file}");
#endif
            }
        }

        // --------------------------
        // Cameras
        // --------------------------
        private void SaveCameraShots()
        {
            if (cameras == null || cameras.Count == 0) return;

            foreach (var cam in cameras)
            {
                if (!cam) continue;

                string camName = GetSanitized(cam);
                string path = Path.Combine(_imageDirPath, $"{camName}#{stepIndex:D4}.{GetImageExtension(imageFormat)}");
                CaptureCameraToFile(cam, imageWidth, imageHeight, imageFormat, captureMsaaSamples, path);
#if UNITY_EDITOR
                Debug.Log($"Saved image -> {path}");
#endif
            }
        }

        private static string GetImageExtension(ImageFormat fmt) =>
            fmt == ImageFormat.JPG ? "jpg" : (fmt == ImageFormat.EXR ? "exr" : "png");

        private string GetSanitized(UnityEngine.Object obj)
        {
            if (obj == null) return "null";
            if (_sanitizedNameCache.TryGetValue(obj, out var cached)) return cached;

            string name = obj.name;
            foreach (char c in Path.GetInvalidFileNameChars())
                name = name.Replace(c.ToString(), "_");

            _sanitizedNameCache[obj] = name;
            return name;
        }

        // --------------------------
        // Capture Implementation
        // --------------------------
        private sealed class CaptureBuffers
        {
            public RenderTexture RT;
            public Texture2D Tex;
            public int W, H, Msaa;
            public bool IsEXR;
            public bool IsTempRT;

            public void Ensure(int w, int h, int msaa, bool exr, bool useTemp)
            {
                if (RT != null && Tex != null && W == w && H == h && Msaa == msaa && IsEXR == exr && IsTempRT == useTemp)
                    return;

                Release(IsTempRT);

                var desc = new RenderTextureDescriptor(w, h)
                {
                    depthBufferBits = 24,
                    msaaSamples = Mathf.Max(1, msaa),
                    sRGB = (QualitySettings.activeColorSpace == ColorSpace.Linear),
                    colorFormat = RenderTextureFormat.Default
                };

                RT = useTemp ? RenderTexture.GetTemporary(desc) : new RenderTexture(desc);
                RT.useMipMap = false;
                RT.autoGenerateMips = false;
                RT.anisoLevel = 0;

                Tex = new Texture2D(
                    w, h,
                    exr ? TextureFormat.RGBAHalf : TextureFormat.RGB24,
                    false, exr);

                W = w; H = h; Msaa = msaa; IsEXR = exr; IsTempRT = useTemp;
            }

            public void Release(bool wasTemp)
            {
                if (RT)
                {
                    if (wasTemp) RenderTexture.ReleaseTemporary(RT);
                    else { RT.Release(); UnityEngine.Object.Destroy(RT); }
                    RT = null;
                }
                if (Tex)
                {
                    UnityEngine.Object.Destroy(Tex);
                    Tex = null;
                }
            }
        }

        private struct CameraState
        {
            public RenderTexture prevTarget;
            public RenderTexture prevActive;
            public bool prevAllowDynRes;

            public Component ppLayer;
            public bool? prevPpEnabled;

            public Component urpData;
            public object prevUrpPost;
            public object prevUrpAA;

            public Component hdrpData;
            public object prevHdrpAA;
            public object prevHdrpVolumeMask;

            public float prevTimeScale;
        }

        private void CaptureCameraToFile(Camera cam, int width, int height, ImageFormat fmt, int msaaSamples, string filePath)
        {
            bool isEXR = (fmt == ImageFormat.EXR);
            var buffers = GetBuffers(cam, width, height, msaaSamples, isEXR);

            var state = new CameraState();
            try
            {
                // Optionally freeze scene so moving objects don't blur between frames
                if (freezeSceneDuringCapture)
                {
                    state.prevTimeScale = Time.timeScale;
                    Time.timeScale = 0f;

                    if (freezeUnscaledAnimators)
                    {
                        _frozenAnimators.Clear();
                        var anims = FindObjectsOfType<Animator>();
                        foreach (var a in anims)
                        {
                            if (a.updateMode == AnimatorUpdateMode.UnscaledTime)
                            {
                                _frozenAnimators.Add((a, a.speed));
                                a.speed = 0f;
                            }
                        }
                    }
                }

                // Bind RT
                state.prevTarget = cam.targetTexture;
                state.prevActive = RenderTexture.active;
                state.prevAllowDynRes = cam.allowDynamicResolution;

                cam.targetTexture = buffers.RT;
                RenderTexture.active = buffers.RT;

                if (disableDynamicResolutionForCapture)
                    cam.allowDynamicResolution = false;

                // Resolve pipeline components once
                state.ppLayer = (R.PPLayerType != null) ? cam.GetComponent(R.PPLayerType) : null;
                state.urpData = (R.URPType != null) ? cam.GetComponent(R.URPType) : null;
                state.hdrpData = (R.HDRPType != null) ? cam.GetComponent(R.HDRPType) : null;

                // Built-in PPv2: disable
                if (disablePostProcessingForCapture && state.ppLayer && R.PPLayer_enabled != null)
                {
                    state.prevPpEnabled = (bool)R.PPLayer_enabled.GetValue(state.ppLayer, null);
                    R.PPLayer_enabled.SetValue(state.ppLayer, false, null);
                }

                // URP: disable PP and AA
                if (state.urpData)
                {
                    if (disablePostProcessingForCapture && R.URP_renderPostProcessing != null)
                    {
                        state.prevUrpPost = R.URP_renderPostProcessing.GetValue(state.urpData, null);
                        R.URP_renderPostProcessing.SetValue(state.urpData, false, null);
                    }
                    if (disableTemporalAAForCapture && R.URP_antialiasing != null)
                    {
                        state.prevUrpAA = R.URP_antialiasing.GetValue(state.urpData, null);
                        var enumType = R.URP_antialiasing.PropertyType;
                        var noneVal = Enum.Parse(enumType, "None", ignoreCase: true);
                        R.URP_antialiasing.SetValue(state.urpData, noneVal, null);
                    }
                }

                // HDRP: disable AA and volumes (or apply capture mask)
                if (state.hdrpData)
                {
                    if (disableTemporalAAForCapture && R.HDRP_antialiasing != null)
                    {
                        state.prevHdrpAA = R.HDRP_antialiasing.GetValue(state.hdrpData, null);
                        var enumType = R.HDRP_antialiasing.PropertyType;
                        var noneVal = Enum.Parse(enumType, "None", ignoreCase: true);
                        R.HDRP_antialiasing.SetValue(state.hdrpData, noneVal, null);
                    }

                    if (disablePostProcessingForCapture && R.HDRP_volumeLayerMask != null)
                    {
                        state.prevHdrpVolumeMask = R.HDRP_volumeLayerMask.GetValue(state.hdrpData, null);
                        var maskToUse = hdrpCaptureVolumeMaskOverride; // 0 => disable all volumes
                        R.HDRP_volumeLayerMask.SetValue(state.hdrpData, maskToUse, null);
                    }
                }

                // Warm-up renders to flush histories (runs while frozen so nothing moves)
                int warms = Mathf.Max(0, warmupRenders);
                for (int i = 0; i < warms; i++) cam.Render();

                // Actual capture frame
                cam.Render();

                // Readback
                buffers.Tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
                buffers.Tex.Apply(false, false);

                // Encode
                byte[] bytes;
                switch (fmt)
                {
                    case ImageFormat.JPG:
                        bytes = buffers.Tex.EncodeToJPG(Mathf.Clamp(jpgQuality, 1, 100));
                        break;
                    case ImageFormat.EXR:
                        bytes = buffers.Tex.EncodeToEXR(Texture2D.EXRFlags.CompressZIP);
                        break;
                    default:
                        bytes = buffers.Tex.EncodeToPNG();
                        break;
                }

                File.WriteAllBytes(filePath, bytes);
            }
            finally
            {
                // Restore pipeline settings
                if (state.ppLayer && state.prevPpEnabled.HasValue && R.PPLayer_enabled != null)
                    R.PPLayer_enabled.SetValue(state.ppLayer, state.prevPpEnabled.Value, null);

                if (state.urpData)
                {
                    if (state.prevUrpPost != null && R.URP_renderPostProcessing != null)
                        R.URP_renderPostProcessing.SetValue(state.urpData, state.prevUrpPost, null);

                    if (state.prevUrpAA != null && R.URP_antialiasing != null)
                        R.URP_antialiasing.SetValue(state.urpData, state.prevUrpAA, null);
                }

                if (state.hdrpData)
                {
                    if (state.prevHdrpAA != null && R.HDRP_antialiasing != null)
                        R.HDRP_antialiasing.SetValue(state.hdrpData, state.prevHdrpAA, null);

                    if (state.prevHdrpVolumeMask != null && R.HDRP_volumeLayerMask != null)
                        R.HDRP_volumeLayerMask.SetValue(state.hdrpData, state.prevHdrpVolumeMask, null);
                }

                // Restore camera targets
                cam.allowDynamicResolution = state.prevAllowDynRes;
                cam.targetTexture = state.prevTarget;
                RenderTexture.active = state.prevActive;

                // Unfreeze scene if we froze it
                if (freezeSceneDuringCapture)
                {
                    if (freezeUnscaledAnimators)
                    {
                        foreach (var (anim, speed) in _frozenAnimators)
                        {
                            if (anim) anim.speed = speed;
                        }
                        _frozenAnimators.Clear();
                    }

                    Time.timeScale = state.prevTimeScale;
                }

                // Dispose buffers if not reusing
                if (!reuseBuffers && _buffersByCamera.TryGetValue(cam, out var bufs))
                {
                    bufs.Release(bufs.IsTempRT);
                    _buffersByCamera.Remove(cam);
                }
            }
        }

        private CaptureBuffers GetBuffers(Camera cam, int w, int h, int msaa, bool exr)
        {
            if (!reuseBuffers)
            {
                var temp = new CaptureBuffers();
                temp.Ensure(w, h, msaa, exr, useTemporaryRT);
                return temp;
            }

            if (!_buffersByCamera.TryGetValue(cam, out var buffers) || buffers == null)
            {
                buffers = new CaptureBuffers();
                _buffersByCamera[cam] = buffers;
            }

            buffers.Ensure(w, h, msaa, exr, useTemporaryRT);
            return buffers;
        }
    }
}
