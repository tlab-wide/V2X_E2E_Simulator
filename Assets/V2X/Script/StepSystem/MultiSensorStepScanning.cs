using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using RGLUnityPlugin;
using AWSIM.PointCloudMapping;
using System.Text;
using unique_identifier_msgs.msg; // for UUID from LineOfSight.GetUUID()


namespace AWSIM.Scanning
{
    /// <summary>
    /// Step scanner for LiDAR + Cameras.
    /// - LiDAR: builds per-sensor RGL subgraphs (local ROS-axes or global ROS world), optional voxel downsampling, saves .pcd.
    /// - Camera: takes one picture per configured camera per step, uses an existing "camera feature" component if present, else falls back to a built-in capture.
    /// - During each step: sets Time.timeScale=0 before capture and restores it after to avoid blur.
    /// - Output root: Assets/{outputDirectoryRoot}/Lidar and Assets/{outputDirectoryRoot}/Camera
    /// - Can optionally group outputs into per-sensor subfolders under Lidar/Camera.
    /// - Trigger via Space key or public API.
    /// </summary>
    public class MultiSensorStepScanning : MonoBehaviour
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

        [Header("LiDAR Downsampling (applies to saved clouds)")] [SerializeField]
        private bool enableDownsampling = true;

        [SerializeField, Min(0.000001f)] private float leafSize = 0.1f;

        // --------------------------
        // Camera Configuration
        // --------------------------
        [Header("Camera Scanning")]
        [SerializeField,
         Tooltip(
             "All camera GameObjects to capture from each step. Each must have a Camera component, and optionally a custom camera feature.")]
        private List<GameObject> cameraGameObjects = new List<GameObject>();

        [SerializeField,
         Tooltip("Override output resolution for fallback capture. If false, uses camera.pixelWidth/Height.")]
        private bool overrideCameraResolution = false;

        [SerializeField, Min(1)] private int fallbackWidth = 1920;
        [SerializeField, Min(1)] private int fallbackHeight = 1080;

        [SerializeField, Range(1, 8), Tooltip("Supersampling multiplier for fallback capture (1 = none).")]
        private int fallbackSupersampling = 1;

        private enum ImageFormat
        {
            PNG,
            JPG
        }

        [SerializeField] private ImageFormat imageFormat = ImageFormat.PNG;
        [SerializeField, Range(1, 100)] private int jpgQuality = 95;

        [SerializeField,
         Tooltip(
             "If your project already has a camera feature component, we will try to use it first by interface/method name. Leave empty to auto-detect.")]
        private string preferredCameraFeatureTypeName = ""; // optional hint

        // --------------------------
        // Ground Truth Logging
        // --------------------------
        [Header("Ground Truth Logging")]
        [SerializeField, Tooltip("List of GroundTruthArea volumes whose seen objects will be UNIONed each step.")]
        private List<GroundTruthArea> groundTruthAreas = new List<GroundTruthArea>();

        [SerializeField,
         Tooltip("Optional reference frame; if set, positions are made relative to this before Unity->ROS conversion.")]
        private Transform rosReference;

        [SerializeField, Tooltip("Subfolder (under outputDirectoryRoot) where per-step CSVs will be written.")]
        private string groundTruthFolderName = "GroundTruth";

        [SerializeField, Tooltip("Write a CSV header row for each step file.")]
        private bool writeCsvHeader = true;

        // internal path for this logger
        private string _groundTruthDirPath;


        // --------------------------
        // Output
        // --------------------------
        [Header("Output")]
        [SerializeField,
         Tooltip("Root folder (inside Assets) where outputs are saved. Subfolders 'Lidar' and 'Camera' are created.")]
        private string outputDirectoryRoot = "SensorSteps";

        [SerializeField, Tooltip("Zero-based step index; auto-increments after each step.")]
        private int stepIndex = 0;

        [SerializeField, Tooltip("If true, create a subfolder per sensor under 'Lidar'/'Camera'.")]
        private bool groupBySensorInSubfolders = true;

        // --------------------------
        // Internals
        // --------------------------
        private Dictionary<GameObject, RGLNodeSequence> sensorSubgraphs;
        private const string TransformNodeId = "ROS_WORLD_TF";
        private const string LocalAxesToRosNodeId = "LOCAL_TO_ROS_AXES";
        private const string DownsampleNodeId = "DOWNSAMPLE";

        private string _lidarDirPath;
        private string _cameraDirPath;

        private readonly Dictionary<UnityEngine.Object, string> _sanitizedNameCache = new();

        // --------------------------
        // Integration Interface (optional)
        // --------------------------
        /// <summary>
        /// If your own camera component implements this, we'll call CaptureAndSave(path).
        /// Otherwise we reflect common method names or fall back to built-in capture.
        /// </summary>
        public interface IStepPhotoSource
        {
            void CaptureAndSave(string absolutePath);
        }

        private void Start()
        {
            if ((sensorGameObjects == null || sensorGameObjects.Count == 0) &&
                (cameraGameObjects == null || cameraGameObjects.Count == 0))
            {
                Debug.LogError("MultiSensorStepScanning: No LiDAR or Camera sensors assigned. Disabling component.");
                enabled = false;
                return;
            }

            string root = Path.Combine(Application.dataPath, outputDirectoryRoot);
            _lidarDirPath = Path.Combine(root, "Lidar");
            _cameraDirPath = Path.Combine(root, "Camera");
            _groundTruthDirPath =  Path.Combine(root, "GroundTruthPos");
            Directory.CreateDirectory(_lidarDirPath);
            Directory.CreateDirectory(_cameraDirPath);
            Directory.CreateDirectory(_groundTruthDirPath);

            sensorSubgraphs = new Dictionary<GameObject, RGLNodeSequence>();
            foreach (var sensorGO in sensorGameObjects)
            {
                if (sensorGO && sensorGO.TryGetComponent<LidarSensor>(out var lidar))
                {
                    lidar.AutomaticCaptureHz = 0;
                    InitializeOrRebuildSubgraph(lidar);
                }
            }


            
        }

        private void OnDestroy()
        {
            if (sensorSubgraphs != null)
            {
                foreach (var sg in sensorSubgraphs.Values)
                    sg?.Clear();
                sensorSubgraphs.Clear();
            }
        }

        public void OnValidate()
        {
            if (fallbackWidth < 1) fallbackWidth = 1;
            if (fallbackHeight < 1) fallbackHeight = 1;
            if (fallbackSupersampling < 1) fallbackSupersampling = 1;

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
            float prevTimeScale = Time.timeScale;
            try
            {
                // Freeze time to avoid blur across both LiDAR and Cameras.
                Time.timeScale = 0f;

                // (Optional) Ensure transforms are settled at this frame boundary.
                // If you prefer, you can use the coroutine below to wait for end-of-frame.
                TriggerLidarScans();
                CaptureCameraShots();

                SaveLidarScans();
                // Camera shots are saved as they're captured.

                SaveGroundTruthCsv();


                stepIndex++;
            }
            catch (Exception ex)
            {
                Debug.LogError($"Step scan failed: {ex}");
            }
            finally
            {
                // Restore time scale
                Time.timeScale = prevTimeScale == 0f ? 1f : prevTimeScale;
            }
        }

        public IEnumerator TriggerAndSaveStepCoroutine()
        {
            float prevTimeScale = Time.timeScale;
            Time.timeScale = 0f;
            yield return new WaitForEndOfFrame(); // ensures all rendering has a stable state

            TriggerLidarScans();
            yield return null;

            // Let the render thread breathe before camera grabs (esp. if many cameras)
            yield return new WaitForEndOfFrame();
            CaptureCameraShots();

            SaveLidarScans();

            stepIndex++;
            Time.timeScale = prevTimeScale == 0f ? 1f : prevTimeScale;
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
                    .AddNodePointsTransform(LocalAxesToRosNodeId, localToRosAxes)
                    .AddNodePointsDownsample(DownsampleNodeId, new Vector3(leafSize, leafSize, leafSize));

                subgraph.SetActive(DownsampleNodeId, enableDownsampling);
                lidar.ConnectToLidarFrame(subgraph);
            }
        }

        private static Matrix4x4 UnityLocalToRosAxesMatrix()
        {
            var m = Matrix4x4.identity;
            m.m00 = 0;
            m.m01 = 0;
            m.m02 = 1;
            m.m03 = 0; // x_ros
            m.m10 = -1;
            m.m11 = 0;
            m.m12 = 0;
            m.m13 = 0; // y_ros
            m.m20 = 0;
            m.m21 = 1;
            m.m22 = 0;
            m.m23 = 0; // z_ros
            m.m30 = 0;
            m.m31 = 0;
            m.m32 = 0;
            m.m33 = 1;
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

                // Per-sensor folder (optional via toggle)
                string lidarBase = _lidarDirPath;
                if (groupBySensorInSubfolders)
                    lidarBase = GetOrCreatePerSensorFolder(_lidarDirPath, sensorName);

                string file = Path.Combine(lidarBase, $"{sensorName}#{stepIndex:D4}.pcd");
                subgraph.SavePcdFile(file);
#if UNITY_EDITOR
                Debug.Log(
                    $"Saved PCD [{(useGlobalCoordinatesForLidar ? "GLOBAL/ROS world" : "LOCAL (ROS axes)")}]-> {file}");
#endif
            }
        }

        // --------------------------
        // Camera
        // --------------------------
        private void CaptureCameraShots()
        {
            if (cameraGameObjects == null || cameraGameObjects.Count == 0) return;

            foreach (var camGO in cameraGameObjects)
            {
                if (!camGO) continue;
                if (!camGO.TryGetComponent<Camera>(out var cam))
                {
                    Debug.LogWarning($"Camera step-scan: '{camGO.name}' has no Camera component. Skipped.");
                    continue;
                }

                string camName = GetSanitized(camGO);
                string ext = imageFormat == ImageFormat.PNG ? "png" : "jpg";

                // Per-camera folder (optional via toggle)
                string cameraBase = _cameraDirPath;
                if (groupBySensorInSubfolders)
                    cameraBase = GetOrCreatePerSensorFolder(_cameraDirPath, camName);

                string file = Path.Combine(cameraBase, $"{camName}#{stepIndex:D4}.{ext}");

                // Temporarily turn camera "on" if it's off, capture, then restore original state.
                WithCameraTemporarilyActive(camGO, cam, () =>
                {
                    // 1) Try preferred/known camera feature component by type name (optional hint).
                    if (!string.IsNullOrEmpty(preferredCameraFeatureTypeName))
                    {
                        var typed = camGO.GetComponent(preferredCameraFeatureTypeName) as MonoBehaviour;
                        if (TryInvokeCustomFeature(typed, file)) return;
                    }

                    // 2) Try interface-based integration.
                    var feature = camGO.GetComponent<MonoBehaviour>() as IStepPhotoSource;
                    if (feature != null)
                    {
                        try
                        {
                            // Ensure the feature Behaviour is enabled while we call it.
                            var b = feature as Behaviour;
                            bool prev = b ? b.enabled : true;
                            if (b && !prev) b.enabled = true;

                            feature.CaptureAndSave(file);

                            if (b) b.enabled = prev;
#if UNITY_EDITOR
                            Debug.Log($"Saved Camera (IStepPhotoSource)-> {file}");
#endif
                            return;
                        }
                        catch (Exception ex)
                        {
                            Debug.LogWarning(
                                $"Camera feature (IStepPhotoSource) failed on '{camGO.name}': {ex.Message}. Falling back.");
                        }
                    }

                    // 3) Try reflection to common method names on any component.
                    if (TryInvokeAnyCameraFeatureMethod(camGO, file)) return;

                    // 4) Fallback: built-in capture.
                    try
                    {
                        BuiltInCapture(cam, file);
#if UNITY_EDITOR
                        Debug.Log($"Saved Camera (fallback)-> {file}");
#endif
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"Fallback camera capture failed for '{camGO.name}': {ex}");
                    }
                });
            }
        }

        private bool TryInvokeCustomFeature(MonoBehaviour comp, string path)
        {
            if (comp == null) return false;
            // Attempt common signatures on the explicitly-preferred component.
            return TryInvokeMethod(comp, "CaptureAndSave", path)
                   || TryInvokeMethod(comp, "TakePicture", path)
                   || TryInvokeMethod(comp, "CaptureToFile", path);
        }

        private bool TryInvokeAnyCameraFeatureMethod(GameObject go, string path)
        {
            var comps = go.GetComponents<MonoBehaviour>();
            foreach (var c in comps)
            {
                if (c == null) continue;
                if (TryInvokeMethod(c, "CaptureAndSave", path) ||
                    TryInvokeMethod(c, "TakePicture", path) ||
                    TryInvokeMethod(c, "CaptureToFile", path))
                {
#if UNITY_EDITOR
                    Debug.Log($"Saved Camera (custom feature '{c.GetType().Name}')-> {path}");
#endif
                    return true;
                }
            }

            return false;
        }

        private bool TryInvokeMethod(MonoBehaviour comp, string methodName, string pathArg)
        {
            if (comp == null) return false;

            var t = comp.GetType();
            var m = t.GetMethod(methodName,
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
                null, new[] { typeof(string) }, null);
            if (m == null) return false;

            var behaviour = comp as Behaviour;
            bool prevEnabled = behaviour ? behaviour.enabled : true;
            if (behaviour && !prevEnabled) behaviour.enabled = true;

            try
            {
                m.Invoke(comp, new object[] { pathArg });
                return true;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"Invoke {t.Name}.{methodName}(string) failed: {ex.Message}");
                return false;
            }
            finally
            {
                if (behaviour) behaviour.enabled = prevEnabled;
            }
        }

        private void BuiltInCapture(Camera cam, string absolutePath)
        {
            // Pick resolution
            int w, h;
            if (overrideCameraResolution)
            {
                w = Mathf.Max(1, fallbackWidth) * fallbackSupersampling;
                h = Mathf.Max(1, fallbackHeight) * fallbackSupersampling;
            }
            else
            {
                int pw = Mathf.Max(1, cam.pixelWidth);
                int ph = Mathf.Max(1, cam.pixelHeight);
                w = pw * fallbackSupersampling;
                h = ph * fallbackSupersampling;
            }

            var prevTarget = cam.targetTexture;
            var prevActive = RenderTexture.active;

            var rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32);
            cam.targetTexture = rt;
            cam.Render();

            RenderTexture.active = rt;
            var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, w, h), 0, 0, false);
            tex.Apply(false);

            byte[] bytes = (imageFormat == ImageFormat.PNG)
                ? ImageConversion.EncodeToPNG(tex)
                : ImageConversion.EncodeToJPG(tex, jpgQuality);

            File.WriteAllBytes(absolutePath, bytes);

            cam.targetTexture = prevTarget;
            RenderTexture.active = prevActive;

            rt.Release();
            Destroy(rt);
            Destroy(tex);
        }

        // --------------------------
        // Utilities
        // --------------------------
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

        private void WithCameraTemporarilyActive(GameObject camGO, Camera cam, Action captureAction)
        {
            bool hadActiveSelf = camGO.activeSelf; // this GO's own active flag
            bool hadEnabled = cam.enabled;

            // Turn on if off
            if (!hadActiveSelf) camGO.SetActive(true);
            if (!hadEnabled) cam.enabled = true;

            try
            {
                // If a parent is disabled, we can't fully activate this GO; warn but still attempt.
                if (!camGO.activeInHierarchy)
                    Debug.LogWarning(
                        $"'{camGO.name}' is inactive in hierarchy (likely due to a disabled parent). Attempting capture anyway.");

                captureAction?.Invoke();
            }
            finally
            {
                // Restore to original state
                cam.enabled = hadEnabled;
                if (!hadActiveSelf) camGO.SetActive(false);
            }
        }

        private static string GetOrCreatePerSensorFolder(string baseDir, string sensorName)
        {
            string dir = Path.Combine(baseDir, sensorName);
            Directory.CreateDirectory(dir);
            return dir;
        }


        // --------------------------
// Ground Truth CSV
// --------------------------
        private void SaveGroundTruthCsv()
        {
            // Collect unique seen transforms from all configured GT areas
            var seen = CollectSeenTransforms();

            // One CSV per step, under outputDirectoryRoot/GroundTruth
            string file = System.IO.Path.Combine(_groundTruthDirPath, $"GT#{stepIndex:D4}.csv");

            try
            {
                using (var sw = new System.IO.StreamWriter(file, false, Encoding.UTF8))
                {
                    if (writeCsvHeader)
                        sw.WriteLine("uuid,name,x_ros,y_ros,z_ros,yaw_deg_ros,step");

                    foreach (var t in seen)
                    {
                        if (!t) continue;

                        // Position: same logic you showed:
                        // var pos = CalculateRelativePosition(seenObjects[j].transform.position);
                        // pos = ROS2Utility.UnityToRosPosition(pos);
                        UnityEngine.Vector3
                            posLocal = CalculateRelativePosition(t.position); // relative to rosReference (if provided)
                        UnityEngine.Vector3 posRos = ROS2Utility.UnityToRosPosition(posLocal); // your existing utility

                        // Rotation (Y): apply -Y Euler (optionally relative to rosReference)
                        float yawUnityDeg = GetRelativeYawDeg(t);
                        float yawRosDeg = -yawUnityDeg;

                        // UUID from LineOfSight
                        string uuidHex = TryGetUuidHex(t);

                        // Name (reuse your own sanitizer if you have one; else fallback to t.gameObject.name)
                        // Name (reuse your own sanitizer if you have one; else fallback to t.gameObject.name)
                        string objName = GetSanitized(t.gameObject);

                        sw.WriteLine(
                            $"{uuidHex},{objName},{posRos.x:F6},{posRos.y:F6},{posRos.z:F6},{yawRosDeg:F3},{stepIndex}");
                    }
                }
#if UNITY_EDITOR
                UnityEngine.Debug.Log($"Saved GroundTruth CSV -> {file}");
#endif
            }
            catch (System.Exception ex)
            {
                UnityEngine.Debug.LogError($"Failed writing GroundTruth CSV '{file}': {ex}");
            }
        }

        /// <summary>Union of seen object transforms across all configured GroundTruthAreas.</summary>
        private System.Collections.Generic.List<UnityEngine.Transform> CollectSeenTransforms()
        {
            var set = new System.Collections.Generic.HashSet<int>(); // instanceID de-dup
            var list = new System.Collections.Generic.List<UnityEngine.Transform>();

            if (groundTruthAreas == null) return list;

            for (int i = 0; i < groundTruthAreas.Count; i++)
            {
                var gta = groundTruthAreas[i];
                if (!gta) continue;

                // GroundTruthArea returns transforms of the seen GameObjects.
                var seen = gta.GetSeenObjects(); // List<Transform>
                for (int j = 0; j < seen.Count; j++)
                {
                    var t = seen[j];
                    if (!t) continue;

                    int id = t.GetInstanceID();
                    if (set.Add(id))
                        list.Add(t);
                }
            }

            return list;
        }

        /// <summary>
        /// Make a world-space position relative to rosReference if provided; otherwise just return the world pos.
        /// (Matches your pattern: CalculateRelativePosition(...) -> ROS2Utility.UnityToRosPosition(...))
        /// </summary>
        private UnityEngine.Vector3 CalculateRelativePosition(UnityEngine.Vector3 worldPos)
        {
            if (rosReference != null)
                return rosReference.InverseTransformPoint(worldPos);
            return worldPos;
        }

        /// <summary>Return target yaw (degrees) relative to rosReference (if set), else world yaw.</summary>
        private float GetRelativeYawDeg(UnityEngine.Transform target)
        {
            var q = target.rotation;
            if (rosReference != null)
                q = UnityEngine.Quaternion.Inverse(rosReference.rotation) * q;

            return q.eulerAngles.y; // Unity yaw
        }

        /// <summary>Read UUID from LineOfSight on this transform (or parent/children). Returns 32-char lowercase hex or "NO_UUID".</summary>
        private string TryGetUuidHex(UnityEngine.Transform t)
        {
            if (!t) return "NO_UUID";

            LineOfSight los = t.GetComponent<LineOfSight>();
            if (!los) los = t.GetComponentInParent<LineOfSight>();
            if (!los) los = t.GetComponentInChildren<LineOfSight>();

            if (los != null)
            {
                UUID u = los.GetUUID();
                if (u != null && u.Uuid != null && u.Uuid.Length > 0)
                    return System.BitConverter.ToString(u.Uuid).Replace("-", "").ToLowerInvariant();
            }

            return "NO_UUID";
        }
    }
}