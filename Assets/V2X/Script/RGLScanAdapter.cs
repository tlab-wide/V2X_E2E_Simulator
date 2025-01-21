using System;
using System.Collections;
using System.Collections.Generic;
using RGLUnityPlugin;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    [RequireComponent(typeof(LidarSensor))]
    public class RGLScanAdapter : MonoBehaviour
    {
        [SerializeField] [Tooltip("Enable/disable point cloud data downsampling")]
        private bool enableDownsampling;

        [SerializeField] [Tooltip("Resolution of point cloud data downsampling")] [Min(0.000001f)]
        private float leafSize;

        private bool isInitialized = false;

        private LidarSensor lidarSensor;

        private RGLNodeSequence rglSubgraphMapping;

        private readonly string rosWorldTransformNodeId = "ROS_WORLD_TF";
        private readonly string downsampleNodeId = "DOWNSAMPLE";
        private readonly string temporalMergeNodeId = "TEMPORAL_MERGE";

        private string outputPcdFilePath;

        private Matrix4x4 worldTransform;

        public void Awake()
        {
            lidarSensor = GetComponent<LidarSensor>();
            // Make sure automatic capture in RGL Lidar Sensor is disabled.
            // We want to perform captures only on demand (after warping).
            lidarSensor.AutomaticCaptureHz = 0;
        }

        // Empty function to make enable checkbox appear in the Inspector
        public void Start()
        {
        }


        public void Initialize(Vector3 worldOriginROS, string outputPcdFilePath)
        {
            if (isInitialized)
            {
                throw new Exception("Attempted to initialize RGLMappingAdapter twice!");
            }

            this.outputPcdFilePath = outputPcdFilePath;

            // Create and connect subgraph
            worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
            worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginROS);
            rglSubgraphMapping = new RGLNodeSequence()
                .AddNodePointsTransform(rosWorldTransformNodeId, worldTransform)
                .AddNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize))
                .AddNodePointsTemporalMerge(temporalMergeNodeId, new RGLField[1] { RGLField.XYZ_F32 });

            rglSubgraphMapping.SetActive(downsampleNodeId, enableDownsampling);

            lidarSensor.ConnectToWorldFrame(rglSubgraphMapping);

            isInitialized = true;
        }

        public void OnValidate()
        {
            if (rglSubgraphMapping == null)
            {
                return;
            }

            rglSubgraphMapping.UpdateNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
            rglSubgraphMapping.SetActive(downsampleNodeId, enableDownsampling);
        }

        public string GetSensorName()
        {
            return gameObject.name;
        }

        public void SavePcd()
        {
            if (rglSubgraphMapping == null)
            {
                Debug.LogWarning("RGLMappingAdapter: skipped saving PCD file - empty point cloud");
                return;
            }

            rglSubgraphMapping.SavePcdFile(outputPcdFilePath);
        }

        // public void Capture()
        // {
        //     if (!isInitialized)
        //     {
        //         throw new Exception("Attempted to run RGLMappingAdapter without initialization!");
        //     }
        //
        //     if (!enabled)
        //     {
        //         return;
        //     }
        //
        //     lidarSensor.Capture();
        //
        //     if (enableDownsampling)
        //     {
        //         int countBeforeDownsample = rglSubgraphMapping.GetPointCloudCount(rosWorldTransformNodeId);
        //         int countAfterDownsample = rglSubgraphMapping.GetPointCloudCount(downsampleNodeId);
        //         bool pointCloudReduced = countAfterDownsample < countBeforeDownsample;
        //         if (!pointCloudReduced)
        //         {
        //             Debug.LogWarning(
        //                 $"Downsampling had no effect for '{name}'. If you see this message often, consider increasing leafSize.");
        //         }
        //     }
        // }


        public void CaptureStepByStep(bool useCarPos)
        {
            rglSubgraphMapping.Clear();
            rglSubgraphMapping = new RGLNodeSequence()
                .AddNodePointsTransform(rosWorldTransformNodeId, worldTransform)
                .AddNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize))
                .AddNodePointsTemporalMerge(temporalMergeNodeId, new RGLField[1] { RGLField.XYZ_F32 });

            rglSubgraphMapping.SetActive(downsampleNodeId, enableDownsampling);

            lidarSensor.ConnectToWorldFrame(rglSubgraphMapping);

            isInitialized = true;


            if (!isInitialized)
            {
                throw new Exception("Attempted to run RGLMappingAdapter without initialization!");
            }

            if (!enabled)
            {
                return;
            }

            lidarSensor.Capture();

            
            
            if (enableDownsampling)
            {
                int countBeforeDownsample = rglSubgraphMapping.GetPointCloudCount(rosWorldTransformNodeId);
                int countAfterDownsample = rglSubgraphMapping.GetPointCloudCount(downsampleNodeId);
                bool pointCloudReduced = countAfterDownsample < countBeforeDownsample;
                if (!pointCloudReduced)
                {
                    Debug.LogWarning(
                        $"Downsampling had no effect for '{name}'. If you see this message often, consider increasing leafSize.");
                }
            }
        }


        public void SavePcdStepByStep(string fileName)
        {
            if (rglSubgraphMapping == null)
            {
                Debug.LogWarning("RGLMappingAdapter: skipped saving PCD file - empty point cloud");
                return;
            }

            rglSubgraphMapping.SavePcdFile($"{outputPcdFilePath}_{GetSensorName()}_{fileName}.pcd");
        }

        // Called in PointCloudMapper.OnDestroy()
        // Must be executed after destroying PointCloudMapper because SavePcd is called there
        // To be refactored when implementing multi-sensor mapping
        public void Destroy()
        {
            rglSubgraphMapping.Clear();
            isInitialized = false;
        }
    }
}