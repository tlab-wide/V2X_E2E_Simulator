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


        private readonly string lidarRaysNodeId = "LIDAR_RAYS";
        private readonly string lidarRangeNodeId = "LIDAR_RANGE";
        private readonly string lidarRingsNodeId = "LIDAR_RINGS";
        private readonly string lidarTimeOffsetsNodeId = "LIDAR_OFFSETS";
        private readonly string lidarPoseNodeId = "LIDAR_POSE";
        private readonly string noiseLidarRayNodeId = "NOISE_LIDAR_RAY";
        private readonly string lidarRaytraceNodeId = "LIDAR_RAYTRACE";
        private readonly string noiseHitpointNodeId = "NOISE_HITPOINT";
        private readonly string noiseDistanceNodeId = "NOISE_DISTANCE";
        private readonly string pointsCompactNodeId = "POINTS_COMPACT";
        private readonly string toLidarFrameNodeId = "TO_LIDAR_FRAME";
        
        
        public void CaptureStepByStep(bool useCarPos,Vector3 worldOriginROS)
        {
           
            
            rglSubgraphMapping.Clear();
            
            // Create and connect subgraph
            worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
            worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginROS);
            rglSubgraphMapping = new RGLNodeSequence()
                // .AddNodePointsTransform(rosWorldTransformNodeId, worldTransform)
                .AddNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize))
                .AddNodePointsTemporalMerge(temporalMergeNodeId, new RGLField[1] { RGLField.XYZ_F32 });
            
            
            // rglSubgraphMapping = new RGLNodeSequence()
            //     .AddNodeRaysFromMat3x4f(lidarRaysNodeId, new Matrix4x4[1] {Matrix4x4.identity})
            //     .AddNodeRaysSetRange(lidarRangeNodeId, new Vector2[1] {new Vector2(0.0f, Mathf.Infinity)})
            //     .AddNodeRaysSetRingIds(lidarRingsNodeId, new int[1] {0})
            //     .AddNodeRaysSetTimeOffsets(lidarTimeOffsetsNodeId, new float[1] {0})
            //     .AddNodeRaysTransform(lidarPoseNodeId, Matrix4x4.identity)
            //     .AddNodeGaussianNoiseAngularRay(noiseLidarRayNodeId, 0, 0)
            //     .AddNodeRaytrace(lidarRaytraceNodeId)
            //     .AddNodeGaussianNoiseAngularHitpoint(noiseHitpointNodeId, 0, 0)
            //     .AddNodeGaussianNoiseDistance(noiseDistanceNodeId, 0, 0, 0);
            
            

            // rglSubgraphMapping.SetActive(downsampleNodeId, enableDownsampling);

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


        public void SavePcdStepByStep(int counter)
        {
            if (rglSubgraphMapping == null)
            {
                Debug.LogWarning("RGLMappingAdapter: skipped saving PCD file - empty point cloud");
                return;
            }

            rglSubgraphMapping.SavePcdFile($"{outputPcdFilePath}_{GetSensorName()}_{counter}.pcd");
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