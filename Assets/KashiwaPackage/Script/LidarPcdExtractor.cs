using System;
using AWSIM.Lanelet;
using System.Collections.Generic;
using System.Linq;
using AWSIM.PointCloudMapping.Geometry;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    /// <summary>
    /// Provide functionality to conduct point cloud mapping along all centerlines in OSM.
    /// If you play your scene, PointCloudMapper will automatically start mapping.
    /// The vehicle keeps warping along centerlines at a interval of <see cref="captureLocationInterval"/> and point cloud data from sensors are captured at every warp point.
    /// PCD file will be outputted when you stop your scene or all locations in the route are captured.
    /// </summary>
    public class LidarPcdExtractor : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Imported OSM file. Mapping is conducted along all centerlines of lanelets in the OSM.")]
        private OsmDataContainer osmContainer;

        [SerializeField]
        [Tooltip(
            "Game object containing sensors to capture pointcloud. It will be warped along centerlines of lanelets.")]
        private GameObject vehicleGameObject;

        [SerializeField] [Tooltip("Result PCD file name. On Editor/Windows, it will be saved in Assets/")]
        private string outputPcdFilePath = "output_pcd";

        [SerializeField] private GameObject rootStaticSensors;
        // [SerializeField] private RGLScanAdapter[] rglStaticScanAdapters; 

        
        [SerializeField] [Tooltip("Result csv file name. On Editor/Windows, it will be saved in Assets/")]
        private string outputCsvFilePath = "result.csv";

        [SerializeField] [Tooltip("Distance in meters between consecutive warps along the centerline of a lanelet.")]
        private float captureLocationInterval = 6f;

        [SerializeField] [Tooltip("World origin in ROS coordinate systems, will be added to every point coordinates")]
        private Vector3 worldOriginROS;


        [SerializeField] private bool LaneletVisualizerIsActive = true;
        
        [SerializeField] [Tooltip("Configurable visualization of the loaded lanelet map")]
        private LaneletVisualizer laneletVisualizer;

        private RGLScanAdapter[] mappingSensors;
        private Queue<Pose> capturePoseQueue;
        private string csvPath;

        
        
        
        private void Start()
        {
            // Initiate csv file with headers 
            csvPath = $"{Application.dataPath}/{outputCsvFilePath}";
            CsvEditorUtils.HandleCsvHeader(csvPath,"ID,X,Y,Z,W_rotation,X_rotation,Y_rotation,Z_rotation\n");
            
            mappingSensors = vehicleGameObject.GetComponentsInChildren<RGLScanAdapter>();
            mappingSensors = mappingSensors.Concat(rootStaticSensors.GetComponentsInChildren<RGLScanAdapter>()).ToArray();   
            
            
            if (mappingSensors == null)
            {
                Debug.LogError(
                    $"Could not find mapping sensor in {vehicleGameObject.name}. Disabling PointCloudMapper!");
                enabled = false;
                return;
            }

            Debug.Log($"Found mapping sensor in {vehicleGameObject.name}: {mappingSensors[0].GetSensorName()}");

            for (int i = 0; i < mappingSensors.Length; i++) //TODO can be removed
            {
                mappingSensors[i].Initialize(worldOriginROS, $"{Application.dataPath}/{outputPcdFilePath}");
            }
            // mappingSensor.Initialize(worldOriginROS, $"{Application.dataPath}/{outputPcdFilePath}");

            var laneletMap = new OsmToLaneletMap(worldOriginROS).Convert(osmContainer.Data);

            var start = Time.realtimeSinceStartup;
            capturePoseQueue = new Queue<Pose>(LaneletMapToPoses(laneletMap, captureLocationInterval));
            var computeTimeMs = (Time.realtimeSinceStartup - start) * 1000f;
            Debug.Log($"Will visit {capturePoseQueue.Count} points; computed in {computeTimeMs} ms");

            if (LaneletVisualizerIsActive)
            {
                laneletVisualizer.Initialize(laneletMap);
                laneletVisualizer.CreateCenterline(transform);
            }
            
        }


        private int counter = 1;

        private void Update()
        {
            Debug.Log($"PointCloudMapper: {capturePoseQueue.Count} captures left");
            if (capturePoseQueue.Count == 0)
            {
                Debug.Log("Entered to saving");
                // SavePcd();
                enabled = false;
                return;
            }

            var currentPose = capturePoseQueue.Dequeue();
            
            
            
            //save car data
            vehicleGameObject.transform.position = currentPose.position;
            vehicleGameObject.transform.rotation = currentPose.rotation;

            

            for (int i = 0; i < mappingSensors.Length; i++)
            {
                string rowLog = $"{counter},";
                string sensorName = mappingSensors[i].name;
                //set position
                var pos = ROS2Utility.UnityToRosPosition(mappingSensors[i].transform.position);
                pos = pos + worldOriginROS;
                rowLog += $"{pos.x},{pos.y},{pos.z}";
            
                //set rotation
            
                //rotation base on bus todo check correctness
                Quaternion r = ROS2Utility.UnityToRosRotation(mappingSensors[i].transform.rotation);
                rowLog += $",{r.w},{r.x},{r.y},{r.z} \n";
            
                CsvEditorUtils.AppendStringToFile(csvPath,rowLog);

            }
            
            
            
            
            foreach (var mappingSensor in mappingSensors)
            {
                mappingSensor.CaptureStepByStep();
                mappingSensor.SavePcdStepByStep(counter);
            }
            counter++;
        }

        public void OnDestroy()
        {
            if (enabled)
            {
                SavePcd();
            }

            foreach (var mappingSensor in mappingSensors)
            {
                mappingSensor.Destroy();    
            }
            
        }

        private void SavePcd()
        {
            foreach (var mappingSensor in mappingSensors)
            {
                Debug.Log($"Writing PCD to {Application.dataPath}/{outputPcdFilePath}");
                mappingSensor.SavePcd();
                Debug.Log("PCL data saved successfully");
            }
        }

        private static IEnumerable<Pose> LaneletMapToPoses(LaneletMap laneletMap, float jumpDistance)
        {
            foreach (var laneletData in laneletMap.Lanelets.Values)
            {
                float distanceVisited = 0.0f;
                Vector3[] centerPoints = laneletData.CalculateCenterline();
                BezierPath bezierPath = new BezierPathFactory().CreateBezierPath(centerPoints);

                while (distanceVisited <= bezierPath.Length)
                {
                    yield return bezierPath.TangentPose(distanceVisited);
                    distanceVisited += jumpDistance;
                }
            }
        }
    }
}