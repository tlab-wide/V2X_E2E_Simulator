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
    [RequireComponent(typeof(SensorSettingController))]
    public class LidarPcdExtractor : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Imported OSM file. Mapping is conducted along all centerlines of lanelets in the OSM.")]
        private OsmDataContainer osmContainer;

        [Header("New log scan setting")] [SerializeField]
        private bool readPosesFromFile;

        [SerializeField] private TextAsset positionsOfCar;
        [SerializeField] private int iterationNumber = 5;
        [SerializeField] private float waitAfterEachScan = 1.0f;
        [SerializeField] private SensorSettingController sensorSettingController;

        [Header("Normal settings")]
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

        [SerializeField] private bool useCarPos = false;
        [SerializeField] private float waitTime;

        [SerializeField] [Tooltip("Distance in meters between consecutive warps along the centerline of a lanelet.")]
        private float captureLocationInterval = 6f;

        [SerializeField] [Tooltip("World origin in ROS coordinate systems, will be added to every point coordinates")]
        private Vector3 worldOriginROS;


        [SerializeField] private bool LaneletVisualizerIsActive = true;

        [SerializeField] private LogAroundCar logAroundCar;

        [SerializeField] [Tooltip("Configurable visualization of the loaded lanelet map")]
        private LaneletVisualizer laneletVisualizer;

        private RGLScanAdapter[] mappingSensors;
        private List<Pose> capturePoseQueue;
        private string csvPath;
        private float startTime;
        private ScannerCar scannerCar;


        private void Start()
        {
            startTime = Time.time;


            // Initiate csv file with headers 
            csvPath = $"{Application.dataPath}/{outputCsvFilePath}";
            CsvEditorUtils.HandleCsvHeader(csvPath,
                "state,pose_index,Iteration,shift_x,shift_y,shift_z,angle,Name,X,Y,Z,W_rotation,X_rotation,Y_rotation,Z_rotation\n");

            mappingSensors = vehicleGameObject.GetComponentsInChildren<RGLScanAdapter>();
            mappingSensors = mappingSensors.Concat(rootStaticSensors.GetComponentsInChildren<RGLScanAdapter>())
                .ToArray();


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

            if (readPosesFromFile)
            {
                capturePoseQueue = new List<Pose>();
                List<List<string>> poses = CsvEditorUtils.ReadCsvFile(positionsOfCar);


                for (int i = 0; i < poses.Count; i++)
                {
                    Vector3 pos = new Vector3(float.Parse(poses[i][0]), float.Parse(poses[i][1]),
                        float.Parse(poses[i][2]));

                    pos = pos - worldOriginROS;

                    pos = ROS2Utility.RosToUnityPosition(pos);


                    Vector3 rotation = new Vector3(float.Parse(poses[i][3]), float.Parse(poses[i][4]),
                        float.Parse(poses[i][5]));
                    Quaternion rotationQuaternion = Quaternion.Euler(rotation);

                    // rotationQuaternion =  ROS2Utility.RosToUnityRotation(rotationQuaternion);

                    Debug.Log($"** position :{pos} , {rotation}");
                    Pose newPose = new Pose(pos, rotationQuaternion);
                    capturePoseQueue.Add(newPose);
                }
            }
            else
            {
                capturePoseQueue = new List<Pose>(LaneletMapToPoses(laneletMap, captureLocationInterval).ToList());
            }

            var computeTimeMs = (Time.realtimeSinceStartup - start) * 1000f;
            Debug.Log($"Will visit {capturePoseQueue.Count} points; computed in {computeTimeMs} ms");

            if (LaneletVisualizerIsActive)
            {
                laneletVisualizer.Initialize(laneletMap);
                laneletVisualizer.CreateCenterline(transform);
            }


            scannerCar = vehicleGameObject.GetComponent<ScannerCar>();
            sensorSettingController = this.GetComponent<SensorSettingController>();
        }


        private int counterPos = 0;
        private int counterIteration = 0;
        private int stateSensorSetting = 0;
        private float lastScanTime;


        private void Update()
        {
            //wait time mechanism 
            if (Time.time - startTime < waitTime && Time.time - lastScanTime < waitAfterEachScan)
            {
                return;
            }

            if (counterIteration >= iterationNumber) // all process finished
            {
                return;
            }


            //this section implimented for i j k in update methode compatible with unity class structure 
            Debug.Log(
                $"stateSensor {stateSensorSetting} , counter pos {counterPos}, counter iteration {counterIteration} ");
            stateSensorSetting = sensorSettingController.SetupNextState();
            if (stateSensorSetting == -1)
            {
                stateSensorSetting = sensorSettingController.ResetState();
                counterPos++;
                if (counterPos >= capturePoseQueue.Count)
                {
                    counterPos = 0;
                    counterIteration++;
                    if (counterIteration >= iterationNumber)
                    {
                        return;
                    }
                }
            }

            Debug.Log($"exit with {stateSensorSetting} ");


            Debug.Log($"PointCloudMapper: {capturePoseQueue.Count} captures left");
            if (capturePoseQueue.Count == 0)
            {
                Debug.Log("Entered to saving");
                // SavePcd();
                enabled = false;
                return;
            }

            // var currentPose = capturePoseQueue.Dequeue();
            var currentPose = capturePoseQueue[counterPos];


            //save car data
            vehicleGameObject.transform.position = currentPose.position;
            vehicleGameObject.transform.rotation = currentPose.rotation;

            if (scannerCar.IsInsideAnything())
            {
                Debug.Log($"The scanner was inside a car in this point: {currentPose.position}");
                return;
            }

            if (logAroundCar != null)
            {
                logAroundCar.CaptureLog(stateSensorSetting, counterPos,
                    sensorSettingController.GetPos(stateSensorSetting),
                    sensorSettingController.GetAngle(stateSensorSetting), counterIteration);
            }


            for (int i = 0; i < mappingSensors.Length; i++)
            {
                Vector3 shifPos = sensorSettingController.GetPos(stateSensorSetting);
                float angle = sensorSettingController.GetAngle(stateSensorSetting);

                string rowLog = $"{counterPos},{stateSensorSetting},{counterIteration},{mappingSensors[i].name},";
                rowLog += $"{shifPos.x},{shifPos.y},{shifPos.z},{angle},";
                //set position
                var pos = ROS2Utility.UnityToRosPosition(mappingSensors[i].transform.position);
                pos = pos + worldOriginROS;
                rowLog += $"{pos.x},{pos.y},{pos.z}";

                //set rotation

                //rotation base on bus todo check correctness
                Quaternion r = ROS2Utility.UnityToRosRotation(mappingSensors[i].transform.rotation);
                rowLog += $",{r.w},{r.x},{r.y},{r.z} \n";

                CsvEditorUtils.AppendStringToFile(csvPath, rowLog);
            }


            foreach (var mappingSensor in mappingSensors)
            {
                mappingSensor.CaptureStepByStep(useCarPos);

                string fileName = $"{stateSensorSetting}_{counterPos}_{iterationNumber}";
                mappingSensor.SavePcdStepByStep(fileName);
            }

            lastScanTime = Time.time;
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