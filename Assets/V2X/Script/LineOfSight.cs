using System;
using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using NUnit.Framework;
using std_msgs.msg;
using unique_identifier_msgs.msg;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;

public class LineOfSight : MonoBehaviour
{
    [SerializeField] private Transform pointsParent;
    public List<Transform> points;
    

    [SerializeField] private Transform cube;
    [SerializeField] private BoxState boxState;
    [SerializeField] public int minimumNumberOfPointsVisible = 1;
    // public int MinimumNumberOfPointsVisible { get { return minimumNumberOfPointsVisible; } }

    

    private List<MockSensor> observableSensor = new List<MockSensor>();
    private Dictionary<MockSensor, int> observableSensorWitCount = new Dictionary<MockSensor, int>();
    private UUID uuid;


    public int MinimumNumberOfPointsVisible()
    {
        return minimumNumberOfPointsVisible;
    }


    public List<Transform> Points()
    {
        return points;
    }
    private void Awake()
    {
        Transform[] childs = pointsParent.GetComponentsInChildren<Transform>();

        points = new List<Transform>();

        for (int i = 1; i < childs.Length; i++)
        {
            points.Add(childs[i]);
        }

        uuid = new UUID();
        GenerateRandomBytes(uuid.Uuid);

        // if (LineOfSightManagerJobs.Instance != null ||  LineOfSightManagerJobs.Instance.IsJobsRunning())
        // {
        //     LineOfSightManagerJobs.Instance.RegisterTarget(this);
        // }
        
    }
    
    
    
    

    public static System.Random random = new System.Random();

    public static void GenerateRandomBytes(byte[] uuid)
    {
        byte[] randomBytes = new byte[uuid.Length];

        random.NextBytes(randomBytes); // Fill the byte array with random values

        for (int i = 0; i < uuid.Length; i++)
        {
            uuid[i] = randomBytes[i];
        }
    }


    public UUID GetUUID()
    {
        return uuid;
    }


    public static UUID GenerateUUid()
    {
        UUID uuid = new UUID();
        GenerateRandomBytes(uuid.Uuid);
        return uuid;
    }


    private void OnEnable()
    {
        setColor(0f, BoxState.Red);
        // if (!LineOfSightManagerJobs.Instance.IsJobsRunning())
        // {
        StartCoroutine(CheckLineOfSight());
        // }
        
    }


    public IEnumerator CheckLineOfSight(bool fastMode = false)
    {
        if (!fastMode)
            yield return null; //wait one frame

        Scenario currentScenario = BlindScenarioManager.Instance.getCurrentScenario();
        List<MockSensor> carLidars = currentScenario.getLidars();
        List<MockSensor> carCameras = currentScenario.getCameras();
        List<MockSensor> rsuSensors = currentScenario.getRSUsensors();
        List<MockSensor> rsuCameras = currentScenario.getRsuCameras();


        // Transform busLidar = BlindScenarioManager.Instance.getBusLidar();
        // Transform crossroadLidar = BlindScenarioManager.Instance.getRSUsensor();

        while (this != null && this.gameObject.activeSelf)
        {
            //find there is vision or not
            bool seenByRSU = false;
            bool seenByOBU = false;

            // bus lidars 
            List<Transform> seenObjectsByLidar = new List<Transform>();
            for (int j = 0; j < carLidars.Count; j++)
            {
                int numberOfSeenBusLidars = 0;
                for (int i = 0; i < points.Count; i++)
                {
                    Transform startPoint = points[i];
                    if (carLidars[j].haveLineOfSight(startPoint))
                    {
                        numberOfSeenBusLidars++;
                        seenObjectsByLidar.Add(startPoint);
                    }
                    carLidars[j].UpdateLastUpdateTime();
                }

                //visualization
                if (numberOfSeenBusLidars >= BlindScenarioManager.Instance.GetBusLidarThreshold() &&
                    numberOfSeenBusLidars >= minimumNumberOfPointsVisible)
                {
                    seenByOBU = true;
                    //just for log system
                    if (!observableSensor.Contains(carLidars[j]))
                    {
                        observableSensor.Add(carLidars[j]);
                        observableSensorWitCount[carLidars[j]] = numberOfSeenBusLidars;
                    }

                    carLidars[j].addObjectToObserved(this.transform, numberOfSeenBusLidars);


                    // BlindScenarioManager.Instance.AddObservedCar(this.transform);
                    // Debug.Log("now is green");
                }
                else
                {
                    //just for log system
                    observableSensor.Remove(carLidars[j]);
                    carLidars[j].removeObjectFromObserved(this.transform);
                }
            }


            // RSU lidar 
            List<Transform> seenObjectsByRSU = new List<Transform>();

            for (int j = 0; j < rsuSensors.Count; j++)
            {
                int numberOfSeenRSUs = 0;
                for (int i = 0; i < points.Count; i++)
                {
                    Transform startPoint = points[i];
                    if (rsuSensors[j].haveLineOfSight(startPoint))
                    {
                        numberOfSeenRSUs++;
                        seenObjectsByRSU.Add(startPoint);
                    }
                    rsuSensors[j].UpdateLastUpdateTime();
                }

                //visualization
                if (numberOfSeenRSUs >= BlindScenarioManager.Instance.GetRsuThreshold() &&
                    numberOfSeenRSUs >= minimumNumberOfPointsVisible)
                {
                    seenByRSU = true;
                    //just for log system
                    if (!observableSensor.Contains(rsuSensors[j]))
                    {
                        observableSensor.Add(rsuSensors[j]);
                        observableSensorWitCount[rsuSensors[j]] = numberOfSeenRSUs;
                    }

                    rsuSensors[j].addObjectToObserved(this.transform, numberOfSeenRSUs);
                }
                else
                {
                    observableSensor.Remove(rsuSensors[j]);
                    rsuSensors[j].removeObjectFromObserved(this.transform);
                }
            }


            // bus cameras 
            List<Transform> seenObjectsCameras = new List<Transform>();
            for (int j = 0; j < carCameras.Count; j++)
            {
                int numberOfSeenBusCameras = 0;
                for (int i = 0; i < points.Count; i++)
                {
                    Transform startPoint = points[i];
                    if (carCameras[j].haveLineOfSight(startPoint))
                    {
                        numberOfSeenBusCameras++;
                        seenObjectsCameras.Add(startPoint);
                    }
                    carCameras[j].UpdateLastUpdateTime();
                }

                //visualization
                if (numberOfSeenBusCameras >= BlindScenarioManager.Instance.GetBusCameraThreshold() &&
                    numberOfSeenBusCameras >= minimumNumberOfPointsVisible)
                {
                    seenByOBU = true;
                    if (!observableSensor.Contains(carCameras[j]))
                    {
                        observableSensor.Add(carCameras[j]);
                        observableSensorWitCount[carCameras[j]] = numberOfSeenBusCameras;
                    }

                    carCameras[j].addObjectToObserved(this.transform, numberOfSeenBusCameras);
                }
                else
                {
                    observableSensor.Remove(carCameras[j]);
                    carCameras[j].removeObjectFromObserved(this.transform);
                }
            }


            //Rsu Camera
            List<Transform> seenObjectsByRsuCameras = new List<Transform>();
            for (int j = 0; j < rsuCameras.Count; j++)
            {
                int numberOfSeenRsuCameras = 0;
                for (int i = 0; i < points.Count; i++)
                {
                    Transform startPoint = points[i];
                    if (rsuCameras[j].haveLineOfSight(startPoint))
                    {
                        numberOfSeenRsuCameras++;
                        seenObjectsByRsuCameras.Add(startPoint);
                    }
                    rsuCameras[j].UpdateLastUpdateTime();
                }

                //visualization
                if (numberOfSeenRsuCameras >= BlindScenarioManager.Instance.GetRsuCameraThreshold() &&
                    numberOfSeenRsuCameras >= minimumNumberOfPointsVisible)
                {
                    seenByRSU = true;
                    if (!observableSensor.Contains(rsuCameras[j]))
                    {
                        observableSensor.Add(rsuCameras[j]);
                        observableSensorWitCount[rsuCameras[j]] = numberOfSeenRsuCameras;
                    }

                    rsuCameras[j].addObjectToObserved(this.transform, numberOfSeenRsuCameras);
                }
                else
                {
                    observableSensor.Remove(rsuCameras[j]);
                    rsuCameras[j].removeObjectFromObserved(this.transform);
                }
            }


            // select box color
            if (seenByOBU)
            {
                float notSeenPercentage = ((float)points.Count - seenObjectsByLidar.Count) / points.Count;
                setColor(1, BoxState.Green);
                // Debug.Log("now is green");
            }
            else if (seenByRSU)
            {
                // float notSeenPercentage = ((float)points.Count - seenObjectsByRSU.Count) / points.Count;
                setColor(1, BoxState.Purple);
                // Debug.Log("yellow");
            }
            else
            {
                setColor(1, BoxState.Red);
                // Debug.Log("now is red");
            }


            if (!fastMode)
            {
                yield return new WaitForSeconds(1 / BlindScenarioManager.Instance.GetFrequency());
            }
        }
    }

    public void checkImmidiately()
    {
        Scenario currentScenario = BlindScenarioManager.Instance.getCurrentScenario();
        List<MockSensor> lidars = currentScenario.getLidars();
        List<MockSensor> busCameras = currentScenario.getCameras();
        List<MockSensor> rsuSensors = currentScenario.getRSUsensors();
        List<MockSensor> rsuCameras = currentScenario.getRsuCameras();

        bool seenByRSU = false;
        bool seenByOBU = false;

        // Transform busLidar = BlindScenarioManager.Instance.getBusLidar();
        // Transform crossroadLidar = BlindScenarioManager.Instance.getRSUsensor();


        //find there is vision or not


        // lidars handeling 
        List<Transform> seenObjectsByLidar = new List<Transform>();
        for (int j = 0; j < lidars.Count; j++)
        {
            int numberOfSeenBusLidars = 0;
            for (int i = 0; i < points.Count; i++)
            {
                Transform startPoint = points[i];
                if (lidars[j].haveLineOfSight(startPoint))
                {
                    numberOfSeenBusLidars++;
                    seenObjectsByLidar.Add(startPoint);
                }
            }

            //visualization
            if (numberOfSeenBusLidars >= BlindScenarioManager.Instance.GetBusLidarThreshold() &&
                numberOfSeenBusLidars >= minimumNumberOfPointsVisible)
            {
                seenByOBU = true;
                //just for log system
                if (!observableSensor.Contains(lidars[j]))
                {
                    observableSensor.Add(lidars[j]);
                    observableSensorWitCount[lidars[j]] = numberOfSeenBusLidars;
                }

                lidars[j].addObjectToObserved(this.transform, numberOfSeenBusLidars);


                // BlindScenarioManager.Instance.AddObservedCar(this.transform);
                // Debug.Log("now is green");
            }
            else
            {
                //just for log system
                observableSensor.Remove(lidars[j]);
                lidars[j].removeObjectFromObserved(this.transform);
            }
        }


        // RSU handeling 
        List<Transform> seenObjectsByRSU = new List<Transform>();

        for (int j = 0; j < rsuSensors.Count; j++)
        {
            int numberOfSeenRSUs = 0;
            for (int i = 0; i < points.Count; i++)
            {
                Transform startPoint = points[i];
                if (rsuSensors[j].haveLineOfSight(startPoint))
                {
                    numberOfSeenRSUs++;
                    seenObjectsByRSU.Add(startPoint);
                }
            }

            //visualization
            if (numberOfSeenRSUs >= BlindScenarioManager.Instance.GetRsuThreshold() &&
                numberOfSeenRSUs >= minimumNumberOfPointsVisible)
            {
                seenByRSU = true;
                //just for log system
                if (!observableSensor.Contains(rsuSensors[j]))
                {
                    observableSensor.Add(rsuSensors[j]);
                    observableSensorWitCount[rsuSensors[j]] = numberOfSeenRSUs;
                }

                rsuSensors[j].addObjectToObserved(this.transform, numberOfSeenRSUs);
            }
            else
            {
                observableSensor.Remove(rsuSensors[j]);
                rsuSensors[j].removeObjectFromObserved(this.transform);
            }
        }


        // bus cameras 
        List<Transform> seenObjectsCameras = new List<Transform>();
        for (int j = 0; j < busCameras.Count; j++)
        {
            int numberOfSeenBusCameras = 0;
            for (int i = 0; i < points.Count; i++)
            {
                Transform startPoint = points[i];
                if (busCameras[j].haveLineOfSight(startPoint))
                {
                    numberOfSeenBusCameras++;
                    seenObjectsCameras.Add(startPoint);
                }
            }

            //visualization
            if (numberOfSeenBusCameras >= BlindScenarioManager.Instance.GetBusCameraThreshold() &&
                numberOfSeenBusCameras >= minimumNumberOfPointsVisible)
            {
                seenByOBU = true;
                if (!observableSensor.Contains(busCameras[j]))
                {
                    observableSensor.Add(busCameras[j]);
                    observableSensorWitCount[busCameras[j]] = numberOfSeenBusCameras;
                }

                busCameras[j].addObjectToObserved(this.transform, numberOfSeenBusCameras);
            }
            else
            {
                observableSensor.Remove(busCameras[j]);
                busCameras[j].removeObjectFromObserved(this.transform);
            }
        }


        //Rsu Camera
        List<Transform> seenObjectsByRsuCameras = new List<Transform>();
        for (int j = 0; j < rsuCameras.Count; j++)
        {
            int numberOfSeenRsuCameras = 0;
            for (int i = 0; i < points.Count; i++)
            {
                Transform startPoint = points[i];
                if (rsuCameras[j].haveLineOfSight(startPoint))
                {
                    numberOfSeenRsuCameras++;
                    seenObjectsByRsuCameras.Add(startPoint);
                }
            }

            //visualization
            if (numberOfSeenRsuCameras >= BlindScenarioManager.Instance.GetRsuCameraThreshold() &&
                numberOfSeenRsuCameras >= minimumNumberOfPointsVisible)
            {
                seenByRSU = true;
                if (!observableSensor.Contains(rsuCameras[j]))
                {
                    observableSensor.Add(rsuCameras[j]);
                    observableSensorWitCount[rsuCameras[j]] = numberOfSeenRsuCameras;
                }

                rsuCameras[j].addObjectToObserved(this.transform, numberOfSeenRsuCameras);
            }
            else
            {
                observableSensor.Remove(rsuCameras[j]);
                rsuCameras[j].removeObjectFromObserved(this.transform);
            }
        }


        // select box color
        if (seenByOBU)
        {
            // float notSeenPercentage = ((float)points.Count - seenObjectsByLidar.Count) / points.Count;
            setColor(1, BoxState.Green);
            // Debug.Log("now is green");
        }
        else if (seenByRSU)
        {
            // float notSeenPercentage = ((float)points.Count - seenObjectsByRSU.Count) / points.Count;
            setColor(1, BoxState.Purple);
            // Debug.Log("yellow");
        }
        else
        {
            setColor(1, BoxState.Red);
            // Debug.Log("now is red");
        }
    }


    public BoxState GetCarBoxState()
    {
        return this.boxState;
    }

    public List<MockSensor> GetObservableSensors()
    {
        return observableSensor;
    }

    public int GetNumberOfDetectedPoint(MockSensor mockSensor)
    {
        return observableSensorWitCount[mockSensor];
    }


    public void setColor(float antiTransparency, BoxState boxState)
    {
        bool isItSkinnedMesh = true;

        this.boxState = boxState;
        var skinnedMesh = cube.GetComponent<SkinnedMeshRenderer>();
        Material material;

        if (skinnedMesh == null)
        {
            material = cube.GetComponent<MeshRenderer>().material;
            isItSkinnedMesh = false;
        }
        else
        {
            isItSkinnedMesh = true;
            material = skinnedMesh.material;
        }

        material.SetFloat("AntiTransparency", antiTransparency * 2);


        switch (boxState)
        {
            case BoxState.Green:
                material.SetColor("Base_Color", Color.green);
                break;
            case BoxState.Red:
                material.SetColor("Base_Color", Color.red);
                break;
            case BoxState.Purple:
                material.SetColor("Base_Color", Color.blue);
                break;
        }

        if (isItSkinnedMesh)
        {
            cube.GetComponent<SkinnedMeshRenderer>().material = material;
        }
        else
        {
            cube.GetComponent<MeshRenderer>().material = material;
        }
    }


    public byte GetTypeOfObject()
    {
        string carName = gameObject.name;

        string truck = "Truck";
        string bus = "Bus";
        string car = "Car";
        string van = "Van";
        string hatchback = "Hatchback";
        string taxi = "Taxi";


        if (carName.IndexOf(truck) >= 0)
        {
            // Debug.Log("Truck catched");
            return ObjectClassification.TRUCK;
        }

        if (carName.IndexOf(bus) >= 0)
        {
            // Debug.Log("Bus catched");
            return ObjectClassification.BUS;
        }

        if (carName.IndexOf(car) >= 0 || carName.IndexOf(van) >= 0 || carName.IndexOf(taxi) >= 0 ||
            carName.IndexOf(hatchback) >= 0)
        {
            // Debug.Log("cars catched");
            return ObjectClassification.CAR;
        }
        else
        {
            //todo complete other types, currently in our environment we still do not have require any more
            // Debug.Log("cars Pedestrian");
            return ObjectClassification.PEDESTRIAN;
        }
    }


    public enum BoxState
    {
        Purple,
        Green,
        Red
    }


    // public void SetDetectedSensors(List<MockSensor> sensors)
    // {
    //     bool detectedByRsu = false;
    //     bool detectedByBus = false;
    //
    //     foreach (var sensor in sensors)
    //     {
    //         MockSensor.MockSensorType sensorType = sensor.GetMockSensorType();
    //         if (sensorType == MockSensor.MockSensorType.OBU_LIDAR || sensorType == MockSensor.MockSensorType.OBU_CAMERA)
    //         {
    //             detectedByBus = true;
    //         }
    //
    //         if (sensorType == MockSensor.MockSensorType.RSU_Lidar || sensorType == MockSensor.MockSensorType.RSU_CAMERA)
    //         {
    //             detectedByRsu = true;
    //         }
    //     }
    //
    //     if (detectedByBus)
    //     {
    //         //make it green
    //         setColor(1, BoxState.Green);
    //     }
    //     else if (detectedByRsu)
    //     {
    //         //make it yellow
    //         setColor(1, BoxState.Purple);
    //     }
    //     else
    //     {
    //         //make it red
    //         setColor(1, BoxState.Red);
    //     }
    //
    //
    //     // You might want to update visual indicators, notify other systems, etc.
    //     // For demonstration, we log the detected sensors.
    //     string sensorNames = "";
    //     foreach (MockSensor sensor in sensors)
    //     {
    //         sensorNames += sensor.name + ", ";
    //     }
    //
    //     Debug.Log($"[{name}] Detected by sensors: {sensorNames}");
    // }
    
    public void SetDetectedSensors2(List<MockSensor> sensors)
    {
        bool detectedByRsu = false;
        bool detectedByBus = false;

        // Determine which sensor types detected this object.
        foreach (var sensor in sensors)
        {
            MockSensor.MockSensorType sensorType = sensor.GetMockSensorType();
            if (sensorType == MockSensor.MockSensorType.OBU_LIDAR || sensorType == MockSensor.MockSensorType.OBU_CAMERA)
            {
                detectedByBus = true;
            }
            if (sensorType == MockSensor.MockSensorType.RSU_Lidar || sensorType == MockSensor.MockSensorType.RSU_CAMERA)
            {
                detectedByRsu = true;
            }
        }

        // Update color based on sensor types.
        if (detectedByBus)
        {
            // Make it green if any bus sensor detects the object.
            setColor(1, BoxState.Green);
        }
        else if (detectedByRsu)
        {
            // Make it purple if only RSU sensors detect the object.
            setColor(1, BoxState.Purple);
        }
        else
        {
            // Otherwise, mark it as red.
            setColor(1, BoxState.Red);
        }

        // Update internal lists and counts.
        observableSensor = sensors;
        observableSensor.Clear();
        foreach (MockSensor sensor in sensors)
        {
            if (observableSensorWitCount.ContainsKey(sensor))
                observableSensorWitCount[sensor]++;
            else
                observableSensorWitCount[sensor] = 1;
        }

        // Log the detected sensor names.
        // string sensorNames = "";
        // foreach (MockSensor sensor in sensors)
        // {
        //     sensorNames += sensor.name + ", ";
        // }
        // Debug.Log($"[{name}] Detected by sensors: {sensorNames}");
    }
}