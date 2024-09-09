using System;
using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using std_msgs.msg;
using unique_identifier_msgs.msg;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;

public class LineOfSight : MonoBehaviour
{
    [SerializeField] private Transform pointsParent;
    private List<Transform> points;


    [SerializeField] private Transform cube;
    [SerializeField] private BoxState boxState;

    private List<MockSensor> observableSensor = new List<MockSensor>();
    private UUID uuid;


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

    // private void OnEnable()
    // {
    //     setColor(0f, BoxState.Green);
    //     StartCoroutine(CheckLineOfSight());
    // }

    private void OnEnable()
    {
        setColor(0f, BoxState.Green);
        StartCoroutine(CheckLineOfSight());
    }


    public IEnumerator CheckLineOfSight(bool fastMode = false)
    {
        if (!fastMode)
            yield return null; //wait one frame

        Scenario currentScenario = BlindScenarioManager.Instance.getCurrentScenario();
        List<MockSensor> lidars = currentScenario.getLidars();
        List<MockSensor> busCameras = currentScenario.getCameras();
        List<MockSensor> rsuSensors = currentScenario.getRSUsensors();
        List<MockSensor> rsuCameras = currentScenario.getRsuCameras();


        // Transform busLidar = BlindScenarioManager.Instance.getBusLidar();
        // Transform crossroadLidar = BlindScenarioManager.Instance.getRSUsensor();

        while (this != null && this.gameObject.activeSelf)
        {
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
                if (numberOfSeenBusLidars >= BlindScenarioManager.Instance.GetBusLidarThreshold())
                {
                    //just for log system
                    if (!observableSensor.Contains(lidars[j]))
                    {
                        observableSensor.Add(lidars[j]);
                    }

                    lidars[j].addObjectToObserved(this.transform);


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
                if (numberOfSeenRSUs >= BlindScenarioManager.Instance.GetRsuThreshold())
                {
                    //just for log system
                    if (!observableSensor.Contains(rsuSensors[j]))
                    {
                        observableSensor.Add(rsuSensors[j]);
                    }

                    rsuSensors[j].addObjectToObserved(this.transform);
                }
                else
                {
                    observableSensor.Remove(rsuSensors[j]);
                    rsuSensors[j].removeObjectFromObserved(this.transform);
                }
            }


            // lidar cameras 
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
                if (numberOfSeenBusCameras >= BlindScenarioManager.Instance.GetRsuThreshold())
                {
                    if (!observableSensor.Contains(busCameras[j]))
                    {
                        observableSensor.Add(busCameras[j]);
                    }

                    busCameras[j].addObjectToObserved(this.transform);
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
                if (numberOfSeenRsuCameras >= BlindScenarioManager.Instance.GetRsuCameraThreshold())
                {
                    if (!observableSensor.Contains(rsuCameras[j]))
                    {
                        observableSensor.Add(rsuCameras[j]);
                    }

                    rsuCameras[j].addObjectToObserved(this.transform);
                }
                else
                {
                    observableSensor.Remove(rsuCameras[j]);
                    rsuCameras[j].removeObjectFromObserved(this.transform);
                }
            }
            


            // select box color
            if (seenObjectsByLidar.Count >= BlindScenarioManager.Instance.GetBusLidarThreshold() ||
                seenObjectsCameras.Count >= BlindScenarioManager.Instance.GetBusCameraThreshold())
            {
                float notSeenPercentage = ((float)points.Count - seenObjectsByLidar.Count) / points.Count;
                // setColor(1 - notSeenPercentage, BoxState.Green);
                setColor(1, BoxState.Green);
                // Debug.Log("now is green");
            }
            else if (seenObjectsByRSU.Count >= BlindScenarioManager.Instance.GetRsuThreshold() ||
                     seenObjectsByRsuCameras.Count >= BlindScenarioManager.Instance.GetRsuCameraThreshold())
            {
                // float notSeenPercentage = ((float)points.Count - seenObjectsByRSU.Count) / points.Count;
                // setColor(1 - notSeenPercentage, BoxState.Yellow);
                setColor(1, BoxState.Yellow);
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

    public BoxState GetCarBoxState()
    {
        return this.boxState;
    }

    public List<MockSensor> getObservableSensors()
    {
        return observableSensor;
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
            case BoxState.Yellow:
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
        Yellow,
        Green,
        Red
    }
}