using System;
using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using NUnit.Framework;
using std_msgs.msg;
using unique_identifier_msgs.msg;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;

public class TrackableObject : MonoBehaviour
{
    public static System.Random random = new System.Random();

    [SerializeField] private Transform pointsParent;
    public List<Transform> points;


    // [SerializeField] private Transform cube;
    [SerializeField] private BoxState boxState;
    [SerializeField] public int minimumNumberOfPointsVisible = 2;
    [SerializeField] public float frequency = 2.0f;
    
    [SerializeField] private Transform redBoudingBox;
    [SerializeField] private Transform GreenBoudingBox;
    [SerializeField] private Transform PurpleBoudingBox;


    [SerializeField]private List<MockSensorRaycast> sensorsInZone = new List<MockSensorRaycast>();

    // private Dictionary<MockSensor, int> observableSensorWitCount = new Dictionary<MockSensor, int>();
    private UUID uuid;


    private float waitTimeForUpdate;
    private float passedTime = 0;

    private void Awake()
    {
        passedTime =
            Random.Range(-0.6f,
                0.0f); // this line of code make the update point varient which leads to more smooth updates in run time
        waitTimeForUpdate = 1.0f / frequency;
        Transform[] childs = pointsParent.GetComponentsInChildren<Transform>();

        points = new List<Transform>();

        for (int i = 1; i < childs.Length; i++)
        {
            points.Add(childs[i]);
        }

        uuid = new UUID();
        GenerateRandomBytes(uuid.Uuid);
    }

    public static void GenerateRandomBytes(byte[] uuid)
    {
        byte[] randomBytes = new byte[uuid.Length];

        random.NextBytes(randomBytes); // Fill the byte array with random values

        for (int i = 0; i < uuid.Length; i++)
        {
            uuid[i] = randomBytes[i];
        }
    }

    private void OnEnable()
    {
        setColor( BoxState.Red);
    }


    // Update is called once per frame
    void Update()
    {
        passedTime += Time.deltaTime;
        if (passedTime > waitTimeForUpdate)
        {
            passedTime = 0;

            //bus sensors
            List<MockSensorRaycast> busSensorRaycasts = RayCastManager.Instance.getBusSensors();
            foreach (MockSensorRaycast sensorRaycast in busSensorRaycasts)
            {
                CheckIsInZone(sensorRaycast);
            }

            //Rsu sensors

            List<MockSensorRaycast> RsuSensorRaycasts = RayCastManager.Instance.getRsuSensors();
            foreach (MockSensorRaycast sensorRaycast in RsuSensorRaycasts)
            {
                CheckIsInZone(sensorRaycast);
            }

            
            Debug.Log($"len sensor zone {sensorsInZone.Count}");
            
            bool detectedByBus = false;
            bool detectedByRsu = false;

            foreach (MockSensorRaycast sensor in sensorsInZone)
            {
                Debug.Log("I search for the sensor ");
                if (sensor.CheckIsDetected(this.GetInstanceID()))
                {
                    if (sensor.GetIsRsu())
                    {
                        detectedByRsu = true;
                    }
                    else
                    {
                        detectedByBus = true;
                    }
                }
            }
            
            

            if (detectedByBus)
            {
                // setColor(1, BoxState.Green);
                setColor(BoxState.Green);
            }
            else if (detectedByRsu)
            {
                setColor(BoxState.Purple);
            }
            else
            {
                setColor(BoxState.Red);
            }
        }
    }
    
    


    private List<MockSensorRaycast> SensorsInZoneTemp = new List<MockSensorRaycast>();

    private void CheckIsInZone(MockSensorRaycast sensor)
    {
        if (sensor.gameObject.activeInHierarchy && math.distance(sensor.transform.position, this.transform.position) <
            RayCastManager.Instance.getMaxDistanceForSensorDetection())
        {
            if (!sensorsInZone.Contains(sensor))
            {
                sensorsInZone.Add(sensor);

                sensor.subscribeToSensor(gameObject.GetInstanceID(), this);
            }
        }
        else
        {
            if (sensorsInZone.Contains(sensor))
            {
                sensorsInZone.Remove(sensor);
                sensor.unsubscribeFromSensor(gameObject.GetInstanceID());
            }
        }
    }

    public void setColor(BoxState boxState)
    {
        redBoudingBox.gameObject.SetActive(false);
        PurpleBoudingBox.gameObject.SetActive(false);
        GreenBoudingBox.gameObject.SetActive(false);
        
        switch (boxState)
        {
            case BoxState.Red:
                redBoudingBox.gameObject.SetActive(true);
                break;
            case BoxState.Green:
                GreenBoudingBox.gameObject.SetActive(true);
                break;
            case BoxState.Purple:
                PurpleBoudingBox.gameObject.SetActive(true);
                break;
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
            return ObjectClassification.TRUCK;
        }

        if (carName.IndexOf(bus) >= 0)
        {
            return ObjectClassification.BUS;
        }

        if (carName.IndexOf(car) >= 0 || carName.IndexOf(van) >= 0 || carName.IndexOf(taxi) >= 0 ||
            carName.IndexOf(hatchback) >= 0)
        {
            return ObjectClassification.CAR;
        }
        else
        {
            //todo complete other types, currently in our environment we still do not have require any more
            return ObjectClassification.PEDESTRIAN;
        }
    }

    // public int GetNumberOfDetectedPoint(MockSensor mockSensor)
    // {
    //     return observableSensorWitCount[mockSensor];
    // }


    private void OnDestroy()
    {
        foreach (MockSensorRaycast sensor in SensorsInZoneTemp)
        {
            sensor.unsubscribeFromSensor(gameObject.GetInstanceID());
        }
    }

    public enum BoxState
    {
        Purple,
        Green,
        Red
    }
}