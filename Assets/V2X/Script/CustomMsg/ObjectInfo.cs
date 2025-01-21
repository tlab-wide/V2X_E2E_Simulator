using System;
using System.Collections.Generic;
using System.Numerics;
using AWSIM;
using ROS2;
using UnityEngine;
using CooperativeObjectInfoMessage = dm_cooperative_msgs.msg.CooperativeObjectInfoMessage;
using Environment = AWSIM.Environment;
using dm_object_info_msgs.msg;
using unique_identifier_msgs.msg;
using Unity.VisualScripting;
using UnityEngine.Serialization;
using Coordinate = CoordinateSharp.Coordinate;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class ObjectInfo : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 1;

    // [SerializeField] private ulong stationID = 24;

    public string Topic = "/v2x/cooperative";
    public string TopicGroundTruth = "/v2x/cooperativeGroundTruth";
    public string rsuId = "0x1100";
    public byte sensorId = 1;


    public string frameId = "obj";

    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";
    [SerializeField] private byte test;


    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise dimensionNoise;
    private NoiseSetting.Noise probabilityNoise;


    // Dictionary to store UUIDs (16-byte array) and their associated integers
    private Dictionary<string, int> uuidDictionary = new Dictionary<string, int>();

    // Random number generator
    private System.Random random = new System.Random();


    public QoSSettings QosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 1,
    };

    // IPublisher<CooperativeObjectInfoMessage> objectPublisher;
    // IPublisher<CooperativeObjectInfoMessage> objectPublisherGroundTruth;

    IPublisher<ObjectInfoArray> objectPublisher;
    IPublisher<ObjectInfoArray> objectPublisherGroundTruth;


    // private CooperativeObjectInfoMessage msg;
    // private CooperativeObjectInfoMessage msgGroundTruth;

    private ObjectInfoArray msg;
    private ObjectInfoArray msgGroundTruth;

    // Start is called before the first frame update
    void Awake()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);


        // msg = new CooperativeObjectInfoMessage();
        // msgGroundTruth = new CooperativeObjectInfoMessage();

        msg = new ObjectInfoArray();
        msgGroundTruth = new ObjectInfoArray();


        // Create publisher.
        var qos = QosSettings.GetQoSProfile();
        // objectPublisher = SimulatorROS2Node.CreatePublisher<CooperativeObjectInfoMessage>(Topic, qos);
        objectPublisher = SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(Topic, qos);
        // objectPublisherGroundTruth =
        //     SimulatorROS2Node.CreatePublisher<CooperativeObjectInfoMessage>(TopicGroundTruth, qos);
        objectPublisherGroundTruth =
            SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(TopicGroundTruth, qos);


        // log state of sensor
        Transform firstSensor = this.sensors[0].transform;
        Vector3 pos = ROS2Utility.UnityToRosPosition(firstSensor.position);
        pos = pos + Environment.Instance.MgrsOffsetPosition;
        Quaternion rot = ROS2Utility.UnityToRosRotation(firstSensor.rotation);

        // Debug.Log($"{firstSensor.name} RSU ***|");
        // Debug.Log(pos);
        // Debug.Log(rot);
        // Debug.Log($"{rot.x}, {rot.y},{rot.z} ,{rot.w}");
        // Debug.Log(firstSensor.rotation.eulerAngles.y);
        
        
    }

    private void CheckMockSensors()
    {
        // ObjectInfoArray objectInfoArray = new ObjectInfoArray();
        //
        // objectInfoArray.Array = objectInfos.ToArray();
        // List<PredictedObject> predictedObjects = new List<PredictedObject>();
        List<dm_object_info_msgs.msg.ObjectInfo> objectInfos = new List<dm_object_info_msgs.msg.ObjectInfo>();
        List<dm_object_info_msgs.msg.ObjectInfo>
            objectInfosGroundTruth = new List<dm_object_info_msgs.msg.ObjectInfo>();
        List<Transform> haveSeen = new List<Transform>();


        for (int i = 0; i < sensors.Count; i++)
        {
            List<Transform> seenObjects = sensors[i].GetSeenObjects();

            for (int j = 0; j < seenObjects.Count; j++)
            {
                // remove duplications
                if (haveSeen.Contains(seenObjects[j]))
                {
                    continue;
                }

                haveSeen.Add(seenObjects[j]);
                //add to list

                objectInfos.Add(handlObjectInfo(seenObjects[j], true));
                objectInfosGroundTruth.Add(handlObjectInfo(seenObjects[j], false));
            }
        }



        msg.Array = objectInfos.ToArray();
        
        msgGroundTruth.Array = objectInfosGroundTruth.ToArray();
      
        
        objectPublisher.Publish(msg);

        if (!TopicGroundTruth.Equals("None"))
        {
            objectPublisherGroundTruth.Publish(msgGroundTruth);
        }
        //todo prediction for RSU 
    }

    private float timer;


    private dm_object_info_msgs.msg.ObjectInfo handlObjectInfo(Transform seenObject, bool byNoise = true)
    {
        dm_object_info_msgs.msg.ObjectInfo objectInfo = new dm_object_info_msgs.msg.ObjectInfo();


        //position 
        var pos = ROS2Utility.UnityToRosPosition(seenObject.transform.position);
        pos = pos + Environment.Instance.MgrsOffsetPosition;


        //add noise

        pos = byNoise ? positionNoise.ApplyNoiseOnVector(pos) : pos;

        string latLong = ConvertToLatLong(pos.x, pos.y);

        string lat = latLong.Split(' ')[0];
        string longitude = latLong.Split(' ')[1];
        // objectInfo.Object_location.Latitude.Value = float.Parse(lat);
        objectInfo.Object_location.Latitude.Value = (int)(float.Parse(lat) * 10000000);

        // objectInfo.Object_location.Longitude.Value = float.Parse(longitude);
        objectInfo.Object_location.Longitude.Value = (int)(float.Parse(longitude) * 10000000);

        objectInfo.Object_location.Altitude.Value = (int)(pos.z * 100);

        // objectInfo.Object_location.Geodetic_system.Value = 4326;
        objectInfo.Object_location.Geodetic_srid.Value = 4326;

        //Existancy
        objectInfo.Existency.Value =
            (byte)((int)(byNoise ? (probabilityNoise.ApplyNoiseOnFloat(0.8f) * 101) : 0.8f * 101));


        //Rotation
        float rotation = CalculateAngleFromNorth(seenObject.transform);
        objectInfo.Direction.Value.Value = (ushort)(rotation * 80);


        // Debug.Log("ss1");
        //type 
        NPCVehicle npcVehicle = seenObject.GetComponent<NPCVehicle>();
        LineOfSight lineOfSight = seenObject.GetComponent<LineOfSight>();


        //Id setup
        UUID uuid = lineOfSight.GetUUID();
        int generated_id = GetIntForUUID(uuid.Uuid);
        objectInfo.Id.Value = (ulong)generated_id;

        //speed setup 
        Rigidbody rigidbody = seenObject.GetComponent<Rigidbody>();

        


        // Debug.Log("ss2");
        objectInfo.Object_class = new ObjectClass[5];

        for (int i = 0; i < objectInfo.Object_class.Length; i++)
        {
            ObjectClass objectClass = new ObjectClass();
            objectClass.Confidence.Value = 0;
            objectClass.Id.Value = (byte)i;
            objectInfo.Object_class[i] = objectClass;
        }

        // objectInfo.Time TODO 
        objectInfo.Time = new TimestampIts();
        objectInfo.Time.Value = GetITSTimeInMilliseconds();


        ObjectId objectIdSource = new ObjectId();
        objectIdSource.Value = GenerateObjectId(rsuId, sensorId, (ushort)generated_id);

        objectInfo.Information_source_list = new[] { objectIdSource };

        
        //based on msg document
        // public const byte UNKNOWN = 0;
        // public const byte VEHICLE = 1;
        // public const byte PERSON = 2;
        // public const byte ANIMAL = 3;
        // public const byte OTHER = 4;
        
        if (npcVehicle != null)
        {
            //add noise
            Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,
                npcVehicle.Bounds.extents.y * 2, npcVehicle.Bounds.extents.z * 2);
            dimensions = byNoise ? dimensionNoise.ApplyNoiseOnVector(dimensions) : dimensions;

            objectInfo.Size.Length.Value.Value = (ushort)(dimensions.z * 100);
            objectInfo.Size.Width.Value.Value = (ushort)(dimensions.x * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(dimensions.y * 100);


            ObjectClass objectClassTarget = new ObjectClass();
            objectClassTarget.Id.Value = 1;
            objectClassTarget.Confidence.Value = 100;

            objectInfo.Object_class[1] = objectClassTarget;
            
            
            //setup velocity
            objectInfo.Speed.Value.Value = (short)(Vector3.Magnitude(rigidbody.linearVelocity) * 100);
            
            
        }
        else
        {
            ObjectClass objectClassTarget = new ObjectClass();
            objectClassTarget.Id.Value = 2;
            objectClassTarget.Confidence.Value = 100;

            objectInfo.Object_class[2] = objectClassTarget;


            objectInfo.Size.Length.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Width.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(1.7f * 100);
            
            //setup velocity
            objectInfo.Speed.Value.Value = (short) (seenObject.GetComponent<MoveablePedestrian>().GetSpeed() * 100);

            // objectInfo.Object_class = new ObjectClass[1];
            // objectInfo.Object_class[0].Id.Value = (byte)7; // todo type of car has bug
        }


        return objectInfo;
    }

    public void Update()
    {
        timer += Time.deltaTime;
        // timer += Time.fixedDeltaTime;


        var interval = 1.0f / (Hz * 2);
        if (timer + 0.00001f < interval)
            return;

        timer = 0;

        CheckMockSensors();
    }


    private string ConvertToLatLong(float x, float y)
    {
        int xIntValue = (int)x;
        int yIntValue = (int)y;

        double extraEasting = x - xIntValue; // in meters
        double extraNorthing = y - yIntValue; // in meters


        // Example MGRS coordinate
        // string mgrsCoordinate = "54SVE0454272957"; // Replace with your MGRS coordinate
        string mgrsCoordinate = Environment.Instance.MgrsGridZone + xIntValue.ToString("D5") + yIntValue.ToString("D5");

        // Convert MGRS to geographic coordinates (latitude and longitude)
        Coordinate coordFromMGRS = Parse(mgrsCoordinate);

        // Get the current latitude in degrees
        double latitude = coordFromMGRS.Latitude.ToDouble();

        // Calculate the impact of 1 meter movement on latitude and longitude
        double latShiftPerMeter = 1 / 111320.0; // Approx. 1 meter in degrees of latitude
        double longShiftPerMeter =
            1 / (111320.0 *
                 Math.Cos(latitude * Math.PI / 180.0)); // 1 meter in degrees of longitude, adjusted by latitude

        // Calculate the shifts
        double latitudeShift = extraNorthing * latShiftPerMeter;
        double longitudeShift = extraEasting * longShiftPerMeter;

        // Adjust the latitude and longitude
        double adjustedLatitude = latitude + latitudeShift;
        double adjustedLongitude = coordFromMGRS.Longitude.ToDouble() + longitudeShift;

        // Output the result
        return $"{adjustedLatitude} {adjustedLongitude}";
    }


    // I don't know why but i copy it from the main library !!!!!!
    // There is an issue when i import the library based on standard framework that don't 
    // recognise the Parse function therefore we solver the problem in this manner 
    private static Coordinate Parse(string value)
    {
        Coordinate coordinate = (Coordinate)null;
        if (Coordinate.TryParse(value, out coordinate))
            return coordinate;
        throw new FormatException(string.Format("Input Coordinate \"{0}\" was not in a correct format.",
            (object)value));
    }


    // Calculate the Y angle from North
    public float CalculateAngleFromNorth(Transform target)
    {
        // Ensure the angle is within the 0-360 range using Mathf.Repeat
        return (target.rotation.eulerAngles.y + 90) % 360;
    }


    public static ulong GetITSTimeInMilliseconds()
    {
        // Get the current Unix time in milliseconds
        long unixTimeMillis = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        // Debug.Log("time--");
        // Debug.Log(unixTimeMillis);
        unixTimeMillis = unixTimeMillis - 1072882800000;
        // Debug.Log("time++");
        // Debug.Log(unixTimeMillis);
        return (ulong)unixTimeMillis;
    }


    // Function to convert byte array to a string (for use as a dictionary key)
    private string ByteArrayToString(byte[] byteArray)
    {
        return BitConverter.ToString(byteArray).Replace("-", "").ToLower();
    }

    // Function to retrieve an integer for a given UUID
    public int GetIntForUUID(byte[] uuid)
    {
        // Convert the UUID byte array to a string to use as a dictionary key
        string uuidKey = ByteArrayToString(uuid);

        // Check if the UUID is already in the dictionary
        if (uuidDictionary.ContainsKey(uuidKey))
        {
            // Return the existing integer
            return uuidDictionary[uuidKey];
        }
        else
        {
            // Generate a random integer
            int newInt = random.Next(1, int.MaxValue);

            // Store the UUID and its associated integer in the dictionary
            uuidDictionary.Add(uuidKey, newInt);

            // Return the new integer
            return newInt;
        }
    }


    public static ulong GenerateObjectId(string rsuId, byte sensorId, ushort detectedObjectId)
    {
        // Convert the RSU ID from hex to ulong
        uint rsuPart = Convert.ToUInt32(rsuId, 16);

        // Create a 16-byte array to hold the final object ID
        byte[] FinalobjectIdBytes = new byte[8];

        // Insert the RSU part into the first 4 bytes
        BitConverter.GetBytes(rsuPart).CopyTo(FinalobjectIdBytes, 0);

        // Insert the object ID into the next 2 bytes
        BitConverter.GetBytes(detectedObjectId).CopyTo(FinalobjectIdBytes, 4);

        // Insert the sensor ID into the next 1 byte
        BitConverter.GetBytes(sensorId).CopyTo(FinalobjectIdBytes, 6);

        // Ensure the last byte is set to 0x01
        FinalobjectIdBytes[7] = 0x01;

        return BitConverter.ToUInt64(FinalobjectIdBytes, 0);
    }
}