using System;
using System.Collections.Generic;
using System.Numerics;
using AWSIM;
using ROS2;
using UnityEngine;
// using CooperativeObjectInfoMessage = dm_cooperative_msgs.msg.CooperativeObjectInfoMessage;
using Environment = AWSIM.Environment;
using dm_object_info_msgs.msg;
using unique_identifier_msgs.msg;
using Unity.VisualScripting;
using UnityEngine.Serialization;
using Coordinate = CoordinateSharp.Coordinate;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using UnityEngine;
using System.Collections;

public class ObjectInfo : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 10;

    [SerializeField] private bool isGroundTruth = false;

    // [SerializeField] private ulong stationID = 24;

    [FormerlySerializedAs("Topic")] public string topic = "/v2x/cooperative";

    [FormerlySerializedAs("TopicGroundTruth")]
    public string topicGroundTruth = "/v2x/cooperativeGroundTruth";

    public string rsuId = "0x1100";
    // public byte sensorId = 1;


    public string frameId = "obj";

    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";

    [Header("Delayed message")] [SerializeField]
    private List<MessageDelay<ObjectInfoArray>> messageDelays;


    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise dimensionNoise;
    private NoiseSetting.Noise probabilityNoise;


    // Dictionary to store UUIDs (16-byte array) and their associated integers
    private Dictionary<string, int> uuidDictionary = new Dictionary<string, int>();

    // Random number generator
    private System.Random random = new System.Random();

    [SerializeField] private static string Z_utm = "54S";

    [SerializeField] private static float E_utm = 404542.530f;

    [SerializeField] private static float N_utm = 3972957.923f;


    private Vector3 utm_vector;


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
        utm_vector = new Vector3(E_utm, N_utm, 0);
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
        objectPublisher = SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(topic, qos);
        // objectPublisherGroundTruth =
        //     SimulatorROS2Node.CreatePublisher<CooperativeObjectInfoMessage>(TopicGroundTruth, qos);
        objectPublisherGroundTruth =
            SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(topicGroundTruth, qos);


        // log state of sensor
        // Transform firstSensor = this.sensors[0].transform;
        // Vector3 pos = ROS2Utility.UnityToRosPosition(firstSensor.position);
        // pos = pos + Environment.Instance.MgrsOffsetPosition;
        // Quaternion rot = ROS2Utility.UnityToRosRotation(firstSensor.rotation);

        // Debug.Log($"{firstSensor.name} RSU ***|");
        // Debug.Log(pos);
        // Debug.Log(rot);
        // Debug.Log($"{rot.x}, {rot.y},{rot.z} ,{rot.w}");
        // Debug.Log(firstSensor.rotation.eulerAngles.y);
        Debug.Log("NEW test is the best");
        Debug.Log(this.transform.rotation.eulerAngles);


        InitializeDelaySystem();
    }

    // private void CheckMockSensors()
    // {
    //     // Collect all sightings across sensors, grouped by object instance ID
    //     var sightings = new Dictionary<int, List<Transform>>();
    //
    //     for (int i = 0; i < sensors.Count; i++)
    //     {
    //         List<Transform> seenObjects = sensors[i].GetSeenObjects();
    //         for (int j = 0; j < seenObjects.Count; j++)
    //         {
    //             var t = seenObjects[j];
    //             if (t == null) continue;
    //
    //             int id = t.GetInstanceID();
    //             if (!sightings.TryGetValue(id, out var bucket))
    //             {
    //                 bucket = new List<Transform>();
    //                 sightings[id] = bucket;
    //             }
    //             bucket.Add(t); // keep all sightings; we’ll emit once per key later
    //         }
    //     }
    //
    //     // Build outgoing messages once per unique object (per instance ID)
    //     List<dm_object_info_msgs.msg.ObjectInfo> objectInfos = new List<dm_object_info_msgs.msg.ObjectInfo>();
    //     List<dm_object_info_msgs.msg.ObjectInfo> objectInfosGroundTruth = new List<dm_object_info_msgs.msg.ObjectInfo>();
    //
    //     foreach (var kvp in sightings)
    //     {
    //         // If you ever need per-object metadata (e.g., "how many sensors saw this"),
    //         // kvp.Value.Count gives you that.
    //         Transform representative = kvp.Value[0];
    //
    //         objectInfos.Add(handlObjectInfo(representative, true));
    //         objectInfosGroundTruth.Add(handlObjectInfo(representative, false));
    //     }
    //
    //     msg.Array = objectInfos.ToArray();
    //     msgGroundTruth.Array = objectInfosGroundTruth.ToArray();
    //
    //     objectPublisher.Publish(msg);
    //
    //     if (!string.IsNullOrEmpty(TopicGroundTruth) && !TopicGroundTruth.Equals("None"))
    //     {
    //         objectPublisherGroundTruth.Publish(msgGroundTruth);
    //     }
    // }


    private void CheckMockSensors()
    {
        // Group sightings by unique instance ID and keep:
        // - a representative Transform for the object
        // - the list of sensors that saw it
        var sightings = new Dictionary<Transform, List<MockSensor>>();

        for (int i = 0; i < sensors.Count; i++)
        {
            var sensor = sensors[i];
            List<Transform> seenObjects = sensor.GetSeenObjects();

            for (int j = 0; j < seenObjects.Count; j++)
            {
                Transform t = seenObjects[j];
                if (t == null) continue;

                // Use the Transform itself as the key
                if (!sightings.TryGetValue(t, out var watchers))
                {
                    watchers = new List<MockSensor>();
                    sightings[t] = watchers;
                }

                // Ensure each sensor is recorded once for this object
                if (!watchers.Contains(sensor))
                {
                    watchers.Add(sensor);
                }
            }
        }

        // Build outgoing messages once per unique object
        var objectInfos = new List<dm_object_info_msgs.msg.ObjectInfo>();
        var objectInfosGroundTruth = new List<dm_object_info_msgs.msg.ObjectInfo>();

        foreach (var kvp in sightings)
        {
            Transform rep = kvp.Key;
            List<MockSensor> watchers = kvp.Value;

            objectInfos.Add(handlObjectInfo(rep, watchers, true));
            objectInfosGroundTruth.Add(handlObjectInfo(rep, watchers, false));
        }


        msg.Array = objectInfos.ToArray();
        msgGroundTruth.Array = objectInfosGroundTruth.ToArray();

        objectPublisher.Publish(msg);


        // Publish GT only when TopicGroundTruth is not null/empty and not the sentinel "None"
        if (!string.IsNullOrEmpty(topicGroundTruth) && !topicGroundTruth.Equals("None"))
        {
            objectPublisherGroundTruth.Publish(msgGroundTruth);
        }

        PublishByDelay(msg, msgGroundTruth);
    }


    private float timer;


    private dm_object_info_msgs.msg.ObjectInfo handlObjectInfo(Transform seenObject, List<MockSensor> sensors,
        bool byNoise = true)
    {
        dm_object_info_msgs.msg.ObjectInfo objectInfo = new dm_object_info_msgs.msg.ObjectInfo();


        //position 
        var pos_pure = ROS2Utility.UnityToRosPosition(seenObject.transform.position);
        // pos = pos + Environment.Instance.MgrsOffsetPosition;

        //add noise

        var pos = byNoise ? positionNoise.ApplyNoiseOnVector(pos_pure) : pos_pure;


        Vector3 pos_utm = pos + utm_vector;
        (double lat, double lon) = GeographicLib.UTMUPS.Reverse(54, true, pos_utm.x, pos_utm.y);


        // objectInfo.Object_location.Latitude.Value = float.Parse(lat);
        objectInfo.Object_location.Latitude.Value = (int)(lat * 10000000);

        // objectInfo.Object_location.Longitude.Value = float.Parse(longitude);
        objectInfo.Object_location.Longitude.Value = (int)(lon * 10000000);

        objectInfo.Object_location.Altitude.Value = (int)((pos.z + Environment.Instance.MgrsOffsetPosition.z) * 100);
        // Debug.Log($"the altitude reported {objectInfo.Object_location.Altitude.Value}");


        // objectInfo.Object_location.Geodetic_system.Value = 4326;
        objectInfo.Object_location.Geodetic_srid.Value = 4326;

        //Existancy
        objectInfo.Existency.Value =
            (byte)((int)(byNoise ? (probabilityNoise.ApplyNoiseOnFloat(0.8f) * 101) : 1f * 101));


        //Orientation
        float rotation = CalculateAngleFromNorth(seenObject.transform);
        objectInfo.Orientation.Value.Value = (ushort)(rotation * 80);

        // Debug.Log($"Rotation: {rotation} ***");
        // Debug.Log("ss1");
        //type 
        NPCVehicle npcVehicle = seenObject.GetComponent<NPCVehicle>();
        LineOfSight lineOfSight = seenObject.GetComponent<LineOfSight>();


        //Id setup
        UUID uuid = lineOfSight.GetUUID();
        int generatedId = GetIntForUUID(uuid.Uuid);
        objectInfo.Id.Value = (ulong)generatedId;

        //get rigidbody
        Rigidbody rigidbody = seenObject.GetComponent<Rigidbody>();

        //direction
        // // ---- direction in the horizontal plane ----
        Vector3 vel = rigidbody.linearVelocity; // current velocity
        // Vector3 vel = seenObject.forward; // current velocity  just for test
        Vector3 dir = new Vector3(vel.x, 0f, vel.z); // ignore vertical component

        float direction = 0;

        if (dir.sqrMagnitude > 0.0001f) // small threshold to skip “almost-stopped”
        {
            dir.Normalize();

            // azimuth (°) clockwise from north (+Z axis)
            direction = Vector3.SignedAngle(Vector3.forward, dir, Vector3.up);
            direction = CalculateAngleFromNorth(direction);
            // Debug.Log($"Azimuth: {direction:0.0}°");
        }
        else
        {
            //not moving object
            direction = rotation;
        }

        objectInfo.Direction.Value.Value = (ushort)(direction * 80);

        // Debug.Log("ss2");
        objectInfo.Object_class = new ObjectClass[1];


        for (int i = 0; i < objectInfo.Object_class.Length; i++)
        {
            ObjectClass objectClass = new ObjectClass();
            objectClass.Confidence.Value = 0;
            objectClass.Id.Value = (byte)i;
            objectInfo.Object_class[i] = objectClass;
        }


        objectInfo.Time = new TimestampIts();
        objectInfo.Time.Value = GetITSTimeInMilliseconds(sensors[0].GetLastUpdateTime());


        objectInfo.Information_source_list = new ObjectId[sensors.Count];
        for (int i = 0; i < sensors.Count; i++)
        {
            ObjectId sourceInfoObjectIdInstance = new ObjectId();
            sourceInfoObjectIdInstance.Value = GenerateObjectId(rsuId, sensors[i].GetSensorId(), (ushort)generatedId);
            objectInfo.Information_source_list[i] = sourceInfoObjectIdInstance;
        }


        ObjectId objectIdInstance = new ObjectId();
        objectIdInstance.Value = objectInfo.Information_source_list[0].Value; // we would send first input 
        objectInfo.Id.Value = objectIdInstance.Value;


        // Debug.Log($"object info size : {objectInfo.Information_source_list.Length} , value is {objectInfo.Id.Value}");


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
            objectClassTarget.Confidence.Value = 101;


            objectInfo.Object_class[0] = objectClassTarget;


            //setup velocity
            objectInfo.Speed.Value.Value = (short)(Vector3.Magnitude(rigidbody.linearVelocity) * 100);
        }
        else
        {
            ObjectClass objectClassTarget = new ObjectClass();
            objectClassTarget.Id.Value = 2;
            objectClassTarget.Confidence.Value = 101;


            objectInfo.Object_class[0] = objectClassTarget;


            objectInfo.Size.Length.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Width.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(1.7f * 100);

            //setup velocity
            // Debug.Log(seenObject.transform.name);
            objectInfo.Speed.Value.Value = (short)(seenObject.GetComponent<ISpeed>().GetSpeed() * 100);

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


    // Calculate the Y angle for ROS rotation style
    public float YawAngluarRotationRosSystem(Transform target)
    {
        return -((target.rotation.eulerAngles.y) % 360);
    }

    // Calculate the Y angle from North for Azimuth
    public float CalculateAngleFromNorth(Transform target)
    {
        return (target.rotation.eulerAngles.y + 90) % 360;
    }

    public float CalculateAngleFromNorth(float target)
    {
        return (target + 90) % 360;
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

    public static ulong GetITSTimeInMilliseconds(long unixTimeMillis)
    {
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


    public ulong GenerateObjectId(string rsuIdHex, byte sensorId, ushort detectedObjectId)
    {
        // Convert RSU ID from hex string to uint
        uint rsuId = Convert.ToUInt32(rsuIdHex, 16);

        // Ensure valid sensor ID range (0-255)
        if (sensorId > 255)
        {
            Debug.LogError($"Sensor ID out of range: {sensorId}");
            return 0;
        }

        // Object ID construction
        ulong objectId = 0;


        if (isGroundTruth)
        {
            // Bits 63-62: "01" (Recognition by Cooperative Roadside Equipment)
            objectId |= (1UL << 63); // Set bit 63 to 1 (0b10 in bits 63-62)
        }
        else
        {
            // Bits 63-62: "01" (Recognition by Cooperative Roadside Equipment)
            objectId |= (1UL << 62); // Set bit 62 to 1 (0b01 in bits 63-62)
        }


        // Bits 61-56: All "0" (Reserved, already zero-initialized)

        // Bits 55-48: Sensor ID
        objectId |= ((ulong)sensorId << 48);

        // Bits 47-32: Detected Object ID
        objectId |= ((ulong)detectedObjectId << 32);

        // Bits 31-0: RSU ID (converted from hex string to uint)
        objectId |= rsuId;

        // Debugging Output in Unity Console
        // Debug.Log($"Generated Object ID (Hex): {objectId:X16}");

        // Convert ulong to a byte array (little-endian by default)
        byte[] objectIdBytes = BitConverter.GetBytes(objectId);

        // Convert byte array to BitArray
        BitArray bitArray = new BitArray(objectIdBytes);

        // Reverse the bit order
        BitArray reversedBits = new BitArray(bitArray.Length);
        for (int i = 0; i < bitArray.Length; i++)
        {
            reversedBits[bitArray.Length - 1 - i] = bitArray[i]; // Reverse the index
        }

        // Convert BitArray to a string for debugging
        string binaryString = "";
        for (int i = 0; i < reversedBits.Length; i++)
        {
            binaryString += reversedBits[i] ? "1" : "0";
            if ((i + 1) % 8 == 0) binaryString += " "; // Space every 8 bits
        }

        // Debug.Log($"Original Bytes (Hex): {BitConverter.ToString(objectIdBytes).Replace("-", " ")}");
        // Debug.Log($"BitArray (Binary): {binaryString}");
        // Debug.Log($"rsuId: {rsuIdHex}, sensorId: {sensorId}, detectedObjectId: {detectedObjectId}");
        // Debug.Log($"Final : {objectId:X16}");
        // Debug.Log($"Final  {objectId}");

        return objectId;
    }


    // Delay System
    private void InitializeDelaySystem()
    {
        var qos = QosSettings.GetQoSProfile();
        //handle general delays
        var generalDelays = NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs();
        for (int i = 0; i < generalDelays.Count; i++)
        {
            var md = generalDelays[i]; // COPY
            var baseTopic = md.delayConfig.isGroundTruth ? topicGroundTruth : topic;

            MessageDelay<ObjectInfoArray> messageDelayConfig = new MessageDelay<ObjectInfoArray>(md.delayConfig);


            messageDelayConfig.SetIPublisher(SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(
                baseTopic + md.GetTopicName(), qos));

            messageDelays.Add(messageDelayConfig); // <- IMPORTANT: write back the mutated struct
        }


        //handle custom delays
        for (int i = 0; i < messageDelays.Count; i++)
        {
            var md = messageDelays[i]; // COPY
            var baseTopic = md.delayConfig.isGroundTruth ? topicGroundTruth : topic;
            md.SetIPublisher(SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(
                baseTopic + md.delayConfig.topicName, qos));

            messageDelays.Add(md); // <- IMPORTANT: write back the mutated struct
        }
    }

    private void PublishByDelay(ObjectInfoArray message, ObjectInfoArray messageGroundTruth)
    {
        for (int i = 0; i < messageDelays.Count; i++)
        {
            var md = messageDelays[i]; // (optional) local copy for clarity

            if (md.delayConfig.isGroundTruth)
            {
                NetworkSimulator.Instance.PublishLate(md, Clone(messageGroundTruth));
            }
            else
            {
                NetworkSimulator.Instance.PublishLate(md, Clone(message));
            }
        }
    }


    // private static ObjectInfoArray Clone(ObjectInfoArray source)
    // {
    //     if (source == null) return null;
    //
    //     // Push the managed state to the native buffer (allocates/updates source._handle)
    //     source.WriteNativeMessage();
    //
    //     // Read back from the native buffer into a brand-new managed instance
    //     var copy = new ObjectInfoArray();
    //     copy.ReadNativeMessage(source.Handle);
    //
    //     return copy;
    // }

    // private static ObjectInfoArray Clone(ObjectInfoArray source)
    // {
    //     if (source == null) return null;
    //
    //     // Create a temporary native message to serialize INTO (not the source’s handle).
    //     var tmpNative = new ObjectInfoArray();
    //     try
    //     {
    //         // Serialize the source into tmpNative’s native struct
    //         source.WriteNativeMessage(tmpNative.Handle);
    //
    //         // Read back into a brand-new managed instance
    //         var copy = new ObjectInfoArray();
    //         copy.ReadNativeMessage(tmpNative.Handle);
    //         return copy;
    //     }
    //     finally
    //     {
    //         // Free the temporary native buffer immediately
    //         tmpNative.Dispose();
    //     }
    // }

    private static ObjectInfoArray Clone(ObjectInfoArray source)
    {
        if (source == null) return null;

        // Create a temporary native message to serialize INTO (not the source’s handle).
        var tmpNative = new ObjectInfoArray();
        tmpNative.Array = source.Array;
        return tmpNative;
    }
}