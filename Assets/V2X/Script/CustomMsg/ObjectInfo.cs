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

public class ObjectInfo : MonoBehaviour
{
    [SerializeField] private List<DetectionSensor> sensors;

    [Tooltip("Also publish the objects seen by the sensors of these ObjectInfo components (one message for several RSUs). " +
             "Each object keeps the RSU ID of the publisher whose sensor saw it.")]
    [SerializeField] private List<ObjectInfo> sources = new List<ObjectInfo>();

    // Sensors of one RSU, identified by its RSU ID.
    [Serializable]
    public class RsuSensors
    {
        public string rsuId = "0x12020001";
        public List<DetectionSensor> sensors = new List<DetectionSensor>();
    }

    [Tooltip("Sensors of further RSUs that are published only in this message (they have no ObjectInfo of their own).")]
    [SerializeField] private List<RsuSensors> rsuSensors = new List<RsuSensors>();

    [Tooltip("Publishing rate (messages per second).")]
    [SerializeField] private float Hz = 10;

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
    private bool enableDelayedMessages = true;
    [SerializeField]private List<MessageDelayConfig> messageDelaysConfigs = new List<MessageDelayConfig>();
    private bool useGeneralDelays = true;     // false for publishers created from config: they list their own profiles
    private List<MessageDelay<ObjectInfoArray>> messageDelays = new List<MessageDelay<ObjectInfoArray>>();


    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise dimensionNoise;
    private NoiseSetting.Noise probabilityNoise;


    // Object numbers (bits 47-32 of the object ID): a random number per object, shared by every ObjectInfo so the
    // same object has the same number on every topic. A number is never given to two objects.
    private static readonly Dictionary<string, ushort> objectNumbers = new Dictionary<string, ushort>();
    private static readonly HashSet<ushort> usedObjectNumbers = new HashSet<ushort>();
    private static readonly System.Random objectNumberRandom = new System.Random();

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetObjectNumbers()
    {
        objectNumbers.Clear();
        usedObjectNumbers.Clear();
    }

    // A sensor together with the RSU ID it reports under.
    private struct Watcher
    {
        public DetectionSensor sensor;
        public uint rsu;
    }

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

    /// <summary>
    /// Sets up a publisher created from config (call while the GameObject is inactive, before Awake).
    /// Network copies come from <paramref name="delays"/> only; the general NetworkSimulator profiles are not used.
    /// </summary>
    public void Configure(string rsuIdHex, List<DetectionSensor> ownSensors, List<RsuSensors> otherRsus, float hz,
        string topicName, string groundTruthTopic, string positionNoise, string rotationNoise, string dimensionNoise,
        string probabilityNoise, List<MessageDelayConfig> delays)
    {
        rsuId = rsuIdHex;
        sensors = ownSensors ?? new List<DetectionSensor>();
        sources = new List<ObjectInfo>();
        rsuSensors = otherRsus ?? new List<RsuSensors>();
        Hz = hz;
        topic = topicName;
        topicGroundTruth = string.IsNullOrEmpty(groundTruthTopic) ? "None" : groundTruthTopic;
        positionNoiseName = positionNoise;
        rotationNoiseName = rotationNoise;
        dimensionNoiseName = dimensionNoise;
        probabilityNoiseName = probabilityNoise;
        messageDelaysConfigs = delays ?? new List<MessageDelayConfig>();
        enableDelayedMessages = messageDelaysConfigs.Count > 0;
        useGeneralDelays = false;
    }

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
        // Debug.Log("NEW test is the best");
        // Debug.Log(this.transform.rotation.eulerAngles);


        if (enableDelayedMessages)
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


    // Sensors of this component and of the source components, each with its RSU ID.
    private List<Watcher> CollectWatchers()
    {
        var watchers = new List<Watcher>();
        void Add(string rsuId, List<DetectionSensor> list)
        {
            if (list == null) return;
            uint rsu = Convert.ToUInt32(rsuId, 16);
            foreach (var sensor in list)
                if (sensor != null && !watchers.Exists(w => w.sensor == sensor && w.rsu == rsu))
                    watchers.Add(new Watcher { sensor = sensor, rsu = rsu });
        }
        Add(rsuId, sensors);
        foreach (var source in sources)
            if (source != null && source != this)
                Add(source.rsuId, source.sensors);
        foreach (var group in rsuSensors)
            if (group != null)
                Add(group.rsuId, group.sensors);
        return watchers;
    }

    private void CheckMockSensors()
    {
        // Every object once, with the sensors that saw it.
        var sightings = new Dictionary<Transform, List<Watcher>>();
        foreach (var watcher in CollectWatchers())
        {
            foreach (Transform t in watcher.sensor.GetSeenObjects())
            {
                if (t == null) continue;
                if (!sightings.TryGetValue(t, out var list))
                    sightings[t] = list = new List<Watcher>();
                list.Add(watcher);
            }
        }

        // Build outgoing messages once per unique object
        var objectInfos = new List<dm_object_info_msgs.msg.ObjectInfo>();
        var objectInfosGroundTruth = new List<dm_object_info_msgs.msg.ObjectInfo>();

        foreach (var kvp in sightings)
        {
            Transform rep = kvp.Key;
            if (rep.tag.Equals("Ego"))
            {
                continue; // it is the ego car
            }
            // Fixed order (RSU ID, then sensor ID), so the object's id does not depend on which sensor saw it first.
            List<Watcher> watchers = kvp.Value;
            watchers.Sort((a, b) => a.rsu != b.rsu ? a.rsu.CompareTo(b.rsu) : a.sensor.GetSensorId().CompareTo(b.sensor.GetSensorId()));
            objectInfos.Add(HandlObjectInfo(rep, watchers, true));
            objectInfosGroundTruth.Add(HandlObjectInfo(rep, watchers, false));
        }


        msg.Array = objectInfos.ToArray();
        msgGroundTruth.Array = objectInfosGroundTruth.ToArray();

        objectPublisher.Publish(msg);


        // Publish GT only when TopicGroundTruth is not null/empty and not the sentinel "None"
        if (!string.IsNullOrEmpty(topicGroundTruth) && !topicGroundTruth.Equals("None"))
        {
            objectPublisherGroundTruth.Publish(msgGroundTruth);
        }
        
        if (enableDelayedMessages) 
            PublishByDelay(msg, msgGroundTruth);
    }


    private float timer;


    private dm_object_info_msgs.msg.ObjectInfo HandlObjectInfo(Transform seenObject, List<Watcher> watchers,
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
            // with noise: 0.8 ± noise, kept inside 0-100 (a noisy value must not reach 101 or wrap below 0)
            (byte)(byNoise ? Mathf.Clamp((int)(probabilityNoise.ApplyNoiseOnFloat(0.8f) * 101), 0, 100) : 101);


        //Orientation
        float rotation = CalculateAngleFromNorth(seenObject.transform);
        objectInfo.Orientation.Value.Value = (ushort)(rotation * 80);

        NPCVehicle npcVehicle = seenObject.GetComponent<NPCVehicle>();
        LineOfSight lineOfSight = seenObject.GetComponent<LineOfSight>();


        //Id setup
        UUID uuid = lineOfSight.GetUUID();
        ushort objectNumber = GetObjectNumber(uuid.Uuid);

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
        objectInfo.Time.Value = GetITSTimeInMilliseconds(watchers[0].sensor.GetLastUpdateTime());


        objectInfo.Information_source_list = new ObjectId[watchers.Count];
        for (int i = 0; i < watchers.Count; i++)
        {
            ObjectId sourceInfoObjectIdInstance = new ObjectId();
            sourceInfoObjectIdInstance.Value = GenerateObjectId(watchers[i].rsu, watchers[i].sensor.GetSensorId(), objectNumber);
            objectInfo.Information_source_list[i] = sourceInfoObjectIdInstance;
        }

        // id = the ID given by the first sensor (lowest RSU ID, then lowest sensor ID)
        objectInfo.Id.Value = objectInfo.Information_source_list[0].Value;


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


        var interval = 1.0f / Hz;
        if (timer + 0.00001f < interval)
            return;

        timer = Mathf.Min(timer - interval, interval);   // keep the remainder so the average rate is Hz

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


    // Object number of the object with this UUID; a new object gets a random number (1 … 65535) not used before.
    private static ushort GetObjectNumber(byte[] uuid)
    {
        string uuidKey = BitConverter.ToString(uuid);
        if (objectNumbers.TryGetValue(uuidKey, out var number))
            return number;

        if (usedObjectNumbers.Count >= ushort.MaxValue)     // all numbers used: start over
        {
            objectNumbers.Clear();
            usedObjectNumbers.Clear();
        }
        do number = (ushort)objectNumberRandom.Next(1, ushort.MaxValue + 1);
        while (!usedObjectNumbers.Add(number));
        objectNumbers[uuidKey] = number;
        return number;
    }


    // Object ID of a roadside-recognised object (spec 3.3.3):
    // bits 63-62 "10", 61-56 reserved, 55-48 sensor ID, 47-32 object number, 31-0 RSU ID.
    public static ulong GenerateObjectId(uint rsuId, byte sensorId, ushort objectNumber)
    {
        ulong objectId = 1UL << 63;
        objectId |= (ulong)sensorId << 48;
        objectId |= (ulong)objectNumber << 32;
        objectId |= rsuId;
        return objectId;
    }


    // Delay System
    private void InitializeDelaySystem()
    {
        var qos = QosSettings.GetQoSProfile();
        //handle general delays
        var generalDelays = useGeneralDelays ? NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs() : new List<MessageDelay<ROS2.Message>>();
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
        for (int i = 0; i < messageDelaysConfigs.Count; i++)
        {
            var md = messageDelaysConfigs[i]; // COPY
            var baseTopic = md.isGroundTruth ? topicGroundTruth : topic;
            
            MessageDelay<ObjectInfoArray> messageDelayConfig = new MessageDelay<ObjectInfoArray>(md);
            messageDelayConfig.SetIPublisher(SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(
                baseTopic + md.topicName, qos));

            messageDelays.Add(messageDelayConfig); // <- IMPORTANT: write back the mutated struct
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