using System;
using System.Collections.Generic;
using System.Text;
using AWSIM;
using ROS2;
using UnityEngine;
using UnityEngine.Serialization;
using dm_object_info_msgs.msg;
using unique_identifier_msgs.msg;
using Coordinate = CoordinateSharp.Coordinate;
using Environment = AWSIM.Environment;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;


// Deprecated
public class ObjectInfoPublisher : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("GroundTruthArea providing objects to publish (no sensors used).")]
    [SerializeField] private GroundTruthArea groundTruthArea;

    [Header("Publish")]
    [SerializeField, Range(1, 50)] private float Hz = 10f;
    [Tooltip("ROS 2 topic to publish (only one topic).")]
    public string topic = "/v2x/cooperative";

    [Header("Flags")]
    [SerializeField] private bool isGroundTruth = false; // kept for ObjectId top bits

    [Header("IDs")]
    public string rsuId = "0x1100";
    [Tooltip("Sensor ID used in Information_source_list when no sensors are present.")]
    [Range(0,255)] public byte sourceSensorId = 0;

    [Header("Coordinate Settings")]
    [SerializeField] private static string Z_utm = "54S";
    [SerializeField] private static float E_utm = 404542.530f;
    [SerializeField] private static float N_utm = 3972957.923f;

    [Header("Noise profiles")]
    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";

    // QoS
    public QoSSettings QosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 1,
    };

    // Internals
    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise dimensionNoise;
    private NoiseSetting.Noise probabilityNoise;

    private Vector3 utm_vector;
    private IPublisher<ObjectInfoArray> objectPublisher;
    private ObjectInfoArray msg;
    private float timer;

    // UUID → int map
    private readonly Dictionary<string, int> uuidDictionary = new Dictionary<string, int>();
    private readonly System.Random random = new System.Random();

    private void Awake()
    {
        if (groundTruthArea == null)
        {
            Debug.LogError("[ObjectInfoPublisher] GroundTruthArea reference is missing.");
        }

        utm_vector = new Vector3(E_utm, N_utm, 0);

        positionNoise   = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise   = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise  = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise= NoiseSetting.Instance.GetNoise(probabilityNoiseName);

        msg = new ObjectInfoArray();

        var qos = QosSettings.GetQoSProfile();
        objectPublisher = SimulatorROS2Node.CreatePublisher<ObjectInfoArray>(topic, qos);
    }

    private void Update()
    {
        timer += Time.deltaTime;
        var period = 1f / Mathf.Max(1f, Hz);
        if (timer + 0.00001f < period) return;
        timer = 0f;

        PublishFromArea();
    }

    private void PublishFromArea()
    {
        if (groundTruthArea == null) return;

        var objectInfos = new List<dm_object_info_msgs.msg.ObjectInfo>();

        foreach (var go in groundTruthArea.GetSeenObjects())
        {
            if (!go) continue;
            var info = BuildObjectInfo(go.transform, byNoise: !isGroundTruth);
            if (info != null) objectInfos.Add(info);
        }

        msg.Array = objectInfos.ToArray();
        objectPublisher.Publish(msg);
    }

    // similar object to HandlObjectInfo 
    private dm_object_info_msgs.msg.ObjectInfo BuildObjectInfo(Transform seenObject, bool byNoise)
    {
        var objectInfo = new dm_object_info_msgs.msg.ObjectInfo();

        // --- Position (Unity → ROS, then UTM → WGS84) ---
        var pos_pure = ROS2Utility.UnityToRosPosition(seenObject.position);
        var pos = byNoise ? positionNoise.ApplyNoiseOnVector(pos_pure) : pos_pure;

        Vector3 pos_utm = pos + utm_vector;
        (double lat, double lon) = GeographicLib.UTMUPS.Reverse(54, true, pos_utm.x, pos_utm.y);

        objectInfo.Object_location.Latitude.Value  = (int)(lat * 10000000);
        objectInfo.Object_location.Longitude.Value = (int)(lon * 10000000);
        objectInfo.Object_location.Altitude.Value  = (int)((pos.z + Environment.Instance.MgrsOffsetPosition.z) * 100);
        objectInfo.Object_location.Geodetic_srid.Value = 4326;

        // --- Existency ---
        objectInfo.Existency.Value = (byte)((int)(byNoise ? (probabilityNoise.ApplyNoiseOnFloat(0.8f) * 101) : 1f * 101));

        // --- Orientation (azimuth from north) ---
        float rotation = CalculateAngleFromNorth(seenObject);
        objectInfo.Orientation.Value.Value = (ushort)(rotation * 80);

        // --- ID (UUID if available; stable fallback otherwise) ---
        var los = seenObject.GetComponent<LineOfSight>();
        byte[] uuidBytes = los != null ? (los.GetUUID()?.Uuid ?? null) : null;
        if (uuidBytes == null || uuidBytes.Length == 0)
            uuidBytes = HashTo16Bytes(GetHierarchyPath(seenObject)); // stable fallback

        int generatedId = GetIntForUUID(uuidBytes);
        objectInfo.Id.Value = (ulong)generatedId;

        // --- Direction & Speed ---
        var rb = seenObject.GetComponent<Rigidbody>();
        Vector3 vel = rb ? rb.linearVelocity : Vector3.zero;

        Vector3 dirXZ = new Vector3(vel.x, 0f, vel.z);
        float directionDeg;
        if (dirXZ.sqrMagnitude > 0.0001f)
        {
            dirXZ.Normalize();
            directionDeg = Vector3.SignedAngle(Vector3.forward, dirXZ, Vector3.up);
            directionDeg = CalculateAngleFromNorth(directionDeg);
        }
        else
        {
            directionDeg = rotation; // not moving → use heading
        }
        objectInfo.Direction.Value.Value = (ushort)(directionDeg * 80);

        // --- Classification, Size, Speed ---
        objectInfo.Object_class = new ObjectClass[1];

        var npcVehicle = seenObject.GetComponent<NPCVehicle>();
        if (npcVehicle != null)
        {
            Vector3 dims = new Vector3(npcVehicle.Bounds.extents.x * 2,
                                       npcVehicle.Bounds.extents.y * 2,
                                       npcVehicle.Bounds.extents.z * 2);
            dims = byNoise ? dimensionNoise.ApplyNoiseOnVector(dims) : dims;

            objectInfo.Size.Length.Value.Value = (ushort)(dims.z * 100);
            objectInfo.Size.Width.Value.Value  = (ushort)(dims.x * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(dims.y * 100);

            var cls = new ObjectClass();
            cls.Id.Value = 1;   // VEHICLE
            cls.Confidence.Value = 101;
            objectInfo.Object_class[0] = cls;

            objectInfo.Speed.Value.Value = (short)(Vector3.Magnitude(vel) * 100);
        }
        else
        {
            var cls = new ObjectClass();
            cls.Id.Value = 2;   // PERSON
            cls.Confidence.Value = 101;
            objectInfo.Object_class[0] = cls;

            objectInfo.Size.Length.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Width.Value.Value  = (ushort)(0.5f * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(1.7f * 100);

            var sp = seenObject.GetComponent<ISpeed>();
            objectInfo.Speed.Value.Value = (short)((sp != null ? sp.GetSpeed() : 0f) * 100);
        }

        // --- Time (no sensors → use current time) ---
        objectInfo.Time = new TimestampIts();
        objectInfo.Time.Value = GetITSTimeInMilliseconds(DateTimeOffset.UtcNow.ToUnixTimeMilliseconds());

        // --- Information source list (no sensors → single source using configured sensorId) ---
        objectInfo.Information_source_list = new ObjectId[1];
        var src = new ObjectId();
        src.Value = GenerateObjectId(rsuId, sourceSensorId, (ushort)generatedId);
        objectInfo.Information_source_list[0] = src;

        // Ensure Id matches the first source (as in your original flow)
        objectInfo.Id.Value = src.Value;

        return objectInfo;
    }

    // ==== helpers copied from your original style ====

    public float YawAngluarRotationRosSystem(Transform target)
        => -((target.rotation.eulerAngles.y) % 360);

    public float CalculateAngleFromNorth(Transform target)
        => (target.rotation.eulerAngles.y + 90) % 360;

    public float CalculateAngleFromNorth(float target)
        => (target + 90) % 360;

    public static ulong GetITSTimeInMilliseconds(long unixTimeMillis)
        => (ulong)(unixTimeMillis - 1072882800000);

    private string ByteArrayToString(byte[] byteArray)
        => BitConverter.ToString(byteArray).Replace("-", "").ToLower();

    public int GetIntForUUID(byte[] uuid)
    {
        string key = ByteArrayToString(uuid);
        if (uuidDictionary.TryGetValue(key, out var val)) return val;
        int newInt = random.Next(1, int.MaxValue);
        uuidDictionary.Add(key, newInt);
        return newInt;
    }

    public ulong GenerateObjectId(string rsuIdHex, byte sensorId, ushort detectedObjectId)
    {
        uint rsuIdVal = Convert.ToUInt32(rsuIdHex, 16);
        if (sensorId > 255)
        {
            Debug.LogError($"Sensor ID out of range: {sensorId}");
            return 0;
        }

        ulong objectId = 0;
        if (isGroundTruth) objectId |= (1UL << 63);
        else               objectId |= (1UL << 62);

        objectId |= ((ulong)sensorId << 48);
        objectId |= ((ulong)detectedObjectId << 32);
        objectId |= rsuIdVal;
        return objectId;
    }

    private static string GetHierarchyPath(Transform t)
    {
        var sb = new StringBuilder();
        while (t != null)
        {
            sb.Insert(0, "/" + t.name);
            t = t.parent;
        }
        return sb.ToString();
    }

    private static byte[] HashTo16Bytes(string s)
    {
        // stable 16-byte id from a string (e.g., transform hierarchy path)
        using (var md5 = System.Security.Cryptography.MD5.Create())
            return md5.ComputeHash(Encoding.UTF8.GetBytes(s));
    }

    private static Coordinate Parse(string value)
    {
        Coordinate c = null;
        if (Coordinate.TryParse(value, out c)) return c;
        throw new FormatException($"Input Coordinate \"{value}\" was not in a correct format.");
    }
}
