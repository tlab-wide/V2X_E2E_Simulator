using System;
using System.Collections.Generic;
using AWSIM;
using ROS2;
using UnityEngine;
using CooperativeObjectInfoMessage = dm_cooperative_msgs.msg.CooperativeObjectInfoMessage;
using Environment = AWSIM.Environment;
using dm_object_info_msgs.msg;
using Coordinate = CoordinateSharp.Coordinate;

public class ObjectInfo : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 1;

    [SerializeField] private ulong stationID = 24;
    [SerializeField] private ulong sensorType = 1;

    public string Topic = "/v2x/cooperative";
    public string TopicGroundTruth = "/v2x/cooperativeGroundTruth";

    public string frameId = "obj";

    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";


    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise dimensionNoise;
    private NoiseSetting.Noise probabilityNoise;


    public QoSSettings QosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 1,
    };

    IPublisher<CooperativeObjectInfoMessage> objectPublisher;
    IPublisher<CooperativeObjectInfoMessage> objectPublisherGroundTruth;

    private CooperativeObjectInfoMessage msg;
    private CooperativeObjectInfoMessage msgGroundTruth;

    // Start is called before the first frame update
    void Awake()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);

        msg = new CooperativeObjectInfoMessage();
        msgGroundTruth = new CooperativeObjectInfoMessage();


        // Create publisher.
        var qos = QosSettings.GetQoSProfile();
        objectPublisher = SimulatorROS2Node.CreatePublisher<CooperativeObjectInfoMessage>(Topic, qos);
        objectPublisherGroundTruth =
            SimulatorROS2Node.CreatePublisher<CooperativeObjectInfoMessage>(TopicGroundTruth, qos);
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


                // old section 

                //
                // //rotation base on bus //todo check correctness 
                // var r = ROS2Utility.UnityToRosRotation(seenObjects[j].rotation);
                // //apply noise
                // r = rotationNoise.RotateQuaternionAroundY(r);
                //
                // predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.X = r.x;
                // predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Y = r.y;
                // predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Z = r.z;
                // predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.W = r.w;
                //
                // predictedObject.Existence_probability = 0.8f;
                // // predictedObject.Existence_probability = probabilityNoise.ApplyNoiseToDecrease(1);
                //
                //
                // NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();
                //
                // autoware_auto_perception_msgs.msg.ObjectClassification objectClassification =
                //     new autoware_auto_perception_msgs.msg.ObjectClassification();
                // //
                // if (npcVehicle != null)
                // {
                //     //add noise
                //     Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,
                //         npcVehicle.Bounds.extents.y * 2, npcVehicle.Bounds.extents.z * 2);
                //     dimensions = dimensionNoise.ApplyNoiseOnVector(dimensions);
                //
                //
                //     dimensions = ROS2Utility.UnityToRosPosition(dimensions);
                //
                //     predictedObject.Shape.Dimensions.X = dimensions.x;
                //     predictedObject.Shape.Dimensions.Y =
                //         Math.Abs(dimensions.y); // dimension no need to be minus when convert to autoware coordination
                //     predictedObject.Shape.Dimensions.Z = dimensions.z;
                //
                //     predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z + dimensions.z;
                //
                //     // predictedObject.Shape.Dimensions.X = 4.7f;
                //     // predictedObject.Shape.Dimensions.Y = 1.93f;
                //     // predictedObject.Shape.Dimensions.Z = 1.6f;
                //
                //     // objectClassification.Label = lineOfSight.GetTypeOfObject();
                //     
                //     objectClassification.Label = 1;
                // }
                // else
                // {
                //     predictedObject.Shape.Dimensions.X = 0.5f;
                //     predictedObject.Shape.Dimensions.Y = 0.5f;
                //     predictedObject.Shape.Dimensions.Z = 1.7f;
                //     predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z =
                //         pos.z + predictedObject.Shape.Dimensions.Z / 2;
                //     objectClassification.Label = 7;
                // }
                //
                //
                // //handling uuid and type
                // LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                // if (lineOfSight != null)
                // {
                //     objectClassification.Probability = 1;
                //     // objectClassification.Probability = probabilityNoise.ApplyNoiseToDecrease(1);
                //     predictedObject.Classification = new autoware_auto_perception_msgs.msg.ObjectClassification[]
                //         { objectClassification };
                //     // predictedObject.Object_id = lineOfSight.GetUUID();
                //     predictedObject.Object_id.Uuid[0] = (byte)j;
                //     predictedObject.Object_id.Uuid[1] = (byte)stationID;
                //     // predictedObject.Object_id = LineOfSight.GenerateUUid();
                // }
                // else
                // {
                //     throw new Exception("Detected an object without lineOfSight component");
                // }
                //
                // // predictedObjects.Add(predictedObject);
            }
        }

        msg.Station_id = stationID;
        msg.Sensor_type = sensorType;

        msg.Object_info_array.Array = objectInfos.ToArray();

        msgGroundTruth.Station_id = stationID;
        msgGroundTruth.Sensor_type = sensorType;

        msgGroundTruth.Object_info_array.Array = objectInfosGroundTruth.ToArray();

        //pos station (we mention first element position of sensors as pos station)
        if (sensors.Count != 0)
        {
            UnityEngine.Vector3 posStation = sensors[0].transform.position;
            posStation = ROS2Utility.UnityToRosPosition(posStation);
            posStation = posStation + Environment.Instance.MgrsOffsetPosition;

            msg.Station_pose.Pose.Position.X = posStation.x;
            msg.Station_pose.Pose.Position.Y = posStation.y;
            msg.Station_pose.Pose.Position.Z = posStation.z;

            msgGroundTruth.Station_pose.Pose.Position.X = posStation.x;
            msgGroundTruth.Station_pose.Pose.Position.Y = posStation.y;
            msgGroundTruth.Station_pose.Pose.Position.Z = posStation.z;
        }
        else
        {
            throw new Exception("No sensors have been found in RSU");
        }


        objectPublisher.Publish(msg);

        objectPublisherGroundTruth.Publish(msgGroundTruth);

        //todo prediction for RSU 
    }

    private float timer;


    private dm_object_info_msgs.msg.ObjectInfo handlObjectInfo(Transform seenObjects, bool byNoise = true)
    {
        dm_object_info_msgs.msg.ObjectInfo objectInfo = new dm_object_info_msgs.msg.ObjectInfo();


        //position 
        var pos = ROS2Utility.UnityToRosPosition(seenObjects.transform.position);
        pos = pos + Environment.Instance.MgrsOffsetPosition;


        //add noise

        pos = byNoise ? positionNoise.ApplyNoiseOnVector(pos) : pos;

        string latLong = ConvertToLatLong(pos.x, pos.y);

        string lat = latLong.Split(' ')[0];
        string longitude = latLong.Split(' ')[1];
        // objectInfo.Object_location.Latitude.Value = float.Parse(lat);
        objectInfo.Object_location.Latitude.Value = (int)(float.Parse(lat) * 1000000);


        // objectInfo.Object_location.Longitude.Value = float.Parse(longitude);
        objectInfo.Object_location.Longitude.Value = (int)(float.Parse(longitude) * 1000000);

        objectInfo.Object_location.Altitude.Value = (int)pos.z;

        objectInfo.Object_location.Geodetic_system.Value = 4326;

        //Existancy
        objectInfo.Existency.Value =
            (byte)((int)(byNoise ? (probabilityNoise.ApplyNoiseOnFloat(0.8f) * 101) : 0.8f * 101));


        //Rotation
        float rotation = CalculateAngleFromNorth(seenObjects.transform.forward);
        objectInfo.Direction.Value.Value = (ushort)(rotation * 100);


        //type 
        NPCVehicle npcVehicle = seenObjects.GetComponent<NPCVehicle>();
        LineOfSight lineOfSight = seenObjects.GetComponent<LineOfSight>();

        // Debug.Log($"Line of Sight => {lineOfSight == null} - {lineOfSight.transform.name}");
        if (npcVehicle != null)
        {
            //add noise
            Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,
                npcVehicle.Bounds.extents.y * 2, npcVehicle.Bounds.extents.z * 2);
            dimensions = byNoise ? dimensionNoise.ApplyNoiseOnVector(dimensions) : dimensions;

            objectInfo.Size.Length.Value.Value = (ushort)(dimensions.z * 100);
            objectInfo.Size.Width.Value.Value = (ushort)(dimensions.x * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(dimensions.y * 100);

            // objectInfo.Object_class = new ObjectClass[1];
            // objectInfo.Object_class[0].Id.Value = lineOfSight.GetTypeOfObject();//todo type of car has bug 
        }
        else
        {
            objectInfo.Size.Length.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Width.Value.Value = (ushort)(0.5f * 100);
            objectInfo.Size.Height.Value.Value = (ushort)(1.7f * 100);
            
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
    // Calculate the angle from North using Atan2
    public float CalculateAngleFromNorth(Vector3 direction)
    {
        // Project the direction onto the XZ plane
        direction.y = 0;

        // Calculate the angle using Atan2
        float angle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;

        // Ensure the angle is within 0-360 range
        if (angle < 0)
        {
            angle += 360;
        }

        return angle;
    }
}