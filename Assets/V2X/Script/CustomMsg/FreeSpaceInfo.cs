using System;
using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using AWSIM;
using dm_cooperative_msgs.msg;
using dm_freespace_info_msgs.msg;
using ROS2;
using UnityEngine;
using v2x_msgs.msg;
using CooperativeFreespaceInfoMessage = dm_cooperative_msgs.msg.CooperativeFreespaceInfoMessage;
using CooperativeObjectInfoMessage = dm_cooperative_msgs.msg.CooperativeObjectInfoMessage;
using Environment = AWSIM.Environment;

public class FreeSpaceInfo : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 1;

    [SerializeField] private ulong stationID = 24;
    [SerializeField] private ulong sensorType = 1;

    public string Topic = "/v2x/cooperative";

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

    private CooperativeObjectInfoMessage msg;

    // Start is called before the first frame update
    void Awake()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);

        msg = new CooperativeObjectInfoMessage();


        // Create publisher.
        var qos = QosSettings.GetQoSProfile();
        objectPublisher = SimulatorROS2Node.CreatePublisher<CooperativeObjectInfoMessage>(Topic, qos);
    }

    private void CheckMockSensors()
    {
        List<PredictedObject> predictedObjects = new List<PredictedObject>();
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


                PredictedObject predictedObject = new PredictedObject();


                var pos = ROS2Utility.UnityToRosPosition(seenObjects[j].transform.position);
                pos = pos + Environment.Instance.MgrsOffsetPosition;

                //add noise
                pos = positionNoise.ApplyNoiseOnVector(pos);

                // Debug.Log("test position ros pos");
                // Debug.Log(pos);

                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.X = pos.x;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Y = pos.y;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z;


                //rotation base on bus //todo check correctness 
                var r = ROS2Utility.UnityToRosRotation(seenObjects[j].rotation);
                //apply noise
                r = rotationNoise.RotateQuaternionAroundY(r);

                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.X = r.x;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Y = r.y;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Z = r.z;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.W = r.w;

                predictedObject.Existence_probability = 0.8f;
                // predictedObject.Existence_probability = probabilityNoise.ApplyNoiseToDecrease(1);


                NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();

                autoware_auto_perception_msgs.msg.ObjectClassification objectClassification =
                    new autoware_auto_perception_msgs.msg.ObjectClassification();

                if (npcVehicle != null)
                {
                    //add noise
                    Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,
                        npcVehicle.Bounds.extents.y * 2, npcVehicle.Bounds.extents.z * 2);
                    dimensions = dimensionNoise.ApplyNoiseOnVector(dimensions);


                    dimensions = ROS2Utility.UnityToRosPosition(dimensions);

                    predictedObject.Shape.Dimensions.X = dimensions.x;
                    predictedObject.Shape.Dimensions.Y =
                        Math.Abs(dimensions.y); // dimension no need to be minus when convert to autoware coordination
                    predictedObject.Shape.Dimensions.Z = dimensions.z;

                    predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z + dimensions.z;

                    // predictedObject.Shape.Dimensions.X = 4.7f;
                    // predictedObject.Shape.Dimensions.Y = 1.93f;
                    // predictedObject.Shape.Dimensions.Z = 1.6f;

                    // objectClassification.Label = lineOfSight.GetTypeOfObject();
                    objectClassification.Label = 1;
                }
                else
                {
                    predictedObject.Shape.Dimensions.X = 0.5f;
                    predictedObject.Shape.Dimensions.Y = 0.5f;
                    predictedObject.Shape.Dimensions.Z = 1.7f;
                    predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z =
                        pos.z + predictedObject.Shape.Dimensions.Z / 2;
                    objectClassification.Label = 7;
                }


                //handling uuid and type
                LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                if (lineOfSight != null)
                {
                    objectClassification.Probability = 1;
                    // objectClassification.Probability = probabilityNoise.ApplyNoiseToDecrease(1);
                    predictedObject.Classification = new autoware_auto_perception_msgs.msg.ObjectClassification[]
                        { objectClassification };
                    // predictedObject.Object_id = lineOfSight.GetUUID();
                    predictedObject.Object_id.Uuid[0] = (byte)j;
                    predictedObject.Object_id.Uuid[1] = (byte)stationID;
                    // predictedObject.Object_id = LineOfSight.GenerateUUid();
                }
                else
                {
                    throw new Exception("Detected an object without lineOfSight component");
                }

                predictedObjects.Add(predictedObject);
            }
        }


        msg.Station_id = stationID;
        msg.Sensor_type = sensorType;

        //pos station (we mention first element position of sensors as pos station)
        if (sensors.Count != 0)
        {
            UnityEngine.Vector3 posStation = sensors[0].transform.position;
            posStation = ROS2Utility.UnityToRosPosition(posStation);
            posStation = posStation + Environment.Instance.MgrsOffsetPosition;

            msg.Station_pose.Pose.Position.X = posStation.x;
            msg.Station_pose.Pose.Position.Y = posStation.y;
            msg.Station_pose.Pose.Position.Z = posStation.z;
        }
        else
        {
            throw new Exception("No sensors have been found in RSU");
        }


        objectPublisher.Publish(msg);

        //todo prediction for RSU 
    }

    private float timer;

    public void FixedUpdate()
    {
        timer += Time.deltaTime;
        // timer += Time.fixedDeltaTime;


        var interval = 1.0f / (Hz * 2);
        if (timer + 0.00001f < interval)
            return;

        timer = 0;

        CheckMockSensors();
    }
}