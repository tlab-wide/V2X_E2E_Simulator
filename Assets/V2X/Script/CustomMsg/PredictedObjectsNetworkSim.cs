using System;
using System.Collections;
using System.Collections.Generic;
using autoware_perception_msgs.msg;
using AWSIM;
using ROS2;
using UnityEngine;
using v2x_msgs.msg;
using Environment = AWSIM.Environment;

public class PredictedObjectsNetworkSim : MonoBehaviour
{
    [SerializeField] private List<MockDetectionSensor> sensors;

    [SerializeField] private float Hz = 1;
    [SerializeField] private ulong stationID = 24;
    [SerializeField] private ulong sensorType = 1;

    public string Topic = "/v2x/cooperative";
    public string frameId = "obj";

    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";

    // ---- Delay system ----
    [Header("Delayed message")]
    [SerializeField] private bool enableDelayedMessages = false; // default false
    [SerializeField] private List<MessageDelayConfig> messageDelaysConfigs = new List<MessageDelayConfig>();
    private readonly List<MessageDelay<V2XPerception>> messageDelays = new List<MessageDelay<V2XPerception>>();

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

    IPublisher<V2XPerception> poseStampedPublisher;
    private V2XPerception msg;

    void Awake()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);

        msg = new V2XPerception();
        msg.Predicted_objects.Header = new std_msgs.msg.Header()
        {
            Frame_id = frameId,
        };

        var qos = QosSettings.GetQoSProfile();
        poseStampedPublisher = SimulatorROS2Node.CreatePublisher<V2XPerception>(Topic, qos);

        if (enableDelayedMessages)
            InitializeDelaySystem();
    }

    private void InitializeDelaySystem()
    {
        var qos = QosSettings.GetQoSProfile();

        // 1) Add global (“general”) delays
        var generalDelays = NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs();
        foreach (var md in generalDelays)
        {
            var delay = new MessageDelay<V2XPerception>(md.delayConfig);
            delay.SetIPublisher(
                SimulatorROS2Node.CreatePublisher<V2XPerception>(Topic + md.GetTopicName(), qos)
            );
            messageDelays.Add(delay);
        }

        // 2) Add component-local custom delays
        foreach (var md in messageDelaysConfigs)
        {
            var delay = new MessageDelay<V2XPerception>(md);
            delay.SetIPublisher(
                SimulatorROS2Node.CreatePublisher<V2XPerception>(Topic + md.topicName, qos)
            );
            messageDelays.Add(delay);
        }
    }

    private void PublishByDelay(V2XPerception message)
    {
        if (!enableDelayedMessages) return;

        foreach (var md in messageDelays)
        {
            NetworkSimulator.Instance.PublishLate(md, Clone(message));
        }
    }

    private static V2XPerception Clone(V2XPerception source)
    {
        if (source == null) return null;
        var copy = new V2XPerception
        {
            Station_id = source.Station_id,
            Sensor_type = source.Sensor_type,
            Station_pose = source.Station_pose,
            Predicted_objects = source.Predicted_objects
        };
        return copy;
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
                if (!seenObjects[j].gameObject.activeInHierarchy || haveSeen.Contains(seenObjects[j]))
                    continue;

                haveSeen.Add(seenObjects[j]);
                PredictedObject predictedObject = new PredictedObject();

                var pos = ROS2Utility.UnityToRosPosition(seenObjects[j].transform.position);
                pos = pos + Environment.Instance.MgrsOffsetPosition;
                pos = positionNoise.ApplyNoiseOnVector(pos);

                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.X = pos.x;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Y = pos.y;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z;

                var r = ROS2Utility.UnityToRosRotation(seenObjects[j].rotation);
                r = rotationNoise.RotateQuaternionAroundY(r);
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.X = r.x;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Y = r.y;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Z = r.z;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.W = r.w;

                predictedObject.Existence_probability = 0.8f;

                ISpeed ispeed = seenObjects[j].GetComponentInParent<ISpeed>();
                geometry_msgs.msg.Vector3 linearVelocity = new geometry_msgs.msg.Vector3();
                linearVelocity.X = ispeed.GetSpeed();
                predictedObject.Kinematics.Initial_twist_with_covariance.Twist.Linear = linearVelocity;

                NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();
                var objectClassification = new autoware_perception_msgs.msg.ObjectClassification();

                if (npcVehicle != null)
                {
                    Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,
                        npcVehicle.Bounds.extents.y * 2, npcVehicle.Bounds.extents.z * 2);
                    dimensions = dimensionNoise.ApplyNoiseOnVector(dimensions);
                    dimensions = ROS2Utility.UnityToRosPosition(dimensions);

                    predictedObject.Shape.Dimensions.X = dimensions.x;
                    predictedObject.Shape.Dimensions.Y = Math.Abs(dimensions.y);
                    predictedObject.Shape.Dimensions.Z = dimensions.z;
                    predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z + dimensions.z;
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

                LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                if (lineOfSight != null)
                {
                    objectClassification.Probability = 1;
                    predictedObject.Classification = new[] { objectClassification };
                    predictedObject.Object_id.Uuid[0] = (byte)j;
                    predictedObject.Object_id.Uuid[1] = (byte)stationID;
                }
                else
                {
                    throw new Exception("Detected an object without lineOfSight component");
                }

                predictedObjects.Add(predictedObject);
            }
        }

        msg.Predicted_objects.Objects = predictedObjects.ToArray();
        msg.Station_id = stationID;
        msg.Sensor_type = sensorType;

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

        msg.Predicted_objects.Header.Stamp = SimulatorROS2Node.GetCurrentRosTime();

        poseStampedPublisher.Publish(msg);
        PublishByDelay(msg); // conditional
    }

    private float timer;

    public void FixedUpdate()
    {
        timer += Time.deltaTime;
        var interval = 1.0f / (Hz * 2);
        if (timer + 0.00001f < interval)
            return;

        timer = 0;
        CheckMockSensors();
    }
}
