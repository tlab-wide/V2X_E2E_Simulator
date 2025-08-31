using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using ROS2;
using Unity.VisualScripting;
using UnityEngine;
using v2x_msgs.msg;
using ObjectClassification = autoware_perception_msgs.msg.ObjectClassification;

public class DetectedObjectsAutoware : MonoBehaviour
{
    [SerializeField] private List<MockDetectionSensor> sensors;
    [SerializeField] private float Hz = 10;

    public string Topic = "/OBU/Sensing";
    public string frameId = "bus";
    [SerializeField] private Transform busTransform;

    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";

    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise dimensionNoise;
    private NoiseSetting.Noise probabilityNoise;

    // ---- Delay system (disabled by default) ----
    [Header("Delayed message")]
    [SerializeField] private bool enableDelayedMessages = false; // default false
    [SerializeField] private List<MessageDelayConfig> messageDelaysConfigs = new List<MessageDelayConfig>();
    private readonly List<MessageDelay<autoware_perception_msgs.msg.DetectedObjects>> messageDelays
        = new List<MessageDelay<autoware_perception_msgs.msg.DetectedObjects>>();

    public QoSSettings QosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 1,
    };

    IPublisher<autoware_perception_msgs.msg.DetectedObjects> sensorDetectedPublisher;
    private autoware_perception_msgs.msg.DetectedObjects msg;

    void Awake()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);

        msg = new autoware_perception_msgs.msg.DetectedObjects();
        msg.Header = new std_msgs.msg.Header()
        {
            Frame_id = frameId,
        };

        var qos = QosSettings.GetQoSProfile();
        sensorDetectedPublisher =
            SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.DetectedObjects>(Topic, qos);

        if (enableDelayedMessages)
            InitializeDelaySystem();
    }

    private void InitializeDelaySystem()
    {
        var qos = QosSettings.GetQoSProfile();

        // 1) Global (“general”) delay topics from NetworkSimulator
        var generalDelays = NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs();
        foreach (var md in generalDelays)
        {
            var delay = new MessageDelay<autoware_perception_msgs.msg.DetectedObjects>(md.delayConfig);
            delay.SetIPublisher(
                SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.DetectedObjects>(Topic + md.GetTopicName(), qos)
            );
            messageDelays.Add(delay);
        }

        // 2) Component-local delays from inspector
        foreach (var md in messageDelaysConfigs)
        {
            var delay = new MessageDelay<autoware_perception_msgs.msg.DetectedObjects>(md);
            delay.SetIPublisher(
                SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.DetectedObjects>(Topic + md.topicName, qos)
            );
            messageDelays.Add(delay);
        }
    }

    private void PublishByDelay(autoware_perception_msgs.msg.DetectedObjects message)
    {
        if (!enableDelayedMessages) return;

        foreach (var md in messageDelays)
        {
            NetworkSimulator.Instance.PublishLate(md, Clone(message));
        }
    }

    private static autoware_perception_msgs.msg.DetectedObjects Clone(autoware_perception_msgs.msg.DetectedObjects source)
    {
        if (source == null) return null;

        var copy = new autoware_perception_msgs.msg.DetectedObjects
        {
            Header = source.Header,
            // Shallow copy is typically fine; switch to .Clone() if you mutate after publish.
            Objects = (autoware_perception_msgs.msg.DetectedObject[])source.Objects?.Clone()
        };
        return copy;
    }

    private Vector3 CalculateRelativePosition(Vector3 objectPos)
    {
        Vector3 relativePosition = objectPos - busTransform.position;
        Vector3 relativePositionRotated = Quaternion.Inverse(busTransform.rotation) * relativePosition;
        return relativePositionRotated;
    }

    private void CheckMockSensors()
    {
        List<autoware_perception_msgs.msg.DetectedObject> objects =
            new List<autoware_perception_msgs.msg.DetectedObject>();
        List<Transform> haveSeen = new List<Transform>();
        for (int i = 0; i < sensors.Count; i++)
        {
            List<Transform> seenObjects = sensors[i].GetSeenObjects();

            for (int j = 0; j < seenObjects.Count; j++)
            {
                // remove duplications
                if (haveSeen.Contains(seenObjects[j]) || !seenObjects[j].gameObject.activeInHierarchy)
                    continue;

                haveSeen.Add(seenObjects[j]);

                autoware_perception_msgs.msg.DetectedObject DetectedObject =
                    new autoware_perception_msgs.msg.DetectedObject();

                var pos = CalculateRelativePosition(seenObjects[j].transform.position);
                pos = ROS2Utility.UnityToRosPosition(pos);

                // add noise
                pos = positionNoise.ApplyNoiseOnVector(pos);

                DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.X = pos.x;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Y = pos.y;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z = pos.z;
                DetectedObject.Kinematics.Orientation_availability = 2;

                // speed
                DetectedObject.Kinematics.Has_twist = true;
                ISpeed ispeed = seenObjects[j].gameObject.GetComponentInParent<ISpeed>();
                if (ispeed == null)
                    ispeed = seenObjects[j].transform.GetComponent<ISpeed>();

                geometry_msgs.msg.Vector3 linearVelocity = new geometry_msgs.msg.Vector3();
                float magnitude = ispeed.GetSpeed();
                linearVelocity.X = magnitude;
                DetectedObject.Kinematics.Twist_with_covariance.Twist.Linear = linearVelocity;

                // rotation based on bus
                Quaternion r =
                    ROS2Utility.UnityToRosRotation(Quaternion.Inverse(busTransform.rotation) * seenObjects[j].rotation);

                // apply noise
                r = rotationNoise.RotateQuaternionAroundY(r);

                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.X = r.x;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.Y = r.y;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.Z = r.z;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.W = r.w;

                DetectedObject.Existence_probability = probabilityNoise.ApplyNoiseToDecrease(1);

                // dimensions
                NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();

                if (npcVehicle != null)
                {
                    Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2, npcVehicle.Bounds.extents.y * 2,
                        npcVehicle.Bounds.extents.z * 2);
                    dimensions = dimensionNoise.ApplyNoiseOnVector(dimensions);
                    dimensions = ROS2Utility.UnityToRosPosition(dimensions);

                    DetectedObject.Shape.Dimensions.X = dimensions.x;
                    DetectedObject.Shape.Dimensions.Y = Math.Abs(dimensions.y);
                    DetectedObject.Shape.Dimensions.Z = dimensions.z;

                    // center of mass alignment
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z = pos.z + dimensions.z / 2;
                }
                else
                {
                    DetectedObject.Shape.Dimensions.X = 0.7;
                    DetectedObject.Shape.Dimensions.Y = 0.7;
                    DetectedObject.Shape.Dimensions.Z = 1.7;

                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z =
                        pos.z + DetectedObject.Shape.Dimensions.Z / 2;
                }

                // type
                LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                if (lineOfSight != null)
                {
                    var objectClassification = new ObjectClassification
                    {
                        Label = lineOfSight.GetTypeOfObject(),
                        Probability = 1
                    };
                    DetectedObject.Classification = new ObjectClassification[] { objectClassification };
                }
                else
                {
                    throw new Exception("Detected an object without lineOfSight component");
                }

                objects.Add(DetectedObject);
            }
        }

        msg.Objects = objects.ToArray();

        msg.Header.Stamp = SimulatorROS2Node.GetCurrentRosTime();
        sensorDetectedPublisher.Publish(msg);

        // Conditionally publish delayed/lossy copies
        PublishByDelay(msg);
    }

    private float timer;

    public void FixedUpdate()
    {
        timer += Time.deltaTime;

        // HZ * 2 to align with AWSIM's deltaTime cadence used elsewhere
        var interval = 1.0f / (Hz * 2);
        if (timer + 0.00001f < interval)
            return;

        timer = 0;
        CheckMockSensors();
    }
}
