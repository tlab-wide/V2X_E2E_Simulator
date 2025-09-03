using System;
using System.Collections;
using System.Collections.Generic;
using autoware_perception_msgs.msg;
using AWSIM;
using ROS2;
using UnityEngine;
using v2x_msgs.msg;
using Environment = AWSIM.Environment;

public class PredictedObjectsAutoware : MonoBehaviour
{
    [SerializeField] private List<DetectionSensor> sensors;

    [SerializeField] private float Hz = 1;

    [SerializeField] private ulong stationID = 24;
    [SerializeField] private ulong sensorType = 1;

    public string Topic = "/v2x/cooperative_pure";
    public string frameId = "obj";

    [SerializeField] private string positionNoiseName = "default noise";
    [SerializeField] private string rotationNoiseName = "default noise";
    [SerializeField] private string dimensionNoiseName = "default noise";
    [SerializeField] private string probabilityNoiseName = "default noise";

    // ---- Delay system (mirrors ObjectInfo pattern) ----
    [Header("Delayed message")] [SerializeField]
    private bool enableDelayedMessages = true;
    [SerializeField] private List<MessageDelayConfig> messageDelaysConfigs = new List<MessageDelayConfig>();
    private readonly List<MessageDelay<PredictedObjects>> messageDelays = new List<MessageDelay<PredictedObjects>>();

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

    IPublisher<PredictedObjects> poseStampedPublisher;

    private PredictedObjects msg;

    void Awake()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);

        msg = new PredictedObjects
        {
            Header = new std_msgs.msg.Header()
            {
                Frame_id = frameId,
            }
        };

        var qos = QosSettings.GetQoSProfile();
        poseStampedPublisher = SimulatorROS2Node.CreatePublisher<PredictedObjects>(Topic, qos);

        if (enableDelayedMessages)
            InitializeDelaySystem();
    }

    private void InitializeDelaySystem()
    {
        var qos = QosSettings.GetQoSProfile();

        // 1) Add global (“general”) delays (copied from NetworkSimulator configs)
        var generalDelays = NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs();
        for (int i = 0; i < generalDelays.Count; i++)
        {
            var md = generalDelays[i]; // copy
            // For PredictedObjects we don’t have GT/non-GT split, so always base on Topic
            var baseTopic = Topic;

            var delay = new MessageDelay<PredictedObjects>(md.delayConfig);
            delay.SetIPublisher(
                SimulatorROS2Node.CreatePublisher<PredictedObjects>(baseTopic + md.GetTopicName(), qos)
            );
            messageDelays.Add(delay);
        }

        // 2) Add component-local custom delays
        for (int i = 0; i < messageDelaysConfigs.Count; i++)
        {
            var md = messageDelaysConfigs[i]; // copy
            var baseTopic = Topic;

            var delay = new MessageDelay<PredictedObjects>(md);
            delay.SetIPublisher(
                SimulatorROS2Node.CreatePublisher<PredictedObjects>(baseTopic + md.topicName, qos)
            );
            messageDelays.Add(delay);
        }
    }

    private void PublishByDelay(PredictedObjects message)
    {
        for (int i = 0; i < messageDelays.Count; i++)
        {
            var md = messageDelays[i];
            // Clone to decouple the async send from our live buffer
            NetworkSimulator.Instance.PublishLate(md, Clone(message));
        }
    }

    // Lightweight clone is sufficient (ObjectInfo uses a shallow copy too)
    private static PredictedObjects Clone(PredictedObjects source)
    {
        if (source == null) return null;

        var copy = new PredictedObjects();
        // Header is a struct-like ROS msg; shallow copy is fine
        copy.Header = source.Header;

        // Copy array reference (fast). If you prefer, you can shallow-clone the array object:
        // copy.Objects = (PredictedObject[])source.Objects?.Clone();
        copy.Objects = source.Objects;

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

                // Noise
                pos = positionNoise.ApplyNoiseOnVector(pos);

                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.X = pos.x;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Y = pos.y;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z;

                // Rotation
                var r = ROS2Utility.UnityToRosRotation(seenObjects[j].rotation);
                r = rotationNoise.RotateQuaternionAroundY(r);

                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.X = r.x;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Y = r.y;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Z = r.z;
                predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.W = r.w;

                // Twist speed (x as forward)
                ISpeed ispeed = seenObjects[j].GetComponentInParent<ISpeed>();
                geometry_msgs.msg.Vector3 linearVelocity = new geometry_msgs.msg.Vector3();
                linearVelocity.X = ispeed.GetSpeed();
                predictedObject.Kinematics.Initial_twist_with_covariance.Twist.Linear = linearVelocity;

                predictedObject.Existence_probability = probabilityNoise.ApplyNoiseToDecrease(1);

                NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();
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
                }
                else
                {
                    predictedObject.Shape.Dimensions.X = 0.5f;
                    predictedObject.Shape.Dimensions.Y = 0.5f;
                    predictedObject.Shape.Dimensions.Z = 1.7f;
                    predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z =
                        pos.z + predictedObject.Shape.Dimensions.Z / 2;
                }

                // Classification + uuid
                LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                if (lineOfSight != null)
                {
                    var oc = new autoware_perception_msgs.msg.ObjectClassification();
                    oc.Label = lineOfSight.GetTypeOfObject();
                    oc.Probability = 1;
                    predictedObject.Classification = new autoware_perception_msgs.msg.ObjectClassification[] { oc };
                    predictedObject.Object_id = lineOfSight.GetUUID();
                }
                else
                {
                    throw new Exception("Detected an object without lineOfSight component");
                }
                FillCovariancesInPlace(predictedObject);
                predictedObjects.Add(predictedObject);
            }
        }

        msg.Header.Stamp = SimulatorROS2Node.GetCurrentRosTime();
        msg.Objects = predictedObjects.ToArray();

        // Publish immediately on the base topic
        poseStampedPublisher.Publish(msg);

        // Also publish delayed/lossy copies on derived topics
        if (enableDelayedMessages)
            PublishByDelay(msg);
        
        
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
    
    [Header("Msg Default fillers")]
    [SerializeField] private float posStdDevXY   = 0.5f; // m
    [SerializeField] private float posStdDevZ    = 1.0f; // m
    [SerializeField] private float yawStdDevDeg  = 5f;   // deg
    [SerializeField] private float linVelStdDev  = 0.5f; // m/s
    [SerializeField] private float angVelStdDevDeg = 5f; // deg/s


    //v
    private void FillCovariancesInPlace(PredictedObject po)
    {
        // Pose 6x6: [x y z roll pitch yaw]
        var poseCov  = po.Kinematics.Initial_pose_with_covariance.Covariance;
        // Twist 6x6: [vx vy vz vroll vpitch vyaw]
        var twistCov = po.Kinematics.Initial_twist_with_covariance.Covariance;

        if (poseCov == null || poseCov.Length != 36)  throw new Exception("Pose covariance array invalid");
        if (twistCov == null || twistCov.Length != 36) throw new Exception("Twist covariance array invalid");

        Array.Clear(poseCov,  0, poseCov.Length);
        Array.Clear(twistCov, 0, twistCov.Length);

        double sx2   = posStdDevXY * posStdDevXY;
        double sy2   = sx2;
        double sz2   = posStdDevZ  * posStdDevZ;

        double yawRad = Mathf.Deg2Rad * yawStdDevDeg;
        double syaw2  = yawRad * yawRad;

        // If roll/pitch are basically unknown, give large but non-zero variance.
        double rpd   = Mathf.Deg2Rad * 30f;  // 30 deg
        double sroll2  = rpd * rpd;
        double spitch2 = sroll2;

        // Diagonal for PoseWithCovariance
        poseCov[0]  = sx2;     // x
        poseCov[7]  = sy2;     // y
        poseCov[14] = sz2;     // z
        poseCov[21] = sroll2;  // roll
        poseCov[28] = spitch2; // pitch
        poseCov[35] = syaw2;   // yaw

        // Velocities
        double svlin2 = linVelStdDev * linVelStdDev;
        double avRad  = Mathf.Deg2Rad * angVelStdDevDeg;
        double svang2 = avRad * avRad;

        twistCov[0]  = svlin2; // vx
        twistCov[7]  = svlin2; // vy
        twistCov[14] = svlin2; // vz
        twistCov[21] = svang2; // vroll
        twistCov[28] = svang2; // vpitch
        twistCov[35] = svang2; // vyaw
    }


}