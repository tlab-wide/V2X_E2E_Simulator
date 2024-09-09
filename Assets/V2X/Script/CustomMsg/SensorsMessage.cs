using System;
using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using AWSIM;
using ROS2;
using sensor_objects_msgs.msg;
using unique_identifier_msgs.msg;
using UnityEngine;
using v2x_msgs.msg;
using DetectedObjects = AWSIM.DetectedObjects;
using ObjectClassification = autoware_auto_perception_msgs.msg.ObjectClassification;

public class SensorsMessage : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 1;

    [SerializeField] private ulong stationID = 24;
    [SerializeField] private ulong sensorType = 1;

    public string Topic = "/v2x/Sensing";

    [SerializeField] private Transform busTransform;

    public QoSSettings QosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 1,
    };

    IPublisher<autoware_auto_perception_msgs.msg.DetectedObjects> sensorDetectedPublisher;

    CooperativeObjectsMessage test;

    // Start is called before the first frame update
    void Awake()
    {
        // Create publisher.
        var qos = QosSettings.GetQoSProfile();
        sensorDetectedPublisher =
            SimulatorROS2Node.CreatePublisher<autoware_auto_perception_msgs.msg.DetectedObjects>(Topic, qos);
    }

    private Vector3 CalculateRelativePosition(Vector3 objectPos)
    {
        Vector3 relativePosition = objectPos - busTransform.position;

        // Rotate the relative position based on the source's rotation
        Vector3 relativePositionRotated = Quaternion.Inverse(busTransform.rotation) * relativePosition;


        return relativePosition;
    }


    private void CheckMockSensors()
    {
        List<autoware_auto_perception_msgs.msg.DetectedObject> objects =
            new List<autoware_auto_perception_msgs.msg.DetectedObject>();
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


                autoware_auto_perception_msgs.msg.DetectedObject DetectedObject =
                    new autoware_auto_perception_msgs.msg.DetectedObject();

                var pos = CalculateRelativePosition(seenObjects[j].transform.position);
                pos = ROS2Utility.UnityToRosPosition(pos);

                Debug.Log("test position ros pos");
                Debug.Log(pos);

                DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.X = pos.x;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Y = pos.y;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z = pos.z;
                DetectedObject.Kinematics.Orientation_availability = 2;


                //rotation base on bus todo check correctness
                var r = ROS2Utility.UnityToRosRotation(Quaternion.Inverse(busTransform.rotation) *
                                                       seenObjects[j].rotation);
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.X = r.x;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.Y = r.y;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.Z = r.z;
                DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.W = r.w;


                //handling dimension
                NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();

                if (npcVehicle != null)
                {
                    DetectedObject.Shape.Dimensions.X = npcVehicle.Bounds.extents.x * 2;
                    DetectedObject.Shape.Dimensions.Y = npcVehicle.Bounds.extents.y * 2;
                    DetectedObject.Shape.Dimensions.Z = npcVehicle.Bounds.extents.z * 2;
                }
                else
                {
                    DetectedObject.Shape.Dimensions.X = 0.7f;
                    DetectedObject.Shape.Dimensions.Y = 0.7f;
                    DetectedObject.Shape.Dimensions.Z = 1.7f;
                }

                //handling type
                LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                if (lineOfSight != null)
                {
                    ObjectClassification objectClassification = new ObjectClassification();
                    objectClassification.Label = lineOfSight.GetTypeOfObject();
                    objectClassification.Probability = 1;
                    DetectedObject.Classification = new ObjectClassification[] { objectClassification };
                }
                else
                {
                    throw new Exception("Detected an object without lineOfSight component");
                }


                objects.Add(DetectedObject);
            }
        }

        autoware_auto_perception_msgs.msg.DetectedObjects msg =
            new autoware_auto_perception_msgs.msg.DetectedObjects();


        msg.Objects = objects.ToArray();

        sensorDetectedPublisher.Publish(msg);
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