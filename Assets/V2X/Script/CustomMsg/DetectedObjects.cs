using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using autoware_auto_perception_msgs.msg;
using AWSIM;
using dm_rsu_msgs.msg;
using geometry_msgs.msg;
using ROS2;
using std_msgs.msg;
using unique_identifier_msgs.msg;
using UnityEngine;
using v2x_msgs.msg;
using Pose = geometry_msgs.msg.Pose;
using Transform = UnityEngine.Transform;
using Vector3 = geometry_msgs.msg.Vector3;

namespace AWSIM
{
    public class DetectedObjects : MonoBehaviour
    {
        [SerializeField] private List<MockSensor> sensors;

        [SerializeField] private float Hz = 1;

        [SerializeField] private ulong stationID = 24;
        [SerializeField] private ulong sensorType = 1;

        public string Topic = "/v2x/cooperative";

        public string frameId = "obj";
        
        public QoSSettings QosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        IPublisher<CooperativeObjectsMessage> poseStampedPublisher;
        // IPublisher<> poseStampedPublisher;
        
        private CooperativeObjectsMessage msg;

        // Start is called before the first frame update
        void Awake()
        {
            msg = new CooperativeObjectsMessage();

            msg.Station_pose.Header = new std_msgs.msg.Header()
            {
                Frame_id = frameId,
            };


            // Create publisher.
            var qos = QosSettings.GetQoSProfile();
            poseStampedPublisher = SimulatorROS2Node.CreatePublisher<CooperativeObjectsMessage>(Topic, qos);

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


                        // Debug.Log("test position ros pos");
                        // Debug.Log(pos);

                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.X = pos.x;
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Y = pos.y;
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z;
                        
                        
                        //rotation base on bus //todo check correctness 
                        var r = ROS2Utility.UnityToRosRotation(seenObjects[j].rotation);
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.X = r.x;
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Y = r.y;
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.Z = r.z;
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation.W = r.w;


                        NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();

                        if (npcVehicle != null)
                        {
                            predictedObject.Shape.Dimensions.X = npcVehicle.Bounds.extents.x * 2;
                            predictedObject.Shape.Dimensions.Y = npcVehicle.Bounds.extents.y * 2;
                            predictedObject.Shape.Dimensions.Z = npcVehicle.Bounds.extents.z * 2;
                        }
                        else
                        {
                            predictedObject.Shape.Dimensions.X = 0.7f;
                            predictedObject.Shape.Dimensions.Y = 0.7;
                            predictedObject.Shape.Dimensions.Z = 1.7f;
                        }
                        
                        
                        //handling uuid and type
                        LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                        if (lineOfSight != null)
                        {
                            autoware_auto_perception_msgs.msg.ObjectClassification objectClassification = new autoware_auto_perception_msgs.msg.ObjectClassification();
                            objectClassification.Label = lineOfSight.GetTypeOfObject();
                            objectClassification.Probability = 1;
                            predictedObject.Classification = new autoware_auto_perception_msgs.msg.ObjectClassification[]{objectClassification};
                            predictedObject.Object_id = lineOfSight.GetUUID();
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


                poseStampedPublisher.Publish(msg);

                //todo prediction for RSU 
                
        }
        
        
        private float timer;

        public void FixedUpdate()
        {
            timer += Time.deltaTime;
            // timer += Time.fixedDeltaTime;


            var interval = 1.0f / (Hz*2);
            if (timer + 0.00001f < interval)
                return;

            timer = 0;

            CheckMockSensors();
        }
    }
}