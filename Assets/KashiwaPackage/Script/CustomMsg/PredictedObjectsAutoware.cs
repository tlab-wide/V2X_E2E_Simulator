using System;
using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using AWSIM;
using ROS2;
using Unity.Mathematics;
using UnityEngine;
using v2x_msgs.msg;
using Environment = AWSIM.Environment;

public class PredictedObjectsAutoware : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 1;

    [SerializeField] private ulong stationID = 24;
    [SerializeField] private ulong sensorType = 1;

    public string Topic = "/v2x/cooperative_pure";

    public string frameId = "obj";
    
    [SerializeField] private string positionNoiseName="default noise";
    [SerializeField] private string rotationNoiseName="default noise";
    [SerializeField] private string dimensionNoiseName="default noise";
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

    IPublisher<PredictedObjects> poseStampedPublisher;

    private PredictedObjects msg;

    // Start is called before the first frame update
    void Start()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);
        
        
        msg = new PredictedObjects();

        msg.Header = new std_msgs.msg.Header()
        {
            Frame_id = frameId,
        };


        // Create publisher.
        var qos = QosSettings.GetQoSProfile();
        poseStampedPublisher = SimulatorROS2Node.CreatePublisher<PredictedObjects>(Topic, qos);

        StartCoroutine(CheckMockSensors());
    }

    IEnumerator CheckMockSensors()
    {
        while (true)
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

                    // DetectedObject.Existence_probability = 1f;
                    predictedObject.Existence_probability = probabilityNoise.ApplyNoiseToDecrease(1);

                    NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();

                    if (npcVehicle != null)
                    {
                        //add noise
                        Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,npcVehicle.Bounds.extents.y * 2,npcVehicle.Bounds.extents.z * 2);
                        dimensions = dimensionNoise.ApplyNoiseOnVector(dimensions);



                        dimensions = ROS2Utility.UnityToRosPosition(dimensions);
                        
                        predictedObject.Shape.Dimensions.X = dimensions.x;
                        predictedObject.Shape.Dimensions.Y = math.abs( dimensions.y); // dimension no need to be minus when convert to autoware coordination
                        predictedObject.Shape.Dimensions.Z = dimensions.z;

                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z + dimensions.z;

                        // predictedObject.Shape.Dimensions.X = 4.7f;
                        // predictedObject.Shape.Dimensions.Y = 1.93f;
                        // predictedObject.Shape.Dimensions.Z = 1.6f;
                    }
                    else
                    {
                        predictedObject.Shape.Dimensions.X = 0.5f;
                        predictedObject.Shape.Dimensions.Y = 0.5f;
                        predictedObject.Shape.Dimensions.Z = 1.7f;
                        predictedObject.Kinematics.Initial_pose_with_covariance.Pose.Position.Z = pos.z + predictedObject.Shape.Dimensions.Z / 2;
                    }


                    //handling uuid and type
                    LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                    if (lineOfSight != null)
                    {
                        autoware_auto_perception_msgs.msg.ObjectClassification objectClassification =
                            new autoware_auto_perception_msgs.msg.ObjectClassification();
                        objectClassification.Label = lineOfSight.GetTypeOfObject();
                        // objectClassification.Label = 1;
                        //objectClassification.Probability = probabilityNoise.ApplyNoiseToDecrease(1);
                        objectClassification.Probability = 1;
                        predictedObject.Classification = new autoware_auto_perception_msgs.msg.ObjectClassification[]
                            { objectClassification };
                        // predictedObject.Object_id = lineOfSight.GetUUID();
                        // predictedObject.Object_id.Uuid[0] = (byte)j;
                        // predictedObject.Object_id.Uuid[1] = (byte)stationID;
                        predictedObject.Object_id = LineOfSight.GenerateUUid();
                    }
                    else
                    {
                        throw new Exception("Detected an object without lineOfSight component");
                    }

                    predictedObjects.Add(predictedObject);
                }
            }

            msg.Header.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            msg.Objects = predictedObjects.ToArray();

            
            
            poseStampedPublisher.Publish(msg);

            //todo prediction for RSU 

            yield return new WaitForSeconds(1 / Hz);
        }
    }
}
