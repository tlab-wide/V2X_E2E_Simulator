using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using ROS2;
using UnityEngine;
using v2x_msgs.msg;
using ObjectClassification = autoware_auto_perception_msgs.msg.ObjectClassification;

public class DetectedObjectsAutoware : MonoBehaviour
{
    [SerializeField] private List<MockSensor> sensors;

    [SerializeField] private float Hz = 10;
    
    

    public string Topic = "/OBU/Sensing";
    public string frameId = "bus";
    [SerializeField] private Transform busTransform;

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

    IPublisher<autoware_auto_perception_msgs.msg.DetectedObjects> sensorDetectedPublisher;

    CooperativeObjectsMessage test;
    private autoware_auto_perception_msgs.msg.DetectedObjects msg;
    

    
    // Start is called before the first frame update
    void Start()
    {
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        dimensionNoise = NoiseSetting.Instance.GetNoise(dimensionNoiseName);
        probabilityNoise = NoiseSetting.Instance.GetNoise(probabilityNoiseName);


        msg = new autoware_auto_perception_msgs.msg.DetectedObjects();
        msg.Header = new std_msgs.msg.Header()
        {
            Frame_id = frameId,
        };
        
        
        // Create publisher.
        var qos = QosSettings.GetQoSProfile();
        sensorDetectedPublisher =
            SimulatorROS2Node.CreatePublisher<autoware_auto_perception_msgs.msg.DetectedObjects>(Topic, qos);
        
        StartCoroutine(CheckMockSensors());
    }

    private Vector3 CalculateRelativePosition(Vector3 objectPos)
    {
        Vector3 relativePosition = objectPos - busTransform.position;

        // Rotate the relative position based on the source's rotation
        Vector3 relativePositionRotated = Quaternion.Inverse(busTransform.rotation) * relativePosition;

        // Debug.Log("**--releative pos--**");
        // Debug.Log(relativePositionRotated);
        return relativePositionRotated;
    }

    IEnumerator CheckMockSensors()
    {
        while (true)
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

                    //add noise
                    pos = positionNoise.ApplyNoiseOnVector(pos);
                    
                    

                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.X = pos.x;
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Y = pos.y;
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z = pos.z;
                    DetectedObject.Kinematics.Orientation_availability = 2;
                    
                    
                    //rotation base on bus todo check correctness
                    Quaternion r = ROS2Utility.UnityToRosRotation(Quaternion.Inverse(busTransform.rotation) *seenObjects[j].rotation);
                    
                    //apply noise
                    r = rotationNoise.RotateQuaternionAroundY(r);
                    
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.X = r.x;
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.Y = r.y;
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.Z = r.z;
                    DetectedObject.Kinematics.Pose_with_covariance.Pose.Orientation.W = r.w;

                    // DetectedObject.Existence_probability = 1f;
                    DetectedObject.Existence_probability = probabilityNoise.ApplyNoiseToDecrease(1);

                    //handling dimension
                    NPCVehicle npcVehicle = seenObjects[j].GetComponent<NPCVehicle>();

                    if (npcVehicle != null)
                    {
                        //add noise
                        Vector3 dimensions = new Vector3(npcVehicle.Bounds.extents.x * 2,npcVehicle.Bounds.extents.y * 2,npcVehicle.Bounds.extents.z * 2);
                        dimensions = dimensionNoise.ApplyNoiseOnVector(dimensions);
                        
                        
                        dimensions = ROS2Utility.UnityToRosPosition(dimensions);
                        
                        DetectedObject.Shape.Dimensions.X = dimensions.x;
                        DetectedObject.Shape.Dimensions.Y = Math.Abs( dimensions.y); // dimension no need to be minus when convert to autoware coordination
                        DetectedObject.Shape.Dimensions.Z = dimensions.z;
                        
                        // this line of code allign center of mass 
                        DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z = pos.z + dimensions.z/2;
                        
                        
                    }
                    else
                    {
                        DetectedObject.Shape.Dimensions.X = 0.7;
                        DetectedObject.Shape.Dimensions.Y = 0.7;
                        DetectedObject.Shape.Dimensions.Z = 1.7;
                        
                        DetectedObject.Kinematics.Pose_with_covariance.Pose.Position.Z = pos.z + DetectedObject.Shape.Dimensions.Z/2;
                    }

                    //handling type
                    LineOfSight lineOfSight = seenObjects[j].GetComponent<LineOfSight>();
                    if (lineOfSight != null)
                    {
                        autoware_auto_perception_msgs.msg.ObjectClassification objectClassification = new autoware_auto_perception_msgs.msg.ObjectClassification();
                        objectClassification.Label = lineOfSight.GetTypeOfObject();
                        objectClassification.Probability = 1;
                        // objectClassification.Probability = probabilityNoise.ApplyNoiseToDecrease(1);;
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

            
            yield return new WaitForSeconds(1 / Hz);
        }
    }
}
