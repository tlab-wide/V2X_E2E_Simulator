using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using AWSIM;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;
using Object = System.Object;
using String = std_msgs.msg.String;

[RequireComponent(typeof(BoxCollider))]
public class LogAroundCar : MonoBehaviour
{
    [SerializeField] private float radius = 50.0f;

    [FormerlySerializedAs("logPathCars")] [SerializeField]
    private string logPath = "HumanCarsAround.csv";


    // [SerializeField] private bool checkHumans;
    // [SerializeField] private bool checkCars;


    [SerializeField] private string detectionNoiseName = "default";
    [SerializeField] private string positionNoiseName = "default";
    [SerializeField] private string rotationNoiseName = "default";
    [SerializeField] private string speedNoiseName = "default";


    [SerializeField] private List<Transform> humanTransforms = new List<Transform>();
    [SerializeField] private List<Transform> carTransforms = new List<Transform>();


    private string myVariable;
    private NoiseSetting.Noise detectionNoise;
    private NoiseSetting.Noise positionNoise;
    private NoiseSetting.Noise rotationNoise;
    private NoiseSetting.Noise speedNoise;

    // Start is called before the first frame update
    void Start()
    {
        // set up header of CSV files
        HandleCsvHeader();

        detectionNoise = NoiseSetting.Instance.GetNoise(detectionNoiseName);
        positionNoise = NoiseSetting.Instance.GetNoise(positionNoiseName);
        rotationNoise = NoiseSetting.Instance.GetNoise(rotationNoiseName);
        speedNoise = NoiseSetting.Instance.GetNoise(speedNoiseName);
    }


    // private void OnTriggerExit(Collider other)
    // {
    //     // Debug.Log(other.name);
    //     if (other.gameObject.GetComponentInParent<NPCVehicle>() != null)
    //     {
    //         carTransforms.Remove(other.gameObject.GetComponentInParent<NPCVehicle>().transform);
    //     }
    //     else if (other.gameObject.GetComponentInParent<NPCPedestrian>() != null)
    //     {
    //         humanTransforms.Remove(other.gameObject.GetComponentInParent<NPCPedestrian>().transform);
    //     }
    //     else if (other.GameObject().layer == 6) //6 layer is Vehicle
    //     {
    //         //just for bus
    //         Rigidbody rigidbody = FindRigidbodyInHierarchy(other.GameObject());
    //         if (rigidbody != null)
    //         {
    //             carTransforms.Remove(rigidbody.transform);
    //         }
    //     }
    // }
    //
    // private void OnTriggerEnter(Collider other)
    // {
    //     if (checkCars && other.gameObject.GetComponentInParent<NPCVehicle>() != null)
    //     {
    //         carTransforms.Add(other.gameObject.GetComponentInParent<NPCVehicle>().transform);
    //     }
    //     else if (checkHumans && other.gameObject.GetComponentInParent<NPCPedestrian>() != null)
    //     {
    //         humanTransforms.Add(other.gameObject.GetComponentInParent<NPCPedestrian>().transform);
    //     }
    //     else if (other.GameObject().layer == 6) //6 layer is Vehicle
    //     {
    //         //just for bus
    //         Rigidbody rigidbody = FindRigidbodyInHierarchy(other.GameObject());
    //         if (rigidbody != null)
    //         {
    //             if (!carTransforms.Contains(rigidbody.transform))
    //             {
    //                 carTransforms.Add(rigidbody.transform);
    //             }
    //         }
    //     }
    // }


    // Call this method with the collider's gameObject to find the Rigidbody in its ancestors
    public Rigidbody FindRigidbodyInHierarchy(GameObject childObject)
    {
        // Check if the current gameObject has a Rigidbody
        Rigidbody rb = childObject.GetComponent<Rigidbody>();

        // If found, return the Rigidbody
        if (rb != null)
        {
            return rb;
        }

        // If there's no parent, return null (reached the top of the hierarchy without finding a Rigidbody)
        if (childObject.transform.parent == null)
        {
            return null;
        }

        // Recursively check the parent
        return FindRigidbodyInHierarchy(childObject.transform.parent.gameObject);
    }

    public void CaptureLog(int state, int posIndex,Vector3 posShift, float angle,int iterationNumber =1  )
    {
        ScanTheEnvironment();

        // Debug.Log("Capture Log is called");
        for (int i = 0; i < carTransforms.Count; i++)
        {
            if (UnityEngine.Random.Range(0.0f, 1) < detectionNoise.GetNoiseVector().x)
                continue;


            string row = dataRowGenerator(carTransforms[i],state,posIndex,posShift,angle,iterationNumber);
            if (row.Equals(""))
            {
                carTransforms.RemoveAt(i);
                i--;
                continue;
            }

            AppendStringToFile(logPath, row);
        }


        for (int i = 0; i < humanTransforms.Count; i++)
        {
            if (UnityEngine.Random.Range(0.0f, 1) < detectionNoise.SampleNoiseVector().x)
                continue;
            string row = dataRowGenerator(humanTransforms[i],state,posIndex,posShift,angle,iterationNumber);
            if (row.Equals(""))
            {
                carTransforms.RemoveAt(i);
                i--;
                continue;
            }

            AppendStringToFile(logPath, row);
        }
    }


    private string dataRowGenerator(Transform transform, int state, int posIndex,Vector3 posShift, float angle,int iterationNumber = 0)
    {
        try
        {
            if (transform is null || transform.GetComponent<LineOfSight>() is null)
            {
                Debug.LogWarning($"{transform.name} is null and can not save log");
                return "";
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning("Object removed");
            return "";
        }


        LineOfSight lineOfSightComponent = transform.GetComponent<LineOfSight>();
        string row;

        //time
        builtin_interfaces.msg.Time rosTime = SimulatorROS2Node.GetCurrentRosTime();


        //set position
        var pos = ROS2Utility.UnityToRosPosition(transform.position);
        pos = pos + AWSIM.Environment.Instance.MgrsOffsetPosition;
        pos = positionNoise.ApplyNoiseOnVector(pos);


        //set speed
        Rigidbody rigidbody = transform.GetComponent<Rigidbody>();
        Vector3 speed = rigidbody.velocity;
        speed = speedNoise.ApplyNoiseOnVector(speed);
        speed = ROS2Utility.UnityToRosPosition(speed);


        //rotation base on bus todo check correctness
        Vector3 rot = transform.rotation.eulerAngles;
        rot = rotationNoise.ApplyNoiseOnVector(rot);
        Quaternion r = ROS2Utility.UnityToRosRotation(Quaternion.Euler(rot));


        if (lineOfSightComponent is null)
        {
            // the autonomous car :)
            row =
                $"{state},{posIndex},{iterationNumber},{posShift.x},{posShift.y},{posShift.z},{angle},{transform.name},{pos.x},{pos.y},{pos.z},{r.w},{r.x},{r.y},{r.z},{speed.x},{speed.y},{speed.z},{rosTime.Sec},{rosTime.Nanosec},{Time.frameCount}\n";
        }
        else
        {
            // StartCoroutine(lineOfSightComponent.CheckLineOfSight()); //this line of code is very important
            lineOfSightComponent.checkImmidiately(); // todo i never test this function specifically for this component but it tested on the similar log components 
            //it will recaculate line of sight for the element to each sensor 
            List<MockSensor> sensors = lineOfSightComponent.GetObservableSensors();
            string sensorsNames = "";
            for (int i = 0; i < sensors.Count; i++)
            {
                sensorsNames += sensors[i].getName() + $"*{lineOfSightComponent.GetNumberOfDetectedPoint(sensors[i])}*" + "|";
            }


            row =
                $"{state},{posIndex},{iterationNumber},{posShift.x},{posShift.y},{posShift.z},{angle},{transform.name},{pos.x},{pos.y},{pos.z},{r.w},{r.x},{r.y},{r.z},{speed.x},{speed.y},{speed.z},{rosTime.Sec},{rosTime.Nanosec} ,{Time.frameCount},{lineOfSightComponent.GetCarBoxState()},{sensorsNames}\n";
        }


        return row;
    }

    // public IEnumerator WaitForNFrame(int n)
    // {
    //     for (int i = 0; i < n; i++)
    //     {
    //         yield return null;
    //     }
    // }

    public static void AppendStringToFile(string filePath, string content)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            writer.Write(content);
        }
    }

    private void HandleCsvHeader()
    {
        if (logPath != "" && !File.Exists(logPath))
        {
            AppendStringToFile(logPath,
                "state,pos_index,iteration_number,pos_shift_x,pos_shift_y,pos_shift_z,rotation_angle,Name,X,Y,Z,W rotation,X rotation,Y rotation,Z rotation,Speed_X,Speed_Y,Speed_Z,Time_sec,Time_nano,Frame,Box_State,Sensor Names \n");
        }

   
    }


    private void ScanTheEnvironment()
    {
        Collider[] hitColliders = Physics.OverlapSphere(this.transform.position, radius);

        carTransforms = new List<Transform>();
        humanTransforms = new List<Transform>();

        // Debug.Log($"number founded colliders {hitColliders.Length}");

        foreach (var other in hitColliders)
        {
            if (other.gameObject.GetComponentInParent<NPCVehicle>() != null)
            {
                Transform parentTransform = other.gameObject.GetComponentInParent<NPCVehicle>().transform;

                if (!carTransforms.Contains(parentTransform))
                {
                    carTransforms.Add(other.gameObject.GetComponentInParent<NPCVehicle>().transform);
                }
            }
            else if (other.gameObject.GetComponentInParent<NPCPedestrian>() != null)
            {
                Transform parentTransform = other.gameObject.GetComponentInParent<NPCPedestrian>().transform;

                if (!humanTransforms.Contains(parentTransform))
                {
                    humanTransforms.Add(other.gameObject.GetComponentInParent<NPCPedestrian>().transform);
                }
            }
            else if (other.GameObject().layer == 6) //6 layer is Vehicle
            {
                //just for bus
                Rigidbody rigidbody = FindRigidbodyInHierarchy(other.GameObject());
                if (rigidbody != null)
                {
                    if (!carTransforms.Contains(rigidbody.transform))
                    {
                        carTransforms.Add(rigidbody.transform);
                    }
                }
            }
        }
    }
}