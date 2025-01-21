using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using AWSIM;
using Unity.VisualScripting;
using UnityEngine;
using Object = System.Object;
using String = std_msgs.msg.String;

[RequireComponent(typeof(BoxCollider))]
public class LogArea : MonoBehaviour
{
    [SerializeField] private string logPathHumans;
    [SerializeField] private string logPathCars;


    [SerializeField] private bool checkHumans;
    [SerializeField] private bool checkCars;

    [SerializeField] private float waitTime = 0.05f;
    [SerializeField] private int waitForFrame = 2;
    [SerializeField] private bool ultraMode = true;

    [SerializeField] private CheckpointJumper checkpointJumper;
    
    private ILogIndex logIndexBus;
    
    [SerializeField]private List<Transform> humanTransforms = new List<Transform>();
    [SerializeField]private List<Transform> carTransforms = new List<Transform>();


    private string myVariable;

    // Start is called before the first frame update
    void Start()
    {
        logIndexBus = checkpointJumper != null ? checkpointJumper : null;
        
        // set up header of CSV files
        StartCoroutine(HandleCsvHeader());

        StartCoroutine(SaveLogs(waitTime, ultraMode));
    }


    private void OnTriggerExit(Collider other)
    {
        // Debug.Log("entered");
        // Debug.Log(other.name);

        // Debug.Log(other.name);
        if (other.gameObject.GetComponentInParent<NPCVehicle>() != null)
        {
            carTransforms.Remove(other.gameObject.GetComponentInParent<NPCVehicle>().transform);
        }
        else if (other.gameObject.GetComponentInParent<NPCPedestrian>() != null)
        {
            humanTransforms.Remove(other.gameObject.GetComponentInParent<NPCPedestrian>().transform);
        }
        else if (other.GameObject().layer == 6) //6 layer is Vehicle
        {
            //just for bus
            Rigidbody rigidbody = FindRigidbodyInHierarchy(other.GameObject());
            if (rigidbody != null)
            {
                carTransforms.Remove(rigidbody.transform);
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        // Debug.Log($"bug bus : {other.name}");
        
        if (checkCars && other.gameObject.GetComponentInParent<NPCVehicle>() != null)
        {
            carTransforms.Add(other.gameObject.GetComponentInParent<NPCVehicle>().transform);
        }
        else if (checkHumans && other.gameObject.GetComponentInParent<NPCPedestrian>() != null)
        {
            if (!humanTransforms.Contains(other.gameObject.GetComponentInParent<NPCPedestrian>().transform))
            {
                // LineOfSight lineOfSightComponent = transform.GetComponent<LineOfSight>();
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


    private IEnumerator SaveLogs(float waitingTime, bool ultraMode = false)
    {
        // this line can be remove, but i rather to wait one step in start of logSystem
        yield return WaitForNFrame(UnityEngine.Random.Range(20, 30));

        if (!ultraMode)
        {
            while (true)
            {
                for (int i = 0; i < carTransforms.Count; i++)
                {
                    string row = dataRowGenerator(carTransforms[i]);
                    if (row.Equals(""))
                    {
                        Debug.Log($"removing {i}");
                        carTransforms.RemoveAt(i);
                        i--;
                        continue;
                    }

                    AppendStringToFile(logPathCars, row);
                }


                for (int i = 0; i < humanTransforms.Count; i++)
                {
                    string row = dataRowGenerator(humanTransforms[i]);
                    if (row.Equals(""))
                    {
                        Debug.Log($"removing {i}");
                        humanTransforms.RemoveAt(i);
                        i--;
                        continue;
                    }

                    AppendStringToFile(logPathHumans, row);
                }

                // foreach (Transform human in humanTransforms)
                // {
                //     string row = dataRowGenerator(human);
                //     ;
                //     // $"{human.name},{human.position.x},{human.position.y},{human.position.z},{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff},{Time.frameCount}\n";
                //     AppendStringToFile(logPathHumans, row);
                // }

                yield return new WaitForSeconds(waitingTime);
            }
        }
        else
        {
            while (true)
            {
                // foreach (Transform car in carTransforms)
                // {
                //     string row = dataRowGenerator(car);
                //     AppendStringToFile(logPathCars, row);
                // }
                //
                //
                // foreach (Transform human in humanTransforms)
                // {
                //     string row = dataRowGenerator(human);
                //     // $"{human.name},{human.position.x},{human.position.y},{human.position.z},{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff},{Time.frameCount}\n";
                //     AppendStringToFile(logPathHumans, row);
                // }

                for (int i = 0; i < carTransforms.Count; i++)
                {
                    string row = dataRowGenerator(carTransforms[i]);
                    if (row.Equals("")) // it means object has dead
                    {
                        Debug.Log($"removing {i}");
                        carTransforms.RemoveAt(i);
                        i--;
                        continue;
                    }

                    AppendStringToFile(logPathCars, row);
                }


                for (int i = 0; i < humanTransforms.Count; i++)
                {
                    string row = dataRowGenerator(humanTransforms[i]);
                    if (row.Equals(""))
                    {
                        Debug.Log($"removing {i}");
                        humanTransforms.RemoveAt(i);
                        i--;
                        continue;
                    }

                    AppendStringToFile(logPathHumans, row);
                }

                yield return WaitForNFrame(waitForFrame);
            }
        }
    }

    private string dataRowGenerator(Transform transform)
    {
        try
        {
            // if (transform is null || transform.GetComponent<LineOfSight>() is null)
            if (transform is null)
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
        lineOfSightComponent?.checkImmidiately(); //to ensure state of jumpers
        string row;

        //time
        builtin_interfaces.msg.Time rosTime = SimulatorROS2Node.GetCurrentRosTime();


        //set position
        var pos = ROS2Utility.UnityToRosPosition(transform.position);
        pos = pos + AWSIM.Environment.Instance.MgrsOffsetPosition;


        //rotation base on bus todo check correctness
        Quaternion r = ROS2Utility.UnityToRosRotation(transform.rotation);


        int stateBus = logIndexBus?.GetIndex() ?? 0;
        
        if (lineOfSightComponent is null)
        {
            
            row =
                $"{transform.name},{pos.x},{pos.y},{pos.z},{r.w},{r.x},{r.y},{r.z},{rosTime.Sec},{rosTime.Nanosec},{Time.frameCount},,,{stateBus}\n";
        }
        else
        {
            List<MockSensor> sensors = lineOfSightComponent.GetObservableSensors();
            string sensorsNames = "";
            for (int i = 0; i < sensors.Count; i++)
            {
                sensorsNames += sensors[i].getName()+ $"*{lineOfSightComponent.GetNumberOfDetectedPoint(sensors[i])}*" + "|";
            }


            row =
                $"{transform.name},{pos.x},{pos.y},{pos.z},{r.w},{r.x},{r.y},{r.z},{rosTime.Sec},{rosTime.Nanosec} ,{Time.frameCount},{lineOfSightComponent.GetCarBoxState()},{sensorsNames},{stateBus}\n";
        }


        return row;
    }

    public IEnumerator WaitForNFrame(int n)
    {
        for (int i = 0; i < n; i++)
        {
            yield return null;
        }
    }

    public static void AppendStringToFile(string filePath, string content)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            writer.Write(content);
        }
    }

    private IEnumerator HandleCsvHeader()
    {
        if (checkCars && logPathCars != "" && !File.Exists(logPathCars))
        {
            AppendStringToFile(logPathCars,
                "Name,X,Y,Z,W rotation,X rotation,Y rotation,Z rotation,Time_sec,Time_nano,Frame,Box_State,Sensor Names,Index\n");
        }

        if (checkHumans && logPathHumans != "" && !File.Exists(logPathHumans))
        {
            AppendStringToFile(logPathHumans,
                "Name,X,Y,Z,W rotation,X rotation,Y rotation,Z rotation,Time_sec,Time_nano,Frame,Box_State,Sensor Names,Index\n");
        }

        yield return null;
    }
}