using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using AWSIM;
using Unity.VisualScripting;
using UnityEngine;
using Random = UnityEngine.Random;

public class LogAutonomous : MonoBehaviour
{
    [SerializeField] private string logPathCars;
    [SerializeField] private string spawnSelectionLogPath = "AutonomousSpawnValidation.csv";
    [SerializeField, Tooltip("Optional prefix for autonomous logs (applied to both CSV files).")] private string logFilePrefix = "Auto";

    // public int waitFrames = 30;
    public float waitTime = 0.005f;


    private List<Transform> carCollisions =new List<Transform>();
    private List<Transform> humanCollisions = new List<Transform>();

    private Rigidbody rb;
    private bool spawnSelectionHeaderWritten;
    private string _runLogPathCars;
    private string _runSpawnSelectionLogPath;

    private void Awake()
    {
        rb = transform.GetComponent<Rigidbody>();
        _runLogPathCars = BuildRunFilePath(logPathCars);
        _runSpawnSelectionLogPath = BuildRunFilePath(spawnSelectionLogPath);
        // set up header of CSV files
        StartCoroutine(HandleCsvHeaderBus());
        EnsureSpawnSelectionLogHeader();
        
        StartCoroutine(SaveLogs());
    }


    
    //log entire map
    private IEnumerator SaveLogs()
    {
        // this line can be remove, but i rather to wait one step in start of logSystem
        yield return WaitForNFrame(Random.Range(3,30)); 
        
        while (true)
        {
            string rowLog = "";
            
            //set position
            var pos = ROS2Utility.UnityToRosPosition(transform.position);
            pos = pos + AWSIM.Environment.Instance.MgrsOffsetPosition;
            rowLog += $"{pos.x},{pos.y},{pos.z}";
            
            //set rotation
            
            //rotation base on bus todo check correctness
            Quaternion r = ROS2Utility.UnityToRosRotation(transform.rotation);
            rowLog += $",{r.w},{r.x},{r.y},{r.z}";
            
            
            //velocity
            Vector3 speed = rb.linearVelocity;
            speed= ROS2Utility.UnityToRosPosition(speed);
            rowLog +=  $",{speed.x},{speed.y},{speed.z}";
            
            
            //Acceleration
            Vector3 acc = rb.linearVelocity;
            acc= ROS2Utility.UnityToRosPosition(acc);
            rowLog +=  $",{acc.x},{acc.y},{acc.z}";
            
            //add car collisions
            string carCollisionNames = makeListToString(carCollisions);
            rowLog += $",{carCollisionNames}";
            
            //add car collisions
            string humanCollisionNames = makeListToString(humanCollisions);
            rowLog += $",{humanCollisionNames},";

            //time
            builtin_interfaces.msg.Time time = SimulatorROS2Node.GetCurrentRosTime();

            rowLog += $"{time.Sec},{time.Nanosec} \n";
            
            
            CsvEditorUtils.AppendStringToFile(_runLogPathCars,rowLog);
            yield return new WaitForSeconds(waitTime);
            // yield return WaitForNFrame(waitFrames);
        }
    }

    private string makeListToString(List<Transform> transforms)
    {
        if (transforms.Count ==0)
        {
            return " ";
        }
        
        string result ="";
        for (int i = 0; i < transforms.Count; i++)
        {
            result += transforms[i].name+"|";
        }

        result = result.Remove(result.Length-1);

        return result;

    }


    public IEnumerator  WaitForNFrame(int n)
    {
        for (int i = 0; i < n; i++)
        {
            yield return null;
        }
    }

    // public int getWaitFrames()
    // {
    //     return waitFrames;
    // }
    
    
    private IEnumerator HandleCsvHeaderBus() //todo can be remove and use CsvEditorUtils
    {
        if (_runLogPathCars != "" && !File.Exists(_runLogPathCars))
        {
            CsvEditorUtils.AppendStringToFile(_runLogPathCars, "X,Y,Z,W rotation,X rotation,Y rotation,Z rotation, X velocity ,Y velocity, Z velocity,X acceleration,Y acceleration, Z acceleration, list of car collisions, list of human collisions , time ,nano \n");
        }
        yield return null;
    }

    private void EnsureSpawnSelectionLogHeader()
    {
        if (spawnSelectionHeaderWritten)
            return;

        if (string.IsNullOrEmpty(_runSpawnSelectionLogPath))
        {
            spawnSelectionHeaderWritten = true;
            return;
        }

        if (!File.Exists(_runSpawnSelectionLogPath))
        {
            CsvEditorUtils.AppendStringToFile(_runSpawnSelectionLogPath, "midpoint_segment,spawn_lane,sample_index,distance_to_midpoint_m,estimated_time_s,target_time_s,reached_cruise,target_x,target_y,target_z,ros_time_sec,ros_time_nsec\n");
        }

        spawnSelectionHeaderWritten = true;
    }
    
    
    
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

    private void OnTriggerEnter(Collider other)
    {
        //check human or car
        if (other.GameObject().layer == 6)
        {
            
            Rigidbody rb = FindRigidbodyInHierarchy(other.transform.GameObject());
            if (rb is null)
            {
                Debug.LogWarning($"Rigidbody is not found in vehicle (gameobject:{other.GameObject().name})");
                return;
            }

            Transform parent = rb.transform;

            if (!humanCollisions.Contains(parent))
            {
                humanCollisions.Add(parent);
            }

        }
    }
    
    
    private void OnTriggerExit(Collider other)
    {
        //check human or car
        if (other.GameObject().layer == 6)
        {
            
            Rigidbody rb = FindRigidbodyInHierarchy(other.transform.GameObject());
            if (rb is null)
            {
                Debug.LogWarning($"Rigidbody is not found in vehicle (gameobject:{other.GameObject().name})");
                return;
            }

            Transform parent = rb.transform;

            humanCollisions.Remove(parent);

        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.GameObject().layer == 6)
        {
            
            Rigidbody rb = FindRigidbodyInHierarchy(collision.transform.GameObject());
            if (rb is null)
            {
                Debug.LogWarning($"Rigidbody is not found in vehicle (gameobject:{collision.transform.name})");
                return;
            }

            Transform parent = rb.transform;
            
            if (!carCollisions.Contains(parent))
            {
                carCollisions.Add(parent);
            }

        }
    }


    private void OnCollisionExit(Collision other)
    {
        if (other.transform.GameObject().layer == 6)
        {
            
            Rigidbody rb = FindRigidbodyInHierarchy(other.transform.GameObject());
            if (rb is null)
            {
                Debug.LogWarning($"Rigidbody is not found in vehicle (gameobject:{other.transform.name})");
                return;
            }

            Transform parent = rb.transform;

            carCollisions.Remove(parent);

        }
    }

    public void LogSpawnSelection(string midpointSegment, string spawnLane, int sampleIndex, float distanceToMidpoint, float estimatedTime, float targetTime, bool reachedCruise, Vector3 targetPosition)
    {
        EnsureSpawnSelectionLogHeader();
        if (string.IsNullOrEmpty(_runSpawnSelectionLogPath))
            return;

        builtin_interfaces.msg.Time time = SimulatorROS2Node.GetCurrentRosTime();
        string row =
            $"{midpointSegment},{spawnLane},{sampleIndex},{distanceToMidpoint},{estimatedTime},{targetTime},{(reachedCruise ? 1 : 0)},{targetPosition.x},{targetPosition.y},{targetPosition.z},{time.Sec},{time.Nanosec}\n";
        CsvEditorUtils.AppendStringToFile(_runSpawnSelectionLogPath, row);
    }

    private string BuildRunFilePath(string basePath)
    {
        if (string.IsNullOrEmpty(basePath))
            return string.Empty;

        string directory = Path.GetDirectoryName(basePath);
        string name = Path.GetFileNameWithoutExtension(basePath);
        string ext = Path.GetExtension(basePath);
        if (string.IsNullOrEmpty(ext))
            ext = ".csv";
        string prefix = string.IsNullOrEmpty(logFilePrefix) ? string.Empty : (logFilePrefix + "_");
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string runName = $"{prefix}{name}_{timestamp}{ext}";
        if (string.IsNullOrEmpty(directory))
            return runName;
        return Path.Combine(directory, runName);
    }
}
