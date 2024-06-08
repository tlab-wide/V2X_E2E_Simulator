using System;
using System.Collections;
using System.Collections.Generic;
using std_msgs.msg;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UIElements;

public class MockSensor : MonoBehaviour
{
    [SerializeField] private string name = "sensor1";
    [SerializeField] private string category = "LIDAR";

    [SerializeField] private MockSensorType mockSensorType = MockSensorType.OBU_LIDAR;

    [SerializeField] private float Hfov = 180;

    [FormerlySerializedAs("observableAngleY")] [SerializeField]
    private float Vfov = 180;

    [SerializeField] private List<Transform> seenObjects = new List<Transform>();
    [SerializeField] private float maxDistance = 50;

    private Viewcone myCone;


    public MockSensorType getMockSensorType()
    {
        return mockSensorType;
    }

    private void OnEnable()
    {
        BlindScenarioManager.Instance.getCurrentScenario().addSensorsList(this, seenObjects);
    }

    public string getName()
    {
        return this.name;
    }

    private void Start()
    {
        ConfigFile.Instance.AddToConfig(this);
        CheckConfigExist();

        //setupCone
        myCone = GetComponentInChildren<Viewcone>();
        if (myCone != null)
        {
            if (ConfigFile.Instance.IsConeActive(mockSensorType))
            {
                myCone.Length = maxDistance;
                myCone.Rebuild();

                // TO reduce 
                float Xangle = Hfov >= 70 ? 70 : Hfov;
                Xangle = Xangle * Mathf.Deg2Rad;

                float Yangle = Vfov >= 70 ? 70 : Vfov;
                Yangle = Yangle * Mathf.Deg2Rad;

                myCone.transform.localScale = new Vector3((float)(Mathf.Tan(Xangle) * myCone.Length),
                    (float)(Mathf.Tan(Yangle) * myCone.Length), 1);
            }
            else
            {
                myCone.gameObject.SetActive(false);
            }
        }
    }

    public bool haveLineOfSight(Transform targetPoint)
    {
        if (!this.gameObject.activeSelf)
        {
            // Debug.Log("I know its off");
            return false;
        }

        // if (CheckRaycast(targetPoint.position, this.transform.position) && objectInview(targetPoint))
        if (objectInview(targetPoint) && CheckRaycast(targetPoint.position, this.transform.position))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private bool objectInview(Transform targetPoint)
    {
        Vector3 targetVector = targetPoint.position - this.transform.position;

        //this if reduce amount of process and ignore far car from the max depth in calculation
        if (targetVector.magnitude > maxDistance)
        {
            return false;
        }

        Vector3 targetInXY = Vector3.ProjectOnPlane(targetVector, this.transform.right);
        float angleY = Vector3.Angle(this.transform.forward, targetInXY);
        if (angleY > 90)
        {
            angleY = 180 - angleY;
        }

        if (angleY > Vfov)
        {
            return false;
        }


        Vector3 targetInXZ = Vector3.ProjectOnPlane(targetVector, this.transform.up);
        float angleX = Vector3.Angle(this.transform.forward, targetInXZ);


        if (angleX > Hfov)
        {
            return false;
        }
        else
        {
            return true;
        }
    }


    public void addObjectToObserved(Transform targetPoint)
    {
        if (!seenObjects.Contains(targetPoint))
        {
            seenObjects.Add(targetPoint);
        }
    }

    public void removeObjectFromObserved(Transform targetPoint)
    {
        seenObjects.Remove(targetPoint);
    }


    private bool CheckRaycast(Vector3 startPos, Vector3 targetPos)
    {
        Vector3 direction = targetPos - startPos;
        Ray ray = new Ray(startPos, direction);


        RaycastHit hit;

        // Cast the ray and ignore the specified layer.
        bool hasLine = Physics.Raycast(ray, out hit, maxDistance);
        if (hasLine && hit.collider.gameObject.tag.Equals("MockLidar"))
        {
            Debug.DrawLine(ray.origin, hit.point, Color.green);
            return true;
        }
        else
        {
            if (hasLine)
            {
                Debug.DrawRay(ray.origin, direction, Color.yellow);
            }
            else
            {
                Debug.DrawRay(ray.origin, -ray.direction * maxDistance,
                    Color.red);
            }

            return false;
        }
    }

    public List<Transform> GetSeenObjects()
    {
        RemoveDeletedObjects();
        return seenObjects;
    }

    private void RemoveDeletedObjects()
    {
        int i = 0;

        while (i<seenObjects.Count)
        {
            if (seenObjects[i] == null)
            {
                seenObjects.RemoveAt(i);
            }
            else
            {
                i++;
            }
        }
    }

    //this function returns feature of sensor as class that can store in config file
    public ConfigFile.ConfigSensor GetConfigSensor()
    {
        Vector3 angles = transform.localRotation.eulerAngles;

        ConfigFile.ConfigSensor configSensor = new ConfigFile.ConfigSensor(mockSensorType, name, category,
            this.gameObject.activeSelf, transform.localPosition.x, transform.localPosition.y,
            transform.localPosition.z, angles.x, angles.y, angles.z);

        return configSensor;
    }


    private void CheckConfigExist()
    {
        ConfigFile.ConfigSensor configSensor = ConfigFile.Instance.GetSensorConfig(name);

        if (configSensor == null)
        {
            Debug.LogWarning($"{name} : config is null");
            return;
        }

        if (!configSensor.isEnable)
        {
            Debug.Log("i know is dissables");
            this.gameObject.SetActive(false);
            return;
        }

        //set positions
        this.transform.localPosition = new Vector3(configSensor.x_pos, configSensor.y_pos, configSensor.z_pos);

        transform.localRotation =
            Quaternion.Euler(new Vector3(configSensor.x_rotation, configSensor.y_rotation, configSensor.z_rotation));
        category = configSensor.category;

        maxDistance = configSensor.maxDistance;

        // cone setup
        ConfigFile.SensorCategory sensorCategory = ConfigFile.Instance.GetMyCategoryConfig(category);

        if (sensorCategory == null)
        {
            Debug.Log("category is null");
            return;
        }

        Hfov = sensorCategory.HFOV;
        Vfov = sensorCategory.VFOV;
    }


    [Serializable]
    public enum MockSensorType
    {
        OBU_LIDAR = 1,
        RSU_RSU = 2,
        OBU_CAMERA = 3,
        RSU_CAMERA = 4,
    }
}