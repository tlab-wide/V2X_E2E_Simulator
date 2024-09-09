using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using std_msgs.msg;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;

public class ConfigFile : Singleton<ConfigFile>
{
    // Create a field for the save file.
    [SerializeField] private string saveFileTemplate = "./configTemplate.json";
    [SerializeField] private string saveFileOriginal = "./config.json";


    // Create a GameData field.
    ConfigData configData = null;
    private ConfigData configDataTemplate = new ConfigData();

    void Awake()
    {
        configData = readFile();
        StartCoroutine(saveTemplateLogSystem(1));
    }

    private void Start()
    {
        SetupHintLines();
    }


    // this function will on/off showing lines in the screen for 
    // observability of the cars
    private void SetupHintLines()
    {
        // Debug.Log("inja omad");
        if (configData != null)
        {
            BlindScenarioManager.Instance.SetShowCameraLine(configData.isCamerasActive);
            BlindScenarioManager.Instance.SetShowLidarLine(configData.isLidarActive);
            BlindScenarioManager.Instance.SetShowRsuLine(configData.isRsuActive);
        }
    }


    //returns true if can load and cant find or not exist the file will return false
    public ConfigSensor GetSensorConfig(string name)
    {
        if (configData is null)
        {
            Debug.Log("No config exist");
            return null;
        }

        for (int i = 0; i < configData.AV_sensors.Count; i++)
        {
            if (configData.AV_sensors[i].name.Equals(name))
            {
                return configData.AV_sensors[i];
            }
        }

        for (int i = 0; i < configData.RSU_sonsors.Count; i++)
        {
            if (configData.RSU_sonsors[i].name.Equals(name))
            {
                return configData.RSU_sonsors[i];
            }
        }

        return null;
    }

    public SensorCategory GetMyCategoryConfig(string category)
    {
        if (configData is null)
        {
            Debug.Log("No config exist");
            return null;
        }

        for (int i = 0; i < configData.SensorCategoriesList.Count; i++)
        {
            if (configData.SensorCategoriesList[i].categoryName.Equals(category))
            {
                return configData.SensorCategoriesList[i];
            }
        }

        return null;
    }


    //add sensor details to template config 
    public void AddToConfig(MockSensor mockSensor)
    {
        if (configDataTemplate is null)
        {
            configDataTemplate = new ConfigData();
        }

        ConfigSensor sensor = mockSensor.GetConfigSensor();


        // Each sensor is either in AVs or RSUs.
        if (MockSensor.MockSensorType.OBU_LIDAR == mockSensor.getMockSensorType() ||
            MockSensor.MockSensorType.OBU_CAMERA == mockSensor.getMockSensorType())
        {
            configDataTemplate.AV_sensors.Add(sensor);
        }
        else
        {
            configDataTemplate.RSU_sonsors.Add(sensor);
        }
    }

    // private float[] convertVectorToArray(Vector3 vector3)
    // {
    //     return new float[] { vector3[0], vector3[1], vector3[2] };
    // }


    public ConfigData readFile()
    {
        // Does the file exist?
        if (File.Exists(saveFileOriginal))
        {
            // Read the entire file and save its contents.
            string fileContents = File.ReadAllText(saveFileOriginal);

            // Deserialize the JSON data 
            //  into a pattern matching the GameData class.

            return JsonUtility.FromJson<ConfigData>(fileContents);
        }

        Debug.LogWarning("+++ no found config file +++");
        return null;
    }

    public bool IsConeActive(MockSensor.MockSensorType mockSensorType)
    {
        if (configData is null)
        {
            return false;
        }

        switch (mockSensorType)
        {
            case MockSensor.MockSensorType.RSU_RSU:
                return configData.isRsuConeActive;
            case MockSensor.MockSensorType.OBU_LIDAR:
                return configData.isLidarConeActive;
            case MockSensor.MockSensorType.OBU_CAMERA:
                return configData.isCameraConeActive;
            case MockSensor.MockSensorType.RSU_CAMERA:
                return configData.isRsuCameraConeActive;
        }

        return false;
    }

    private IEnumerator saveTemplateLogSystem(float waitBeforeSave)
    {
        yield return new WaitForSeconds(waitBeforeSave);
        SensorCategory rsu = new SensorCategory("RSU", 15, 30);
        SensorCategory lidar = new SensorCategory("LIDAR", 15, 90);
        SensorCategory camera_leopard_180 = new SensorCategory("leopard_180", 90, 90);
        SensorCategory camera_leopard_40 = new SensorCategory("leopard_40", 20, 90);
        SensorCategory camera_tier_c1 = new SensorCategory("tier_c1", 60, 90);


        configDataTemplate.SensorCategoriesList.Add(rsu);
        configDataTemplate.SensorCategoriesList.Add(lidar);
        configDataTemplate.SensorCategoriesList.Add(camera_leopard_180);
        configDataTemplate.SensorCategoriesList.Add(camera_leopard_40);
        configDataTemplate.SensorCategoriesList.Add(camera_tier_c1);

        writeFile();
    }

    public void writeFile()
    {
        // Serialize the object into JSON and save string.
        string jsonString = JsonUtility.ToJson(configDataTemplate, true);

        // Write JSON to file.
        File.WriteAllText(saveFileTemplate, jsonString);
    }


    [System.Serializable]
    public class ConfigData
    {
        //these variables are case sensitive and must match the strings "firstName" and "lastName" in the JSON.
        public bool isCamerasActive;
        public bool isRsuActive;
        public bool isLidarActive;
        public bool isCameraConeActive;
        public bool isRsuConeActive;
        public bool isLidarConeActive;
        public bool isRsuCameraConeActive;


        public List<SensorCategory> SensorCategoriesList;
        public List<ConfigSensor> AV_sensors;
        public List<ConfigSensor> RSU_sonsors;


        public ConfigData()
        {
            this.isCamerasActive = true;
            this.isRsuActive = true;
            this.isLidarActive = true;


            this.AV_sensors = new List<ConfigSensor>();
            this.RSU_sonsors = new List<ConfigSensor>();
            this.SensorCategoriesList = new List<SensorCategory>();
        }
    }


    //subclass
    [System.Serializable]
    public class ConfigSensor
    {
        public MockSensor.MockSensorType SensorAsType;

        // public string SensorTypeAsString;
        public string name;
        public string category;
        public bool isEnable;
        public float maxDistance;
        public float x_pos;
        public float y_pos;
        public float z_pos;
        public float x_rotation;
        public float y_rotation;
        public float z_rotation;

        public ConfigSensor(MockSensor.MockSensorType sensorAsType, string name, string category, bool isEnable,
            float x_pos,
            float y_pos, float z_pos, float x_rotation, float y_rotation, float z_rotation, float maxDistance = 50)
        {
            this.SensorAsType = sensorAsType;
            // this.SensorTypeAsString = sensorAsType.DisplayName();
            this.name = name;
            this.category = category;
            this.isEnable = isEnable;
            this.x_pos = x_pos;
            this.y_pos = y_pos;
            this.z_pos = z_pos;

            this.x_rotation = x_rotation;
            this.y_rotation = y_rotation;
            this.z_rotation = z_rotation;

            this.maxDistance = maxDistance;
        }
    }


    //subclass
    [Serializable]
    public class SensorCategory
    {
        public string categoryName;
        public float HFOV;
        public float VFOV;

        public SensorCategory(string categoryName, float hfov, float vfov)
        {
            this.categoryName = categoryName;
            this.HFOV = hfov;
            this.VFOV = vfov;
        }
    }
}