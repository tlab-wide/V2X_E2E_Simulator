using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;
using YamlDotNet.Core.Tokens;

public class Scenario : MonoBehaviour
{
    private static Color rsuLineColor = Color.green;
    private static Color lidarLineColor = Color.blue;
    private static Color cameraLineColor = Color.cyan;
    private static Color cameraRsuLineColor = Color.green;


    [SerializeField] private List<MockDetectionSensor> carCameras; 
    [SerializeField] private List<MockDetectionSensor> carLidars;

    [FormerlySerializedAs("RSUs")] [SerializeField]
    private List<MockDetectionSensor> rsuSensors;

    [SerializeField] private List<MockDetectionSensor> rsuCameras;

    [SerializeField] private Transform mainBusCamera;


    // [SerializeField] private GameObject linePoolParent;

    [SerializeField] private Dictionary<MockDetectionSensor, List<Transform>> lines;

    [SerializeField] private Transform preferredPositionForMainCamera = null;


    private LineRenderer[] lineRenderersPool;

    private void Awake()
    {
        if (lines == null)
        {
            lines = new Dictionary<MockDetectionSensor, List<Transform>>();
        }
    }


    private void OnEnable()
    {
        if (preferredPositionForMainCamera != null)
        {
            Camera camera = Camera.main;
            // Debug.Log("khast taghir beder");
            // Debug.Log(camera.gameObject.name);

            camera.transform.position = preferredPositionForMainCamera.position;
            camera.transform.rotation = preferredPositionForMainCamera.rotation;
        }
        else
        {
            // Debug.Log("vard entekhab nashodesh");
        }
    }


    // depricated
    // private void Start()
    // {
    //     if (linePoolParent != null)
    //     {
    //         lineRenderersPool = linePoolParent.GetComponentsInChildren<LineRenderer>();
    //     }
    // }


    public void addSensorsList(MockDetectionSensor mockDetectionSensor, List<Transform> target)
    {
        if (lines == null)
        {
            lines = new Dictionary<MockDetectionSensor, List<Transform>>();
        }

        if (lines.ContainsKey(mockDetectionSensor))
        {
            lines[mockDetectionSensor] = target;
        }
        else
        {
            lines.Add(mockDetectionSensor, target);
        }
    }


    public void toggleCam()
    {
        mainBusCamera.gameObject.SetActive(!mainBusCamera.gameObject.activeSelf);
    }


    public void turnOffScenario()
    {
        mainBusCamera.gameObject.SetActive(false);
    }


    public void setCameraActivation(bool isActive)
    {
        mainBusCamera.gameObject.SetActive(isActive);
    }

    public List<MockDetectionSensor> getLidars()
    {
        return carLidars;
    }

    public List<MockDetectionSensor> getCameras()
    {
        return carCameras;
    }

    public List<MockDetectionSensor> getRSUsensors()
    {
        return rsuSensors;
    }

    public List<MockDetectionSensor> getRsuCameras()
    {
        return rsuCameras;
    }

    public Transform getMainCamera()
    {
        return mainBusCamera;
    }

    // depricated
    // private void Update()
    // {
    //     // draw lines and config line renderers
    //     if (linePoolParent.activeSelf && lineRenderersPool != null && lineRenderersPool.Length != 0)
    //     {
    //         int lineIndex = 0;
    //         foreach (KeyValuePair<MockSensor, List<Transform>> line in lines)
    //         {
    //             for (int i = 0; i < line.Value.Count; i++)
    //             {
    //                 Transform target = line.Value[i];
    //                 lineRenderersPool[lineIndex].enabled = true;
    //                 try
    //                 {
    //                     lineRenderersPool[lineIndex].SetPosition(0, line.Key.transform.position);
    //                     lineRenderersPool[lineIndex].SetPosition(1, target.position);
    //                 }
    //                 catch (Exception e)
    //                 {
    //                     // this try catch remove the objects that have been remove because they reach at the end. 
    //                     line.Value.Remove(target);
    //                     continue;
    //                 }
    //
    //
    //                 //chose color of line
    //                 Material material = lineRenderersPool[lineIndex].material;
    //                 switch (line.Key.getMockSensorType())
    //                 {
    //                     case MockSensor.MockSensorType.OBU_LIDAR:
    //                         if (BlindScenarioManager.Instance.GetShowLidarLine())
    //                         {
    //                             material.SetColor("_BaseColor", lidarLineColor);
    //                         }
    //                         else
    //                         {
    //                             continue;
    //                         }
    //
    //                         break;
    //                     case MockSensor.MockSensorType.RSU_RSU:
    //                         if (BlindScenarioManager.Instance.GetShowRsuLine())
    //                         {
    //                             material.SetColor("_BaseColor", rsuLineColor);
    //                         }
    //                         else
    //                         {
    //                             continue;
    //                         }
    //
    //                         break;
    //                     case MockSensor.MockSensorType.OBU_CAMERA:
    //                         if (BlindScenarioManager.Instance.GetShowCameraLine())
    //                         {
    //                             material.SetColor("_BaseColor", cameraLineColor);
    //                         }
    //                         else
    //                         {
    //                             continue;
    //                         }
    //
    //                         break;
    //                     case MockSensor.MockSensorType.RSU_CAMERA:
    //                         if (BlindScenarioManager.Instance.GetShowRsuCameraLine())
    //                         {
    //                             material.SetColor("_BaseColor", cameraRsuLineColor);
    //                         }
    //                         else
    //                         {
    //                             continue;
    //                         }
    //
    //                         break;
    //                 }
    //
    //                 lineRenderersPool[lineIndex].material = material;
    //
    //                 lineIndex++;
    //             }
    //         }
    //
    //         while (lineIndex < lineRenderersPool.Length)
    //         {
    //             lineRenderersPool[lineIndex].enabled = false;
    //             lineIndex++;
    //         }
    //     }
    // }
}