using System;
using System.Collections;
using System.Collections.Generic;
using std_msgs.msg;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using UnityEngine.Serialization;
using UnityEngine.UI;

public class BlindScenarioManager : Singleton<BlindScenarioManager>
{
    


    // Start is called before the first frame update

    [SerializeField] private List<Scenario> scenarios;

    // [SerializeField] private Transform mainCamera;


    // [SerializeField] private Transform busLidar;

    [SerializeField] private Scenario currentScenario;
    // [SerializeField] private Transform currentRSU;

    [SerializeField] private int busLidarThreshold = 3;
    [SerializeField] private int busCameraThreshold = 3;
    [FormerlySerializedAs("RSUThreshold")] [SerializeField] private int rsuThreshold = 2;
    [SerializeField] private int rsuCameraThreshold = 2;

    [SerializeField] private float checkFrequency = 2;




    
    //to use this feature you must add serialize field for them to attach them to something but additional lines approximately are deprecated from this project and use gizmos only 
    private Toggle cameraLinesToggle;
    private Toggle rsuLinesToggle;
    private Toggle lidarLinesToggle;


    //serializeField is just for test remove it in future
    private bool showCameraLine = false;
    private bool showLidarLine = false;
    private bool showRsuLine = false;
    private bool showRsuCameraLine = false;


    private LineRenderer[] lineRenderersPool;


    //this function call by UI
    public void SetShowCameraLine(Toggle toggle)
    {
        showCameraLine = toggle.isOn;
    }

    //this function call by config
    public void SetShowCameraLine(bool active)
    {
        showCameraLine = active;
        cameraLinesToggle.isOn = active;
    }

    //this function call by UI
    public void SetShowRsuLine(Toggle toggle)
    {
        showRsuLine = toggle.isOn;
    }

    //this function call by config
    public void SetShowRsuLine(bool active)
    {
        showRsuLine = active;
        rsuLinesToggle.isOn = active;
    }

    //this function call by UI
    public void SetShowLidarLine(Toggle toggle)
    {
        showLidarLine = toggle.isOn;
    }

    //this function call by config
    public void SetShowLidarLine(bool active)
    {
        showLidarLine = active;
        lidarLinesToggle.isOn = active;
    }

    public bool GetShowLidarLine()
    {
        return showLidarLine;
    }

    public bool GetShowRsuLine()
    {
        return showRsuLine;
    }

    public bool GetShowCameraLine()
    {
        return showCameraLine;
    }

    public bool GetShowRsuCameraLine()
    {
        return showRsuCameraLine;
    }
    
    private void Awake()
    {

        //check active scenario
        currentScenario = null;
        for (int i = 0; i < scenarios.Count; i++)
        {
            if (scenarios[i].gameObject.activeSelf)
            {
                currentScenario = scenarios[i];
            }
        }

        //if all scenarios are off do that 
        if (currentScenario == null)
        {
            scenarios[0].gameObject.SetActive(true);
            currentScenario = scenarios[0];
        }
    }


    public void ChangScenario(Text btn)
    {
        deactivateCurrentScanrio();

        int currentScenarioIndex = int.Parse(btn.text) - 1;
        currentScenario = scenarios[currentScenarioIndex];

        ActivateScenario(currentScenarioIndex);
    }

    private void deactivateCurrentScanrio()
    {
        //deactivate current scenario
        currentScenario.turnOffScenario();
        currentScenario.transform.gameObject.SetActive(false);
    }

    // public void SwapCamera()
    // {
    //     //toggle cameras
    //     bool cameraState = mainCamera.gameObject.activeSelf;
    //     mainCamera.gameObject.SetActive(!cameraState);
    //     currentScenario.setCameraActivation(cameraState);
    // }


    public Scenario getCurrentScenario()
    {
        return currentScenario;
    }


    public int GetBusLidarThreshold()
    {
        return busLidarThreshold;
    }

    public int GetBusCameraThreshold()
    {
        return busCameraThreshold;
    }

    public int GetRsuThreshold()
    {
        return rsuThreshold;
    }
    
    public int GetRsuCameraThreshold()
    {
        return rsuCameraThreshold;
    }
    


    public void SetBusLidarThreshold(Text text)
    {
        try
        {
            busLidarThreshold = int.Parse(text.text);
        }
        catch (Exception e)
        {
            Debug.Log(e);
        }
    }

    public void SetRsuThreshold(Text text)
    {
        try
        {
            rsuThreshold = int.Parse(text.text);
        }
        catch (Exception e)
        {
            Console.WriteLine(e);
        }
    }


    //for deactivation just need to setActive to false
    //but for activation you need some assignments that have been done in this funcitons 
    private void ActivateScenario(int index)
    {
        currentScenario = scenarios[index];
        currentScenario.transform.gameObject.SetActive(true);


        // // if main camera is active bus camera must be off, on the other hand if main camera is off the bus camera will turn on  
        // currentScenario.setCameraActivation(!mainCamera.gameObject.activeSelf);
    }


    public float GetFrequency()
    {
        return checkFrequency;
    }


    public void ResetScene()
    {
        SceneManager.LoadScene(0);
    }
    
}