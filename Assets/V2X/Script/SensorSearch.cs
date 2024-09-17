using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

public class SensorSettingController : MonoBehaviour
{
    [SerializeField] private BlindScenarioManager blindScenarioManagerReference;

    [SerializeField] private List<Transform> scanSensors;


    [SerializeField] private float minAngle = 0;
    [SerializeField] private float maxAngles = 360;
    [SerializeField] private int angleSteps = 2;


    [SerializeField] private Vector3 startPos = Vector3.zero;
    [SerializeField] private Vector3 endPos = Vector3.forward;
    [SerializeField] private int movementSteps = 2;


    private Scenario currentScenario;

    private List<Transform> sensors = new List<Transform>();

    private List<Vector3> initialPoses = new List<Vector3>();
    private List<Quaternion> initialRotation = new List<Quaternion>();

    private List<float> finalTestAngles = new List<float>();
    private List<Vector3> finalTestPoses = new List<Vector3>(); //this field is additive

    private int numberOfConditions;


    void Start()
    {
        currentScenario = blindScenarioManagerReference.getCurrentScenario();
        Debug.Log($"state of current scenario {currentScenario} is {currentScenario == null}");

        // collecting RSU sensor references in entire scene 
        foreach (MockSensor mockSensor in currentScenario.getRsuCameras())
        {
            sensors.Add(mockSensor.transform);
        }

        foreach (MockSensor mockSensor in currentScenario.getRSUsensors())
        {
            sensors.Add(mockSensor.transform);
        }

        foreach (Transform sensor in scanSensors)
        {
            sensors.Add(sensor);
        }

        for (int i = 0; i < sensors.Count; i++)
        {
            initialPoses.Add(sensors[i].position);
            initialRotation.Add(sensors[i].rotation);
        }

        //angle pre calculation  
        float stepSizeAngle = (maxAngles - minAngle) / (angleSteps);
        for (int i = 0; i < angleSteps; i++)
        {
            finalTestAngles.Add(minAngle + stepSizeAngle * i);
        }

        //movement pre calculation  
        Vector3 stepSizePos = (endPos - startPos) / (movementSteps);
        Vector3 targetPos = Vector3.zero;
        // finalTestPoses.Add(Vector3.zero);
        for (int i = 0; i < movementSteps; i++)
        {
            for (int j = 0; j < movementSteps; j++)
            {
                for (int k = 0; k < movementSteps; k++)
                {
                    targetPos = startPos + stepSizePos.z * i * Vector3.forward;
                    // finalTestPoses.Add(targetPos);

                    targetPos = targetPos + stepSizePos.y * j * Vector3.up;
                    // finalTestPoses.Add(targetPos);

                    targetPos = targetPos + stepSizePos.x * k * Vector3.right;
                    finalTestPoses.Add(targetPos);
                }
            }
        }

        Debug.Log("List state**");
        Debug.Log(finalTestPoses.ToList());
        Debug.Log(finalTestPoses);
        for (int i = 0; i < finalTestPoses.Capacity; i++)
        {
            Debug.Log(finalTestPoses[i]);
        }

        numberOfConditions = finalTestAngles.Count * finalTestPoses.Count;
    }


    private int state = 0;

    public int SetupNextState()
    {
        int angleStateIndex = GetAngleStateIndex(state);
        int posStateIndex = GetPosStateIndex(state);

        if (posStateIndex >= finalTestPoses.Count)
        {
            return -1; //it means iteration is completed
        }

        state++;
        applySetting(finalTestPoses[posStateIndex], finalTestAngles[angleStateIndex]);


        return state - 1;
    }

    public int ResetState()
    {
        state = 0;
        return state;
    }

    public Vector3 GetPos(int state)
    {
        if (this.isActiveAndEnabled == false)
        {
            return Vector3.zero;
        }
        int posStateIndex = GetPosStateIndex(state);
        return finalTestPoses[posStateIndex];
    }

    public float GetAngle(int state)
    {
        if (this.isActiveAndEnabled == false)
        {
            return 0;
        }
        int angleStateIndex = GetAngleStateIndex(state);
        return finalTestAngles[angleStateIndex];
    }


    public int GetState()
    {
        return state;
    }


    private void applySetting(Vector3 pos, float angle)
    {
        for (int i = 0; i < sensors.Count; i++)
        {
            sensors[i].position = initialPoses[i] + pos;
            Quaternion firstRotation = initialRotation[i];
            Vector3 newRotation = firstRotation.eulerAngles + Vector3.up * angle;
            sensors[i].eulerAngles = newRotation;
        }
    }


    public int GetAngleStateIndex(int stateIndex)
    {
        int angleStateIndex = stateIndex % finalTestAngles.Count;
        return angleStateIndex;
    }

    public int GetPosStateIndex(int stateIndex)
    {
        int posStateIndex = stateIndex / finalTestAngles.Count;
        return posStateIndex;
    }
}