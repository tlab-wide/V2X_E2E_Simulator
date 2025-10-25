using System;
using AWSIM.TrafficSimulation;
using Unity.VisualScripting.YamlDotNet.Core.Tokens;
using UnityEngine;

public class TestTrafficRemovingNpc : MonoBehaviour
{
    [SerializeField] TrafficManager trafficManager;
    [SerializeField] Transform centerTransform;
    [SerializeField] float radius;
    [SerializeField] KeyCode RemoveKey = KeyCode.R;

    private void FixedUpdate()
    {
        if (Input.GetKeyDown(RemoveKey))
        {
            trafficManager.RemoveVehiclesInRadius(centerTransform, radius);
        }
    }
}
