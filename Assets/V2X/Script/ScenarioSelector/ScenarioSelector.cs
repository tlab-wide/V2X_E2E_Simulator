using System.Collections.Generic;
using UnityEngine;

public class ScenarioSelector : MonoBehaviour
{
    [SerializeField] private List<Transform> scenarios;

    [SerializeField] private Transform bus;
    [SerializeField] private Transform car;

    [SerializeField] private Transform startPoint1;
    [SerializeField] private Transform startPoint2;
    [SerializeField] private Transform startPoint3;
    [SerializeField] private Transform startPoint4;

    private Transform targetTransform;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Optionally, you can initialize all scenarios to be inactive at the start
        DeactivateAllScenarios();
        ActivateScenario(1);


        if (bus.gameObject.activeInHierarchy)
        {
            targetTransform = bus.transform;
        }
        else
        {
            targetTransform = car.transform;
        }
    }

    // Method to activate a specific scenario and deactivate all others
    public void ActivateScenario(int scenarioIndex)
    {
        // Check if the scenarioIndex is within the bounds of the scenarios list
        if (scenarioIndex >= 0 && scenarioIndex < scenarios.Count)
        {
            // Deactivate all scenarios first
            DeactivateAllScenarios();

            // Activate the selected scenario
            scenarios[scenarioIndex].gameObject.SetActive(true);
        }
        else
        {
            Debug.LogWarning("Scenario index out of range: " + scenarioIndex);
        }
    }

    // Method to deactivate all scenarios
    private void DeactivateAllScenarios()
    {
        foreach (Transform scenario in scenarios)
        {
            scenario.gameObject.SetActive(false);
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // You can add any update logic here if needed
        if (Input.GetKeyDown(KeyCode.Alpha1)) // For the '1' key
        {
            ActivateScenario(1);
        }

        if (Input.GetKeyDown(KeyCode.Alpha2)) // For the '2' key
        {
            ActivateScenario(2);
        }

        if (Input.GetKeyDown(KeyCode.Alpha3)) // For the '3' key
        {
            ActivateScenario(3);
        }

        if (Input.GetKeyDown(KeyCode.Alpha4)) // For the '4' key
        {
            ActivateScenario(4);
        }

        if (Input.GetKeyDown(KeyCode.Alpha5)) // For the '5' key
        {
            ActivateScenario(5);
        }

        if (Input.GetKeyDown(KeyCode.Alpha6)) // For the '6' key
        {
            ActivateScenario(6);
        }

        if (Input.GetKeyDown(KeyCode.Alpha7)) // For the '7' key
        {
            ActivateScenario(7);
        }

        if (Input.GetKeyDown(KeyCode.Alpha8)) // For the '8' key
        {
            ActivateScenario(8);
        }

        if (Input.GetKeyUp(KeyCode.Alpha9)) // For the '9' key
        {
            ActivateScenario(9);
        }

        if (Input.GetKeyDown(KeyCode.Alpha0)) // For the '0' key
        {
            ActivateScenario(0);
        }

        if (Input.GetKeyDown(KeyCode.Q))
        {
            targetTransform.transform.position = startPoint1.position;
            targetTransform.transform.rotation = startPoint1.rotation;
            targetTransform.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
            targetTransform.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            targetTransform.transform.position = startPoint2.position;
            targetTransform.transform.rotation = startPoint2.rotation;
            targetTransform.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
            targetTransform.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        }


        if (Input.GetKeyDown(KeyCode.F))
        {
            targetTransform.transform.position = startPoint3.position;
            targetTransform.transform.rotation = startPoint3.rotation;
            targetTransform.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
            targetTransform.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        }

        if (Input.GetKeyDown(KeyCode.X))
        {
            targetTransform.transform.position = startPoint4.position;
            targetTransform.transform.rotation = startPoint4.rotation;
            targetTransform.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
            targetTransform.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        }
    }
}