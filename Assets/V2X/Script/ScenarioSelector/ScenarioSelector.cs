using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ScenarioSelector : MonoBehaviour
{
    // [SerializeField] private TMP_Dropdown dropdown;
    
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


    public void ApplyDropDownData(TMP_Dropdown dropdown)
    {
        int idPeaked = dropdown.value;
        DeactivateAllScenarios();
        ActivateScenario(idPeaked);
    }
}