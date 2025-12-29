using UnityEngine;
using System.Collections.Generic;

public class ToggleByKey : MonoBehaviour
{
    [Header("Key to toggle the objects")]
    public KeyCode toggleKey = KeyCode.None;

    [Header("Objects to toggle")]
    public List<GameObject> objectsToToggle = new List<GameObject>();

    private void Update()
    {
        if (toggleKey == KeyCode.None) return;

        if (Input.GetKeyDown(toggleKey))
        {
            ToggleObjects();
        }
    }

    private void ToggleObjects()
    {
        foreach (var obj in objectsToToggle)
        {
            if (obj != null)
                obj.SetActive(!obj.activeSelf);
        }
    }
}