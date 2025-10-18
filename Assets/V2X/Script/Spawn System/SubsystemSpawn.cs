using UnityEngine;
using System.Collections.Generic; // Required for using Lists

/// <summary>
/// A helper class to pair a GameObject with a specific KeyCode.
/// The [System.Serializable] attribute is crucial for it to show up in the Inspector.
/// </summary>
[System.Serializable]
public class SubsystemEntry
{
    public string name; // An optional name for easier identification in the Inspector
    public GameObject subsystemObject;
    public KeyCode toggleKey = KeyCode.None;
}


/// <summary>
/// This class manages a list of subsystems, where each can be toggled
/// by its own unique key.
/// </summary>
public class SubsystemSpawn : MonoBehaviour
{
    [Tooltip("The list of subsystems, each with its own toggle key.")]
    public List<SubsystemEntry> subsystems;

    // Update is called once per frame
    void Update()
    {
        // If the list is empty or not assigned, do nothing.
        if (subsystems == null) return;
        
        // Loop through each entry in our list
        foreach (SubsystemEntry entry in subsystems)
        {
            // Check if the specific key for THIS entry was pressed down
            if (Input.GetKeyDown(entry.toggleKey))
            {
                // Make sure the GameObject for this entry has been assigned
                if (entry.subsystemObject != null)
                {
                    // Toggle the active state of this specific GameObject
                    bool isActive = entry.subsystemObject.activeSelf;
                    entry.subsystemObject.SetActive(!isActive);
                }
            }
        }
    }
}