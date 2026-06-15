using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ToggleButtonCameraMask : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private List<Camera> targetCameras;   // If left empty, falls back to Camera.main
    [SerializeField] private TextMeshProUGUI buttonLabel;      // The UI.Text on your button

    [Header("Culling Masks")]
    [Tooltip("Mask used when the toggle is in its default (ON) state.")]
    [SerializeField] private LayerMask defaultLayerMask;

    [Tooltip("Mask used when the toggle is in its alternate (OFF) state.")]
    [SerializeField] private LayerMask alternateLayerMask;

    // Tracks which mask is currently applied. True = defaultLayerMask
    private bool usingDefault = true;

    private void Reset()
    {
        if (targetCameras == null || targetCameras.Count == 0)
        {
            targetCameras = new List<Camera>();
            targetCameras.Add(Camera.main);
        }
        if (buttonLabel == null) buttonLabel = GetComponentInChildren<TextMeshProUGUI>(true);
    }

    private void Awake()
    {
        if (targetCameras == null || targetCameras.Count == 0)
        {
            targetCameras = new List<Camera>();
            if (Camera.main != null)
                targetCameras.Add(Camera.main);
            else
                Debug.LogWarning($"{nameof(ToggleButtonCameraMask)} on '{gameObject.name}': targetCameras is empty and Camera.main is null. Assign cameras in the Inspector.");
        }
    }

    private void Start()
    {
        // Initialize to default mask & label
        ApplyMask(usingDefault);
    }

    /// <summary>
    /// Hook this up to your Button's OnClick.
    /// Each call toggles bounding box visibility and the button text.
    /// Also changes the camera culling mask if target cameras are assigned.
    /// </summary>
    public void ToggleBoundingBox()
    {
        usingDefault = !usingDefault;
        LineOfSight.ShowBoxes = usingDefault;

        // Immediately hide/show all active boxes without waiting for the next coroutine tick
        var allLos = FindObjectsOfType<LineOfSight>();
        foreach (var los in allLos)
            los.ApplyBoxVisibility();

        ApplyMask(usingDefault);
    }

    private void ApplyMask(bool useDefault)
    {
        if (targetCameras == null || targetCameras.Count == 0)
        {
            Debug.LogWarning($"{nameof(ToggleButtonCameraMask)} on '{gameObject.name}': No cameras to apply mask to.");
            return;
        }

        foreach (var targetCamera in targetCameras)
        {
            if (targetCamera == null) continue;
            targetCamera.cullingMask = useDefault ? defaultLayerMask : alternateLayerMask;
        }

        if (buttonLabel != null)
        {
            buttonLabel.text = useDefault ? "Bounding Box On" : "Bounding Box Off";
        }
    }
}
