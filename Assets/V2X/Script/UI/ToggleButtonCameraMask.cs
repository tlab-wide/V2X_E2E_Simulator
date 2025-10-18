using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ToggleButtonCameraMask : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Camera targetCamera;   // If left empty, falls back to Camera.main
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
        if (targetCamera == null) targetCamera = Camera.main;
        if (buttonLabel == null) buttonLabel = GetComponentInChildren<TextMeshProUGUI>(true);
    }

    private void Awake()
    {
        if (targetCamera == null) targetCamera = Camera.main;
    }

    private void Start()
    {
        // Initialize to default mask & label
        ApplyMask(usingDefault);
    }

    /// <summary>
    /// Hook this up to your Button's OnClick.
    /// Each call toggles the camera culling mask and the button text.
    /// </summary>
    public void ToggleBoundingBox()
    {
        usingDefault = !usingDefault;
        ApplyMask(usingDefault);
    }

    private void ApplyMask(bool useDefault)
    {
        if (targetCamera == null)
        {
            Debug.LogWarning($"{nameof(ToggleButtonCameraMask)}: No Camera assigned and no main camera found.");
            return;
        }

        targetCamera.cullingMask = useDefault ? defaultLayerMask : alternateLayerMask;

        if (buttonLabel != null)
        {
            buttonLabel.text = useDefault ? "Bounding Box On" : "Bounding Box Off";
        }
    }

    // Optional helpers if you want to change masks at runtime from other scripts
    public void SetDefaultMask(LayerMask mask)
    {
        defaultLayerMask = mask;
        if (usingDefault) ApplyMask(true);
    }

    public void SetAlternateMask(LayerMask mask)
    {
        alternateLayerMask = mask;
        if (!usingDefault) ApplyMask(false);
    }
}
