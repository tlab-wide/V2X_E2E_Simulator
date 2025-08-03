using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
#if UNITY_EDITOR
using UnityEditor;  // for SceneAsset
#endif

public class BuildSafeSceneLoader : MonoBehaviour
{
    [Header("Drag your .unity Scenes here (Editor only)")]
#if UNITY_EDITOR
    [SerializeField]
    private List<SceneAsset> sceneAssets = new List<SceneAsset>();
#endif

    [Tooltip("Automatically populated from SceneAssets")]
    [SerializeField]
    private List<string> sceneNames = new List<string>();

    private void OnValidate()
    {
#if UNITY_EDITOR
        // keep names in sync with assets
        sceneNames.Clear();
        foreach (var asset in sceneAssets)
        {
            if (asset != null)
                sceneNames.Add(asset.name);
        }
#endif
    }

    private void Start()
    {
        LoadAllAdditive();
    }

    /// <summary>
    /// Loads every scene in the sceneNames list additively.
    /// </summary>
    public void LoadAllAdditive()
    {
        if (sceneNames == null || sceneNames.Count == 0)
        {
            Debug.LogWarning($"[{nameof(BuildSafeSceneLoader)}] No scenes assigned!");
            return;
        }

        foreach (var name in sceneNames)
        {
            if (string.IsNullOrEmpty(name))
            {
                Debug.LogWarning($"[{nameof(BuildSafeSceneLoader)}] Encountered empty scene name – skipping.");
                continue;
            }

            SceneManager.LoadSceneAsync(name, LoadSceneMode.Additive);
        }
    }

    /// <summary>
    /// Call this at runtime (e.g. from your spectator UI) to override the list of scenes
    /// you want to load.
    /// </summary>
    public void SetScenesByName(List<string> names)
    {
        sceneNames = new List<string>(names);
    }
}