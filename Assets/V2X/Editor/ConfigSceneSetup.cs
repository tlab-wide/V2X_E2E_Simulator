// Adds (or updates) the ConfigBootstrap object of the open scene.
// Menu: V2X > Config > Add Config Bootstrap To Scene. Batch mode: -executeMethod V2X.Config.ConfigSceneSetup.SetupBatch
//   [-exportScene <scene path>] [-exportScenario <name>]  (also exports the scene to config/ when the folder is missing)
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace V2X.Config
{
    public static class ConfigSceneSetup
    {
        const string LidarPrefab = "Assets/V2X/Prefabs/Pseudo Sensors/RSU.prefab";
        const string CameraPrefab = "Assets/V2X/Prefabs/Pseudo Sensors/RSU_Camera.prefab";
        static readonly string[] Replaced = { "Pseudo Sensors setup", "CooL4 Object Info (DM)" };

        [MenuItem("V2X/Config/Add Config Bootstrap To Scene")]
        public static void SetupMenu()
        {
            Debug.Log("[ConfigSceneSetup] " + Setup());
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }

        public static void SetupBatch()
        {
            var scene = ConfigLoader.GetArg("-exportScene");
            if (!string.IsNullOrEmpty(scene))
                EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            if (ConfigLoader.GetArg("-exportRaw") != null)
                Debug.Log("[ConfigSceneSetup] " + ConfigExporter.ExportRawSensors(ConfigLoader.GetArg("-exportScenario") ?? "kashiwa_cool4"));
            if (!Directory.Exists(ConfigLoader.ConfigRoot()))
                Debug.Log("[ConfigSceneSetup] " + ConfigExporter.Export(ConfigLoader.GetArg("-exportScenario") ?? "kashiwa_cool4"));
            Debug.Log("[ConfigSceneSetup] " + Setup());
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
            EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene());
            EditorApplication.Exit(0);
        }

        public static string Setup()
        {
            var all = Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            var bootstrap = Object.FindObjectsByType<ConfigBootstrap>(FindObjectsInactive.Include, FindObjectsSortMode.None).FirstOrDefault();
            if (bootstrap == null)
            {
                var go = new GameObject("Config Bootstrap");
                go.transform.SetAsFirstSibling();
                bootstrap = go.AddComponent<ConfigBootstrap>();
            }
            var so = new SerializedObject(bootstrap);
            so.FindProperty("lidarPrefab").objectReferenceValue = AssetDatabase.LoadAssetAtPath<GameObject>(LidarPrefab);
            so.FindProperty("cameraPrefab").objectReferenceValue = AssetDatabase.LoadAssetAtPath<GameObject>(CameraPrefab);
            var raw = ConfigExporter.RawSensorPrefabs();
            var rawList = so.FindProperty("rawSensorPrefabs");
            rawList.arraySize = raw.Count;
            for (int i = 0; i < raw.Count; i++) rawList.GetArrayElementAtIndex(i).objectReferenceValue = raw[i];
            var list = so.FindProperty("sceneObjectsReplacedByConfig");
            var targets = new List<GameObject>();
            foreach (var name in Replaced)
            {
                var t = all.FirstOrDefault(x => x.name == name);
                if (t != null) targets.Add(t.gameObject);
            }
            list.arraySize = targets.Count;
            for (int i = 0; i < targets.Count; i++) list.GetArrayElementAtIndex(i).objectReferenceValue = targets[i];
            so.ApplyModifiedPropertiesWithoutUndo();
            return $"ConfigBootstrap ready; replaces: {string.Join(", ", targets.Select(t => t.name))}";
        }
    }
}
