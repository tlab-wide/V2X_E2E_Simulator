// Inspector of ConfigBootstrap: the scenario is picked from the folders in config/scenarios, with buttons to open
// the folder and to make the scenario the default (config/default_scenario.txt).
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace V2X.Config
{
    [CustomEditor(typeof(ConfigBootstrap))]
    public class ConfigBootstrapEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            var root = ConfigLoader.ConfigRoot();
            var scenarios = ConfigLoader.ListScenarios(root);
            var defaultFile = Path.Combine(root, "default_scenario.txt");
            var current = File.Exists(defaultFile) ? File.ReadAllText(defaultFile).Trim() : "";

            var prop = serializedObject.FindProperty("scenario");
            var options = new[] { $"(default: {(current == "" ? "none" : current)})" }.Concat(scenarios).ToArray();
            int index = string.IsNullOrEmpty(prop.stringValue) ? 0 : scenarios.IndexOf(prop.stringValue) + 1;
            if (index == 0 && !string.IsNullOrEmpty(prop.stringValue))
            {
                EditorGUILayout.HelpBox($"Scenario '{prop.stringValue}' is not in {root}/scenarios.", MessageType.Warning);
                index = 0;
            }
            int picked = EditorGUILayout.Popup(new GUIContent("Scenario", "Folder under config/scenarios. Default = config/default_scenario.txt. " +
                                                                           "--scenario on the command line wins."), index, options);
            if (picked != index)
                prop.stringValue = picked == 0 ? "" : scenarios[picked - 1];

            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.FlexibleSpace();
                var name = string.IsNullOrEmpty(prop.stringValue) ? current : prop.stringValue;
                using (new EditorGUI.DisabledScope(string.IsNullOrEmpty(prop.stringValue) || prop.stringValue == current))
                    if (GUILayout.Button("Make default", GUILayout.Width(110)))
                    {
                        File.WriteAllText(defaultFile, prop.stringValue + "\n");
                        prop.stringValue = "";
                    }
                using (new EditorGUI.DisabledScope(string.IsNullOrEmpty(name)))
                    if (GUILayout.Button("Open folder", GUILayout.Width(110)))
                        EditorUtility.RevealInFinder(Path.Combine(root, "scenarios", name, "scenario.yaml"));
            }
            if (scenarios.Count == 0)
                EditorGUILayout.HelpBox($"No scenarios in {root}. The scene's own setup is used.", MessageType.Info);
            EditorGUILayout.HelpBox("The files are read when Play starts. Edit them in the web viewer (Settings) or by hand.", MessageType.None);

            EditorGUILayout.Space();
            DrawPropertiesExcluding(serializedObject, "m_Script", "scenario");
            serializedObject.ApplyModifiedProperties();
        }
    }
}
