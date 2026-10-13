using UnityEditor;
using UnityEngine;

namespace V2X.Paths.Editor
{
    [CustomEditor(typeof(PathConfigLoader))]
    public class PathConfigLoaderEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var loader = (PathConfigLoader)target;
            GUILayout.Space(8f);
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Load Paths Now"))
                {
                    loader.LoadPaths();
                }
                if (GUILayout.Button("Save Paths Now"))
                {
                    loader.SavePaths();
                }
            }
        }
    }
}
