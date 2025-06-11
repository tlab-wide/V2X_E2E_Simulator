using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MapSetting))]
public class MapSettingEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        MapSetting myScript = (MapSetting)target;

        if(GUILayout.Button("Replace Children"))
        {
            myScript.replaceTheChilds();
        }

        if(GUILayout.Button("Build Object"))
        {
            myScript.CreateAll();
        }
        
        if(GUILayout.Button("Clean Objects"))
        {
            myScript.CleanObjects();
        }
        
        if(GUILayout.Button("Swap Trees"))
        {
            myScript.SwapTrees();
        }
        
        if(GUILayout.Button("remove objects without any meshrender"))
        {
            myScript.removeColliderOfEmptyMeshRenders();
        }

        if(GUILayout.Button("creat traffic light box from csv"))
        {
            myScript.ReadCSVAndInstantiate();
        }
        
        if(GUILayout.Button("Rename sequential"))
        {
            myScript.RenameChildrenSequential();
        }
        
        if(GUILayout.Button("csv from lanelet"))
        {
            myScript.MakeCsvFromLaneLet();
        }
        
        if(GUILayout.Button("Remove un named objects"))
        {
            myScript.RemoveUnnamedObjects();
        }
        
        
        
    }
}
