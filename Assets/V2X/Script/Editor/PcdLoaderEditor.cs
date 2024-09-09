using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PcdLoader))]
public class PcdLoaderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        
        
        PcdLoader myScript = (PcdLoader)target;
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
        
        
        
        
    }
}