
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


//Inspectorに絶対座標を出すスクリプト
//https://taiyakisun.hatenablog.com/entry/2021/03/23/223852

[CustomEditor(typeof(Transform))]
public class viewWorldPotision : Editor
{
    Transform _t =null; 

    private void OnEnable()
    {
         _t = target as Transform;
    }

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        EditorGUILayout.Vector3Field("World Position", _t.position);
    }
}

/*
using UnityEngine;
using UnityEditor;

public static class DebugMenu
{
    [MenuItem("Debug/Print Global Position")]
    public static void PrintGlobalPosition()
    {
        if (Selection.activeGameObject != null)
        {
            Debug.Log(Selection.activeGameObject.name + " is at " + Selection.activeGameObject.transform.position);
        }
    }

*/

