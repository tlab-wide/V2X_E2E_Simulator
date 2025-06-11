using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;

public class PcdLoader : MonoBehaviour
{
    [SerializeField] private List<TextAsset> _textAssets;
    [SerializeField] private Transform parent;

    [SerializeField] private GameObject prefabPoint;


    [SerializeField] private float sampleRate = 0.01f;
    // Start is called before the first frame update


    [SerializeField] private float createdObj = 0;


    private void Start()
    {
        SwapTrees();
    }

    public void CreateAll()
    {
        
        Debug.Log("called");
        for (int i = 0; i < _textAssets.Count; i++)
        {
            string pos = _textAssets[i].text;
            string[] points = pos.Split('\n');

            for (int j = 0; j < points.Length; j++)
            {
                if (Random.Range(0.0f, 1.0f) < sampleRate)
                {
                    //todo instantiate the new object
                    InstantiateNewObject(points[j]);
                    createdObj += 1;
                }
            }
        }
    }


    public void InstantiateNewObject(string data)
    {
        string[] SparsedPointData = data.Split(',');

        if (SparsedPointData[0] == "" || SparsedPointData[1] == "" || SparsedPointData[2] == "")
        {
            return;
        }

        float x = float.Parse(SparsedPointData[0]);
        float y = float.Parse(SparsedPointData[1]);
        float z = float.Parse(SparsedPointData[2]);
        Vector3 newPos = new Vector3(x, y, z);


        Instantiate(prefabPoint, newPos, Quaternion.identity, parent);
    }

    public void CleanObjects()
    {
        createdObj = 0;
        Transform[] transforms = parent.GetComponentsInChildren<Transform>();
        Debug.Log(transforms.Length);
        for (int i = 1; i < transforms.Length; i++)
        {
            //remove all childs
            GameObject.DestroyImmediate(transforms[i].gameObject);
        }
    }

    
    
    [SerializeField] private GameObject treeObject;

    [SerializeField] private Vector3 shiftVector3 = new Vector3(0, 0, -1);

    [SerializeField] private Vector3 treeScale = new Vector3(0.3f, 0.3f, 0.3f);

    [SerializeField] private bool removePrev = false;
    
    public void SwapTrees()
    {

        GameObject[] trees = GameObject.FindGameObjectsWithTag("OldTree");
        foreach (var prevTree in trees)
        {
            Mesh mesh = prevTree.GetComponent<MeshFilter>().mesh;
            Vector3[] vertices = mesh.vertices;

            Vector3 sum = Vector3.zero;
            for (int i = 0; i < vertices.Length; i++)
            {
                sum += vertices[i];
            }

            Vector3 avg = sum / vertices.Length;

            Vector3 newPos = avg;
            newPos += shiftVector3;

            GameObject newTreeObject = Instantiate(treeObject, Vector3.zero, Quaternion.identity);
            newTreeObject.transform.localScale = treeScale;
            newTreeObject.transform.position = newPos;
            newTreeObject.transform.parent = prevTree.transform.parent;

            //todo  delete the tree 
            if (removePrev)
            {
                DestroyImmediate(prevTree.gameObject);
            }
        }
    }


    
    
}