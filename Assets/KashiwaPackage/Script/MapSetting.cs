using System.Collections;
using System.Collections.Generic;
using AWSIM.TrafficSimulation;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;

public class MapSetting : MonoBehaviour
{
    [Header("Replacer")] [SerializeField] private Transform replaceParent;

    [FormerlySerializedAs("newParentToReplace")] [SerializeField]
    private Transform newParentToJoin;

    [SerializeField] private GameObject prefabObjToReplace;
    [SerializeField] private int overSampling = 1;


    [Header("Point cloud")] [SerializeField]
    private List<TextAsset> _textAssets;

    [SerializeField] private Transform parentCloudPoints;

    [SerializeField] private GameObject prefabCloudPoint;
    [SerializeField] private GameObject prefabCloudPointWithGizmos;

    [SerializeField] private bool activeIntesity;


    [SerializeField] private float sampleRatePCD = 0.01f;
    // Start is called before the first frame update

    [SerializeField] private Color baseColorPCD = Color.yellow;


    [SerializeField] private float createdObj = 0;


    //read from csv create objects as child
    [Header("Traffic lights boxes")]
    public Transform parentObject; // The parent object to hold the instantiated prefabs

    public GameObject prefab; // The prefab to instantiate
    public List<TextAsset> csvFiles; // The CSV file containing positions

    public void ReadCSVAndInstantiate()
    {
        foreach (TextAsset csvFile in csvFiles)
        {
            // Split the CSV into lines
            string[] lines = csvFile.text.Split('\n');
            foreach (string line in lines)
            {
                // Split each line into x, y, z valuesa
                string[] values = line.Split(',');
                if (values.Length <3)
                {
                    values = line.Split(' ');
                }
                
                if (values.Length == 3)
                {
                    // Parse the position values
                    float x, y, z;
                    if (float.TryParse(values[0], out x) && float.TryParse(values[1], out y) &&
                        float.TryParse(values[2], out z))
                    {
                        // Instantiate the prefab at the parsed position and convert axis 
                        Vector3 position = new Vector3(-y, z, x);
                        Instantiate(prefab, position, Quaternion.identity, parentObject);
                    }
                }
            }
        }
    }


    private void Start()
    {
        // SwapTrees();
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
                if (Random.Range(0.0f, 1.0f) < sampleRatePCD)
                {
                    //todo instantiate the new object
                    InstantiateNewObject(points[j]);
                    createdObj += 1;
                }
            }
        }
    }


    public void replaceTheChilds()
    {
        Debug.Log("catched");

        TrafficLane[] trafficLanes = replaceParent.GetComponentsInChildren<TrafficLane>();


        for (int i = 0; i < trafficLanes.Length; i++)
        {
            Vector3[] wayPoints = trafficLanes[i].Waypoints;
            for (int j = 0; j < wayPoints.Length; j++)
            {
                Vector3 stepSize;
                if (wayPoints.Length > j + 1)
                {
                    stepSize = (wayPoints[j + 1] - wayPoints[j]) / overSampling;
                    for (int k = 0; k < overSampling; k++)
                    {
                        Debug.Log("WE ARE IN");
                        Instantiate(prefabObjToReplace, wayPoints[j] + k * stepSize, Quaternion.identity,
                            newParentToJoin);
                    }
                }
                else
                {
                    Instantiate(prefabObjToReplace, wayPoints[j], Quaternion.identity, newParentToJoin);
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

        // Debug.Log(SparsedPointData[0]);
        float x = float.Parse(SparsedPointData[0]);
        float y = float.Parse(SparsedPointData[1]);
        float z = float.Parse(SparsedPointData[2]);
        Vector3 newPos = new Vector3(x, z, y);


        if (!activeIntesity)
        {
            Instantiate(prefabCloudPoint, newPos, Quaternion.identity, parentCloudPoints);
            return;
        }
        else
        {
            // this mode requires significant amount of ram 

            GameObject newPoint =
                Instantiate(prefabCloudPointWithGizmos, newPos, Quaternion.identity, parentCloudPoints);
            //apply intensity
            if (SparsedPointData.Length > 3)
            {
                if (SparsedPointData[3] != "")
                {
                    float intensity = float.Parse(SparsedPointData[3]);
                    intensity /= 15000;
                    newPoint.gameObject.GetComponent<GizmoPCD>().gizmoColor = baseColorPCD * intensity;
                }
            }
        }
    }

    public void CleanObjects()
    {
        createdObj = 0;
        Transform[] transforms = parentCloudPoints.GetComponentsInChildren<Transform>();
        Debug.Log(transforms.Length);
        for (int i = 1; i < transforms.Length; i++)
        {
            //remove all childs
            GameObject.DestroyImmediate(transforms[i].gameObject);
        }
    }


    [SerializeField] private GameObject[] treeObjects;


    [SerializeField] private Vector3 shiftVector3 = new Vector3(0, 0, -1);

    [SerializeField] private Vector3 treeScale = new Vector3(0.3f, 0.3f, 0.3f);

    [SerializeField] private bool removePrev = false;

    public void SwapTrees()
    {
        GameObject[] trees = GameObject.FindGameObjectsWithTag("OldTree");
        foreach (var prevTree in trees)
        {
            Mesh mesh = prevTree.GetComponent<MeshFilter>().sharedMesh;
            Vector3[] vertices = mesh.vertices;

            Vector3 sum = Vector3.zero;
            for (int i = 0; i < vertices.Length; i++)
            {
                sum += vertices[i];
            }

            Vector3 avg = sum / vertices.Length;

            Vector3 newPos = avg;
            newPos += shiftVector3;


            GameObject SelectTreePrefab = SelectRandomTreePrefabObjects();
            GameObject newTreeObject = Instantiate(SelectTreePrefab, Vector3.zero, Quaternion.identity);
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


    private GameObject SelectRandomTreePrefabObjects()
    {
        int minIndex = 0;
        int maxIndex = treeObjects.Length;

        int selectedIndex = Random.Range(minIndex, maxIndex);

        return treeObjects[selectedIndex];
    }

    // if objects without mesh render has collider lidar sensors get in to problem therefor
    // this function must be used to remove collider object of them 
    [SerializeField] private GameObject rootSearchForEmptyObjects;

    public void removeColliderOfEmptyMeshRenders()
    {
        MeshCollider[] meshColliders = rootSearchForEmptyObjects.GetComponentsInChildren<MeshCollider>();
        // MeshFilter[] meshFilters = rootSearchForEmptyObjects.GetComponentsInChildren<MeshFilter>();

        for (int i = 0; i < meshColliders.Length; i++)
        {
            MeshFilter meshFilters = meshColliders[i].GetComponent<MeshFilter>();

            if (meshFilters.mesh.vertices.Length == 0)
            {
                Destroy(meshColliders[i]);
            }
        }
    }
}