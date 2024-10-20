using System;
using System.Collections;
using System.Collections.Generic;
using AWSIM;
using AWSIM.TrafficSimulation;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;
using Environment = AWSIM.Environment;
using Random = UnityEngine.Random;

public class MapSetting : MonoBehaviour
{
    [Header("Lanelet To Json ")] [SerializeField]
    private GameObject parentOfLanelet;


    [Header("Rename Tool")] [SerializeField]
    private string baseName = "lanelet";

    [SerializeField] private GameObject parentOfRename;


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
                if (values.Length < 3)
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


    public void RenameChildrenSequential()
    {
        TrafficLane[] trafficLanes = parentOfRename.GetComponentsInChildren<TrafficLane>();
        int counter = 1;

        foreach (TrafficLane lane in trafficLanes)
        {
            lane.transform.gameObject.name = baseName + counter;
            counter++;
        }
    }

    public void MakeCsvFromLaneLet()
    {
        TrafficLane[] trafficLanes = parentOfLanelet.GetComponentsInChildren<TrafficLane>();
        LaneLets laneLets = new LaneLets();

        for (int i = 0; i < trafficLanes.Length; i++)
        {
            Lane lane = new Lane(trafficLanes[i].name);
            Debug.Log(trafficLanes[i].name);
            lane.waypoints = lane.ConvertVector3ArrayToFloatList(trafficLanes[i].Waypoints);
            lane.prevLanes = lane.ExtractNameOfLanesFromList(trafficLanes[i].PrevLanes);
            lane.nextLanes = lane.ExtractNameOfLanesFromList(trafficLanes[i].NextLanes);

            if (trafficLanes[i].StopLine != null)
            {
                StopLine stopLine = trafficLanes[i].StopLine;

                Vector3 stopLinePoseP1 = ROS2Utility.UnityToRosPosition(
                                             new Vector3(stopLine.Points[0][0], stopLine.Points[0][1],
                                                 stopLine.Points[0][2])) +
                                         Environment.Instance.MgrsOffsetPosition;


                Vector3 stopLinePoseP2 = ROS2Utility.UnityToRosPosition(
                                             new Vector3(stopLine.Points[1][0], stopLine.Points[1][1],
                                                 stopLine.Points[1][2])) +
                                         Environment.Instance.MgrsOffsetPosition;


                lane.stopLinePoseP1 = new List<float>()
                    { stopLinePoseP1.x, stopLinePoseP1.y, stopLinePoseP1.z };
                lane.stopLinePoseP2 = new List<float>()
                    { stopLinePoseP2.x, stopLinePoseP2.y, stopLinePoseP2.z };

                if (stopLine.TrafficLight != null)
                {
                    TrafficLight trafficLight = stopLine.TrafficLight;

                    TrafficLightLaneletID laneletID = trafficLight.GetComponent<TrafficLightLaneletID>();


                    if (laneletID != null)
                    {
                        lane.trafficlightsWayIDs.Add(laneletID.wayID);
                    }
                }
            }

            laneLets.LaneLetsArray.Add(lane);
        }

        string save = JsonUtility.ToJson(laneLets, true);
        CsvEditorUtils.AppendStringToFile("laneletToJson.json", save);
        Debug.Log($"Done {laneLets.LaneLetsArray.Count}");
    }

    [Serializable]
    public class LaneLets
    {
        public List<Lane> LaneLetsArray;

        public LaneLets()
        {
            LaneLetsArray = new List<Lane>();
        }
    }

    [Serializable]
    public class Vector3Custom
    {
        public float x;
        public float y;
        public float z;

        public Vector3Custom(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }


    [Serializable]
    public class Lane
    {
        public string name;
        public List<Vector3Custom> waypoints;
        public List<string> prevLanes;
        public List<string> nextLanes;
        public List<long> trafficlightsWayIDs;
        public List<float> stopLinePoseP1;
        public List<float> stopLinePoseP2;

        public Lane(string name)
        {
            this.name = name;
            this.waypoints = new List<Vector3Custom>();
            this.prevLanes = new List<string>();
            this.nextLanes = new List<string>();
            this.stopLinePoseP1 = new List<float>();
            this.stopLinePoseP2 = new List<float>();
            this.trafficlightsWayIDs = new List<long>();
        }

        public List<Vector3Custom> ConvertVector3ArrayToFloatList(Vector3[] vector3List)
        {
            List<Vector3Custom> floatList = new List<Vector3Custom>();

            foreach (Vector3 vector in vector3List)
            {
                Vector3 converted = ROS2Utility.UnityToRosPosition(vector);
                converted = converted + Environment.Instance.MgrsOffsetPosition;
                Vector3Custom floatVector = new Vector3Custom(converted.x, converted.y, converted.z);

                floatList.Add(floatVector);
            }

            return floatList;
        }

        public List<string> ExtractNameOfLanesFromList(List<TrafficLane> trafficLanes)
        {
            List<string> names = new List<string>();

            if (trafficLanes.Count == 0 || trafficLanes == null)
            {
                return names;
            }


            foreach (TrafficLane trafficLane in trafficLanes)
            {
                if (trafficLane == null)
                {
                    Debug.Log("skipped");
                    continue;
                }

                names.Add(trafficLane.transform.name);
            }

            return names;
        }
    }


    public void RemoveUnnamedObjects()
    {
        // Find all root GameObjects in the current scene
        GameObject[] rootObjects = UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects();
        
        // Create a list to store objects that need to be removed
        List<GameObject> objectsToRemove = new List<GameObject>();

        // Iterate through all root objects and their children
        foreach (GameObject root in rootObjects)
        {
            CheckAndRemoveUnnamedObjects(root, objectsToRemove);
        }

        // Remove all the unnamed objects found
        foreach (GameObject obj in objectsToRemove)
        {
            Debug.Log("remove something");
            Destroy(obj);
        }
    }
    
    
    void CheckAndRemoveUnnamedObjects(GameObject obj, List<GameObject> objectsToRemove)
    {

        // Check if the object's name is null or empty
        if (string.IsNullOrEmpty(obj.name))
        {
            objectsToRemove.Add(obj);
        }

        // Recursively check all child objects
        foreach (Transform child in obj.transform)
        {
            CheckAndRemoveUnnamedObjects(child.gameObject, objectsToRemove);
        }
    }
    
}