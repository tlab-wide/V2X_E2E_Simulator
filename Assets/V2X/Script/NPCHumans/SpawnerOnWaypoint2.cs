using System.Collections;
using System.Collections.Generic;
using Cinemachine;
using UnityEngine;
using UnityEngine.Serialization;

public class SpawnerOnWaypoint2 : MonoBehaviour
{
    [SerializeField] private List<SpawnPoint> spawnPoints = new List<SpawnPoint>();

    [SerializeField] private float minWaitingTime = 5f;
    [SerializeField] private float maxWaitingTime = 50;
    [SerializeField] private float randomSelectedWaitingTime = 0f;
    [SerializeField] private bool spawnRandomEnable = true;

    [SerializeField] private List<GameObject> prefabPool;


    private float passedTime = 0f;

    // Start is called before the first frame update
    void Start()
    {
        randomSelectedWaitingTime = Random.Range(minWaitingTime, maxWaitingTime);
    }

    // Update is called once per frame
    void Update()
    {
        passedTime += Time.deltaTime;
        if (spawnRandomEnable)
        {
            if (passedTime >= randomSelectedWaitingTime)
            {
                // Make the random  selection
                passedTime = 0f;
                randomSelectedWaitingTime = Random.Range(minWaitingTime, maxWaitingTime);

                GameObject selectedObject = GetRandomObjectToSpawn();
                SpawnPoint selectedSpawnPoint = GetRandomSpawnPoint();
                Transform selectedTransform = selectedSpawnPoint.ChooseRandomSpawnPoint();
                
                GameObject gameObject = Instantiate(selectedObject, selectedTransform.position, selectedTransform.rotation);
                gameObject.transform.parent = this.transform;
                
                WaypointFollower waypointFollower = gameObject.GetComponent<WaypointFollower>();
                waypointFollower.SetWaypointSystem(selectedSpawnPoint.GetWaypointSystem());
            }
        }
    }

    private GameObject GetRandomObjectToSpawn()
    {
        int randomIndex = Random.Range(0, prefabPool.Count);
        return prefabPool[randomIndex].gameObject;
    }

    private SpawnPoint GetRandomSpawnPoint()
    {
        int randomIndex = Random.Range(0, spawnPoints.Count);
        return spawnPoints[randomIndex];
    }
}

[System.Serializable]
public class SpawnPoint
{
    [SerializeField] private WaypointSystem waypointSystem;
    [SerializeField] private List<Transform> prfereedSpawnPoints;

    public Transform ChooseRandomSpawnPoint()
    {
        return prfereedSpawnPoints[Random.Range(0, prfereedSpawnPoints.Count)];
    }

    public WaypointSystem GetWaypointSystem()
    {
        return waypointSystem;
    }
}