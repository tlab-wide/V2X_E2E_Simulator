using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;


// designed for cars
public class SpawnerOnWaypoint : MonoBehaviour
{
    [SerializeField] private Transform initialSapwnPoint;
    [SerializeField] private GameObject spawnerPrefab;
    [SerializeField] private WaypointSystem waypointSystem;

    [SerializeField] private bool spawnRandomEnable = false;
    [SerializeField] private float minWaitingTime = 5f;
    [SerializeField] private float maxWaitingTime = 300f;
    [SerializeField]private float randomSelectedWaitingTime = 0f;
    
    [SerializeField]private KeyCode spawnKey = KeyCode.O;

    private Vector3 spawnPos;
    private Quaternion spawnRot;
    private float passedTime = 0f;

    private void Awake()
    {
        randomSelectedWaitingTime = Random.Range(minWaitingTime, maxWaitingTime);
        spawnPos = initialSapwnPoint.position;
        spawnRot = initialSapwnPoint.rotation;
    }


    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(spawnKey))
        {
            // Instantiate the object at spawnPos and spawnRot
            if (spawnerPrefab != null)
            {
                GameObject gameObject = Instantiate(spawnerPrefab, spawnPos, spawnRot);
                WaypointFollower waypointFollower = gameObject.GetComponent<WaypointFollower>();
                waypointFollower.SetWaypointSystem(waypointSystem);
                gameObject.transform.parent = this.transform;
                Debug.Log("Spawned an object at the waypoint.");
            }
            else
            {
                Debug.LogError("Spawner prefab is not assigned!");
            }
        }

        passedTime += Time.deltaTime;
        if (spawnRandomEnable)
        {
            if (passedTime >= randomSelectedWaitingTime)
            {
                passedTime = 0f;
                randomSelectedWaitingTime = Random.Range(minWaitingTime, maxWaitingTime);
                // Instantiate the object at spawnPos and spawnRot
                if (spawnerPrefab != null)
                {
                    GameObject gameObject = Instantiate(spawnerPrefab, spawnPos, spawnRot);
                    WaypointFollower waypointFollower = gameObject.GetComponent<WaypointFollower>();
                    waypointFollower.SetWaypointSystem(waypointSystem);
                    gameObject.transform.parent = this.transform;
                    Debug.Log("Spawned an object at the waypoint.");
                }
                else
                {
                    Debug.LogError("Spawner prefab is not assigned!");
                }
            }
        }
    }
}