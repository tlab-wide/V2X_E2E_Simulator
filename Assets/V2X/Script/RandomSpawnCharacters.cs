using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class RandomSpawnCharacters : MonoBehaviour
{
    [SerializeField] private Transform ObjectPool;
    [SerializeField] private int eachPosWaitFrame = 4;

    private Transform[] childAreas;
    private LineOfSight[] humans;
    private float[] areaWeights;

    // Start is called before the first frame update
    void Awake()
    {
        childAreas = this.GetComponentsInChildren<Transform>();
        humans = ObjectPool.GetComponentsInChildren<LineOfSight>();
        CalculateAreaWeights();
    }
    
    // Method to calculate the area of each child area and normalize them to create weights
    void CalculateAreaWeights()
    {
        areaWeights = new float[childAreas.Length];
        float totalArea = 0;

        for (int i = 1; i < childAreas.Length; i++) // start from 1 to skip the parent
        {
            Vector3 scale = childAreas[i].localScale;
            float area = scale.x * scale.z; // assuming the areas are rectangles
            areaWeights[i] = area;
            totalArea += area;
        }

        // Normalize the areaWeights to sum to 1
        for (int i = 1; i < areaWeights.Length; i++)
        {
            areaWeights[i] /= totalArea;
        }
    }
    
    // Method to pick a weighted random section
    int GetWeightedRandomSection()
    {
        float randomValue = Random.value; // Get a random float between 0 and 1
        float cumulative = 0.0f;

        for (int i = 1; i < areaWeights.Length; i++) // start from 1 to skip the parent
        {
            cumulative += areaWeights[i];
            if (randomValue <= cumulative)
            {
                return i;
            }
        }

        return 1; // default fallback, though this should never be hit
    }


    void SamplePoints()
    {
        // first element is the father and we ignore that 

        for (int i = 0; i < humans.Length; i++)
        {
            // Todo  we must make changes here and make selection based on area of child areas
            int selectArea = GetWeightedRandomSection();
            // Debug.Log(selectArea);
            Transform selectedTransform = childAreas[selectArea];

            Vector3 centerPos = selectedTransform.position;

            Quaternion rotation = Quaternion.Euler(0, selectedTransform.rotation.eulerAngles.y, 0);

            Vector3 scale = selectedTransform.localScale;

            float x = Random.Range(-scale.x / 2, scale.x / 2);
            float z = Random.Range(-scale.z / 2, scale.z / 2);

            Vector3 samplePos = new Vector3(x, 0, z);

            samplePos = rotation * samplePos;


            Vector3 finalPos = samplePos + centerPos;

            humans[i].transform.position = finalPos;
        }
    }

    // Update is called once per frame

    private int timer = 0;

    void Update()
    {
        
        if (Time.frameCount - timer > (eachPosWaitFrame))
        {
            timer = Time.frameCount;
            SamplePoints();
        }
    }
}