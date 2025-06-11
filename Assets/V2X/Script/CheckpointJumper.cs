using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CheckpointJumper : MonoBehaviour,ILogIndex
{
    [SerializeField] private float stopTime = 100;
    [SerializeField] private Transform targetCar;
    [SerializeField]private int currentIndex = 0;

    private float timer = 0;
    
    private List<Transform> positions = new List<Transform>();

    // Start is called before the first frame update
    void Start()
    {
        Transform[] transforms = this.GetComponentsInChildren<Transform>();
        for (int i = 1; i < transforms.Length; i++)
        {
            positions.Add(transforms[i]);
        }
        
        SetupNewPos(currentIndex);
    }

    // Update is called once per frame
    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= stopTime)
        {
            Debug.Log($"{currentIndex} index pos process is finished");
        }

        if (timer >= stopTime && currentIndex < positions.Count-1)
        {
            timer = 0;
            currentIndex++;
            SetupNewPos(currentIndex);
        }
    }

    private void SetupNewPos(int index)
    {
        targetCar.position = positions[index].position;
        targetCar.rotation = positions[index].rotation;
    }

    public int GetIndex()
    {
        return currentIndex;
    }
}