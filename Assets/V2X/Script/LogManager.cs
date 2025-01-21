using System;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using Object = System.Object;
using Random = UnityEngine.Random;
using String = std_msgs.msg.String;

public class LogManager : Singleton<LogManager>
{
    [SerializeField] private Transform carsParent;
    [SerializeField] private string logPathCars;

    public int waitFrames = 30;
    public float waitTime = 0.005f;


    private void Awake()
    {
        // set up header of CSV files
        StartCoroutine(HandleCsvHeaderBus());
        
        StartCoroutine(SaveLogs());
    }


    
    //log entire map
    private IEnumerator SaveLogs()
    {
        // this line can be remove, but i rather to wait one step in start of logSystem
        yield return WaitForNFrame(Random.Range(3,30)); 
        
        while (true)
        {
            List<Transform> carTransforms = carsParent.GetComponentsInChildren<Transform>().ToList();
        
            //remove parent transform
            carTransforms.RemoveAt(0);


            List<string> rows = new List<string>();
            for (int i = 0; i < carTransforms.Count; i++)
            {
                Transform car = carTransforms[i];
                string row = $"{car.name},{car.position.x},{car.position.y},{car.position.z},{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff},{Time.frameCount}\n";

                rows.Add(row);
                
                // AppendStringToFile(logPathCars, row);
                // yield return null;
            }
            
            AppendStringToFile(logPathCars,rows);
            yield return new WaitForSeconds(waitTime);
            // yield return WaitForNFrame(waitFrames);
        }
    }
    
    private static void AppendStringToFile(string filePath, List<string> contents)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            foreach (var content in contents)
            {
                writer.Write(content);
            }
        }
    }
    
    private static void AppendStringToFile(string filePath, string content)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            writer.Write(content);
        }
    }


    private IEnumerator test()
    {
        int n = 0;
        while (true)
        {
            n++;
            Debug.Log(n);
            Debug.Log(Time.frameCount);
            Debug.Log($"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}");
            // yield return new WaitForSeconds(1);
            yield return WaitForNFrame(300);
            
        }
        
    }

    public IEnumerator  WaitForNFrame(int n)
    {
        for (int i = 0; i < n; i++)
        {
            yield return null;
        }
    }

    public int getWaitFrames()
    {
        return waitFrames;
    }
    
    
    private IEnumerator HandleCsvHeaderBus()
    {
        if (logPathCars != "" && !File.Exists(logPathCars))
        {
            AppendStringToFile(logPathCars, "Name,X,Y,Z, Time, Frame \n");
        }
        yield return null;
    }
}
