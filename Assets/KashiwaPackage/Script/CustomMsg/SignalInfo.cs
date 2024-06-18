using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;


namespace AWSIM
{
    
    public class SignalInfo: MonoBehaviour
    {
        [SerializeField,
        Tooltip("traffic_signals publication frequency - number of publications per second"),
        Range(1.0f, 100.0f)]
        int outputHz = 10;

        [SerializeField]private int stationId = 1;
        [SerializeField]private TrafficLight[] allTrafficLights;

        
        public class OutputData
        {
            public int stationId;
            public TrafficLight[] trafficLights;
        }

        public delegate void OnOutputDataDelegate(OutputData outputData);
        public OnOutputDataDelegate OnOutputData;

        OutputData outputData = new OutputData();
        float timer = 0;

        void Start()
        {
            outputData.trafficLights = allTrafficLights.ToArray();
            outputData.stationId = stationId;
        }

        void FixedUpdate()
        {
            // Update timer.
            timer += Time.deltaTime;

            // Matching output to hz.
            var interval = 1.0f / (int)outputHz;
            interval -= 0.00001f;
            if (timer < interval)
                return;
            timer = 0;
            
            OnOutputData.Invoke(outputData);
        }
    }
}