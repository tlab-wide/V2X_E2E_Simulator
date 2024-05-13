using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Pedestrian straight back and forth.
    /// </summary>
    [RequireComponent(typeof(NPCPedestrian))]
    public class SimplePedestrianWalkerController2 : MonoBehaviour
    {
       [SerializeField] float duration;
        [SerializeField] float speed;
        [SerializeField] TrafficLight trafficLight;
        [SerializeField] private float CheckTrafficLightTime = 1.0f;

        NPCPedestrian npcPedestrian;
        Vector3 startPosition;
        Quaternion startRotation;
        Vector3 currentPosition;
        Quaternion currentRotation;

        void Awake()
        {
            npcPedestrian = GetComponent<NPCPedestrian>();
            startPosition = transform.position;
            startRotation = transform.rotation;
            currentPosition = transform.position;
            currentRotation = transform.rotation;
        }

        void Start()
        {
            StartCoroutine(Loop());
        }

        IEnumerator Loop()
        {
            while (true)
            {
                yield return WaitForTrafficLight(CheckTrafficLightTime);
                yield return MoveForwardRoutine(duration, speed);
                yield return RotateRoutine(0.5f, 360f);
                yield return WaitForTrafficLight(CheckTrafficLightTime);
                yield return MoveForwardRoutine(duration, speed);
                yield return RotateRoutine(0.5f, 360f);
                var npcTransformPos = npcPedestrian.transform.position;

                // reset
                npcPedestrian.SetPosition(startPosition);
                npcPedestrian.SetRotation(startRotation);
                currentPosition = startPosition;
                currentRotation = startRotation;
            }
        }

        IEnumerator MoveForwardRoutine(float duration, float speed)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                currentPosition += currentRotation * Vector3.forward * speed * Time.fixedDeltaTime;
                npcPedestrian.SetPosition(currentPosition);
            }
        }

        IEnumerator RotateRoutine(float duration, float angularSpeed)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                currentRotation *= Quaternion.AngleAxis(angularSpeed * Time.fixedDeltaTime, Vector3.up);
                npcPedestrian.SetRotation(currentRotation);
            }
        }

        IEnumerator WaitForTrafficLight(float waitTime)
        {
            while (!CheckTrafficLight())
            {
                yield return new WaitForSeconds(waitTime);
            }
        }

        private bool CheckTrafficLight()
        {
            if (trafficLight is null)
            {
                return true;
            }
            
            var trafficLightBulbData = trafficLight.GetBulbData();
            //Fill TrafficSignal with bulbData
            byte color = 0;
            foreach (var bulbData in trafficLightBulbData)
            {
                if (isBulbTurnOn(bulbData.Status))
                {
                    color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
                }
            }

            //when the traffic light is red we can pass the street 
            if (color == autoware_perception_msgs.msg.TrafficLightElement.RED)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        
        private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
        {
            return bulbStatus == TrafficLight.BulbStatus.SOLID_ON || bulbStatus == TrafficLight.BulbStatus.FLASHING;
        }
    }
}