using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class SimpleCyclistMovement : MonoBehaviour
{
    
    [SerializeField] float duration = 7;
    [SerializeField] float speed = 3;
    [SerializeField] TrafficLight trafficLight;
    [SerializeField] private float CheckTrafficLightTime = 1.0f;
    [SerializeField] private float waitTime = 5;


    private Rigidbody npcCyclist;
    Vector3 startPosition;
    Quaternion startRotation;
    Vector3 currentPosition;
    Quaternion currentRotation;
    private Animator animator;


    void Awake()
    {
        npcCyclist = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;
        currentPosition = transform.position;
        currentRotation = transform.rotation;
        animator = this.transform.GetComponent<Animator>();
    }

    void Start()
    {
        StartCoroutine(Loop());
    }

    IEnumerator Loop()
    {
        while (true)
        {
            yield return new WaitForSeconds(waitTime);
            yield return WaitForTrafficLight(CheckTrafficLightTime);
            yield return MoveForwardRoutine(duration, -speed);
            yield return RotateRoutine(180, 80);

            yield return new WaitForSeconds(waitTime);
            yield return WaitForTrafficLight(CheckTrafficLightTime);
            yield return MoveForwardRoutine(duration, +speed);
            yield return RotateRoutine(180, 80);
            // var npcTransformPos = npcPedestrian.transform.position;

            // reset
            npcCyclist.MovePosition(startPosition);
            npcCyclist.MoveRotation(startRotation);
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
            npcCyclist.MovePosition(currentPosition);
        }
    }

    // IEnumerator RotateRoutine(float duration, float angularSpeed)
    // {
    //     var startTime = Time.fixedTime;
    //     while (Time.fixedTime - startTime < duration)
    //     {
    //         yield return new WaitForFixedUpdate();
    //         currentRotation *= Quaternion.AngleAxis(angularSpeed * Time.fixedDeltaTime, Vector3.up);
    //         npcCyclist.MoveRotation(currentRotation);
    //     }
    // }

    IEnumerator RotateRoutine(float angle, float angularSpeed)
    {
        float currentAngle = transform.rotation.eulerAngles.y;
        float targetAngle = (currentAngle + angle) % 360;

        if (targetAngle > currentAngle)
        {
            while (transform.eulerAngles.y < targetAngle)
            {
                yield return new WaitForFixedUpdate();
                // Debug.Log(transform.eulerAngles.y);
                transform.eulerAngles += Vector3.up * angularSpeed * Time.fixedDeltaTime;
            }
        }
        else
        {
            while ((currentAngle <= transform.eulerAngles.y) ||
                   (transform.eulerAngles.y < targetAngle))
            {
                yield return new WaitForFixedUpdate();
                // Debug.Log("--" + transform.eulerAngles.y + "__" + targetAngle);
                transform.eulerAngles += Vector3.up * angularSpeed * Time.fixedDeltaTime;
            }
        }
    }

    public float NormalizeEulerAngles(float eulerAngles)
    {
        eulerAngles = Mathf.Repeat(eulerAngles, 360f);

        // Optional: Restrict to a specific range like -180 to 180 degrees
        eulerAngles = Mathf.Clamp(eulerAngles, -180f, 180f);

        return eulerAngles;
    }

    IEnumerator WaitForTrafficLight(float waitTime)
    {
        while (!CheckTrafficLight())
        {
            animator.enabled = false;
            yield return new WaitForSeconds(waitTime);
        }


        animator.enabled = true;
    }

    private bool CheckTrafficLight()
    {
        if (trafficLight is null || !trafficLight.gameObject.activeSelf || !trafficLight.gameObject.activeInHierarchy)
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