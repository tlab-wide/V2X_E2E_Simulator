using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BicycleMoveRotate : MonoBehaviour
{
    [SerializeField] float duration =7;

    [SerializeField] float speed = 3;


    private Rigidbody npcCyclist;
    Vector3 startPosition;
    Quaternion startRotation;
    Vector3 currentPosition;
    Quaternion currentRotation;

    void Awake()
    {
        npcCyclist = GetComponent<Rigidbody>();
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
            yield return MoveForwardRoutine(duration, -speed);
            yield return RotateRoutine(0.5f, 360f);
            yield return MoveForwardRoutine(duration, -speed);
            yield return RotateRoutine(0.5f, 360f);
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

    IEnumerator RotateRoutine(float duration, float angularSpeed)
    {
        var startTime = Time.fixedTime;
        while (Time.fixedTime - startTime < duration)
        {
            yield return new WaitForFixedUpdate();
            currentRotation *= Quaternion.AngleAxis(angularSpeed * Time.fixedDeltaTime, Vector3.up);
            npcCyclist.MoveRotation(currentRotation);
        }
    }
}
