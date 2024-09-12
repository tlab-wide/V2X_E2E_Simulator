using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM;
using UnityEngine;

public class ScannerCar : MonoBehaviour
{
    private Rigidbody rigidbody;

    private bool isInside = false;

    private List<Transform> carsList = new List<Transform>(); // 

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = this.GetComponent<Rigidbody>();
    }

    public bool IsInsideAnything()
    {
        return isInside;
    }
    
    private void OnTriggerEnter(Collider other)
    {
        Transform carTransform = other.transform;
        if (other.gameObject.layer == 6) //layer vehicle 
        {
            if (!carsList.Contains(carTransform))
            {
                carsList.Append(carTransform);
                isInside = true;
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        Transform carTransform = other.transform;

        if (carsList.Contains(carTransform))
        {
            carsList.Remove(carTransform);
            if (carsList.Count == 0)
            {
                isInside = false;
            }
        }
    }
}