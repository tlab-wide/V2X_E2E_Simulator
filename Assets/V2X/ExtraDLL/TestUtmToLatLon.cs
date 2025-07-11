using System;
using AWSIM;
using UnityEngine;

public class TestUtmToLatLon : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    
    
    [SerializeField] private static float E_utm = 404542.531f;

    [SerializeField] private static float N_utm = 3972958.000f;

    [SerializeField] private Transform target;
    
    private Vector3 utm_vector;


    private void Start()
    {
        utm_vector = new Vector3(E_utm, N_utm, 0);
        Vector3 pos_utm = ROS2Utility.UnityToRosPosition(target.transform.position) + utm_vector;
        (double lat, double lon) =  GeographicLib.UTMUPS.Reverse(54,true,pos_utm.x,pos_utm.y);
        Debug.Log("|||||||||||||||###||||||||||||||");
        Debug.Log($"name:{this.gameObject.name},{lat},{lon}");
    }


   
}
