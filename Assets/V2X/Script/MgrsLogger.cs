using AWSIM;
using UnityEngine;

public class MgrsLogger : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Vector3 pos = ROS2Utility.UnityToRosPosition(transform.position);
        pos += Environment.Instance.MgrsOffsetPosition;
        Debug.Log($"MGRS Logger sensor name: {gameObject.name}, position: ({pos.x:F3}, {pos.y:F3}, {pos.z:F3})");
    }
}