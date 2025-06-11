using UnityEngine;
using UnityEngine.VFX;
using Unity.Collections;
using UnitySensors.Sensor.LiDAR;

public class VFXPointCloudRenderer : MonoBehaviour
{
    [SerializeField] VisualEffect vfx;
    [SerializeField] LiDARSensor sensor;

    void Update()
    {
        // after your sensor has updated the NativeArray<PointXYZI>:
        Vector4[] cpuArray = new Vector4[sensor.pointsNum];
        for (int i = 0; i < cpuArray.Length; i++)
        {
            var p = sensor.pointCloud.points[i];
            cpuArray[i] = new Vector4(p.position.x, p.position.y, p.position.z, p.intensity);
        }

        vfx.SetUInt("PointCount", (uint)cpuArray.Length);
        // vfx.SetVector4("PointCloudBuffer", cpuArray);
    }
}
