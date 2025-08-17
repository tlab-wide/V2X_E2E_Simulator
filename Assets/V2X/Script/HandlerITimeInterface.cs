using UnityEngine;
using UnitySensors.Interface.Std;

public class HandlerITimeInterface : MonoBehaviour,ITimeInterface
{
    private float _time;

    

    float ITimeInterface.time => Time.time;
}
