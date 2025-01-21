using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class NoiseSetting : Singleton<NoiseSetting>
{
    [SerializeField]
    private List<Noise> noises;
    // Start is called before the first frame update

    // public Vector3 ApplyNoiseToVector3(string noiseName  ,Vector3 vector3)
    // {
    //     for (int i = 0; i < noises.Count; i++)
    //     {
    //         if (noises[i].GetName().Equals(noiseName))
    //         {
    //             return vector3 + noises[i].GetNoiseVector();
    //         }
    //     }
    //
    //     Debug.LogWarning($"{noiseName} is not defined in noise types");
    //     return vector3;
    // }
    //
    // public float ApplyNoiseToFloat(string noiseName  ,float number)
    // {
    //     for (int i = 0; i < noises.Count; i++)
    //     {
    //         if (noises[i].GetName().Equals(noiseName))
    //         {
    //             return number + noises[i].GetNoiseVector().x;
    //         }
    //     }
    //
    //     Debug.LogWarning($"{noiseName} is not defined in noise types");
    //     return number;
    // }

    public Noise GetNoise(string noiseName)
    {
        for (int i = 0; i < noises.Count; i++)
        {
            if (noises[i].GetName().Equals(noiseName))
            {
                return noises[i];
            }
        }
        
        Debug.LogWarning("I have return default noise pattern because I can not match names");
        return noises[0];
    }
    
    [Serializable]
    public class  Noise
    {
        [SerializeField] private string name;
        [Range(0f, 0.5f)] [SerializeField] private float xNoise;
        [Range(0f, 0.5f)] [SerializeField] private float yNoise;
        [Range(0f, 0.5f)] [SerializeField] private float zNoise;

        public string GetName()
        {
            return name;    
        }

        public Vector3 SampleNoiseVector()
        {
            return new Vector3(Random.Range(-xNoise, xNoise), Random.Range(-yNoise, yNoise), Random.Range(-zNoise, zNoise));
        }

        public Vector3 GetNoiseVector()
        {
            return new Vector3(xNoise, yNoise, zNoise);
        }

        public Vector3 ApplyNoiseOnVector(Vector3 inputVector3)
        {
            return SampleNoiseVector() + inputVector3;
        }
        
        public float ApplyNoiseOnFloat(float input)
        {
            return SampleNoiseVector().x + input;
        }

        public float ApplyNoiseToDecrease(float input)
        {
            return input - Random.Range(0, xNoise);
        }
        
        
        public Quaternion RotateQuaternionAroundY(Quaternion originalQuaternion)
        {
            // Creating a rotation quaternion based on the Y axis rotation
            Quaternion rotation = Quaternion.Euler(SampleNoiseVector());

            // Applying the rotation to the original quaternion
            Quaternion rotatedQuaternion = rotation * originalQuaternion;

            return rotatedQuaternion;
        }
    }


}
