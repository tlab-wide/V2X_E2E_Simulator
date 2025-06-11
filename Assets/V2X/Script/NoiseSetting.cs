using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class NoiseSetting : Singleton<NoiseSetting>
{
    [SerializeField] private List<Noise> noises;

   
    
    
    
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

    private void Awake()
    {
        CheckNoisesArrayExistance();
    }

    private void CheckNoisesArrayExistance()
    {
        if (noises == null)
        {
            noises = new List<Noise>();
        }

        if (noises.Count == 0)
        {
            noises.Add(new Noise());
        }
    }


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
    public class Noise
    {
        [SerializeField] private string name = "Default";
        [Range(0f, 0.5f)] [SerializeField] private float xNoise;
        [Range(0f, 0.5f)] [SerializeField] private float yNoise;
        [Range(0f, 0.5f)] [SerializeField] private float zNoise;
        [SerializeField] private float gaussianMean= 0 ;
        [SerializeField] private float gaussianStdDev = 0.25f;
        [SerializeField] private bool IsUniform = true;


        public string GetName()
        {
            return name;
        }

        public Vector3 SampleNoiseVector()
        {
            if (IsUniform)
            {
                return new Vector3(Random.Range(-xNoise, xNoise), Random.Range(-yNoise, yNoise),
                    Random.Range(-zNoise, zNoise));
            }
            else
            {
                // Gaussian noise using Box-Muller transform
                return new Vector3(
                    GenerateGaussian(gaussianMean, gaussianStdDev * xNoise),
                    GenerateGaussian(gaussianMean, gaussianStdDev * yNoise),
                    GenerateGaussian(gaussianMean, gaussianStdDev * zNoise)
                );
                
            }
        }
        
       

        public static float GenerateGaussian(float mean, float stdDev)
        {
            float u1 = 1.0f - Random.value; // Uniform(0,1] random number
            float u2 = 1.0f - Random.value;

            float randStdNormal =
                Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                Mathf.Sin(2.0f * Mathf.PI * u2); // Standard normal (mean 0, std dev 1)
            return mean + stdDev * randStdNormal; // Scale to desired mean and std dev
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
            return input - Math.Abs(SampleNoiseVector().x);
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