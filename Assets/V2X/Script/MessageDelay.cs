using System;
using System.Collections;
using dm_object_info_msgs.msg;
using ROS2;
using UnityEngine;
using System;
using System.Collections;
using UnityEngine;
using UnityEngine.Serialization;

[System.Serializable]
public struct MessageDelay<T> where T : Message
{
    [SerializeField] public MessageDelayConfig delayConfig;
    
    public IPublisher<T> publisher;
    
    public MessageDelay(float delay, IPublisher<T> publisher, string topicName, float packetLostProbability, bool isGroundTruth)
    {
        this.delayConfig = new MessageDelayConfig(delay,packetLostProbability,topicName,isGroundTruth);
        this.publisher = publisher;
    }

    public MessageDelay(MessageDelayConfig delayConfig)
    {
        this.delayConfig = delayConfig;
        this.publisher = null;
    }
    public void SetIPublisher(IPublisher<T> publisher)
    {
        this.publisher = publisher;
    }
    
    public IEnumerator PublishAfterDelay(T message)
    {
        // Sample a delay with variance, clamp to >= 0
        float sampledDelayMs = Mathf.Max(
            0f,
            delayConfig.delayMilliseconds + UnityEngine.Random.Range(-delayConfig.delayVariance, delayConfig.delayVariance)
        );

        if (sampledDelayMs > 0f)
            yield return new WaitForSeconds(sampledDelayMs / 1000f);

        // Debug.Log($"Delayed msg '{delayConfig.topicName}' for {sampledDelayMs:0.##} milliseconds.");
        publisher?.Publish(message);
    }

    public float GetDelayTime() => delayConfig.delayMilliseconds;
    public float GetProbabilityPacketLoss() => delayConfig.packetLostProbability;
    public string GetTopicName() => delayConfig.topicName;
}

public class MessageDelayConfig
{
    [SerializeField] [Min(0f)] public float delayMilliseconds;
    [SerializeField] [Range(0f, 1f)] public float packetLostProbability;
    [SerializeField] public string topicName;
    [SerializeField] public bool isGroundTruth; 

    [SerializeField] [Min(0f)] public float delayVariance; // ± variance in ms
    [SerializeField] [Range(0f, 1f)] public float packetLostVariance; // ± variance
    

    public MessageDelayConfig(
        float delayMilliseconds,
        float packetLostProbability,
        string topicName,
        bool isGroundTruth,
        float delayVariance = 0f,
        float packetLostVariance = 0f
    )
    {
        this.delayMilliseconds = delayMilliseconds;
        this.packetLostProbability = packetLostProbability;
        this.topicName = topicName;
        this.isGroundTruth = isGroundTruth;
        this.delayVariance = delayVariance;
        this.packetLostVariance = packetLostVariance;
    }
}