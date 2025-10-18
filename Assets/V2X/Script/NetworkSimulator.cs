using System;
using System.Collections.Generic;
using ROS2;
using UnityEngine;

public class NetworkSimulator : Singleton<NetworkSimulator>
{
    [SerializeField] private List<MessageDelay<Message>> generalDelayMessagesConfigs = new List<MessageDelay<Message>>();
    
    public void PublishLate<T>(MessageDelay<T> messageDelay, T message) where T : Message
    {
        var cfg = messageDelay.delayConfig;

        // Sample a packet-loss probability with variance, clamp to [0,1]
        float sampledLossP = Mathf.Clamp01(
            cfg.packetLossProbability + UnityEngine.Random.Range(-cfg.packetLossVariance, cfg.packetLossVariance)
        );
        

        // Drop the packet based on the sampled probability
        if (UnityEngine.Random.value < sampledLossP)
        {
            // Debug.Log($"Dropped msg on '{cfg.topicName}' (p={sampledLossP:0.###})");
            return;
        }

        StartCoroutine(messageDelay.PublishAfterDelay(message));
    }

    public List<MessageDelay<Message>> GetGeneralDelayMessagesConfigs()
    {
        return generalDelayMessagesConfigs;
    }
    
}
