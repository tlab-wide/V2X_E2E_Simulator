using System.Collections;
using System.Collections.Generic;
// using autoware_auto_perception_msgs.msg;
using autoware_perception_msgs.msg;
using ROS2;
using UnityEngine;
using v2x_msgs.msg;

namespace AWSIM
{
    [RequireComponent(typeof(CustomV2I))]
    public class CustomV2IPublisher : MonoBehaviour
    {
        public enum TrafficSignalID{
            RelationID,
            WayID
        }

        public TrafficSignalID trafficSignalID;

        [SerializeField, Tooltip("On this topic, the traffic_signals are published")]
        string trafficSignalsTopic = "/v2x/traffic_signals";

        [Header("Delayed message")]
        [SerializeField] private bool enableDelayedMessages = false; // default false
        [SerializeField] private List<MessageDelayConfig> messageDelaysConfigs = new List<MessageDelayConfig>();
        private readonly List<MessageDelay<V2XSignals>> messageDelays = new List<MessageDelay<V2XSignals>>();

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        private IPublisher<V2XSignals> cooperativeSignalMessagePublisher;
        private V2XSignals cooperativeSignalsMessage;

        CustomV2I v2iComponent;

        void Start()
        {
            v2iComponent = GetComponent<CustomV2I>();
            v2iComponent.OnOutputData += UpdateMessageAndPublish;

            cooperativeSignalsMessage = new V2XSignals();

            var qos = qosSettings.GetQoSProfile();
            cooperativeSignalMessagePublisher = SimulatorROS2Node.CreatePublisher<V2XSignals>(trafficSignalsTopic, qos);

            if (enableDelayedMessages)
                InitializeDelaySystem();
        }

        private void InitializeDelaySystem()
        {
            var qos = qosSettings.GetQoSProfile();

            // 1) Global (“general”) delay topics from NetworkSimulator
            var generalDelays = NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs();
            foreach (var md in generalDelays)
            {
                var delay = new MessageDelay<V2XSignals>(md.delayConfig);
                delay.SetIPublisher(
                    SimulatorROS2Node.CreatePublisher<V2XSignals>(trafficSignalsTopic + md.GetTopicName(), qos)
                );
                messageDelays.Add(delay);
            }

            // 2) Component-local delay topics from inspector
            foreach (var md in messageDelaysConfigs)
            {
                var delay = new MessageDelay<V2XSignals>(md);
                delay.SetIPublisher(
                    SimulatorROS2Node.CreatePublisher<V2XSignals>(trafficSignalsTopic + md.topicName, qos)
                );
                messageDelays.Add(delay);
            }
        }

        private void PublishByDelay(V2XSignals message)
        {
            if (!enableDelayedMessages) return;

            foreach (var md in messageDelays)
            {
                NetworkSimulator.Instance.PublishLate(md, Clone(message));
            }
        }

        private static V2XSignals Clone(V2XSignals src)
        {
            if (src == null) return null;

            // shallow copy of ROS messages is usually sufficient as long as you don't mutate after publish
            var copy = new V2XSignals
            {
                Station_id = src.Station_id,
                Station_pose = src.Station_pose,
                Traffic_signals = src.Traffic_signals
            };
            return copy;
        }

        void UpdateMessageAndPublish(CustomV2I.OutputData outputData)
        {
            UpdateTrafficSignalArrayMsg(outputData);
            cooperativeSignalMessagePublisher.Publish(cooperativeSignalsMessage);
            PublishByDelay(cooperativeSignalsMessage); // conditional delayed/lossy copies
        }

        private void UpdateTrafficSignalArrayMsg(CustomV2I.OutputData data)
        {
            var trafficSignalList = new List<TrafficSignal>();
            var allRelationID = new List<long>();
            foreach (var trafficLight in data.trafficLights)
            {
                var trafficLightLaneletID = trafficLight.GetComponentInParent<TrafficLightLaneletID>();
                if (trafficLightLaneletID != null)
                {
                    var ids = new List<long>();

                    if (trafficSignalID == TrafficSignalID.RelationID)
                    {
                        ids = trafficLightLaneletID.relationID;
                    }
                    else if (trafficSignalID == TrafficSignalID.WayID)
                    {
                        ids.Add(trafficLightLaneletID.wayID);
                    }

                    foreach (var relationID in ids)
                    {
                        if (allRelationID.Contains(relationID))
                            continue;

                        var trafficSignalMsg = new TrafficSignal
                        {
                            Traffic_signal_id = (int)relationID
                        };

                        var trafficLightBulbData = trafficLight.GetBulbData();
                        var trafficLightElementList = new List<autoware_perception_msgs.msg.TrafficSignalElement>();

                        foreach (var bulbData in trafficLightBulbData)
                        {
                            if (isBulbTurnOn(bulbData.Status))
                            {
                                var trafficLightElementMsg = new autoware_perception_msgs.msg.TrafficSignalElement
                                {
                                    Color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color),
                                    Shape = V2IROS2Utility.UnityToRosBulbShape(bulbData.Type),
                                    Status = V2IROS2Utility.UnityToRosBulbStatus(bulbData.Status),
                                    Confidence = 1.0f
                                };
                                trafficLightElementList.Add(trafficLightElementMsg);
                            }
                        }

                        trafficSignalMsg.Elements = trafficLightElementList.ToArray();
                        trafficSignalList.Add(trafficSignalMsg);
                        allRelationID.Add(relationID);
                    }
                }
            }

            cooperativeSignalsMessage.Station_pose.Header.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            cooperativeSignalsMessage.Station_id = (ulong)data.stationId;
            cooperativeSignalsMessage.Traffic_signals.Signals = trafficSignalList.ToArray();
        }

        private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
        {
            return bulbStatus == TrafficLight.BulbStatus.SOLID_ON || bulbStatus == TrafficLight.BulbStatus.FLASHING;
        }

        void OnDestroy()
        {
            // Make sure the generic parameter matches the actual publisher type
            SimulatorROS2Node.RemovePublisher<V2XSignals>(cooperativeSignalMessagePublisher);
        }
    }
}
