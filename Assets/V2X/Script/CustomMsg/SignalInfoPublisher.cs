using System.Collections;
using System.Collections.Generic;
using autoware_auto_perception_msgs.msg;
using dm_cooperative_msgs.msg;
using ROS2;
using UnityEngine;
using v2x_msgs.msg;

namespace AWSIM
{
    [RequireComponent(typeof(CustomV2I))]
    public class SignalInfoPublisher : MonoBehaviour
    {
        public enum TrafficSignalID{
            RelationID,
            WayID
        }

        public TrafficSignalID trafficSignalID;

        [SerializeField, Tooltip("On this topic, the traffic_signals are published (as a ")]
        string trafficSignalsTopic = "/v2x/traffic_signals_info";
        
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };
        
        
        // IPublisher<autoware_perception_msgs.msg.TrafficSignalArray> trafficSignalsPublisher;
        // autoware_perception_msgs.msg.TrafficSignalArray trafficSignalArrayMsg;

        private IPublisher<CooperativeSignalInfoMessage>  cooperativeSignalMessagePublisher;
        private CooperativeSignalInfoMessage cooperativeSignalsMessage;
        
        

        SignalInfo signalInfo;

        void Start()
        {
            signalInfo = GetComponent<SignalInfo>();
            signalInfo.OnOutputData += UpdateMessageAndPublish;

            cooperativeSignalsMessage = new CooperativeSignalInfoMessage();

            var qos = qosSettings.GetQoSProfile();
            cooperativeSignalMessagePublisher = SimulatorROS2Node.CreatePublisher<CooperativeSignalInfoMessage>(trafficSignalsTopic, qos);
        }

        void UpdateMessageAndPublish(SignalInfo.OutputData outputData)
        {
            UpdateTrafficSignalArrayMsg(outputData);
            cooperativeSignalMessagePublisher.Publish(cooperativeSignalsMessage);
        }

        private void UpdateTrafficSignalArrayMsg(SignalInfo.OutputData data)
        {
            var trafficSignalList = new List<TrafficSignal>();
            var allRelationID = new List<long>();
            foreach (var trafficLight in data.trafficLights)
            {
                var trafficLightLaneletID = trafficLight.GetComponentInParent<TrafficLightLaneletID>();
                if (trafficLightLaneletID != null)
                {
                    var ids = new List<long>();
                    
                    //i don't know exactly why but we do this
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
                        var trafficSignalMsg = new TrafficSignal();
                        if (allRelationID.Contains(relationID))
                        {
                            continue;
                        }
                        trafficSignalMsg.Map_primitive_id = (int)relationID;
                        //Get bulbData
                        var trafficLightBulbData = trafficLight.GetBulbData();
                        //Fill TrafficSignal with bulbData
                        var trafficLightElementList = new List<autoware_auto_perception_msgs.msg.TrafficLight>();
                        foreach (var bulbData in trafficLightBulbData)
                        {
                            if (isBulbTurnOn(bulbData.Status))
                            {
                                var trafficLightElementMsg = new autoware_auto_perception_msgs.msg.TrafficLight();
                                trafficLightElementMsg.Color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
                                trafficLightElementMsg.Shape = V2IROS2Utility.UnityToRosBulbShape(bulbData.Type);
                                trafficLightElementMsg.Status = V2IROS2Utility.UnityToRosBulbStatus(bulbData.Status);
                                trafficLightElementMsg.Confidence = 1.0f;
                                trafficLightElementList.Add(trafficLightElementMsg);
                            }
                        }
                        
                        //Add TrafficLight signal to list
                        trafficSignalMsg.Lights = trafficLightElementList.ToArray();
                        trafficSignalList.Add(trafficSignalMsg);
                        allRelationID.Add(relationID);
                    }
                }
            }
            cooperativeSignalsMessage.Station_pose.Header.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            cooperativeSignalsMessage.Station_id = (ulong)data.stationId;
            // cooperativeSignalsMessage.Traffic_signals.Signals = trafficSignalList.ToArray();
        }

        private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
        {
            return bulbStatus == TrafficLight.BulbStatus.SOLID_ON || bulbStatus == TrafficLight.BulbStatus.FLASHING;
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_perception_msgs.msg.TrafficSignalArray>(cooperativeSignalMessagePublisher);
        }
    }
}