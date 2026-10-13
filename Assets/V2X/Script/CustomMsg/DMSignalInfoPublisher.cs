using System;
using System.Collections.Generic;
using AWSIM.TrafficSimulation;
using dm_signal_info_msgs.msg;
using ROS2;
using UnityEngine;
using DMSignalInfo = dm_signal_info_msgs.msg.SignalInfo;

namespace AWSIM
{
    /// <summary>
    /// Publishes the signal information of intersections in the CooL4 DM format (dm_signal_info_msgs).
    /// The light colour and the remaining time come from the lighting sequences of each TrafficIntersection,
    /// so every configured intersection is published all the time, wherever the ego vehicle is.
    /// See docs/DMProtocols/SignalInfo for the message layout and the Kashiwa signal ID assignment.
    /// </summary>
    public class DMSignalInfoPublisher : MonoBehaviour
    {
        /// <summary>
        /// Logical traffic signals (信号灯器) of an intersection that always show the same state.
        /// </summary>
        [Serializable]
        public class SignalGroup
        {
            [Tooltip("Free text shown in the Inspector, e.g. the approach (方路) it belongs to.")]
            public string label;

            [Tooltip("Pedestrian signal: a flashing green is published as PERMISSIVE_YELLOW (7, 青点滅).")]
            public bool isPedestrian;

            [Tooltip("Group number in the TrafficIntersection lighting sequences (0-3: vehicle, 10-13: pedestrian).")]
            public int trafficLightGroup;

            [Tooltip("Signal IDs (信号灯器 ID). Vehicle: inbound approach ID << 4 | outbound approach ID. Pedestrian: crossed approach ID << 4.")]
            public byte[] signalIds;

            public SignalGroup() { }

            public SignalGroup(string label, bool isPedestrian, int trafficLightGroup, params byte[] signalIds)
            {
                this.label = label;
                this.isPedestrian = isPedestrian;
                this.trafficLightGroup = trafficLightGroup;
                this.signalIds = signalIds;
            }
        }

        [Serializable]
        public class Intersection
        {
            public string label;

            [Tooltip("Optional. When empty, the TrafficIntersection is searched by GameObject name.")]
            public TrafficIntersection trafficIntersection;

            [Tooltip("GameObject name of the TrafficIntersection, used when the reference above is empty.")]
            public string trafficIntersectionName;

            [Tooltip("CRP ID (交差点 ID) of the intersection.")]
            public uint crpId;

            [Tooltip("Topic with only this intersection. Leave empty to publish it on the aggregated topic only.")]
            public string topic;

            public SignalGroup[] signals;

            public Intersection() { }

            public Intersection(string label, string trafficIntersectionName, uint crpId, string topic, params SignalGroup[] signals)
            {
                this.label = label;
                this.trafficIntersectionName = trafficIntersectionName;
                this.crpId = crpId;
                this.topic = topic;
                this.signals = signals;
            }
        }

        // Maximum list sizes of the CooL4 spec (3.6.2).
        const int MaxSignalIds = 8;
        const int MaxSignalLightInfos = 12;

        // ArrowLightIndication bits: bit6 left, bit4 straight, bit2 right.
        const byte ArrowLeft = 1 << 6;
        const byte ArrowStraight = 1 << 4;
        const byte ArrowRight = 1 << 2;

        [Header("Publish")]
        [SerializeField, Range(1, 50)] float hz = 10f;

        [Tooltip("Topic with every intersection in one SignalInfoArray. This is the topic DM subscribes to.")]
        [SerializeField] string topic = "/signal_info";

        [Tooltip("Pad signal_id_list with 0 (UNKNOWN) up to 8 elements, as the ROS2DMI of the DM platform does.")]
        [SerializeField] bool padSignalIdList = true;

        [SerializeField] List<Intersection> intersections = KashiwaDefaults();

        [Header("Delayed message")]
        [SerializeField] bool enableDelayedMessages = false;
        [SerializeField] List<MessageDelayConfig> messageDelaysConfigs = new List<MessageDelayConfig>();

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        // Publishers of the aggregated topic (index 0) and of each intersection topic, with their delayed copies.
        readonly Dictionary<string, IPublisher<SignalInfoArray>> publishers = new Dictionary<string, IPublisher<SignalInfoArray>>();
        readonly Dictionary<string, List<MessageDelay<SignalInfoArray>>> messageDelays = new Dictionary<string, List<MessageDelay<SignalInfoArray>>>();

        readonly List<TrafficIntersection.GroupTimelineEntry> timeline = new List<TrafficIntersection.GroupTimelineEntry>();
        readonly HashSet<string> warnedGroups = new HashSet<string>();
        float timer;

        void Start()
        {
            var allIntersections = FindObjectsByType<TrafficIntersection>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            foreach (var intersection in intersections)
            {
                if (intersection.trafficIntersection == null)
                    intersection.trafficIntersection = FindIntersection(allIntersections, intersection.trafficIntersectionName);
                if (intersection.trafficIntersection == null)
                    Debug.LogWarning($"[DMSignalInfoPublisher] TrafficIntersection '{intersection.trafficIntersectionName}' of {intersection.label} not found.");
            }

            CreatePublishers(topic);
            foreach (var intersection in intersections)
            {
                if (!string.IsNullOrEmpty(intersection.topic))
                    CreatePublishers(intersection.topic);
            }
        }

        // A GameObject can carry several TrafficIntersection components (e.g. a disabled alternative setup);
        // only an enabled one runs its lighting sequence, so prefer it.
        static TrafficIntersection FindIntersection(TrafficIntersection[] all, string gameObjectName)
        {
            var matches = Array.FindAll(all, x => x.name == gameObjectName);
            var running = Array.FindAll(matches, x => x.isActiveAndEnabled);
            if (running.Length > 1)
                Debug.LogWarning($"[DMSignalInfoPublisher] {running.Length} enabled TrafficIntersections named '{gameObjectName}'; using the first.");
            if (running.Length > 0)
                return running[0];
            return Array.Find(matches, x => x.enabled) ?? (matches.Length > 0 ? matches[0] : null);
        }

        void CreatePublishers(string baseTopic)
        {
            var qos = qosSettings.GetQoSProfile();
            publishers[baseTopic] = SimulatorROS2Node.CreatePublisher<SignalInfoArray>(baseTopic, qos);

            var delays = new List<MessageDelay<SignalInfoArray>>();
            messageDelays[baseTopic] = delays;
            if (!enableDelayedMessages)
                return;

            // Global ("general") delay topics from NetworkSimulator, then component-local ones.
            foreach (var md in NetworkSimulator.Instance.GetGeneralDelayMessagesConfigs())
            {
                var delay = new MessageDelay<SignalInfoArray>(md.delayConfig);
                delay.SetIPublisher(SimulatorROS2Node.CreatePublisher<SignalInfoArray>(baseTopic + md.GetTopicName(), qos));
                delays.Add(delay);
            }
            foreach (var md in messageDelaysConfigs)
            {
                var delay = new MessageDelay<SignalInfoArray>(md);
                delay.SetIPublisher(SimulatorROS2Node.CreatePublisher<SignalInfoArray>(baseTopic + md.topicName, qos));
                delays.Add(delay);
            }
        }

        void Update()
        {
            timer += Time.deltaTime;
            if (timer + 0.00001f < 1f / Mathf.Max(1f, hz))
                return;
            timer = 0f;

            var time = ObjectInfo.GetITSTimeInMilliseconds();
            var all = new List<DMSignalInfo>();
            foreach (var intersection in intersections)
            {
                var infos = BuildSignalInfos(intersection, time);
                all.AddRange(infos);
                if (!string.IsNullOrEmpty(intersection.topic))
                    Publish(intersection.topic, infos);
            }
            Publish(topic, all);
        }

        void Publish(string baseTopic, List<DMSignalInfo> infos)
        {
            // A new message every time, so the delayed copies are never modified after they are queued.
            var message = new SignalInfoArray { Array = infos.ToArray() };
            publishers[baseTopic].Publish(message);
            foreach (var md in messageDelays[baseTopic])
                NetworkSimulator.Instance.PublishLate(md, message);
        }

        List<DMSignalInfo> BuildSignalInfos(Intersection intersection, ulong time)
        {
            var infos = new List<DMSignalInfo>();
            if (intersection.trafficIntersection == null)
                return infos;

            foreach (var signal in intersection.signals)
            {
                if (signal.signalIds == null || signal.signalIds.Length == 0)
                    continue;
                if (!intersection.trafficIntersection.TryGetGroupTimeline(signal.trafficLightGroup, timeline))
                {
                    // The group is never lit by this intersection's lighting sequence (or it has not started yet).
                    var key = $"{intersection.label}/{signal.label}";
                    if (Time.timeSinceLevelLoad > 2f && warnedGroups.Add(key))
                        Debug.LogWarning($"[DMSignalInfoPublisher] {key}: group {signal.trafficLightGroup} has no state in " +
                                         $"'{intersection.trafficIntersection.name}' (not used by its lighting sequence?); not published.");
                    continue;
                }

                var lightInfos = BuildSignalLightInfos(timeline, signal.isPedestrian, out var isCountingDown);

                // More than 8 signal IDs with the same state are split into several SignalInfo (spec 3.6.2).
                for (int start = 0; start < signal.signalIds.Length; start += MaxSignalIds)
                {
                    var info = new DMSignalInfo();
                    info.Crp_id.Value = intersection.crpId;
                    info.Signal_id_list = BuildSignalIdList(signal.signalIds, start);
                    info.Time.Value = time;
                    info.Count_down_stop_flag.Value = isCountingDown ? CountDownStopFlag.OFF : CountDownStopFlag.ON;
                    info.Signal_light_info_list = lightInfos;
                    infos.Add(info);
                }
            }
            return infos;
        }

        SignalId[] BuildSignalIdList(byte[] signalIds, int start)
        {
            var count = Mathf.Min(MaxSignalIds, signalIds.Length - start);
            var list = new SignalId[padSignalIdList ? MaxSignalIds : count];
            for (int i = 0; i < list.Length; i++)
                list[i] = new SignalId { Value = i < count ? signalIds[start + i] : SignalId.UNKNOWN };
            return list;
        }

        /// <summary>
        /// The current light and its upcoming changes. Consecutive sequences showing the same light are merged,
        /// so the first entry holds the time left until the light changes. Times are fixed, so min == max.
        /// </summary>
        static SignalLightInfo[] BuildSignalLightInfos(List<TrafficIntersection.GroupTimelineEntry> timeline, bool isPedestrian, out bool isCountingDown)
        {
            var mains = new List<byte>();
            var arrows = new List<byte>();
            var durations = new List<float>();
            foreach (var entry in timeline)
            {
                ToLightIndication(entry.BulbData, isPedestrian, out var main, out var arrow);
                var last = mains.Count - 1;
                if (last >= 0 && mains[last] == main && arrows[last] == arrow)
                {
                    durations[last] += entry.DurationSec;
                    continue;
                }
                mains.Add(main);
                arrows.Add(arrow);
                durations.Add(entry.DurationSec);
            }

            // The light never changes: the remaining time is unknown and does not count down.
            isCountingDown = mains.Count > 1;
            if (!isCountingDown)
                return new[] { NewSignalLightInfo(mains[0], arrows[0], MinTimeToChange.UNKNOWN) };

            // The timeline ends where the current sequence starts again, so a last entry showing the current
            // light is cut short. Drop it rather than publish a wrong duration.
            var count = mains.Count;
            if (mains[count - 1] == mains[0] && arrows[count - 1] == arrows[0])
                count--;
            count = Mathf.Min(count, MaxSignalLightInfos);

            var lightInfos = new SignalLightInfo[count];
            for (int i = 0; i < count; i++)
                lightInfos[i] = NewSignalLightInfo(mains[i], arrows[i], ToTenthsOfSecond(durations[i]));
            return lightInfos;
        }

        static SignalLightInfo NewSignalLightInfo(byte main, byte arrow, ushort timeToChange)
        {
            var lightInfo = new SignalLightInfo();
            lightInfo.Main_light.Value = main;
            lightInfo.Arrow_light.Value = arrow;
            lightInfo.Min_time_to_change.Value = timeToChange;
            lightInfo.Max_time_to_change.Value = timeToChange;
            return lightInfo;
        }

        // 0.1 s units. 65535 is reserved for UNKNOWN.
        static ushort ToTenthsOfSecond(float seconds)
            => (ushort)Mathf.Clamp(Mathf.RoundToInt(seconds * 10f), 0, MinTimeToChange.UNKNOWN - 1);

        /// <summary>
        /// Bulb data to DE_MainLightIndication and DE_青矢信号表示方向 (CooL4 spec 3.6.2 / 5).
        /// </summary>
        static void ToLightIndication(TrafficLight.BulbData[] bulbData, bool isPedestrian, out byte main, out byte arrow)
        {
            main = MainLightIndication.DARK;
            arrow = 0;
            foreach (var bulb in bulbData)
            {
                if (bulb.Status == TrafficLight.BulbStatus.SOLID_OFF)
                    continue;
                var flashing = bulb.Status == TrafficLight.BulbStatus.FLASHING;

                switch (bulb.Type)
                {
                    case TrafficLight.BulbType.LEFT_ARROW_BULB: arrow |= ArrowLeft; continue;
                    case TrafficLight.BulbType.UP_ARROW_BULB: arrow |= ArrowStraight; continue;
                    case TrafficLight.BulbType.RIGHT_ARROW_BULB: arrow |= ArrowRight; continue;
                }

                switch (bulb.Color)
                {
                    case TrafficLight.BulbColor.RED:
                        main = flashing ? MainLightIndication.FLASHING_RED : MainLightIndication.RED_LIGHT;
                        break;
                    case TrafficLight.BulbColor.YELLOW:
                        main = flashing ? MainLightIndication.FLASHING_YELLOW : MainLightIndication.PERMISSIVE_YELLOW;
                        break;
                    case TrafficLight.BulbColor.GREEN:
                        // 7 means yellow for vehicle signals and flashing green (青点滅) for pedestrian signals.
                        main = isPedestrian && flashing ? MainLightIndication.PERMISSIVE_YELLOW : MainLightIndication.PERMISSIVE_GREEN;
                        break;
                }
            }
        }

        void OnDestroy()
        {
            foreach (var publisher in publishers.Values)
                SimulatorROS2Node.RemovePublisher<SignalInfoArray>(publisher);
            foreach (var delays in messageDelays.Values)
            {
                foreach (var md in delays)
                    SimulatorROS2Node.RemovePublisher<SignalInfoArray>(md.publisher);
            }
        }

        /// <summary>
        /// Signal IDs of the Kashiwa-no-ha bus route (東大_柏の葉実験環境について_信号情報について抜粋).
        /// Approach (方路) IDs are 1-4 per intersection. Vehicle IDs are listed in the same order as the DM
        /// messenger output. Pedestrian groups follow the vehicle phase they run with.
        /// </summary>
        static List<Intersection> KashiwaDefaults() => new List<Intersection>
        {
            new Intersection("東大前 (UTokyo)", "crossroad 14", 0x00000001, "/v2x/todaimae/signal_info",
                new SignalGroup("方路1 east (駅→東大 right turn)", false, 0, 0x11, 0x12, 0x13),
                new SignalGroup("方路2 west", false, 2, 0x21, 0x22, 0x23),
                new SignalGroup("方路3 north (東大→駅 left turn)", false, 1, 0x33, 0x31, 0x32),
                new SignalGroup("Pedestrian crossing 方路3", true, 10, 0x30),
                new SignalGroup("Pedestrian crossing 方路1, 2", true, 11, 0x10, 0x20)),

            new Intersection("がん研 (Ganken)", "crossroad1", 0x000c1001, "/v2x/rsu1/signal_info",
                new SignalGroup("Pedestrian crossing 方路1, 3", true, 11, 0x10, 0x30),
                new SignalGroup("方路1 east", false, 0, 0x11, 0x12, 0x13),
                new SignalGroup("Pedestrian crossing 方路2", true, 10, 0x20),
                new SignalGroup("方路2 south (駅→東大 left turn)", false, 1, 0x22, 0x23, 0x21),
                new SignalGroup("方路3 west (東大→駅 right turn)", false, 2, 0x33, 0x31, 0x32)),

            new Intersection("科警研 (Kakeiken)", "crossroad2", 0x000c3001, "/v2x/rsu2/signal_info",
                new SignalGroup("Pedestrian crossing 方路1", true, 10, 0x10),
                new SignalGroup("方路1 east (駅→東大 right turn)", false, 1, 0x11, 0x12, 0x13),
                new SignalGroup("Pedestrian crossing 方路2, 3", true, 11, 0x20, 0x30),
                new SignalGroup("方路2 south", false, 0, 0x22, 0x23, 0x21),
                new SignalGroup("方路3 north (東大→駅 left turn)", false, 2, 0x33, 0x31, 0x32)),

            new Intersection("若柴 (Wakashiba)", "crossroad3", 0x00000002, "/v2x/rsu4/signal_info",
                new SignalGroup("方路1 north", false, 0, 0x11, 0x12, 0x13, 0x14),
                new SignalGroup("方路2 east", false, 1, 0x22, 0x23, 0x24, 0x21),
                new SignalGroup("方路3 south (駅→東大 left turn)", false, 0, 0x33, 0x34, 0x31, 0x32),
                new SignalGroup("方路4 west (東大→駅 right turn)", false, 1, 0x44, 0x41, 0x42, 0x43),
                new SignalGroup("Pedestrian crossing 方路2, 4", true, 10, 0x20, 0x40),
                new SignalGroup("Pedestrian crossing 方路1, 3", true, 11, 0x10, 0x30)),

            new Intersection("駅前 (Station)", "crossroad4", 0x000c2001, "/v2x/rsu5/signal_info",
                new SignalGroup("Pedestrian crossing 方路1", true, 10, 0x10),
                new SignalGroup("方路1 east (駅→東大 right turn)", false, 1, 0x11, 0x12, 0x13),
                new SignalGroup("Pedestrian crossing 方路2, 3", true, 11, 0x20, 0x30),
                new SignalGroup("方路2 south", false, 0, 0x22, 0x23, 0x21),   // the scene puts the south lights in group 0
                new SignalGroup("方路3 north (東大→駅 left turn)", false, 0, 0x33, 0x31, 0x32)),
        };
    }
}
