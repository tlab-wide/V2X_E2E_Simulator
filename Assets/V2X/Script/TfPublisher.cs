using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

//
// TF publisher for AWSIM (ROS2ForUnity)
// Always (by default) shifts Unity origin to MGRS coordinates using Environment.Instance.MgrsOffsetPosition.
//
namespace AWSIM
{
    [Serializable]
    public class FrameSpec
    {
        [Tooltip("The Unity Transform (sensor or link) you want to publish as a TF child frame.")]
        public Transform sourceTransform;

        [Tooltip("Parent frame in ROS (e.g., map, odom, base_link, sensor_kit_base_link).")]
        public string parentFrame = "base_link";

        [Tooltip("Child frame name in ROS (e.g., camera_link, lidar_link).")]
        public string childFrame = "camera_link";

        [Tooltip("Publish this frame on /tf_static once at startup instead of /tf repeatedly.")]
        public bool isStatic = false;

        [Tooltip("When true, the Unity -> ROS axis conversion will be applied (recommended).")]
        public bool convertUnityToRos = true;

        [Tooltip("Shift Unity origin to MGRS world coordinates using Environment.Instance.MgrsOffsetPosition.")]
        public bool applyMgrsOffset = true; // default true
    }

    public class TfPublisher : MonoBehaviour
    {
        [Header("Frames to Publish")]
        public List<FrameSpec> frames = new List<FrameSpec>();

        [Header("Dynamic publishing rate")]
        [Range(1, 200)]
        public int publishHz = 30;

        [Header("Debug")]
        public bool logOnceOnStart = true;

        private IPublisher<tf2_msgs.msg.TFMessage> tfPub;         
        private IPublisher<tf2_msgs.msg.TFMessage> tfStaticPub;   

        private float timer = 0f;

        private QoSSettings tfQos = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy  = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy     = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth             = 100
        };

        private QoSSettings tfStaticQos = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy  = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            HistoryPolicy     = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth             = 1
        };

        void Start()
        {
            tfPub       = SimulatorROS2Node.CreatePublisher<tf2_msgs.msg.TFMessage>("/tf",        tfQos.GetQoSProfile());
            tfStaticPub = SimulatorROS2Node.CreatePublisher<tf2_msgs.msg.TFMessage>("/tf_static", tfStaticQos.GetQoSProfile());

            var staticTransforms = BuildTransforms(isStaticOnly: true);
            if (staticTransforms.Count > 0)
            {
                var msg = new tf2_msgs.msg.TFMessage() { Transforms = staticTransforms.ToArray() };
                tfStaticPub.Publish(msg);
            }

            if (logOnceOnStart)
            {
                Debug.Log($"[TfPublisher] Dynamic frames: {CountMatching(frames, false)}, " +
                          $"Static frames: {CountMatching(frames, true)}. " +
                          $"Publishing /tf @ {publishHz} Hz.");
            }
        }

        void FixedUpdate()
        {
            timer += Time.deltaTime;
            var interval = 1.0f / Mathf.Max(1, publishHz);
            if (timer < interval) return;
            timer = 0f;

            var dynamicTransforms = BuildTransforms(isStaticOnly: false);
            if (dynamicTransforms.Count == 0) return;

            var msg = new tf2_msgs.msg.TFMessage() { Transforms = dynamicTransforms.ToArray() };
            tfPub.Publish(msg);
        }

        private List<geometry_msgs.msg.TransformStamped> BuildTransforms(bool isStaticOnly)
        {
            var list = new List<geometry_msgs.msg.TransformStamped>();

            foreach (var f in frames)
            {
                if (f == null || f.sourceTransform == null) continue;
                if (f.isStatic != isStaticOnly) continue;

                var ts = new geometry_msgs.msg.TransformStamped()
                {
                    Header = new std_msgs.msg.Header()
                    {
                        Frame_id = string.IsNullOrEmpty(f.parentFrame) ? "base_link" : f.parentFrame,
                        Stamp = SimulatorROS2Node.GetCurrentRosTime()
                    },
                    Child_frame_id = string.IsNullOrEmpty(f.childFrame) ? f.sourceTransform.name : f.childFrame,
                    Transform = new geometry_msgs.msg.Transform()
                };

                Vector3 pU = f.sourceTransform.position;
                Quaternion qU = f.sourceTransform.rotation;

                Vector3 pR = f.convertUnityToRos ? UnityToRosPosition(pU) : pU;
                Quaternion qR = f.convertUnityToRos ? UnityToRosRotation(qU) : qU;

                // ✅ Apply MGRS/world offset if enabled (default true)
                if (f.applyMgrsOffset)
                {
                    pR += GetMgrsOffsetRos();
                }

                ts.Transform.Translation.X = pR.x;
                ts.Transform.Translation.Y = pR.y;
                ts.Transform.Translation.Z = pR.z;

                ts.Transform.Rotation.X = qR.x;
                ts.Transform.Rotation.Y = qR.y;
                ts.Transform.Rotation.Z = qR.z;
                ts.Transform.Rotation.W = qR.w;

                list.Add(ts);
            }
            return list;
        }

        private static int CountMatching(List<FrameSpec> specs, bool isStatic)
        {
            int c = 0;
            foreach (var s in specs) if (s != null && s.isStatic == isStatic) c++;
            return c;
        }

        private static Vector3 UnityToRosPosition(Vector3 pu)
            => new Vector3(pu.z, -pu.x, pu.y);

        private static Quaternion UnityToRosRotation(Quaternion qu)
            => new Quaternion(-qu.z, -qu.x, qu.y, qu.w);

        private static Vector3 GetMgrsOffsetRos()
        {
            try
            {
                var env = AWSIM.Environment.Instance;
                return env != null ? env.MgrsOffsetPosition : Vector3.zero;
            }
            catch
            {
                return Vector3.zero;
            }
        }
    }
}
