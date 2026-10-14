// Map frame (ROS / Autoware, MGRS-local metres) <-> Unity world.
// ROS = (unity.z, -unity.x, unity.y) + MGRS offset. Heading is clockwise from north; Unity yaw 0 looks along +z = east.
using UnityEngine;

namespace V2X.Config
{
    public static class ConfigGeometry
    {
        static Vector3 MgrsOffset => AWSIM.Environment.Instance != null ? AWSIM.Environment.Instance.MgrsOffsetPosition : Vector3.zero;

        public static Vector3 MapToUnity(double x, double y, double z)
        {
            var o = MgrsOffset;
            return new Vector3(-(float)(y - o.y), (float)(z - o.z), (float)(x - o.x));
        }

        public static Vector3 UnityToMap(Vector3 p)
        {
            var o = MgrsOffset;
            return new Vector3(p.z + o.x, -p.x + o.y, p.y + o.z);
        }

        public static Quaternion PoseRotation(Pose pose)
            => Quaternion.Euler(-pose.PitchDeg, pose.HeadingDeg - 90f, pose.RollDeg);

        public static void ApplyPose(Transform t, Pose pose)
        {
            t.SetPositionAndRotation(MapToUnity(pose.X, pose.Y, pose.Z), PoseRotation(pose));
        }

        public static Pose ToPose(Transform t)
        {
            var m = UnityToMap(t.position);
            var fwd = t.forward;
            float heading = Mathf.Atan2(fwd.z, -fwd.x) * Mathf.Rad2Deg;   // east = unity z, north = -unity x
            float pitch = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f)) * Mathf.Rad2Deg;
            float roll = Mathf.DeltaAngle(0f, t.eulerAngles.z);
            return new Pose
            {
                X = System.Math.Round(m.x, 3), Y = System.Math.Round(m.y, 3), Z = System.Math.Round(m.z, 3),
                HeadingDeg = Round1((heading + 360f) % 360f), PitchDeg = Round1(pitch), RollDeg = Round1(roll),
            };
        }

        static float Round1(float v) => Mathf.Round(v * 10f) / 10f;

        /// <summary>
        /// Path of <paramref name="t"/> below <paramref name="root"/>, each step as name[index among same-named siblings]
        /// when the name is not unique, e.g. "Sensors/VelodyneVLP16[2]". Stable and unique within the root.
        /// </summary>
        public static string RelativeKey(Transform root, Transform t)
        {
            var parts = new System.Collections.Generic.List<string>();
            for (var x = t; x != null && x != root; x = x.parent)
            {
                var name = x.name;
                if (x.parent != null)
                {
                    int same = 0, index = 0;
                    foreach (Transform sib in x.parent)
                    {
                        if (sib.name != name) continue;
                        if (sib == x) index = same;
                        same++;
                    }
                    if (same > 1) name += $"[{index}]";
                }
                parts.Add(name);
            }
            parts.Reverse();
            return string.Join("/", parts);
        }
    }
}
