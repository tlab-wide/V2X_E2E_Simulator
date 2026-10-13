using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public enum NPCVehicleSpeedLimitSource
    {
        LaneletSpeedLimit,
        FixedSpeedLimit
    }

    /// <summary>
    /// Config of vehicle behaviors in <see cref="NPCVehicleSimulator"/>.
    /// </summary>
    [System.Serializable]
    public struct NPCVehicleConfig
    {
        /// <summary>
        /// Multiplier used to determine rotational speed from steering angle and vehicle speed.
        /// </summary>
        public const float YawSpeedMultiplier = 0.15f;

        /// <summary>
        /// Rate of change of angular velocity per unit time.<br/>
        /// The higher the value, the faster it can turn, but the more blurred the control becomes.
        /// </summary>
        public const float YawSpeedLerpFactor = 5f;

        /// <summary>
        /// Slow speed at which the vehicle can immediately stop.
        /// </summary>
        public const float SlowSpeed = 5f;

        [Tooltip("Acceleration used when driving normally in m/s^2.")]
        public float Acceleration;

        [Tooltip("Deceleration used for gentle braking in m/s^2.")]
        public float Deceleration;

        [Tooltip("Deceleration used for sudden stops in m/s^2.")]
        public float SuddenDeceleration;

        [Tooltip("Deceleration used for absolute stops in m/s^2.")]
        public float AbsoluteDeceleration;

        [Tooltip("Choose whether NPC vehicles follow the lanelet speed limit or a fixed speed from these settings.")]
        public NPCVehicleSpeedLimitSource SpeedLimitSource;

        [Min(0f)]
        [Tooltip("Fixed NPC speed limit in m/s when Speed Limit Source is set to Fixed Speed Limit.")]
        public float SpeedLimit;

        public static NPCVehicleConfig Default()
            => new NPCVehicleConfig
            {
                Acceleration = 3f,
                Deceleration = 2f,
                SuddenDeceleration = 4f,
                AbsoluteDeceleration = 20f,
                SpeedLimitSource = NPCVehicleSpeedLimitSource.LaneletSpeedLimit,
                SpeedLimit = 13.89f
            };
    }
}
