using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Vehicle ROS2 utility static class.
    /// </summary>
    public static class V2IROS2Utility
    {
        /// <summary>
        /// Convert the TrafficLight.BulbType to ROS msg TrafficLightElement
        /// </summary>
        /// <param name="type"></param>
        /// <returns>Converted BulbType.</returns>
        /// 
        public static byte UnityToRosBulbShape(TrafficLight.BulbType type)
        {
            if (type == TrafficLight.BulbType.ANY_CIRCLE_BULB || type == TrafficLight.BulbType.RED_BULB || type == TrafficLight.BulbType.YELLOW_BULB || type == TrafficLight.BulbType.GREEN_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.CIRCLE;
            else if (type == TrafficLight.BulbType.LEFT_ARROW_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.LEFT_ARROW;
            else if (type == TrafficLight.BulbType.RIGHT_ARROW_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.RIGHT_ARROW;
            else if (type == TrafficLight.BulbType.UP_ARROW_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.UP_ARROW;
            else if (type == TrafficLight.BulbType.DOWN_ARROW_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.DOWN_ARROW;
            else if (type == TrafficLight.BulbType.DOWN_LEFT_ARROW_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.DOWN_LEFT_ARROW;
            else if (type == TrafficLight.BulbType.DOWN_RIGHT_ARROW_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.DOWN_RIGHT_ARROW;
            else if (type == TrafficLight.BulbType.CROSS_BULB)
                return autoware_perception_msgs.msg.TrafficSignalElement.CROSS;
            else
                return autoware_perception_msgs.msg.TrafficSignalElement.UNKNOWN;
        }

        /// <summary>
        /// Convert the TrafficLight.BulbStatus to ROS msg TrafficLightElement
        /// </summary>
        /// <param name="status"></param>
        /// <returns>Converted BulbStatus.</returns>
        /// 
        public static byte UnityToRosBulbStatus(TrafficLight.BulbStatus status)
        {
            if (status == TrafficLight.BulbStatus.SOLID_OFF)
                return autoware_perception_msgs.msg.TrafficSignalElement.SOLID_OFF;
            else if (status == TrafficLight.BulbStatus.SOLID_ON)
                return autoware_perception_msgs.msg.TrafficSignalElement.SOLID_ON;
            else if (status == TrafficLight.BulbStatus.FLASHING)
                return autoware_perception_msgs.msg.TrafficSignalElement.FLASHING;
            else
                return autoware_perception_msgs.msg.TrafficSignalElement.UNKNOWN;
        }

        /// <summary>
        /// Convert the TrafficLight.BulbColor to ROS msg TrafficLightElement
        /// </summary>
        /// <param name="color"></param>
        /// <returns>Converted BulbColor.</returns>
        /// 
        public static byte UnityToRosBulbColor(TrafficLight.BulbColor color)
        {
            if (color == TrafficLight.BulbColor.RED)
                return autoware_perception_msgs.msg.TrafficSignalElement.RED;
            else if (color == TrafficLight.BulbColor.YELLOW)
                return autoware_perception_msgs.msg.TrafficSignalElement.AMBER;
            else if (color == TrafficLight.BulbColor.GREEN)
                return autoware_perception_msgs.msg.TrafficSignalElement.GREEN;
            else if (color == TrafficLight.BulbColor.WHITE)
                return autoware_perception_msgs.msg.TrafficSignalElement.WHITE;
            else
                return autoware_perception_msgs.msg.TrafficSignalElement.UNKNOWN;
        }

    }
}
