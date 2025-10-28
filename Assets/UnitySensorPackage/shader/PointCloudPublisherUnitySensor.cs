using System;
using AWSIM;
using ROS2;
using sensor_msgs.msg;
using std_msgs.msg;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;
using Environment = AWSIM.Environment;

public class PointCloudPublisherUnitySensor : MonoBehaviour
{
    // Fill this from elsewhere (ray-casts, meshes, etc.)
    [SerializeField] private Vector3[] _hit_positions;
    [SerializeField] private bool IsGlobal;

    // Topic + QoS  -----------------------------------------------------------
    private IPublisher<PointCloud2> _pcdPub;
    [SerializeField] private  string TOPIC = "/sim/points";

    private readonly QoSSettings _qos = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 1
    };

    // Re-usable message objects  --------------------------------------------
    private PointCloud2 _msg;
    private PointField[] _fields;
    private const int POINT_STEP = 12; // x,y,z -> 3 × float32
    private byte[] _buffer;

    private RaycastLiDARSensor lidarSensor;

    void Start()
    {
        lidarSensor = GetComponent<RaycastLiDARSensor>();
        lidarSensor.scanCallBackEvent.AddListener(OnNewLidarDataPublish);

        _pcdPub = SimulatorROS2Node.CreatePublisher<PointCloud2>(TOPIC, _qos.GetQoSProfile());

        // --- initialise immutable parts of the message ---------------------
        _fields = new[]
        {
            new PointField { Name = "x", Offset = 0, Datatype = PointField.FLOAT32, Count = 1 },
            new PointField { Name = "y", Offset = 4, Datatype = PointField.FLOAT32, Count = 1 },
            new PointField { Name = "z", Offset = 8, Datatype = PointField.FLOAT32, Count = 1 }
        };

        _msg = new PointCloud2
        {
            Header = new Header { Frame_id = "map" },
            Height = 1, // un-organised cloud
            Is_bigendian = false,
            Is_dense = true,
            Point_step = POINT_STEP,
            Fields = _fields
        };
    }

    void SendData() //  or Update() – depends on sensor timing
    {
        if (_hit_positions == null || _hit_positions.Length == 0) return;

        int pointCount = _hit_positions.Length;
        int cloudByteLen = pointCount * POINT_STEP;

        // allocate/re-use the byte buffer only when size changes
        if (_buffer == null || _buffer.Length != cloudByteLen)
            _buffer = new byte[cloudByteLen];

        // write Vector3s into byte[]  (little-endian)
        for (int i = 0; i < pointCount; ++i)
        {
            // ---- optional coordinate conversion Unity (Z-forward, Y-up) ➜ ROS (X-forward, Z-up)
            // var p = new Vector3(_hit_positions[i].z, -_hit_positions[i].x, _hit_positions[i].y);
            var p = _hit_positions[i];

            int ofs = i * POINT_STEP;
            Buffer.BlockCopy(BitConverter.GetBytes(p.x), 0, _buffer, ofs, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.y), 0, _buffer, ofs + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.z), 0, _buffer, ofs + 8, 4);
        }

        // fill the variable parts of PointCloud2
        _msg.Width = (uint)pointCount;
        _msg.Row_step = (uint)cloudByteLen;
        _msg.Data = _buffer;

        // timestamp (R2FU helper updates both secs & nsecs)
        var h = _msg as MessageWithHeader;
        SimulatorROS2Node.UpdateROSTimestamp(ref h);

        // ==== publish ======================================================
        _pcdPub.Publish(_msg);
    }

    public void OnNewLidarDataPublish()
    {
        if (!enabled)
        {
            return;
        }
        
        _hit_positions = lidarSensor.hitPositions;
        
        
        if (IsGlobal)
        {
            Vector3[] ros_hit_position = new Vector3[_hit_positions.Length];
            for (int i = 0; i < _hit_positions.Length; i++)
            {
                var pos = ROS2Utility.UnityToRosPosition(_hit_positions[i]);
                pos = pos + Environment.Instance.MgrsOffsetPosition;
                ros_hit_position[i] = pos;
            }
            _hit_positions = ros_hit_position;
        }
        else
        {
            //todo merge same sections
            Vector3[] ros_hit_position = new Vector3[_hit_positions.Length];
            for (int i = 0; i < _hit_positions.Length; i++)
            {
                // 1) world → sensor local
                Vector3 pos = this.transform.InverseTransformPoint(_hit_positions[i]);
                
                // var pos = _hit_positions[i] - this.transform.position;
                pos = ROS2Utility.UnityToRosPosition(pos);
                
                ros_hit_position[i] = pos;
            }
            _hit_positions = ros_hit_position;
        }
        SendData();
    }
}