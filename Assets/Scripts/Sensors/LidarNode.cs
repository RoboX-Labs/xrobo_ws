using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // For LaserScan message

public class LidarNode : MonoBehaviour
{
    ROSConnection ros;
    [SerializeField] public string TopicName = "scan";
    [SerializeField] public float publishMessageFrequency = 0.1f;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(TopicName);
    }

    void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            PublishLidarData();
            timeElapsed = 0;
        }
    }

    private void PublishLidarData()
    {
        LaserScanMsg lidarMsg = new LaserScanMsg();

        lidarMsg.header.frame_id = "lidar_frame";
        // ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar_frame

        // Set the angle and range parameters
        lidarMsg.angle_min = 0.0f;
        lidarMsg.angle_max = Mathf.Deg2Rad * 360.0f;
        lidarMsg.angle_increment = Mathf.Deg2Rad * (360.0f / LidarSensor.distances.Length);
        lidarMsg.time_increment = publishMessageFrequency / LidarSensor.distances.Length;
        lidarMsg.scan_time = publishMessageFrequency;
        lidarMsg.range_min = 0.1f; // Minimum range value (e.g., 10 cm)
        lidarMsg.range_max = LidarSensor.MaxRange; // Maximum range value

        // Set the ranges (distance measurements)
        lidarMsg.ranges = new float[LidarSensor.distances.Length];
        for (int i = 0; i < LidarSensor.distances.Length; i++)
        {
            // Ensure values are within range_min and range_max
            lidarMsg.ranges[i] = Mathf.Clamp(LidarSensor.distances[i], lidarMsg.range_min, lidarMsg.range_max);
        }

        // Set the intensities (optional, leave empty if not available)
        // lidarMsg.intensities = new float[LidarSensor.distances.Length];
        // for (int i = 0; i < LidarSensor.distances.Length; i++)
        // {
        //     lidarMsg.intensities[i] = 1.0f; // Example intensity value (constant for now)
        // }

        // Publish the message
        ros.Publish(TopicName, lidarMsg);
    }
}