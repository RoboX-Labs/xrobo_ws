using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosOdom = RosMessageTypes.Nav.OdometryMsg; // Alias for nav_msgs/Odometry
using RosTwist = RosMessageTypes.Geometry.TwistMsg; // Alias for geometry_msgs/Twist
using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg;

public class RobotNode : MonoBehaviour
{
    [Header("ROS Connection and Topics")]
    private ROSConnection ros;
    public string publishTopicName = "odom";
    public string subscribeTopicName = "pose_estimation/ltp/odometry";
    public string cmdVelTopicName = "/cmd_vel";
    private bool isCmdVelReceived = false;

    [Header("Robot Settings")]
    public GameObject robot;
    public float publishMessageFrequency = 0.1f;

    [Header("State Variables")]
    private float timeElapsed;
    private Vector3 position = Vector3.zero;
    private Quaternion rotation = Quaternion.identity;
    private Vector3 linearVelocity = Vector3.zero;
    private Vector3 angularVelocity = Vector3.zero;

    void Start()
    {
        InitializeROS();
    }

    void FixedUpdate()
    {
        HandlePublishing();
        Movement();
    }

    // Initialize ROS Connection and Subscriptions
    private void InitializeROS()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Register Publisher
        ros.RegisterPublisher<RosOdom>(publishTopicName);

        // Register Subscribers
        ros.Subscribe<RosOdom>(subscribeTopicName, OdomChange);
        ros.Subscribe<RosTwist>(cmdVelTopicName, CmdVelChange);

        Debug.Log("ROS Publisher and Subscribers initialized.");
    }

    // Publish Odometry Data at a Fixed Frequency
    private void HandlePublishing()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            PublishOdometry();
            timeElapsed = 0;
        }
    }

    private void PublishOdometry()
    {
        var odomMsg = new RosOdom
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "base_link"
            },
            pose = new RosMessageTypes.Geometry.PoseWithCovarianceMsg
            {
                pose = new RosMessageTypes.Geometry.PoseMsg
                {
                    position = new RosMessageTypes.Geometry.PointMsg(
                        robot.transform.position.z,
                        robot.transform.position.x,
                        robot.transform.position.y
                    ),
                    orientation = new RosMessageTypes.Geometry.QuaternionMsg(
                        robot.transform.rotation.x,
                        robot.transform.rotation.z,
                        robot.transform.rotation.y,
                        robot.transform.rotation.w
                    )
                }
            },
            twist = new RosMessageTypes.Geometry.TwistWithCovarianceMsg
            {
                twist = new RosMessageTypes.Geometry.TwistMsg
                {
                    linear = new RosMessageTypes.Geometry.Vector3Msg(
                        linearVelocity.x,
                        linearVelocity.y,
                        linearVelocity.z
                    ),
                    angular = new RosMessageTypes.Geometry.Vector3Msg(
                        angularVelocity.x,
                        angularVelocity.y,
                        angularVelocity.z
                    )
                }
            }
        };

        ros.Publish(publishTopicName, odomMsg);
    }

    // Handle Robot Movement Based on Subscribed Data
    private void Movement()
    {
        float moveSpeed = 10f; // Speed of the robot
        float rotationSpeed = 100f; // Rotation speed of the robot

        // Mapping the linear and angular velocities to Unity's transform
        if (isCmdVelReceived)
        {

            float moveZ = linearVelocity.x * moveSpeed * Time.deltaTime; // Forward/backward
            float moveX = -linearVelocity.y * moveSpeed * Time.deltaTime; // Left/right strafing
            float rotateY = -angularVelocity.z * rotationSpeed * Time.deltaTime; // Rotation around Y-axis
            robot.transform.Translate(moveX, 0, moveZ);
            robot.transform.Rotate(0, rotateY, 0);
            isCmdVelReceived = false;

        }
    }

    // Callback for /cmd_vel Subscription
    private void CmdVelChange(RosTwist twistMessage)
    {
        linearVelocity = new Vector3(
            (float)twistMessage.linear.x,
            (float)twistMessage.linear.y,
            (float)twistMessage.linear.z
        );

        angularVelocity = new Vector3(
            (float)twistMessage.angular.x,
            (float)twistMessage.angular.y,
            (float)twistMessage.angular.z
        );
        isCmdVelReceived = true;
        Debug.Log($"Received /cmd_vel: Linear = {linearVelocity}, Angular = {angularVelocity}");

    }

    // Callback for /pose_estimation/ltp/odometry Subscription
    private void OdomChange(RosOdom odomMessage)
    {
        position = new Vector3(
            (float)odomMessage.pose.pose.position.x,
            (float)odomMessage.pose.pose.position.y,
            (float)odomMessage.pose.pose.position.z
        );

        rotation = new Quaternion(
            (float)odomMessage.pose.pose.orientation.x,
            (float)odomMessage.pose.pose.orientation.y,
            (float)odomMessage.pose.pose.orientation.z,
            (float)odomMessage.pose.pose.orientation.w
        );

        Debug.Log($"Received /pose_estimation/ltp/odometry: Position = {position}, Rotation = {rotation}");
    }
}