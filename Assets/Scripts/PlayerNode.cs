using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Joy = RosMessageTypes.Sensor.JoyMsg;
using RosOdom = RosMessageTypes.Nav.OdometryMsg;
using TFMessage = RosMessageTypes.Tf2.TFMessageMsg;
using TransformStamped = RosMessageTypes.Geometry.TransformStampedMsg;

[System.Serializable]
public struct PublisherData
{
    public string topicName;
    public float frequency;
    public bool isActive;
}

public class PlayerNode : MonoBehaviour
{
    [Header("ROS Connection and Topics")]
    private ROSConnection ros;
    PlayerController player;

    [Header("Subscriber Settings")]
    public string JoyName = "joy";

    [Header("Publisher Settings")]
    public PublisherData odomPublisher = new PublisherData
    {
        topicName = "odom",
        frequency = 0.5f,
        isActive = true
    };

    public PublisherData tfPublisher = new PublisherData
    {
        topicName = "/tf",
        frequency = 20.0f,
        isActive = true
    };

    [Header("State Variables")]
    private float odomTimeElapsed = 0.0f;
    private float tfTimeElapsed = 0.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Joy>(JoyName, JoyChange);

        ros.RegisterPublisher<RosOdom>(odomPublisher.topicName);
        ros.RegisterPublisher<TFMessage>(tfPublisher.topicName);

        player = GetComponent<PlayerController>();
        player.useSim = true;
    }

    void FixedUpdate()
    {
        if (odomPublisher.isActive)
        {
            odomTimeElapsed += Time.fixedDeltaTime;
            if (odomTimeElapsed >= (1.0f / odomPublisher.frequency))
            {
                odomTimeElapsed = 0.0f;
                PublishOdom();
            }
        }

        if (tfPublisher.isActive)
        {
            tfTimeElapsed += Time.fixedDeltaTime;
            if (tfTimeElapsed >= (1.0f / tfPublisher.frequency))
            {
                tfTimeElapsed = 0.0f;
                PublishTF();
            }
        }
    }

    void JoyChange(Joy joyMessage)
    {
        player.Move(new Vector3(-joyMessage.axes[1], joyMessage.axes[3], joyMessage.axes[0]), new Vector3(0, joyMessage.axes[2], 0));

        if (joyMessage.buttons[0] == 1)
        {
            player.Jump();
        }

        if (joyMessage.buttons[1] == 1)
        {
            player.Shoot();
        }
    }

    private void PublishOdom()
    {
        var odomMessage = new RosOdom
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "odom"
            },
            child_frame_id = "base_link",
            pose = new RosMessageTypes.Geometry.PoseWithCovarianceMsg
            {
                pose = new RosMessageTypes.Geometry.PoseMsg
                {
                    position = new RosMessageTypes.Geometry.PointMsg(
                        player.transform.position.z,   // Unity Z (forward) -> ROS X (forward)
                        -player.transform.position.x,  // Unity X (right) -> ROS Y (left)
                        player.transform.position.y    // Unity Y (up) -> ROS Z (up)
                    ),
                    orientation = new RosMessageTypes.Geometry.QuaternionMsg(
                        player.transform.rotation.z,
                        -player.transform.rotation.x,
                        player.transform.rotation.y,
                        player.transform.rotation.w
                    )
                }
            }
        };

        ros.Publish(odomPublisher.topicName, odomMessage);
    }

    private void PublishTF()
    {
        var odomToBaseLinkTransform = new TransformStamped
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "odom"
            },
            child_frame_id = "base_link",
            transform = new RosMessageTypes.Geometry.TransformMsg
            {
                translation = new RosMessageTypes.Geometry.Vector3Msg(
                    player.transform.position.z,   // Unity Z (forward) -> ROS X (forward)
                    -player.transform.position.x,  // Unity X (right) -> ROS Y (left)
                    player.transform.position.y    // Unity Y (up) -> ROS Z (up)
                ),
                rotation = new RosMessageTypes.Geometry.QuaternionMsg(
                    player.transform.rotation.z,
                    -player.transform.rotation.x,
                    player.transform.rotation.y,
                    player.transform.rotation.w
                )
            }
        };

        TFMessage tfMessage = new TFMessage();
        tfMessage.transforms = new TransformStamped[] { odomToBaseLinkTransform };

        ros.Publish(tfPublisher.topicName, tfMessage);
    }
}