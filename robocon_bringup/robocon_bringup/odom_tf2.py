import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with your odometry topic name
            self.odom_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Set rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()