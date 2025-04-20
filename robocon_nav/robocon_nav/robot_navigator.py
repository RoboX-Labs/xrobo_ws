#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from simple_pid import PID


class RobotNav(Node):

    def __init__(self):
        super().__init__('robot_nav')
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.object_detection_callback,
            10)
        self.yolo_sub  # prevent unused variable warning

        self.cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Initialize PID controllers for linear and angular velocity
        self.linear_pid_x = PID(0.1, 0.01, 0.0, setpoint=0.0)
        self.linear_pid_y = PID(0.2, 0.001, 0.0, setpoint=0.0)

        # PID output limits
        self.linear_pid_x.output_limits = (-0.1, 0.1)  # m/s
        self.linear_pid_y.output_limits = (-0.1, 0.1)  # m/s

        self.ball_detected = False
        self.ball_position = None

    def object_detection_callback(self, msg):
        # Process detections to find the ball
        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id == "32":  # Replace "ball" with the actual class_id for the ball
                    self.ball_detected = True
                    self.ball_position = detection.bbox.center.position
                    self.get_logger().info(
                        f'Ball detected at: x={self.ball_position.x}, y={self.ball_position.y}'
                    )
                    return

        # If no ball is detected
        self.ball_detected = False
        self.ball_position = None
        self.get_logger().info('No ball detected.')

    def move_to_ball(self):
        if not self.ball_detected or not self.ball_position:
            self.get_logger().info('No ball to move toward.')
            return

        # Target screen center coordinates
        mid_screen_x = 320
        mid_screen_y = 240

        set_point_x = mid_screen_x 
        set_point_y = mid_screen_y + 200

        # Calculate the error in x and y coordinates
        error_x = set_point_x - self.ball_position.x
        error_y = set_point_y - self.ball_position.y

        self.get_logger().info(f'Ball position: x={self.ball_position.x}, y={self.ball_position.y}')
        self.get_logger().info(f'Error: x={error_x}, y={error_y}')

        # Use PID controllers to calculate linear and angular velocities
        # twist = Twist()
        # twist.linear.x = error_y * 0.01
        # twist.linear.y = error_x * 0.01
        # self.cmd_vel.publish(twist)

        twist = Twist()
        twist.linear.x = self.linear_pid_x(-error_y)
        twist.linear.y = self.linear_pid_y(-error_x)
        self.cmd_vel.publish(twist)

        self.get_logger().info(f'Linear velocity: {twist.linear.x}, {twist.linear.y}')

        self.grab_ball()

    def grab_ball(self):
        # Simulate grabbing the ball (replace this with actual hardware control logic)
        self.get_logger().info('Grabbing the ball...')
        # Add hardware-specific commands here to control the gripper or manipulator


def main(args=None):
    rclpy.init(args=args)

    robot_nav = RobotNav()

    # Spin the node and periodically call move_to_ball
    try:
        while rclpy.ok():
            rclpy.spin_once(robot_nav, timeout_sec=0.1)
            robot_nav.move_to_ball()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    robot_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()