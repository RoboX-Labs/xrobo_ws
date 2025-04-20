from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', 'src/robocon_bringup/config/rviz2.rviz'],  # Specify the RViz config file
        ),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            output='screen',
            parameters=[
                {'ROS_IP': '127.0.0.1'},
                {'ROS_TCP_PORT': 10000}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),
        Node(
            package='robocon_bringup',
            executable='odom_tf2',
            name='odom_tf2',
            output='screen'
        ),
        Node(
            package='object_detection',
            executable='ros2_yolo',
            name='ros2_yolo',
            output='screen',
        ),
        # Node(
        #     package='robocon_nav',
        #     executable='robot_navigator',
        #     name='robot_navigation',
        #     output='screen',
        # )
    ])