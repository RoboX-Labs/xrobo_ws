from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package='robocon_camera', 
        executable='rs_pose_node', 
        name='rs_pose_node'
    ),
    # Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_t265_link'],
    #     output='screen'
    # ),
])
