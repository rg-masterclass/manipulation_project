from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    basic_grasping_perception_node = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        parameters=[{
            'debug_topics': True
        }]
    )

    get_pose_client_node = Node(
        package='get_cube_pose',
        executable='get_pose_client',
        name='get_pose_client_node'
    )

    return LaunchDescription([
        basic_grasping_perception_node,
        get_pose_client_node
    ])
