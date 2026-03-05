from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS 2 Bridge for Isaac Sim
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_relay',
            parameters=[{'input_topic': '/isaac_scan', 'output_topic': '/scan'}]
        ),
        # STL Monitor
        Node(
            package='fleetsafe_monitor',
            executable='stl_monitor_node.py',
            name='safety_monitor',
            parameters=[{'robot_id': 'fastbot_0'}]
        )
    ])
