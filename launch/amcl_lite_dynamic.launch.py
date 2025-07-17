from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_amcl_lite',
            executable='amcl_lite_node',
            name='amcl_lite_node',
            parameters=[
                {'enable_dynamic_detection': True},
                {'dynamic_detection_threshold': 0.5},
                {'dynamic_weight': 0.7},
                {'sensor_sigma': 0.2}
            ],
            output='screen'
        )
    ])
