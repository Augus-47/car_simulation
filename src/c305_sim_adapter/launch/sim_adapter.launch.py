from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='c305_sim_adapter',
            executable='sim_adapter',
            name='c305_sim_adapter',
            parameters=[{
                'use_sim_time': True,
                'wheelbase': 1.152,
                'track_width': 0.902,
                'steer_gain': 0.5,
                'max_steering_angle': 0.5,
                'max_forward_speed': 0.70,
                'max_reverse_speed': 0.12,
                'command_timeout': 0.5,
                'publish_vehicle_state': True,
            }],
            output='screen',
        ),
    ])
