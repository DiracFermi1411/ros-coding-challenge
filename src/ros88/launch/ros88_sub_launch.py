
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros88',
            executable='ros88',
            name='ros88',
            parameters=[
                {'image_topic': 'my_camera/image'}
            ]
        )
    ])
