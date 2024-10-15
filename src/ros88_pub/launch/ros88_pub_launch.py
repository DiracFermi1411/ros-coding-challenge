from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros88_pub',  # Replace 'ros88' with your package name if different
            executable='ros88_pub',  # Replace with the actual executable name for the publisher node
            name='ros88_pub',
            output='screen',
            parameters=[
                {'image_topic': 'my_camera/image'}
            ]
        )
    ])