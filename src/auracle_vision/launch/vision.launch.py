from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='camera_ros', executable='camera_node', name='camera'),
        Node(package='auracle_vision', executable='waste_detector',
             name='waste_detector', output='screen',
             parameters=[{'conf': 0.5,
                          'image_topic': '/camera/image_raw'}]),
    ])
