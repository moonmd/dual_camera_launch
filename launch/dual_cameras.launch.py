from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera 0 @ 1080p
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera0',
            parameters=[{
                'camera': 0,
                'camera_name': 'camera0',
                'format': 'YUYV',
                'width': 1920,
                'height': 1080,
                'framerate': 15,
            }],
            remappings=[
                ('image_raw', 'camera0/image_raw'),
                ('camera_info', 'camera0/camera_info'),
            ],
        ),
        # Camera 1 @ 1080p
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera1',
            parameters=[{
                'camera': 1,
                'camera_name': 'camera1',
                'format': 'YUYV',
                'width': 1920,
                'height': 1080,
                'framerate': 15,
            }],
            remappings=[
                ('image_raw', 'camera1/image_raw'),
                ('camera_info', 'camera1/camera_info'),
            ],
        ),
    ])

