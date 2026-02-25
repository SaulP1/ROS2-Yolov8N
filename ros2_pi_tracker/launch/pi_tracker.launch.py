from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_pi_tracker',
            executable='tracker_node',
            name='pi_tracker_node',
            output='screen',
            parameters=[{'camera_index': 0, 'model_path': 'yolov8n.pt'}],
            remappings=[
                # Add any necessary topic remappings here
            ]
        ),
    ])