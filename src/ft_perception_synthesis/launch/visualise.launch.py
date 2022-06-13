from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ft_perception_synthesis'),
                    'launch',
                    'synthesis.launch.py'
                ])
            )
        ),
        GroupAction([
            PushRosNamespace('perception'),
            Node(
                package='segmentation',
                executable='visualize',
                parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'visualize.yaml'])]
            )
        ])
    ])
