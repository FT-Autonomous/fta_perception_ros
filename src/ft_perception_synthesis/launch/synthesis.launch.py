from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('perception'),
                Node(package='video_capture',
                     executable='video_capture',
                     name='video_capture',
                     parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'segmentation.yaml'])]),
                Node(package='segmentation',
                     executable='segmentation',
                     name='segmentation',
                     parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'video_capture.yaml'])])
            ])
        ])
