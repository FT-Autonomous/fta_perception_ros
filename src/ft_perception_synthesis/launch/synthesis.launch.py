from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('perception'),
                Node(package='zed_capture',
                     executable='zed_capture',
                     name='video_capture',
                     parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'video_capture.yaml'])]),
                Node(package='segmentation',
                     executable='segmentation',
                     name='segmentation',
                     parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'segmentation.yaml'])])
            ])
        ])
