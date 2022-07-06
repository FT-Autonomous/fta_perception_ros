from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import GroupAction, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="bag"),
        DeclareLaunchArgument("bag", default_value="test_bag"),
        GroupAction(
            actions=[
                PushRosNamespace('perception'),
                Node(package='zed_capture',
                     executable='zed_capture',
                     name='video_capture',
                     parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'video_capture.yaml'])],
                     condition=IfCondition(PythonExpression(["'", LaunchConfiguration("source"), "'== 'zed'"]))),
                ExecuteProcess(
                    cmd=['ros2 bag play --remap /zed:=/perception/zed --loop ', PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'bags', LaunchConfiguration('bag')])],
                    shell=True, 
                    condition=IfCondition(PythonExpression(["'", LaunchConfiguration("source"), "'== 'bag'"]))),
                Node(package='segmentation',
                     executable='segmentation',
                     name='segmentation',
                     parameters=[PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'segmentation.yaml'])])
            ])
        ])
