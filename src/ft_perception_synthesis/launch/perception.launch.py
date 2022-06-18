from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'),'launch','synthesis.launch.py'])
            )
        ),
        GroupAction(actions=[
            PushRosNamespace("perception"),
            Node(package="ft_cluster_cpp",
                 executable="ft_cluster_cpp"),
            Node(package="center_estimation",
                 executable="center_estimation"),
            Node(package="ft_perception_synthesis",
                 executable="perception",
                 name="perception")
        ])
    ])
