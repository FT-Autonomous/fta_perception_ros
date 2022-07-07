from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import GroupAction, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    visualize_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('visualize'), "' == 'yes'"]))
    shared_config_file = PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'config', 'config.yml'])
    
    return LaunchDescription([
        DeclareLaunchArgument("zed_source", default_value="bag"),
        DeclareLaunchArgument("bag_source", default_value="moving_bag"),
        DeclareLaunchArgument("cluster_method", default_value="cpp"),
        DeclareLaunchArgument("visualize", default_value="yes"),
        
        # Necessary Nodes
        Node(package='ft_perception_synthesis',
             executable='perception',
             parameters=[shared_config_file]),
        Node(package='segmentation',
             executable='segmentation',
             name='segmentation',
             parameters=[shared_config_file]),
        Node(package='ft_cluster_cpp',
             name='cluster',
             executable='ft_cluster_cpp'),
        
        ## Necessary Nodes with Variants
        Node(package='zed_capture',
             executable='zed_capture',
             name='zed_capture',
             parameters=[shared_config_file],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration("zed_source"), "'== 'camera'"]))),
        ExecuteProcess(
            cmd=['ros2 bag play --loop ', PathJoinSubstitution([FindPackageShare('ft_perception_synthesis'), 'bags', LaunchConfiguration('bag_source')])],
            shell=True, 
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("zed_source"), "'== 'bag'"]))),

        # Optional Nodes
        GroupAction(actions=[
            Node(package='segmentation',
                 executable='visualize',
                 parameters=[shared_config_file],
                 condition=visualize_condition),
            SetEnvironmentVariable('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python'),
        ]),
        Node(package='cluster',
             executable='viz',
             parameters=[shared_config_file],
             condition=visualize_condition),
        ExecuteProcess(cmd=['rqt_graph'],
                       shell=True,
                       condition=visualize_condition)
    ])

