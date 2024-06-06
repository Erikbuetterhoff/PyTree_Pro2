from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share= get_package_share_directory("test_bt_pkg")

    return LaunchDescription(
        [
            ExecuteProcess(
            cmd=['py-trees-tree-viewer'],
            output='screen'
        ),

            Node(
                package='test_bt_pkg',
                executable="wait_action_server",
                output='screen',
            ),

            Node(
                package='test_bt_pkg',
                executable='main_tree',
                # arguments=[
                #     '-entity', 'UR5',
                #     '-topic', '/robot_description',
                #     '-x', '0.0',
                #     '-y', '0.0',
                #     '-z', '0.0'
                # ],
                output='screen',
            ),
            
        ])