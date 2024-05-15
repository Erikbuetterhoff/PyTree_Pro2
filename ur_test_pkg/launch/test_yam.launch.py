from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import os

def generate_launch_description():
    pkg_share= get_package_share_directory("ur_test_pkg")

    return LaunchDescription(
        [
            Node(
                package = "joy",
                executable = "joy_node",
                name = "joy_node",
                output = "screen",
            ),

            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'test.launch.yaml')
                ),
            ),

        ])