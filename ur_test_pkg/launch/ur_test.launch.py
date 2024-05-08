from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import os
import xacro

def generate_launch_description():
    gazebo_ros_share= get_package_share_directory("gazebo_ros")
    ur_share= get_package_share_directory("ur_description")
    pkg_share = get_package_share_directory("ur_test_pkg")
    turtle_share = get_package_share_directory('turtlebot3_gazebo')

    #os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_share,'models') #keine Ahnung ob das passt

    return LaunchDescription(
        [
            # Node(
            #     package = "rviz2",
            #     executable = "rviz2",
            #     name = "rviz2",
            #     output = "screen",
            #     parameters=[{
            #         "use_sim_time": True,
            #     }],
            # ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
                ),
                launch_arguments={
                    'world': os.path.join(pkg_share, "worlds", "ur_test.world"),
                    "pause" : "false",
                    "gui" : "true",
                    "gdb" : "false",
                    "verbose" : "true",
                }.items()
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
                )       
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ur_share, 'launch', 'view_ur.launch.py')
                ),
                launch_arguments={
                    "ur_type": "ur5",
                }.items()
            ),


            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(turtle_share, "launch", 'spawn_turtlebot3.launch.py')
            #         ),
            #         launch_arguments={
            #             'x_pose': "0",
            #             'y_pose': "0",
            #         }.items()
            # ),

            

            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",    
            # ),

            # Node(
            #     package="robot_state_publisher",
            #     executable="robot_state_publisher",
            #     output="both",
            #     parameters=[robot_description],
            # ),    
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'UR5',
                    '-topic', '/robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen',
            ),
        ])