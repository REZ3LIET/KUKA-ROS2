# Author: REZ3LIET

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Loading Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "verbose": "true"
            # "world": world_file
        }.items()
    )

    # Loading Robot Model
    pkg_dir = get_package_share_directory('kuka_gazebo')
    robot_xacro = os.path.join(pkg_dir, "urdf", "kr70_r2100.urdf.xacro")

    parser = xacro.parse(open(robot_xacro))
    xacro.process_doc(parser)

    robot_description = {"robot_description": parser.toxml()}

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "kuka_arm",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.3",
        ],
        output="both"
    )
    
 
    return LaunchDescription([
        gazebo_node,
        robot_state_publisher,
        spawn_robot_node
    ])