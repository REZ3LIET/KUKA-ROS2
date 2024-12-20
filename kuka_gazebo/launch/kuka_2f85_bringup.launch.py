# Author: REZ3LIET

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    # Loading Gazebo
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': '-r -v1'}.items() 
    )

    # Loading Robot Model
    pkg_dir = get_package_share_directory('kuka_gazebo')
    robot_xacro = Command([
        'xacro ', os.path.join(pkg_dir, 'urdf/kr70_r2100_2f85.urdf.xacro')])
    robot_description = {"robot_description": robot_xacro}

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
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "kuka_arm",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="both"
    )

    # Load Controllers
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output="screen"
    )

    kuka_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kuka_arm_controller'],
        output="screen"
    )

    robotiq_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robotiq_2f85_controller'],
        output="screen"
    )

    load_joint_state_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_controller]
        )
    )

    load_kuka_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_controller,
            on_exit=[kuka_controller]
        )
    )

    load_robotiq_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=kuka_controller,
            on_exit=[robotiq_controller]
        )
    )

    return LaunchDescription([
        ign_gazebo_node,
        robot_state_publisher,
        spawn_robot_node,
        load_joint_state_controller,
        load_kuka_controller,
        load_robotiq_controller
    ])