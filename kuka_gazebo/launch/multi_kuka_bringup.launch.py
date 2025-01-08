# Author: REZ3LIET

import os
import yaml
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

import rclpy.logging
logger = rclpy.logging.get_logger("multi_kuka_bringup.launch")

def generate_launch_description():
    ld = LaunchDescription()
    # Loading Gazebo
    world = os.path.join(get_package_share_directory("kuka_gazebo"), "world/empty_world.sdf")
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': [world, ' -r -v1']}.items()
    )
    ld.add_action(ign_gazebo_node)

    robot_list = [
        {"robot_name": "arm_1", "gripper_name": "", "x": "-2.0", "y": "0.0", "yaw": "0.0"},
        # {"robot_name": "arm_2", "gripper_name": "robotiq_2f_85", "x": "2.0", "y": "0.0", "yaw": "0.0"},
        {"robot_name": "arm_3", "gripper_name": "robotiq_2f_140", "x": "0.0", "y": "-2.0", "yaw": "0.0"},
        # {"robot_name": "arm_4", "gripper_name": "", "x": "0.0", "y": "2.0", "yaw": "0.0"},
    ]

    for robot in robot_list:
        # Loading Gazebo
        kuka_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("kuka_gazebo"), "launch"), "/kuka_bringup.launch.py"]
            ),
            launch_arguments={
                'ign_gz': 'False',
                'use_sim_time': 'True',
                'robot_name': robot["robot_name"],
                'namespace': robot["robot_name"],
                'gripper_name': robot["gripper_name"],
                'position_x': robot["x"],
                'position_y': robot["y"],
                'orientation_yaw': robot["yaw"],
            }.items()
        )
        ld.add_action(kuka_node)

    return ld
