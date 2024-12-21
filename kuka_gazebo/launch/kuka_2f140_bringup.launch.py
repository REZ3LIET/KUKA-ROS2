# Author: REZ3LIET

import os
import yaml
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext

def rewrite_yaml(source_file: str, root_key: str):
    # context = LaunchContext()
    # root_key = root_key.perform(context)
    if not root_key:
        return source_file

    with open(source_file, 'r') as file:
        ori_data = yaml.safe_load(file)

    updated_yaml = {root_key: ori_data}
    dst_path = f"/tmp/{time.time()}.yaml"
    with open(dst_path, 'w') as file:
        yaml.dump(updated_yaml, file)
    return dst_path

def generate_launch_description():
    ign_gz = LaunchConfiguration('ign_gz', default='True')
    robot_name = LaunchConfiguration('robot_name', default='kuka_2f140_arm')
    # namespace = LaunchConfiguration('namespace', default='arm_1')
    namespace = "arm_1"
    position_x = LaunchConfiguration('position_x', default='0.0')
    position_y = LaunchConfiguration('position_y', default='0.0')
    orientation_yaw = LaunchConfiguration('orientation_yaw', default='0.0')

    # Loading Gazebo
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': '-r -v1'}.items(),
        condition=IfCondition(ign_gz)
    )

    # Updating controller parameters
    pkg_dir = get_package_share_directory('kuka_gazebo')
    controller_file = rewrite_yaml(
        source_file=os.path.join(pkg_dir, "config/kuka_2f140_controllers.yaml"),
        root_key=namespace
    )

    # Loading Robot Model
    robot_xacro = Command([
        'xacro ', os.path.join(pkg_dir, 'urdf/kr70_r2100_2f140.urdf.xacro'),
        ' robot_name:=', robot_name,
        ' namespace:=', namespace,
        ' controller_file:=', controller_file,
        ])
    robot_description = {"robot_description": robot_xacro}

    # Publish TF
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        remappings=remappings,
        output="both",
        parameters=[robot_description]
    )

    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", namespace+"/robot_description",
            "-name", robot_name,
            "-robot_namespace", namespace,
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw,
        ],
        output="both"
    )

    # Load Controllers
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '-c', namespace+"/controller_manager"],
        output="screen"
    )

    kuka_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kuka_arm_controller', '-c', namespace+"/controller_manager"],
        output="screen"
    )

    robotiq_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robotiq_2f140_controller', '-c', namespace+"/controller_manager"],
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