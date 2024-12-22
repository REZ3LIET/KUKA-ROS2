from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = ""
    namespace = "arm_1"

    moveit_config = (
        MoveItConfigsBuilder("kuka_arm", package_name="kuka_2f140_moveit")
        .robot_description(file_path="config/kr70_r2100.urdf.xacro", mappings={"namespace": namespace})
        .robot_description_semantic(file_path="config/kr70_r2100.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    planning_scene_parameters={
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_parameters,
            {"use_sim_time": use_sim_time}
        ]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kuka_moveit"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_launch",
        output="screen",
        namespace=namespace,
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        move_group_node,
        # rviz_node
    ])