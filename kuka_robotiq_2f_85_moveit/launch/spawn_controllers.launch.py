from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("kr70_r2100", package_name="kuka_robotiq_2f_85_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
