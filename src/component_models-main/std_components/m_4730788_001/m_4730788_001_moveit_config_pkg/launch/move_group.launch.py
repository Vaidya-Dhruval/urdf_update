from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("m_4730788_001", package_name="m_4730788_001_moveit_config_pkg").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
