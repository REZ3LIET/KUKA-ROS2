from typing import List

MOVE_GROUP_ARM: str = "kuka_arm"
MOVE_GROUP_GRIPPER: str = "robotiq_2f140"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.8]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0]

def joint_names() -> List[str]:
    return [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6"
    ]


def base_link_name() -> str:
    return "link_0"

def end_effector_name() -> str:
    return "link_6"

def gripper_joint_names() -> List[str]:
    return [
        "finger_joint"
    ]
