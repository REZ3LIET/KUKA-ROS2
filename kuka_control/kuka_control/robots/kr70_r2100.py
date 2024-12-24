from typing import List

MOVE_GROUP_ARM: str = "kuka_arm"

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

