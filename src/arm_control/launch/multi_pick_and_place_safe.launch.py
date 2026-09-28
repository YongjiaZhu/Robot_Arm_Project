from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .to_moveit_configs()
    )

    safe_multi_node = Node(
        package="arm_control",
        executable="multi_pick_and_place_safe",
        name="multi_pick_and_place_safe",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        safe_multi_node
    ])
