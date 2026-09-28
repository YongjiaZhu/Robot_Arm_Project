from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .to_moveit_configs()
    )

    benchmark_node = Node(
        package="arm_control",
        executable="obstacle_benchmark",
        name="obstacle_benchmark",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        benchmark_node
    ])
