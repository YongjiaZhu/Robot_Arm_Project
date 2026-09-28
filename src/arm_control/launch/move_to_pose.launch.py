from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # 加载 Panda 的 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .to_moveit_configs()
    )

    # 启动我们自己编写的 C++ 节点
    move_to_pose_node = Node(
        package="arm_control",
        executable="move_to_pose",
        name="move_to_pose",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_to_pose_node
    ])
