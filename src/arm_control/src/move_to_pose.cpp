#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "move_to_pose",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    // 2. 启动 ROS2 回调线程
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // 3. 创建 MoveIt2 控制接口
    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "panda_arm"
    );

    // 4. 基本参数
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);

    // 降低速度，方便观察
    move_group.setMaxVelocityScalingFactor(0.20);
    move_group.setMaxAccelerationScalingFactor(0.20);

    move_group.setStartStateToCurrentState();

    // 等待机械臂状态更新
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(
        logger,
        "Planning frame: %s",
        move_group.getPlanningFrame().c_str()
    );

    RCLCPP_INFO(
        logger,
        "End effector: %s",
        move_group.getEndEffectorLink().c_str()
    );

    // 5. 获取当前末端位姿
    geometry_msgs::msg::PoseStamped current_pose =
        move_group.getCurrentPose();

    RCLCPP_INFO(
        logger,
        "Current position: x=%.3f, y=%.3f, z=%.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    );

    // 6. 设置目标位姿
    // 在当前位置基础上：
    // X 方向移动 3 cm
    // Z 方向抬高 3 cm
    geometry_msgs::msg::PoseStamped target_pose = current_pose;

    target_pose.pose.position.x += 0.03;
    target_pose.pose.position.z += 0.03;

    RCLCPP_INFO(
        logger,
        "Target position: x=%.3f, y=%.3f, z=%.3f",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z
    );

    move_group.setPoseTarget(target_pose);

    // 7. 创建规划结果
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    RCLCPP_INFO(logger, "Planning...");

    bool success =
        static_cast<bool>(move_group.plan(plan));

    // 8. 如果规划成功，执行轨迹
    if (success)
    {
        RCLCPP_INFO(logger, "Planning succeeded.");

        RCLCPP_INFO(logger, "Executing trajectory...");

        bool execute_success =
            static_cast<bool>(move_group.execute(plan));

        if (execute_success)
        {
            RCLCPP_INFO(
                logger,
                "Motion completed successfully!"
            );
        }
        else
        {
            RCLCPP_ERROR(
                logger,
                "Trajectory execution failed."
            );
        }
    }
    else
    {
        RCLCPP_ERROR(
            logger,
            "Motion planning failed."
        );
    }

    // 9. 清除目标
    move_group.clearPoseTargets();

    // 10. 关闭 ROS2
    executor.cancel();

    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();

    return 0;
}
