#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>


bool moveToTarget(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::PoseStamped &target_pose,
    const std::string &target_name,
    const rclcpp::Logger &logger)
{
    RCLCPP_INFO(
        logger,
        "Moving to %s: x=%.3f, y=%.3f, z=%.3f",
        target_name.c_str(),
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z
    );

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    RCLCPP_INFO(
        logger,
        "Planning trajectory to %s...",
        target_name.c_str()
    );

    bool plan_success =
        static_cast<bool>(move_group.plan(plan));

    if (!plan_success)
    {
        RCLCPP_ERROR(
            logger,
            "Planning to %s failed.",
            target_name.c_str()
        );

        move_group.clearPoseTargets();

        return false;
    }

    RCLCPP_INFO(
        logger,
        "Planning to %s succeeded.",
        target_name.c_str()
    );

    bool execute_success =
        static_cast<bool>(move_group.execute(plan));

    if (!execute_success)
    {
        RCLCPP_ERROR(
            logger,
            "Execution to %s failed.",
            target_name.c_str()
        );

        move_group.clearPoseTargets();

        return false;
    }

    RCLCPP_INFO(
        logger,
        "Reached %s successfully.",
        target_name.c_str()
    );

    move_group.clearPoseTargets();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(800)
    );

    return true;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "waypoint_control",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        }
    );

    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "panda_arm"
    );

    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);

    move_group.setMaxVelocityScalingFactor(0.20);
    move_group.setMaxAccelerationScalingFactor(0.20);

    std::this_thread::sleep_for(
        std::chrono::seconds(1)
    );

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

    // 读取当前位姿，作为 Home
    geometry_msgs::msg::PoseStamped home_pose =
        move_group.getCurrentPose();

    RCLCPP_INFO(
        logger,
        "Home position: x=%.3f, y=%.3f, z=%.3f",
        home_pose.pose.position.x,
        home_pose.pose.position.y,
        home_pose.pose.position.z
    );

    // A 点
    geometry_msgs::msg::PoseStamped pose_a = home_pose;

    pose_a.pose.position.x += 0.03;
    pose_a.pose.position.y += 0.05;
    pose_a.pose.position.z += 0.02;

    // B 点
    geometry_msgs::msg::PoseStamped pose_b = home_pose;

    pose_b.pose.position.x += 0.06;
    pose_b.pose.position.y += 0.00;
    pose_b.pose.position.z += 0.05;

    // C 点
    geometry_msgs::msg::PoseStamped pose_c = home_pose;

    pose_c.pose.position.x += 0.03;
    pose_c.pose.position.y -= 0.05;
    pose_c.pose.position.z += 0.02;

    RCLCPP_INFO(
        logger,
        "Start waypoint sequence: Home -> A -> B -> C -> Home"
    );

    if (!moveToTarget(
            move_group,
            pose_a,
            "Point A",
            logger))
    {
        goto shutdown;
    }

    if (!moveToTarget(
            move_group,
            pose_b,
            "Point B",
            logger))
    {
        goto shutdown;
    }

    if (!moveToTarget(
            move_group,
            pose_c,
            "Point C",
            logger))
    {
        goto shutdown;
    }

    if (!moveToTarget(
            move_group,
            home_pose,
            "Home",
            logger))
    {
        goto shutdown;
    }

    RCLCPP_INFO(
        logger,
        "Waypoint sequence completed successfully!"
    );


shutdown:

    executor.cancel();

    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();

    return 0;
}
