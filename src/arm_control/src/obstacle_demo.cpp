#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>


int main(int argc, char * argv[])
{
    // =====================================================
    // 1. 初始化 ROS2
    // =====================================================
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "obstacle_demo",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    // ROS2 回调线程
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        }
    );


    // =====================================================
    // 2. 创建 MoveIt2 接口
    // =====================================================
    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "panda_arm"
    );

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;


    // =====================================================
    // 3. 设置规划参数
    // =====================================================

    // 最长允许规划 15 秒
    move_group.setPlanningTime(15.0);

    // 最多尝试 50 次
    move_group.setNumPlanningAttempts(50);

    // 指定 OMPL 中非常常用的 RRTConnect
    move_group.setPlannerId(
        "RRTConnectkConfigDefault"
    );

    // 降低机械臂速度
    move_group.setMaxVelocityScalingFactor(0.15);
    move_group.setMaxAccelerationScalingFactor(0.15);

    // 目标位置允许 1 cm 误差
    move_group.setGoalPositionTolerance(0.01);

    // 姿态允许一定误差
    move_group.setGoalOrientationTolerance(0.05);


    std::this_thread::sleep_for(
        std::chrono::seconds(1)
    );


    // =====================================================
    // 4. 获取当前机械臂末端位姿
    // =====================================================

    move_group.setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped current_pose =
        move_group.getCurrentPose();


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

    RCLCPP_INFO(
        logger,
        "Current position: x=%.3f, y=%.3f, z=%.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    );


    // =====================================================
    // 5. 创建 BOX 障碍物
    // =====================================================

    moveit_msgs::msg::CollisionObject obstacle;

    obstacle.header.frame_id =
        move_group.getPlanningFrame();

    obstacle.id = "obstacle_box";


    shape_msgs::msg::SolidPrimitive box;

    box.type =
        shape_msgs::msg::SolidPrimitive::BOX;

    box.dimensions.resize(3);


    // =====================================================
    // BOX 尺寸
    // =====================================================

    // X方向厚度：6 cm
    box.dimensions[0] = 0.06;

    // Y方向宽度：10 cm
    box.dimensions[1] = 0.10;

    // Z方向高度：16 cm
    box.dimensions[2] = 0.16;


    // =====================================================
    // 6. BOX 位置
    // =====================================================

    geometry_msgs::msg::Pose obstacle_pose;

    obstacle_pose.orientation.w = 1.0;

    // 放到 Start 与 Target 之间
    obstacle_pose.position.x =
        current_pose.pose.position.x + 0.125;

    obstacle_pose.position.y =
        current_pose.pose.position.y + 0.08;

    obstacle_pose.position.z =
        current_pose.pose.position.z + 0.03;


    obstacle.primitives.push_back(
        box
    );

    obstacle.primitive_poses.push_back(
        obstacle_pose
    );

    obstacle.operation =
        moveit_msgs::msg::CollisionObject::ADD;


    // =====================================================
    // 7. 添加障碍物到 Planning Scene
    // =====================================================

    RCLCPP_INFO(
        logger,
        "Adding obstacle..."
    );

    planning_scene_interface.applyCollisionObject(
        obstacle
    );

    // 等待 RViz / MoveIt 更新场景
    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );


    RCLCPP_INFO(
        logger,
        "Obstacle position: x=%.3f, y=%.3f, z=%.3f",
        obstacle_pose.position.x,
        obstacle_pose.position.y,
        obstacle_pose.position.z
    );


    // =====================================================
    // 8. 创建目标位姿
    // =====================================================

    geometry_msgs::msg::PoseStamped target_pose =
        current_pose;

    // 相对于当前位置移动
    target_pose.pose.position.x += 0.25;
    target_pose.pose.position.y += 0.16;
    target_pose.pose.position.z += 0.06;


    RCLCPP_INFO(
        logger,
        "Target position: x=%.3f, y=%.3f, z=%.3f",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z
    );


    // =====================================================
    // 9. 设置运动目标
    // =====================================================

    move_group.setStartStateToCurrentState();

    move_group.setPoseTarget(
        target_pose
    );


    moveit::planning_interface::
        MoveGroupInterface::Plan plan;


    // =====================================================
    // 10. 开始无碰撞路径规划
    // =====================================================

    RCLCPP_INFO(
        logger,
        "Planning collision-free trajectory..."
    );


    bool planning_success =
        static_cast<bool>(
            move_group.plan(plan)
        );


    // =====================================================
    // 11. 执行
    // =====================================================

    if (planning_success)
    {
        RCLCPP_INFO(
            logger,
            "Collision-free planning succeeded!"
        );

        RCLCPP_INFO(
            logger,
            "Executing trajectory..."
        );


        bool execution_success =
            static_cast<bool>(
                move_group.execute(plan)
            );


        if (execution_success)
        {
            RCLCPP_INFO(
                logger,
                "Obstacle avoidance completed successfully!"
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
            "Planning failed: no collision-free path found."
        );
    }


    // =====================================================
    // 12. 清理
    // =====================================================

    move_group.clearPoseTargets();

    executor.cancel();

    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();

    return 0;
}
