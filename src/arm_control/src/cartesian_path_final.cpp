#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>


// ============================================================
// 参数
// ============================================================

// Cartesian演示速度：8%
constexpr double CART_VELOCITY = 0.08;

// 加速度不能真正为0
constexpr double CART_ACCELERATION = 0.03;


// 回Home稍快
constexpr double HOME_VELOCITY = 0.15;
constexpr double HOME_ACCELERATION = 0.08;


// Cartesian插值步长
// 5 mm
constexpr double EEF_STEP = 0.005;


// 轨迹至少完成98%
constexpr double MIN_FRACTION = 0.98;


// 最终末端误差：1.5 cm
constexpr double POSITION_TOLERANCE = 0.015;


// Home最终最大关节误差
constexpr double HOME_TOLERANCE = 0.05;


// ============================================================
// 笛卡尔规划结果
// ============================================================

struct CartesianResult
{
    bool success = false;

    std::string pattern;

    double distance = 0.0;

    double fraction = 0.0;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    moveit_msgs::msg::RobotTrajectory trajectory;
};


// ============================================================
// 位置误差
// ============================================================

double positionDistance(
    const geometry_msgs::msg::Pose &a,
    const geometry_msgs::msg::Pose &b)
{
    const double dx =
        a.position.x -
        b.position.x;

    const double dy =
        a.position.y -
        b.position.y;

    const double dz =
        a.position.z -
        b.position.z;


    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );
}


// ============================================================
// 打印Pose
// ============================================================

void printPose(
    const std::string &name,
    const geometry_msgs::msg::Pose &pose)
{
    std::cout
        << name
        << "  x="
        << std::fixed
        << std::setprecision(4)
        << pose.position.x
        << "  y="
        << pose.position.y
        << "  z="
        << pose.position.z
        << "\n";
}


// ============================================================
// 当前构型与目标最大关节误差
// ============================================================

double maxJointError(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const moveit::core::JointModelGroup *joint_group,
    const std::vector<double> &target)
{
    auto state =
        move_group.getCurrentState(2.0);

    if (!state)
    {
        return 999.0;
    }


    std::vector<double> current;

    state->copyJointGroupPositions(
        joint_group,
        current
    );


    if (current.size() != target.size())
    {
        return 999.0;
    }


    double error = 0.0;


    for (std::size_t i = 0;
         i < target.size();
         ++i)
    {
        error = std::max(
            error,
            std::fabs(
                current[i] -
                target[i]
            )
        );
    }


    return error;
}


// ============================================================
// 返回Safe Home
// ============================================================

bool moveToHome(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const moveit::core::JointModelGroup *joint_group,
    const std::vector<double> &home,
    const rclcpp::Logger &logger)
{
    const double initial_error =
        maxJointError(
            move_group,
            joint_group,
            home
        );


    if (initial_error <
        HOME_TOLERANCE)
    {
        RCLCPP_INFO(
            logger,
            "Robot already at Safe Home."
        );

        return true;
    }


    for (int attempt = 1;
         attempt <= 4;
         ++attempt)
    {
        std::cout
            << "Return Home attempt "
            << attempt
            << "/4\n";


        move_group.stop();

        move_group.clearPoseTargets();

        move_group.clearPathConstraints();


        move_group.setPlanningPipelineId(
            "ompl"
        );


        move_group.setPlannerId(
            "RRTConnectkConfigDefault"
        );


        move_group.setPlanningTime(
            5.0
        );


        move_group.setNumPlanningAttempts(
            8
        );


        move_group.setMaxVelocityScalingFactor(
            HOME_VELOCITY
        );


        move_group.setMaxAccelerationScalingFactor(
            HOME_ACCELERATION
        );


        move_group.setStartStateToCurrentState();


        if (!move_group.setJointValueTarget(
                home))
        {
            continue;
        }


        moveit::planning_interface::
            MoveGroupInterface::Plan plan;


        const bool plan_success =
            static_cast<bool>(
                move_group.plan(
                    plan
                )
            );


        if (!plan_success)
        {
            continue;
        }


        const bool raw_success =
            static_cast<bool>(
                move_group.execute(
                    plan
                )
            );


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                700
            )
        );


        const double final_error =
            maxJointError(
                move_group,
                joint_group,
                home
            );


        std::cout
            << "Home joint error = "
            << final_error
            << " rad\n";


        // 即使控制器反馈abort，
        // 只要实际已经回Home也视为成功
        if (raw_success ||
            final_error <
            HOME_TOLERANCE)
        {
            return true;
        }
    }


    return false;
}


// ============================================================
// Cartesian轨迹重新时间参数化
//
// computeCartesianPath负责几何路径。
// 这里给路径加入速度和时间信息。
// ============================================================

bool retimeTrajectory(
    const moveit::core::RobotStatePtr &start_state,
    const std::string &group_name,
    moveit_msgs::msg::RobotTrajectory &trajectory)
{
    robot_trajectory::RobotTrajectory robot_traj(
        start_state->getRobotModel(),
        group_name
    );


    robot_traj.setRobotTrajectoryMsg(
        *start_state,
        trajectory
    );


    trajectory_processing::
        IterativeParabolicTimeParameterization iptp;


    const bool success =
        iptp.computeTimeStamps(
            robot_traj,
            CART_VELOCITY,
            CART_ACCELERATION
        );


    if (!success)
    {
        return false;
    }


    robot_traj.getRobotTrajectoryMsg(
        trajectory
    );


    return true;
}


// ============================================================
// 发布轨迹给RViz
// ============================================================

void publishTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::core::RobotStatePtr &start_state,

    const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    moveit_msgs::msg::DisplayTrajectory display;


    display.model_id =
        "panda";


    moveit::core::robotStateToRobotStateMsg(
        *start_state,
        display.trajectory_start
    );


    display.trajectory.push_back(
        trajectory
    );


    publisher->publish(
        display
    );
}


// ============================================================
// 生成路径
//
// Pattern A:
//
//       Start
//         ●
//         │
//         ↓ Z
//         ●──────→ +X
//                 ●
//                 │
//                 ↑ Z
//                 ●
//
// Pattern B:
//
//       Start
//         ●
//         │
//         ↓ Z
//         ●←────── -X
//        ●
//        ↑
//
// 如果+X受限自动尝试-X。
// ============================================================

std::vector<geometry_msgs::msg::Pose>
generateWaypoints(
    const geometry_msgs::msg::Pose &start,
    double distance,
    bool positive_x)
{
    std::vector<
        geometry_msgs::msg::Pose
    > waypoints;


    // --------------------------------------------------------
    // 1. Z向下
    // --------------------------------------------------------

    geometry_msgs::msg::Pose p1 =
        start;


    p1.position.z -=
        distance;


    waypoints.push_back(
        p1
    );


    // --------------------------------------------------------
    // 2. X方向运动
    // --------------------------------------------------------

    geometry_msgs::msg::Pose p2 =
        p1;


    if (positive_x)
    {
        p2.position.x +=
            distance;
    }
    else
    {
        p2.position.x -=
            distance;
    }


    waypoints.push_back(
        p2
    );


    // --------------------------------------------------------
    // 3. Z向上
    // --------------------------------------------------------

    geometry_msgs::msg::Pose p3 =
        p2;


    p3.position.z +=
        distance;


    waypoints.push_back(
        p3
    );


    return waypoints;
}


// ============================================================
// 自动寻找完整Cartesian Path
// ============================================================

CartesianResult findCartesianPath(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &start_pose)
{
    CartesianResult best;


    // 优先15cm
    const std::vector<double> distances =
    {
        0.15,
        0.12,
        0.10
    };


    // 先试+X，再试-X
    const std::vector<bool> directions =
    {
        true,
        false
    };


    for (double distance :
         distances)
    {
        for (bool positive_x :
             directions)
        {
            const std::string pattern =
                positive_x
                ?
                "Z_DOWN -> +X -> Z_UP"
                :
                "Z_DOWN -> -X -> Z_UP";


            std::cout
                << "\n--------------------------------------------\n";


            std::cout
                << "Trying pattern  : "
                << pattern
                << "\n";


            std::cout
                << "Segment distance: "
                << distance
                << " m\n";


            auto waypoints =
                generateWaypoints(
                    start_pose,
                    distance,
                    positive_x
                );


            moveit_msgs::msg::
                RobotTrajectory trajectory;


            move_group.setStartStateToCurrentState();


            // =================================================
            // 核心MoveIt Cartesian API
            // =================================================

            const double fraction =
                move_group.computeCartesianPath(
                    waypoints,
                    EEF_STEP,
                    0.0,
                    trajectory,
                    true
                );


            std::cout
                << "Cartesian fraction = "
                << fraction * 100.0
                << "%\n";


            // 保存目前最完整的一条
            if (fraction >
                best.fraction)
            {
                best.pattern =
                    pattern;

                best.distance =
                    distance;

                best.fraction =
                    fraction;

                best.waypoints =
                    waypoints;

                best.trajectory =
                    trajectory;
            }


            // 满足98%直接采用
            if (fraction >=
                MIN_FRACTION)
            {
                best.success =
                    true;

                return best;
            }
        }
    }


    return best;
}


// ============================================================
// MAIN
// ============================================================

int main(
    int argc,
    char *argv[])
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<
            rclcpp::Node
        >(
            "cartesian_path",

            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true
                )
        );


    auto logger =
        node->get_logger();


    // ========================================================
    // ROS executor
    // ========================================================

    rclcpp::executors::
        SingleThreadedExecutor executor;


    executor.add_node(
        node
    );


    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        }
    );


    // ========================================================
    // MoveIt
    // ========================================================

    const std::string group_name =
        "panda_arm";


    moveit::planning_interface::
        MoveGroupInterface move_group(
            node,
            group_name
        );


    moveit::planning_interface::
        PlanningSceneInterface scene;


    move_group.allowReplanning(
        true
    );


    move_group.setGoalJointTolerance(
        0.001
    );


    std::string ee_link =
        move_group.getEndEffectorLink();


    if (ee_link.empty())
    {
        ee_link =
            "panda_link8";
    }


    // ========================================================
    // 清理上一轮避障实验可能残留的BOX
    // ========================================================

    scene.removeCollisionObjects(
        {
            "benchmark_box",
            "tiny_benchmark_box"
        }
    );


    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            500
        )
    );


    // ========================================================
    // Display trajectory
    // ========================================================

    auto display_publisher =
        node->create_publisher<
            moveit_msgs::msg::DisplayTrajectory
        >(
            "/display_planned_path",
            10
        );


    // ========================================================
    // Robot model
    // ========================================================

    auto initial_state =
        move_group.getCurrentState(
            5.0
        );


    if (!initial_state)
    {
        RCLCPP_ERROR(
            logger,
            "Cannot obtain Panda state."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    const auto *joint_group =
        initial_state
            ->getRobotModel()
            ->getJointModelGroup(
                group_name
            );


    if (!joint_group)
    {
        RCLCPP_ERROR(
            logger,
            "Cannot find panda_arm."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    // ========================================================
    // 固定Safe Home
    // ========================================================

    const std::vector<double> home =
    {
         0.000,
        -0.785,
         0.000,
        -2.356,
         0.000,
         1.571,
         0.785
    };


    std::cout
        << "\n\n";


    std::cout
        << "====================================================\n";


    std::cout
        << "     PANDA CARTESIAN PATH EXPERIMENT\n";


    std::cout
        << "====================================================\n";


    std::cout
        << "Planning group : "
        << group_name
        << "\n";


    std::cout
        << "End effector   : "
        << ee_link
        << "\n";


    std::cout
        << "EEF step       : "
        << EEF_STEP
        << " m\n";


    std::cout
        << "Velocity       : "
        << CART_VELOCITY * 100.0
        << "%\n";


    std::cout
        << "Acceleration   : "
        << CART_ACCELERATION * 100.0
        << "%\n";


    // ========================================================
    // PHASE 1
    // 回Home
    // ========================================================

    std::cout
        << "\n====================================================\n";


    std::cout
        << "PHASE 1 : SAFE HOME\n";


    std::cout
        << "====================================================\n";


    if (!moveToHome(
            move_group,
            joint_group,
            home,
            logger))
    {
        RCLCPP_ERROR(
            logger,
            "Unable to reach Safe Home."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    std::cout
        << "Safe Home reached.\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            1
        )
    );


    // ========================================================
    // 当前末端位置
    // ========================================================

    const geometry_msgs::msg::Pose
        start_pose =
            move_group.getCurrentPose(
                ee_link
            ).pose;


    std::cout
        << "\n";


    printPose(
        "Start pose:",
        start_pose
    );


    // ========================================================
    // PHASE 2
    // Cartesian规划
    // ========================================================

    std::cout
        << "\n====================================================\n";


    std::cout
        << "PHASE 2 : CARTESIAN PATH PLANNING\n";


    std::cout
        << "====================================================\n";


    CartesianResult cartesian =
        findCartesianPath(
            move_group,
            start_pose
        );


    if (!cartesian.success)
    {
        std::cout
            << "\nBest fraction obtained = "
            << cartesian.fraction * 100.0
            << "%\n";


        RCLCPP_ERROR(
            logger,
            "No Cartesian path reached 98%% completeness."
        );


        RCLCPP_ERROR(
            logger,
            "Incomplete trajectory will NOT be executed."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    std::cout
        << "\n====================================================\n";


    std::cout
        << "VALID CARTESIAN PATH FOUND\n";


    std::cout
        << "====================================================\n";


    std::cout
        << "Pattern         : "
        << cartesian.pattern
        << "\n";


    std::cout
        << "Segment distance: "
        << cartesian.distance
        << " m\n";


    std::cout
        << "Fraction        : "
        << cartesian.fraction * 100.0
        << "%\n";


    std::cout
        << "Total geometric path ≈ "
        << cartesian.distance * 3.0
        << " m\n";


    for (std::size_t i = 0;
         i < cartesian.waypoints.size();
         ++i)
    {
        printPose(
            "Waypoint "
            +
            std::to_string(
                i + 1
            )
            +
            ":",
            cartesian.waypoints[i]
        );
    }


    // ========================================================
    // PHASE 3
    // 时间参数化
    // ========================================================

    std::cout
        << "\n====================================================\n";


    std::cout
        << "PHASE 3 : TRAJECTORY TIME PARAMETERIZATION\n";


    std::cout
        << "====================================================\n";


    auto execution_start_state =
        move_group.getCurrentState(
            5.0
        );


    if (!execution_start_state)
    {
        RCLCPP_ERROR(
            logger,
            "Cannot obtain execution start state."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    if (!retimeTrajectory(
            execution_start_state,
            group_name,
            cartesian.trajectory))
    {
        RCLCPP_ERROR(
            logger,
            "Trajectory time parameterization failed."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    std::cout
        << "Time parameterization SUCCESS\n";


    // ========================================================
    // RViz显示
    // ========================================================

    publishTrajectory(
        display_publisher,
        execution_start_state,
        cartesian.trajectory
    );


    std::cout
        << "\nTrajectory displayed in RViz.\n";


    std::cout
        << "Robot starts VERY SLOWLY in 4 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            4
        )
    );


    // ========================================================
    // PHASE 4
    // 真正执行
    // ========================================================

    std::cout
        << "\n====================================================\n";


    std::cout
        << "PHASE 4 : EXECUTE CARTESIAN PATH\n";


    std::cout
        << "====================================================\n";


    moveit::planning_interface::
        MoveGroupInterface::Plan execution_plan;


    execution_plan.trajectory_ =
        cartesian.trajectory;


    const auto begin =
        std::chrono::
            steady_clock::now();


    const bool raw_execution_success =
        static_cast<bool>(
            move_group.execute(
                execution_plan
            )
        );


    const auto end =
        std::chrono::
            steady_clock::now();


    const double execution_time =
        std::chrono::
            duration<double>(
                end - begin
            ).count();


    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            800
        )
    );


    // ========================================================
    // 最终真实位置
    // ========================================================

    const geometry_msgs::msg::Pose
        actual_final_pose =
            move_group.getCurrentPose(
                ee_link
            ).pose;


    const geometry_msgs::msg::Pose
        expected_final_pose =
            cartesian.waypoints.back();


    const double final_position_error =
        positionDistance(
            actual_final_pose,
            expected_final_pose
        );


    const bool target_reached =
        final_position_error <=
        POSITION_TOLERANCE;


    // 最终以真实误差作为主要依据
    const bool final_success =
        cartesian.fraction >=
            MIN_FRACTION
        &&
        target_reached;


    // ========================================================
    // 结果
    // ========================================================

    std::cout
        << "\n====================================================\n";


    std::cout
        << "CARTESIAN PATH RESULT\n";


    std::cout
        << "====================================================\n";


    std::cout
        << "Pattern          : "
        << cartesian.pattern
        << "\n";


    std::cout
        << "Segment distance : "
        << cartesian.distance
        << " m\n";


    std::cout
        << "Total path       : "
        << cartesian.distance * 3.0
        << " m\n";


    std::cout
        << "Fraction         : "
        << cartesian.fraction * 100.0
        << "%\n";


    std::cout
        << "Execution time   : "
        << execution_time
        << " s\n";


    std::cout
        << "Position error   : "
        << final_position_error
        << " m\n";


    std::cout
        << "MoveIt feedback  : "
        << (
            raw_execution_success
            ?
            "SUCCESS"
            :
            "ABORT/FAIL"
        )
        << "\n";


    std::cout
        << "Target reached   : "
        << (
            target_reached
            ?
            "YES"
            :
            "NO"
        )
        << "\n";


    std::cout
        << "FINAL RESULT     : "
        << (
            final_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    // ========================================================
    // CSV保存
    // ========================================================

    const char *home_env =
        std::getenv(
            "HOME"
        );


    const std::string root =
        home_env
        ?
        home_env
        :
        "/home/yj";


    const std::string csv_path =
        root
        +
        "/Robot_Arm_Project/data/"
        "cartesian_path_result.csv";


    std::ofstream csv(
        csv_path
    );


    csv
        << "pattern,"
        << "segment_distance_m,"
        << "total_path_m,"
        << "fraction,"
        << "eef_step_m,"
        << "velocity_scale,"
        << "acceleration_scale,"
        << "execution_time_s,"
        << "position_error_m,"
        << "moveit_feedback,"
        << "success\n";


    csv
        << "\""
        << cartesian.pattern
        << "\","
        << cartesian.distance
        << ","
        << cartesian.distance * 3.0
        << ","
        << cartesian.fraction
        << ","
        << EEF_STEP
        << ","
        << CART_VELOCITY
        << ","
        << CART_ACCELERATION
        << ","
        << execution_time
        << ","
        << final_position_error
        << ","
        << (
            raw_execution_success
            ?
            1
            :
            0
        )
        << ","
        << (
            final_success
            ?
            1
            :
            0
        )
        << "\n";


    csv.close();


    std::cout
        << "CSV              : "
        << csv_path
        << "\n";


    // ========================================================
    // 停留3秒
    // ========================================================

    std::cout
        << "\nHolding Cartesian final pose for 3 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            3
        )
    );


    // ========================================================
    // PHASE 5
    // 返回Home
    // ========================================================

    std::cout
        << "\n====================================================\n";


    std::cout
        << "PHASE 5 : RETURN SAFE HOME\n";


    std::cout
        << "====================================================\n";


    const bool home_success =
        moveToHome(
            move_group,
            joint_group,
            home,
            logger
        );


    std::cout
        << "\n====================================================\n";


    std::cout
        << "FINAL SUMMARY\n";


    std::cout
        << "====================================================\n";


    std::cout
        << "Motion            : "
        << cartesian.pattern
        << "\n";


    std::cout
        << "Segment distance  : "
        << cartesian.distance
        << " m\n";


    std::cout
        << "Total path        : "
        << cartesian.distance * 3.0
        << " m\n";


    std::cout
        << "Cartesian fraction: "
        << cartesian.fraction * 100.0
        << "%\n";


    std::cout
        << "Final error       : "
        << final_position_error
        << " m\n";


    std::cout
        << "Cartesian result  : "
        << (
            final_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "Return Home       : "
        << (
            home_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "====================================================\n";


    executor.cancel();


    if (spin_thread.joinable())
    {
        spin_thread.join();
    }


    rclcpp::shutdown();


    return
        final_success
        ?
        0
        :
        1;
}
