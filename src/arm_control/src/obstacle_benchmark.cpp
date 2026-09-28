#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>


// ============================================================
// 1. 实验参数
// ============================================================

// 长距离任务：末端起点到终点至少 1.0 m
constexpr double MIN_POINT_TO_POINT_TRAVEL = 1.00;


// 真正展示避障时速度非常慢
constexpr double EXEC_VELOCITY = 0.06;

// 加速度不能为0，否则轨迹无法正常起停
constexpr double EXEC_ACCELERATION = 0.02;


// 回Home稍微快一点
constexpr double RESET_VELOCITY = 0.14;
constexpr double RESET_ACCELERATION = 0.06;


// 最终目标误差
constexpr double TARGET_TOLERANCE = 0.05;


// ============================================================
// 障碍物扩大3倍
//
// 上一版：
// 6 × 8 × 12 mm
//
// 现在：
// 18 × 24 × 36 mm
// ============================================================

constexpr double BOX_X = 0.018;
constexpr double BOX_Y = 0.024;
constexpr double BOX_Z = 0.036;


// 场景标定时，每个候选位置测试3次
constexpr int CALIBRATION_TRIALS = 3;


// ============================================================
// 2. 数据结构
// ============================================================

struct PlanResult
{
    bool success = false;

    double planning_time = 0.0;
    double path_length = 0.0;

    std::size_t trajectory_points = 0;

    std::string planner = "NONE";

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;
};


struct TargetCandidate
{
    std::vector<double> joints;

    double displacement = 0.0;

    Eigen::Vector3d ee_position;
};


struct ObstacleScene
{
    bool valid = false;

    std::string name;

    // 希望的位置，例如40%
    double nominal_ratio = 0.0;

    // 最终实际采用的位置
    double actual_ratio = 0.0;

    double local_offset = 0.0;

    std::string direction = "NONE";

    geometry_msgs::msg::Pose pose;

    int calibration_successes = 0;

    double mean_calibration_path = 0.0;

    double calibration_stretch = 0.0;
};


struct ExperimentResult
{
    std::string name;

    double nominal_ratio = 0.0;
    double actual_ratio = 0.0;

    double local_offset = 0.0;

    std::string direction;

    double obstacle_x = 0.0;
    double obstacle_y = 0.0;
    double obstacle_z = 0.0;

    std::string planner = "NONE";

    double search_time = 0.0;

    double path_length = 0.0;

    double stretch = 0.0;

    double execution_time = 0.0;

    double final_error = 0.0;

    bool success = false;
};


// ============================================================
// 3. 数学工具
// ============================================================

double mean(
    const std::vector<double> &values)
{
    if (values.empty())
    {
        return 0.0;
    }

    return
        std::accumulate(
            values.begin(),
            values.end(),
            0.0
        )
        /
        static_cast<double>(
            values.size()
        );
}


// ============================================================
// 关节空间路径长度
//
// L = Σ ||q(k+1) - q(k)||
// ============================================================

double calculatePathLength(
    const moveit::planning_interface::
        MoveGroupInterface::Plan &plan)
{
    const auto &points =
        plan.trajectory_
            .joint_trajectory
            .points;


    if (points.size() < 2)
    {
        return 0.0;
    }


    double total = 0.0;


    for (std::size_t i = 1;
         i < points.size();
         ++i)
    {
        const auto &q0 =
            points[i - 1].positions;

        const auto &q1 =
            points[i].positions;


        const std::size_t n =
            std::min(
                q0.size(),
                q1.size()
            );


        double square_sum = 0.0;


        for (std::size_t j = 0;
             j < n;
             ++j)
        {
            const double dq =
                q1[j] - q0[j];


            square_sum +=
                dq * dq;
        }


        total +=
            std::sqrt(
                square_sum
            );
    }


    return total;
}


// ============================================================
// 当前关节误差
// ============================================================

double maxJointError(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &target)
{
    auto state =
        move_group.getCurrentState(
            2.0
        );


    if (!state)
    {
        return
            std::numeric_limits<double>::
                infinity();
    }


    std::vector<double> current;


    state->copyJointGroupPositions(
        joint_group,
        current
    );


    if (current.size() !=
        target.size())
    {
        return
            std::numeric_limits<double>::
                infinity();
    }


    double error = 0.0;


    for (std::size_t i = 0;
         i < target.size();
         ++i)
    {
        error =
            std::max(
                error,
                std::fabs(
                    current[i]
                    -
                    target[i]
                )
            );
    }


    return error;
}


// ============================================================
// 4. Planning Scene
// ============================================================

bool waitForObject(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &id,

    bool expected,

    double timeout = 3.0)
{
    const auto begin =
        std::chrono::
            steady_clock::now();


    while (true)
    {
        const auto names =
            scene.getKnownObjectNames();


        const bool exists =
            std::find(
                names.begin(),
                names.end(),
                id
            )
            !=
            names.end();


        if (exists ==
            expected)
        {
            return true;
        }


        const auto now =
            std::chrono::
                steady_clock::now();


        if (std::chrono::
                duration<double>(
                    now - begin
                ).count()
            >
            timeout)
        {
            return false;
        }


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    100
                )
        );
    }
}


void removeObstacle(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &id)
{
    scene.removeCollisionObjects(
        {id}
    );


    waitForObject(
        scene,
        id,
        false
    );
}


bool addObstacle(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &frame,

    const std::string &id,

    const geometry_msgs::msg::Pose &pose)
{
    moveit_msgs::msg::
        CollisionObject object;


    object.header.frame_id =
        frame;


    object.id =
        id;


    shape_msgs::msg::
        SolidPrimitive box;


    box.type =
        shape_msgs::msg::
            SolidPrimitive::BOX;


    box.dimensions.resize(
        3
    );


    box.dimensions[0] =
        BOX_X;


    box.dimensions[1] =
        BOX_Y;


    box.dimensions[2] =
        BOX_Z;


    object.primitives.push_back(
        box
    );


    object.primitive_poses.push_back(
        pose
    );


    object.operation =
        moveit_msgs::msg::
            CollisionObject::ADD;


    const bool applied =
        scene.applyCollisionObject(
            object
        );


    return
        applied
        &&
        waitForObject(
            scene,
            id,
            true
        );
}


// ============================================================
// 5. Collision Checking
// ============================================================

bool stateValid(
    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &q,

    const std::string &group_name)
{
    moveit::core::RobotState state(
        reference
    );


    state.setJointGroupPositions(
        joint_group,
        q
    );


    state.update();


    moveit_msgs::msg::
        RobotState state_msg;


    moveit::core::
        robotStateToRobotStateMsg(
            state,
            state_msg
        );


    auto request =
        std::make_shared<
            moveit_msgs::srv::
                GetStateValidity::Request
        >();


    request->robot_state =
        state_msg;


    request->group_name =
        group_name;


    auto future =
        client->async_send_request(
            request
        );


    if (future.wait_for(
            std::chrono::seconds(
                2
            ))
        !=
        std::future_status::ready)
    {
        return false;
    }


    return
        future.get()->valid;
}


// ============================================================
// FK
// ============================================================

Eigen::Vector3d getLinkPosition(
    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &q,

    const std::string &link_name)
{
    moveit::core::RobotState state(
        reference
    );


    state.setJointGroupPositions(
        joint_group,
        q
    );


    state.update();


    return
        state.getGlobalLinkTransform(
            link_name
        ).translation();
}


// ============================================================
// 6. Planning
// ============================================================

PlanResult planWithPlanner(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const std::vector<double> &target,

    const std::string &planner_id,

    const std::string &planner_name,

    double planning_time,

    unsigned int attempts,

    double velocity,

    double acceleration)
{
    PlanResult result;


    result.planner =
        planner_name;


    move_group.stop();


    move_group.clearPoseTargets();

    move_group.clearPathConstraints();


    move_group.setPlanningPipelineId(
        "ompl"
    );


    move_group.setPlannerId(
        planner_id
    );


    move_group.setPlanningTime(
        planning_time
    );


    move_group.setNumPlanningAttempts(
        attempts
    );


    move_group.setMaxVelocityScalingFactor(
        velocity
    );


    move_group.setMaxAccelerationScalingFactor(
        acceleration
    );


    move_group.setStartStateToCurrentState();


    if (!move_group.setJointValueTarget(
            target))
    {
        return result;
    }


    const auto begin =
        std::chrono::
            steady_clock::now();


    result.success =
        static_cast<bool>(
            move_group.plan(
                result.plan
            )
        );


    const auto end =
        std::chrono::
            steady_clock::now();


    result.planning_time =
        std::chrono::
            duration<double>(
                end - begin
            ).count();


    if (result.success)
    {
        result.path_length =
            calculatePathLength(
                result.plan
            );


        result.trajectory_points =
            result.plan
                .trajectory_
                .joint_trajectory
                .points
                .size();
    }


    return result;
}


// ============================================================
// 二次轨迹碰撞验证
// ============================================================

bool validateTrajectory(
    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const moveit::planning_interface::
        MoveGroupInterface::Plan &plan,

    const std::string &group_name)
{
    const auto &points =
        plan.trajectory_
            .joint_trajectory
            .points;


    if (points.empty())
    {
        return false;
    }


    const std::size_t step =
        std::max<
            std::size_t
        >(
            1,
            points.size() /
            60
        );


    for (std::size_t i = 0;
         i < points.size();
         i += step)
    {
        if (!stateValid(
                client,
                reference,
                joint_group,
                points[i].positions,
                group_name))
        {
            return false;
        }
    }


    return
        stateValid(
            client,
            reference,
            joint_group,
            points.back().positions,
            group_name
        );
}


// ============================================================
// 原Baseline是否被障碍物阻断
// ============================================================

bool baselineBlocked(
    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const moveit::planning_interface::
        MoveGroupInterface::Plan &baseline,

    const std::string &group_name)
{
    const auto &points =
        baseline.trajectory_
            .joint_trajectory
            .points;


    for (const auto &point :
         points)
    {
        if (!stateValid(
                client,
                reference,
                joint_group,
                point.positions,
                group_name))
        {
            return true;
        }
    }


    return false;
}


// ============================================================
// RViz轨迹
// ============================================================

void publishTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::planning_interface::
        MoveGroupInterface::Plan &plan)
{
    moveit_msgs::msg::
        DisplayTrajectory display;


    display.model_id =
        "panda";


    display.trajectory_start =
        plan.start_state_;


    display.trajectory.push_back(
        plan.trajectory_
    );


    publisher->publish(
        display
    );
}


// ============================================================
// 7. Home
// ============================================================

bool returnHome(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &home,

    const rclcpp::Logger &logger)
{
    if (maxJointError(
            move_group,
            joint_group,
            home)
        <
        0.04)
    {
        RCLCPP_INFO(
            logger,
            "Robot already at Home."
        );


        return true;
    }


    for (int attempt = 1;
         attempt <= 5;
         ++attempt)
    {
        RCLCPP_INFO(
            logger,
            "Return Home attempt %d/5",
            attempt
        );


        PlanResult plan =
            planWithPlanner(
                move_group,
                home,
                "RRTConnectkConfigDefault",
                "RRTConnect",
                6.0,
                10,
                RESET_VELOCITY,
                RESET_ACCELERATION
            );


        if (!plan.success)
        {
            continue;
        }


        const bool raw =
            static_cast<bool>(
                move_group.execute(
                    plan.plan
                )
            );


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    700
                )
        );


        const double error =
            maxJointError(
                move_group,
                joint_group,
                home
            );


        if (raw ||
            error <
            TARGET_TOLERANCE)
        {
            return true;
        }
    }


    return false;
}


// ============================================================
// 8. 自动寻找长距离Target
//
// 定义：
// Home EE -> Target EE 点到点位移 >= 1.0m
//
// 不再使用错误的“最大工作半径百分比”。
// ============================================================

bool findLongDistanceTarget(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &validity_client,

    const moveit::core::RobotState &home_state,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &home,

    const std::string &group_name,

    const std::string &ee_link,

    std::vector<double> &selected_target,

    double &selected_displacement)
{
    const Eigen::Vector3d home_ee =
        getLinkPosition(
            home_state,
            joint_group,
            home,
            ee_link
        );


    const std::array<
        std::pair<double, double>,
        7
    > bounds =
    {{
        {-2.85,  2.85},
        {-1.70,  1.70},
        {-2.85,  2.85},
        {-3.00, -0.15},
        {-2.85,  2.85},
        { 0.10,  3.65},
        {-2.85,  2.85}
    }};


    std::mt19937 rng(
        20260928
    );


    std::vector<TargetCandidate>
        candidates;


    constexpr int RANDOM_SAMPLES =
        12000;


    // ========================================================
    // 快速FK搜索
    // ========================================================

    for (int sample = 0;
         sample < RANDOM_SAMPLES;
         ++sample)
    {
        std::vector<double> q(
            7,
            0.0
        );


        for (std::size_t j = 0;
             j < 7;
             ++j)
        {
            std::uniform_real_distribution<double>
                distribution(
                    bounds[j].first,
                    bounds[j].second
                );


            q[j] =
                distribution(
                    rng
                );
        }


        const Eigen::Vector3d ee =
            getLinkPosition(
                home_state,
                joint_group,
                q,
                ee_link
            );


        const double displacement =
            (
                ee -
                home_ee
            ).norm();


        if (displacement <
            MIN_POINT_TO_POINT_TRAVEL)
        {
            continue;
        }


        TargetCandidate candidate;


        candidate.joints =
            q;


        candidate.displacement =
            displacement;


        candidate.ee_position =
            ee;


        candidates.push_back(
            candidate
        );
    }


    if (candidates.empty())
    {
        return false;
    }


    std::sort(
        candidates.begin(),
        candidates.end(),

        [](
            const TargetCandidate &a,
            const TargetCandidate &b)
        {
            return
                a.displacement
                >
                b.displacement;
        }
    );


    const std::size_t max_checks =
        std::min<
            std::size_t
        >(
            candidates.size(),
            150
        );


    // ========================================================
    // MoveIt真正验证
    // ========================================================

    for (std::size_t i = 0;
         i < max_checks;
         ++i)
    {
        const auto &candidate =
            candidates[i];


        if (!stateValid(
                validity_client,
                home_state,
                joint_group,
                candidate.joints,
                group_name))
        {
            continue;
        }


        std::cout
            << "Testing Target "
            << i + 1
            << " | EE travel="
            << std::fixed
            << std::setprecision(
                3
            )
            << candidate.displacement
            << " m\n";


        PlanResult test =
            planWithPlanner(
                move_group,
                candidate.joints,
                "RRTConnectkConfigDefault",
                "RRTConnect",
                6.0,
                10,
                RESET_VELOCITY,
                RESET_ACCELERATION
            );


        if (!test.success)
        {
            continue;
        }


        if (!validateTrajectory(
                validity_client,
                home_state,
                joint_group,
                test.plan,
                group_name))
        {
            continue;
        }


        selected_target =
            candidate.joints;


        selected_displacement =
            candidate.displacement;


        return true;
    }


    return false;
}


// ============================================================
// 9. Baseline
//
// RRTConnect × 4
// RRTstar × 2
// 选择最短合法路径
// ============================================================

PlanResult generateBaseline(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &validity_client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &target,

    const std::string &group_name)
{
    std::vector<PlanResult>
        candidates;


    for (int i = 0;
         i < 4;
         ++i)
    {
        PlanResult p =
            planWithPlanner(
                move_group,
                target,
                "RRTConnectkConfigDefault",
                "RRTConnect",
                6.0,
                10,
                EXEC_VELOCITY,
                EXEC_ACCELERATION
            );


        if (p.success &&
            validateTrajectory(
                validity_client,
                reference,
                joint_group,
                p.plan,
                group_name))
        {
            candidates.push_back(
                p
            );
        }
    }


    for (int i = 0;
         i < 2;
         ++i)
    {
        PlanResult p =
            planWithPlanner(
                move_group,
                target,
                "RRTstarkConfigDefault",
                "RRTstar",
                10.0,
                1,
                EXEC_VELOCITY,
                EXEC_ACCELERATION
            );


        if (p.success &&
            validateTrajectory(
                validity_client,
                reference,
                joint_group,
                p.plan,
                group_name))
        {
            candidates.push_back(
                p
            );
        }
    }


    if (candidates.empty())
    {
        return
            PlanResult();
    }


    auto best =
        std::min_element(
            candidates.begin(),
            candidates.end(),

            [](
                const PlanResult &a,
                const PlanResult &b)
            {
                return
                    a.path_length
                    <
                    b.path_length;
            }
        );


    return
        *best;
}


// ============================================================
// 10. 最终避障规划
//
// RRTConnect × 4
// RRTstar × 2
//
// 最终选择有限搜索时间中的最短合法轨迹。
// ============================================================

PlanResult optimizedDetourPlanning(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &validity_client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &target,

    const std::string &group_name)
{
    std::vector<PlanResult>
        candidates;


    double total_search_time =
        0.0;


    // ========================================================
    // RRTConnect × 4
    // ========================================================

    for (int i = 0;
         i < 4;
         ++i)
    {
        PlanResult p =
            planWithPlanner(
                move_group,
                target,
                "RRTConnectkConfigDefault",
                "RRTConnect",
                7.0,
                12,
                EXEC_VELOCITY,
                EXEC_ACCELERATION
            );


        total_search_time +=
            p.planning_time;


        if (p.success &&
            validateTrajectory(
                validity_client,
                reference,
                joint_group,
                p.plan,
                group_name))
        {
            candidates.push_back(
                p
            );
        }
    }


    // ========================================================
    // RRTstar × 2
    // ========================================================

    for (int i = 0;
         i < 2;
         ++i)
    {
        PlanResult p =
            planWithPlanner(
                move_group,
                target,
                "RRTstarkConfigDefault",
                "RRTstar",
                12.0,
                1,
                EXEC_VELOCITY,
                EXEC_ACCELERATION
            );


        total_search_time +=
            p.planning_time;


        if (p.success &&
            validateTrajectory(
                validity_client,
                reference,
                joint_group,
                p.plan,
                group_name))
        {
            candidates.push_back(
                p
            );
        }
    }


    if (candidates.empty())
    {
        PlanResult failed;


        failed.planning_time =
            total_search_time;


        return failed;
    }


    auto best =
        std::min_element(
            candidates.begin(),
            candidates.end(),

            [](
                const PlanResult &a,
                const PlanResult &b)
            {
                return
                    a.path_length
                    <
                    b.path_length;
            }
        );


    PlanResult selected =
        *best;


    selected.planning_time =
        total_search_time;


    return selected;
}


// ============================================================
// 11. 轨迹实际笛卡尔弧长比例
// ============================================================

std::size_t findCartesianRatioIndex(
    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const moveit::planning_interface::
        MoveGroupInterface::Plan &baseline,

    const std::string &tracking_link,

    double ratio)
{
    const auto &points =
        baseline.trajectory_
            .joint_trajectory
            .points;


    if (points.size() <= 1)
    {
        return 0;
    }


    std::vector<double> cumulative(
        points.size(),
        0.0
    );


    Eigen::Vector3d previous =
        getLinkPosition(
            reference,
            joint_group,
            points[0].positions,
            tracking_link
        );


    for (std::size_t i = 1;
         i < points.size();
         ++i)
    {
        const Eigen::Vector3d current =
            getLinkPosition(
                reference,
                joint_group,
                points[i].positions,
                tracking_link
            );


        cumulative[i] =
            cumulative[i - 1]
            +
            (
                current -
                previous
            ).norm();


        previous =
            current;
    }


    const double total =
        cumulative.back();


    const double desired =
        ratio *
        total;


    const auto it =
        std::lower_bound(
            cumulative.begin(),
            cumulative.end(),
            desired
        );


    if (it ==
        cumulative.end())
    {
        return
            points.size() - 1;
    }


    return
        static_cast<std::size_t>(
            std::distance(
                cumulative.begin(),
                it
            )
        );
}


// ============================================================
// 12. 基于Baseline局部方向建立坐标系
//
// tangent = 轨迹局部切向
//
// normal1 / normal2 = 两个与轨迹垂直的方向
//
// 障碍物只在轨迹横截面附近搜索。
// ============================================================

bool getLocalTrajectoryFrame(
    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const moveit::planning_interface::
        MoveGroupInterface::Plan &baseline,

    const std::string &tracking_link,

    double ratio,

    Eigen::Vector3d &anchor,

    Eigen::Vector3d &normal1,

    Eigen::Vector3d &normal2)
{
    const auto &points =
        baseline.trajectory_
            .joint_trajectory
            .points;


    if (points.size() < 3)
    {
        return false;
    }


    const std::size_t index =
        findCartesianRatioIndex(
            reference,
            joint_group,
            baseline,
            tracking_link,
            ratio
        );


    const std::size_t previous_index =
        index >= 2
        ?
        index - 2
        :
        0;


    const std::size_t next_index =
        std::min(
            index + 2,
            points.size() - 1
        );


    anchor =
        getLinkPosition(
            reference,
            joint_group,
            points[index].positions,
            tracking_link
        );


    const Eigen::Vector3d previous =
        getLinkPosition(
            reference,
            joint_group,
            points[
                previous_index
            ].positions,
            tracking_link
        );


    const Eigen::Vector3d next =
        getLinkPosition(
            reference,
            joint_group,
            points[
                next_index
            ].positions,
            tracking_link
        );


    Eigen::Vector3d tangent =
        next -
        previous;


    if (tangent.norm() <
        1e-6)
    {
        return false;
    }


    tangent.normalize();


    normal1 =
        tangent.cross(
            Eigen::Vector3d::UnitZ()
        );


    if (normal1.norm() <
        1e-5)
    {
        normal1 =
            tangent.cross(
                Eigen::Vector3d::UnitY()
            );
    }


    if (normal1.norm() <
        1e-5)
    {
        return false;
    }


    normal1.normalize();


    normal2 =
        tangent.cross(
            normal1
        );


    if (normal2.norm() <
        1e-5)
    {
        return false;
    }


    normal2.normalize();


    return true;
}


// ============================================================
// 13. 稳定性标定
//
// 一个障碍物候选必须：
//
// Start VALID
// Goal VALID
// Baseline BLOCKED
//
// 并且 RRTConnect 连续3次测试全部成功
//
// 才允许进入正式实验。
// ============================================================

bool testStableCandidate(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &validity_client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &target,

    const std::string &group_name,

    const PlanResult &baseline,

    int &success_count,

    double &mean_path)
{
    success_count =
        0;


    std::vector<double> paths;


    for (int trial = 1;
         trial <= CALIBRATION_TRIALS;
         ++trial)
    {
        PlanResult p =
            planWithPlanner(
                move_group,
                target,
                "RRTConnectkConfigDefault",
                "RRTConnect",
                6.0,
                10,
                EXEC_VELOCITY,
                EXEC_ACCELERATION
            );


        if (!p.success)
        {
            continue;
        }


        if (!validateTrajectory(
                validity_client,
                reference,
                joint_group,
                p.plan,
                group_name))
        {
            continue;
        }


        ++success_count;


        paths.push_back(
            p.path_length
        );
    }


    mean_path =
        mean(
            paths
        );


    // ========================================================
    // 必须3/3成功
    // ========================================================

    return
        success_count
        ==
        CALIBRATION_TRIALS;
}


// ============================================================
// 14. 在指定区域寻找稳定障碍物
//
// 比如第一组中心40%
//
// 只允许搜索：
//
// 40%
// 37%
// 43%
// 34%
// 46%
//
// 也就是说：
//
// 仍然严格属于前段区域，
// 不是满工作空间乱找。
// ============================================================

ObstacleScene findStableObstacleNearRegion(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const rclcpp::Client<
        moveit_msgs::srv::
            GetStateValidity
    >::SharedPtr &validity_client,

    const moveit::core::RobotState &reference,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &home,

    const std::vector<double> &target,

    const std::string &group_name,

    const std::string &model_frame,

    const std::string &object_id,

    const std::string &tracking_link,

    const PlanResult &baseline,

    double nominal_ratio,

    const std::string &scene_name)
{
    // ========================================================
    // Baseline附近局部位置
    // ========================================================

    const std::vector<double> ratios =
    {
        nominal_ratio,
        nominal_ratio - 0.03,
        nominal_ratio + 0.03,
        nominal_ratio - 0.06,
        nominal_ratio + 0.06
    };


    // ========================================================
    // 较大的障碍物：
    //
    // 允许中心位于轨迹中心
    // 或离轨迹 6~36mm
    //
    // 仍然属于“贴近原始路线”
    // ========================================================

    const std::vector<double> offsets =
    {
        0.000,
        0.006,
        0.012,
        0.018,
        0.024,
        0.030,
        0.036
    };


    std::vector<ObstacleScene>
        stable_candidates;


    // ========================================================
    // 遍历区域内几个Baseline位置
    // ========================================================

    for (double ratio :
         ratios)
    {
        if (ratio <= 0.05 ||
            ratio >= 0.95)
        {
            continue;
        }


        Eigen::Vector3d anchor;

        Eigen::Vector3d normal1;

        Eigen::Vector3d normal2;


        if (!getLocalTrajectoryFrame(
                reference,
                joint_group,
                baseline.plan,
                tracking_link,
                ratio,
                anchor,
                normal1,
                normal2))
        {
            continue;
        }


        struct Direction
        {
            std::string name;

            Eigen::Vector3d vector;
        };


        // ====================================================
        // 使用局部轨迹法向，而不是全局XYZ
        // ====================================================

        const std::vector<Direction>
            directions =
        {
            {
                "+N1",
                normal1
            },

            {
                "-N1",
                -normal1
            },

            {
                "+N2",
                normal2
            },

            {
                "-N2",
                -normal2
            }
        };


        std::cout
            << "\nTesting "
            << scene_name
            << " near "
            << ratio * 100.0
            << "% baseline\n";


        // ====================================================
        // 中心点先测试一次
        // ====================================================

        {
            removeObstacle(
                scene,
                object_id
            );


            geometry_msgs::msg::Pose pose;


            pose.orientation.w =
                1.0;


            pose.position.x =
                anchor.x();


            pose.position.y =
                anchor.y();


            pose.position.z =
                anchor.z();


            if (addObstacle(
                    scene,
                    model_frame,
                    object_id,
                    pose))
            {
                const bool start_valid =
                    stateValid(
                        validity_client,
                        reference,
                        joint_group,
                        home,
                        group_name
                    );


                const bool goal_valid =
                    stateValid(
                        validity_client,
                        reference,
                        joint_group,
                        target,
                        group_name
                    );


                const bool blocked =
                    baselineBlocked(
                        validity_client,
                        reference,
                        joint_group,
                        baseline.plan,
                        group_name
                    );


                if (start_valid &&
                    goal_valid &&
                    blocked)
                {
                    int successes = 0;

                    double mean_path = 0.0;


                    if (testStableCandidate(
                            move_group,
                            validity_client,
                            reference,
                            joint_group,
                            target,
                            group_name,
                            baseline,
                            successes,
                            mean_path))
                    {
                        ObstacleScene candidate;


                        candidate.valid =
                            true;


                        candidate.name =
                            scene_name;


                        candidate.nominal_ratio =
                            nominal_ratio;


                        candidate.actual_ratio =
                            ratio;


                        candidate.local_offset =
                            0.0;


                        candidate.direction =
                            "CENTER";


                        candidate.pose =
                            pose;


                        candidate.calibration_successes =
                            successes;


                        candidate.mean_calibration_path =
                            mean_path;


                        candidate.calibration_stretch =
                            baseline.path_length >
                                0.0
                            ?
                            mean_path /
                                baseline.path_length
                            :
                            0.0;


                        stable_candidates.push_back(
                            candidate
                        );


                        std::cout
                            << "STABLE candidate:"
                            << " ratio="
                            << ratio
                            << " CENTER"
                            << " success="
                            << successes
                            << "/3"
                            << " stretch="
                            << candidate.calibration_stretch
                            << "\n";
                    }
                }
            }
        }


        // ====================================================
        // 沿局部法向测试
        // ====================================================

        for (const auto &direction :
             directions)
        {
            for (double offset :
                 offsets)
            {
                if (offset <
                    1e-9)
                {
                    continue;
                }


                removeObstacle(
                    scene,
                    object_id
                );


                const Eigen::Vector3d center =
                    anchor
                    +
                    offset
                    *
                    direction.vector;


                geometry_msgs::msg::Pose pose;


                pose.orientation.w =
                    1.0;


                pose.position.x =
                    center.x();


                pose.position.y =
                    center.y();


                pose.position.z =
                    center.z();


                if (!addObstacle(
                        scene,
                        model_frame,
                        object_id,
                        pose))
                {
                    continue;
                }


                std::this_thread::sleep_for(
                    std::chrono::
                        milliseconds(
                            120
                        )
                );


                // =============================================
                // Start / Goal 必须安全
                // =============================================

                if (!stateValid(
                        validity_client,
                        reference,
                        joint_group,
                        home,
                        group_name))
                {
                    continue;
                }


                if (!stateValid(
                        validity_client,
                        reference,
                        joint_group,
                        target,
                        group_name))
                {
                    continue;
                }


                // =============================================
                // 原Baseline必须真正被挡
                // =============================================

                if (!baselineBlocked(
                        validity_client,
                        reference,
                        joint_group,
                        baseline.plan,
                        group_name))
                {
                    continue;
                }


                // =============================================
                // 连续3次规划全部成功才接受
                // =============================================

                int successes = 0;

                double mean_path = 0.0;


                if (!testStableCandidate(
                        move_group,
                        validity_client,
                        reference,
                        joint_group,
                        target,
                        group_name,
                        baseline,
                        successes,
                        mean_path))
                {
                    continue;
                }


                ObstacleScene candidate;


                candidate.valid =
                    true;


                candidate.name =
                    scene_name;


                candidate.nominal_ratio =
                    nominal_ratio;


                candidate.actual_ratio =
                    ratio;


                candidate.local_offset =
                    offset;


                candidate.direction =
                    direction.name;


                candidate.pose =
                    pose;


                candidate.calibration_successes =
                    successes;


                candidate.mean_calibration_path =
                    mean_path;


                candidate.calibration_stretch =
                    baseline.path_length >
                        0.0
                    ?
                    mean_path /
                        baseline.path_length
                    :
                    0.0;


                stable_candidates.push_back(
                    candidate
                );


                std::cout
                    << "STABLE candidate:"
                    << " ratio="
                    << ratio
                    << " direction="
                    << direction.name
                    << " offset="
                    << offset
                    << " success="
                    << successes
                    << "/3"
                    << " stretch="
                    << candidate.calibration_stretch
                    << "\n";


                // 已经有4个稳定候选，
                // 足够进行质量比较
                if (stable_candidates.size()
                    >= 4)
                {
                    break;
                }
            }


            if (stable_candidates.size()
                >= 4)
            {
                break;
            }
        }


        if (stable_candidates.size()
            >= 4)
        {
            break;
        }
    }


    removeObstacle(
        scene,
        object_id
    );


    if (stable_candidates.empty())
    {
        return
            ObstacleScene();
    }


    // ========================================================
    // 选择“合理且稳定”的候选
    //
    // 目标Stretch约1.15：
    //
    // 不希望绕一个巨大圈，
    // 又不能和Baseline完全一样。
    // ========================================================

    constexpr double TARGET_STRETCH =
        1.15;


    auto best =
        std::min_element(
            stable_candidates.begin(),
            stable_candidates.end(),

            [](
                const ObstacleScene &a,
                const ObstacleScene &b)
            {
                const double score_a =
                    std::fabs(
                        a.calibration_stretch
                        -
                        TARGET_STRETCH
                    )
                    +
                    0.20
                    *
                    a.local_offset;


                const double score_b =
                    std::fabs(
                        b.calibration_stretch
                        -
                        TARGET_STRETCH
                    )
                    +
                    0.20
                    *
                    b.local_offset;


                return
                    score_a <
                    score_b;
            }
        );


    return
        *best;
}


// ============================================================
// 15. 执行轨迹并验证目标
// ============================================================

bool executePlanAndVerify(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &target,

    const moveit::planning_interface::
        MoveGroupInterface::Plan &plan,

    double &execution_time,

    double &final_error)
{
    const auto begin =
        std::chrono::
            steady_clock::now();


    const bool raw =
        static_cast<bool>(
            move_group.execute(
                plan
            )
        );


    const auto end =
        std::chrono::
            steady_clock::now();


    execution_time =
        std::chrono::
            duration<double>(
                end - begin
            ).count();


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(
                800
            )
    );


    final_error =
        maxJointError(
            move_group,
            joint_group,
            target
        );


    return
        raw
        ||
        final_error <
            TARGET_TOLERANCE;
}


// ============================================================
// 16. 打印最终结果
// ============================================================

void printResults(
    const PlanResult &baseline,

    double ee_travel,

    const std::vector<
        ExperimentResult
    > &results)
{
    std::cout
        << "\n\n";


    std::cout
        << "====================================================================================================================================================\n";


    std::cout
        << "                                   LONG-RANGE THREE-POSITION OBSTACLE AVOIDANCE\n";


    std::cout
        << "====================================================================================================================================================\n";


    std::cout
        << "End-effector point-to-point travel = "
        << std::fixed
        << std::setprecision(
            3
        )
        << ee_travel
        << " m\n";


    std::cout
        << "Obstacle size = "
        << BOX_X * 1000.0
        << " x "
        << BOX_Y * 1000.0
        << " x "
        << BOX_Z * 1000.0
        << " mm\n";


    std::cout
        << "----------------------------------------------------------------------------------------------------------------------------------------------------\n";


    std::cout
        << std::left

        << std::setw(12)
        << "Scene"

        << std::setw(11)
        << "Region"

        << std::setw(11)
        << "ActualPos"

        << std::setw(10)
        << "Offset"

        << std::setw(10)
        << "Dir"

        << std::setw(12)
        << "Planner"

        << std::setw(12)
        << "Search"

        << std::setw(12)
        << "Path"

        << std::setw(10)
        << "Stretch"

        << std::setw(12)
        << "ExecTime"

        << std::setw(11)
        << "Error"

        << std::setw(10)
        << "Result"

        << "\n";


    std::cout
        << "----------------------------------------------------------------------------------------------------------------------------------------------------\n";


    std::cout
        << std::left

        << std::setw(12)
        << "BASELINE"

        << std::setw(11)
        << "---"

        << std::setw(11)
        << "---"

        << std::setw(10)
        << "---"

        << std::setw(10)
        << "---"

        << std::setw(12)
        << baseline.planner

        << std::fixed
        << std::setprecision(
            4
        )

        << std::setw(12)
        << baseline.planning_time

        << std::setw(12)
        << baseline.path_length

        << std::setw(10)
        << 1.0

        << std::setw(12)
        << "---"

        << std::setw(11)
        << "---"

        << std::setw(10)
        << "SUCCESS"

        << "\n";


    for (const auto &result :
         results)
    {
        const std::string nominal =
            std::to_string(
                static_cast<int>(
                    result.nominal_ratio *
                    100.0
                )
            )
            +
            "%";


        const std::string actual =
            std::to_string(
                static_cast<int>(
                    result.actual_ratio *
                    100.0
                )
            )
            +
            "%";


        std::cout
            << std::left

            << std::setw(12)
            << result.name

            << std::setw(11)
            << nominal

            << std::setw(11)
            << actual

            << std::fixed
            << std::setprecision(
                3
            )

            << std::setw(10)
            << result.local_offset

            << std::setw(10)
            << result.direction

            << std::setw(12)
            << result.planner

            << std::setprecision(
                4
            )

            << std::setw(12)
            << result.search_time

            << std::setw(12)
            << result.path_length

            << std::setw(10)
            << result.stretch

            << std::setw(12)
            << result.execution_time

            << std::setw(11)
            << result.final_error

            << std::setw(10)
            << (
                result.success
                ?
                "SUCCESS"
                :
                "FAIL"
            )

            << "\n";
    }


    std::cout
        << "====================================================================================================================================================\n";
}


// ============================================================
// 17. MAIN
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
            "obstacle_benchmark",

            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true
                )
        );


    auto logger =
        node->get_logger();


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


    // ========================================================
    // RViz trajectory
    // ========================================================

    auto display_publisher =
        node->create_publisher<
            moveit_msgs::msg::
                DisplayTrajectory
        >(
            "/display_planned_path",
            10
        );


    // ========================================================
    // State validity
    // ========================================================

    auto validity_client =
        node->create_client<
            moveit_msgs::srv::
                GetStateValidity
        >(
            "/check_state_validity"
        );


    if (!validity_client->
            wait_for_service(
                std::chrono::seconds(
                    5
                )
            ))
    {
        RCLCPP_ERROR(
            logger,
            "State validity service unavailable."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


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


    const std::string ee_link =
        move_group.getEndEffectorLink();


    // 用腕部轨迹作为障碍物放置参考
    const std::string tracking_link =
        "panda_link7";


    const std::string model_frame =
        initial_state
            ->getRobotModel()
            ->getModelFrame();


    const std::string object_id =
        "benchmark_box";


    // ========================================================
    // Home
    // ========================================================

    std::vector<double> home =
    {
         0.000,
        -0.785,
         0.000,
        -2.356,
         0.000,
         1.571,
         0.785
    };


    removeObstacle(
        scene,
        object_id
    );


    if (!returnHome(
            move_group,
            joint_group,
            home,
            logger))
    {
        RCLCPP_ERROR(
            logger,
            "Cannot establish Home."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    moveit::core::RobotState
        home_state(
            *initial_state
        );


    home_state.setJointGroupPositions(
        joint_group,
        home
    );


    home_state.update();


    // ========================================================
    // PHASE 0:
    // 长距离Target
    // ========================================================

    std::cout
        << "\n\n============================================================\n";


    std::cout
        << "PHASE 0 : LONG-DISTANCE TARGET\n";


    std::cout
        << "============================================================\n";


    std::cout
        << "Required EE point-to-point travel >= "
        << MIN_POINT_TO_POINT_TRAVEL
        << " m\n";


    std::vector<double> target;


    double target_displacement =
        0.0;


    if (!findLongDistanceTarget(
            move_group,
            validity_client,
            home_state,
            joint_group,
            home,
            group_name,
            ee_link,
            target,
            target_displacement))
    {
        RCLCPP_ERROR(
            logger,
            "Unable to find long-distance target."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    std::cout
        << "\nSelected EE point-to-point travel = "
        << target_displacement
        << " m\n";


    // ========================================================
    // PHASE 1:
    // Baseline
    // ========================================================

    std::cout
        << "\n\n============================================================\n";


    std::cout
        << "PHASE 1 : GENERATE FIXED BASELINE\n";


    std::cout
        << "============================================================\n";


    PlanResult baseline =
        generateBaseline(
            move_group,
            validity_client,
            home_state,
            joint_group,
            target,
            group_name
        );


    if (!baseline.success)
    {
        RCLCPP_ERROR(
            logger,
            "Baseline planning failed."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    std::cout
        << "Baseline planner = "
        << baseline.planner
        << "\n";


    std::cout
        << "Baseline path    = "
        << baseline.path_length
        << " rad\n";


    // ========================================================
    // PHASE 2:
    // 先校准全部三个场景
    //
    // 只有三个场景全部稳定找到，
    // 才开始正式执行。
    // ========================================================

    const std::vector<double>
        nominal_ratios =
    {
        0.40,
        0.60,
        0.80
    };


    const std::vector<std::string>
        scene_names =
    {
        "OBSTACLE-1",
        "OBSTACLE-2",
        "OBSTACLE-3"
    };


    std::vector<ObstacleScene>
        calibrated_scenes;


    std::cout
        << "\n\n============================================================\n";


    std::cout
        << "PHASE 2 : CALIBRATE ALL THREE OBSTACLE SCENES\n";


    std::cout
        << "============================================================\n";


    std::cout
        << "Obstacle size = "
        << BOX_X * 1000.0
        << " x "
        << BOX_Y * 1000.0
        << " x "
        << BOX_Z * 1000.0
        << " mm\n";


    for (std::size_t i = 0;
         i < nominal_ratios.size();
         ++i)
    {
        std::cout
            << "\n----------------------------------------\n";


        std::cout
            << "Calibrating "
            << scene_names[i]
            << " around "
            << nominal_ratios[i] * 100.0
            << "% baseline\n";


        std::cout
            << "----------------------------------------\n";


        ObstacleScene selected =
            findStableObstacleNearRegion(
                move_group,
                scene,
                validity_client,
                home_state,
                joint_group,
                home,
                target,
                group_name,
                model_frame,
                object_id,
                tracking_link,
                baseline,
                nominal_ratios[i],
                scene_names[i]
            );


        if (!selected.valid)
        {
            RCLCPP_ERROR(
                logger,
                "No stable 3/3 obstacle configuration found for %s.",
                scene_names[i].c_str()
            );


            RCLCPP_ERROR(
                logger,
                "Formal experiment will NOT start with an incomplete scene set."
            );


            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }


        calibrated_scenes.push_back(
            selected
        );


        std::cout
            << "\nSELECTED "
            << selected.name
            << "\n";


        std::cout
            << "Nominal region       : "
            << selected.nominal_ratio
                * 100.0
            << "%\n";


        std::cout
            << "Actual baseline pos  : "
            << selected.actual_ratio
                * 100.0
            << "%\n";


        std::cout
            << "Local direction      : "
            << selected.direction
            << "\n";


        std::cout
            << "Local offset         : "
            << selected.local_offset
            << " m\n";


        std::cout
            << "Calibration success  : "
            << selected.calibration_successes
            << "/"
            << CALIBRATION_TRIALS
            << "\n";


        std::cout
            << "Mean calibration path: "
            << selected.mean_calibration_path
            << " rad\n";


        std::cout
            << "Calibration stretch  : "
            << selected.calibration_stretch
            << "\n";
    }


    removeObstacle(
        scene,
        object_id
    );


    std::cout
        << "\n========================================\n";


    std::cout
        << "ALL 3 SCENES CALIBRATED SUCCESSFULLY\n";


    std::cout
        << "========================================\n";


    // ========================================================
    // PHASE 3:
    // 执行无障碍Baseline
    // ========================================================

    if (!returnHome(
            move_group,
            joint_group,
            home,
            logger))
    {
        RCLCPP_ERROR(
            logger,
            "Cannot return Home before baseline."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    publishTrajectory(
        display_publisher,
        baseline.plan
    );


    std::cout
        << "\nOriginal Baseline ready."
        << "\nRobot starts in 4 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            4
        )
    );


    double baseline_exec_time =
        0.0;


    double baseline_error =
        0.0;


    if (!executePlanAndVerify(
            move_group,
            joint_group,
            target,
            baseline.plan,
            baseline_exec_time,
            baseline_error))
    {
        RCLCPP_ERROR(
            logger,
            "Baseline execution failed."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    std::cout
        << "\nBASELINE SUCCESS\n";


    std::cout
        << "Final error = "
        << baseline_error
        << " rad\n";


    std::cout
        << "Hold for 3 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            3
        )
    );


    // ========================================================
    // CSV
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
        "three_stable_obstacles.csv";


    std::ofstream csv(
        csv_path
    );


    csv
        << "scene,"
        << "nominal_ratio,"
        << "actual_ratio,"
        << "offset_m,"
        << "direction,"
        << "obstacle_x,"
        << "obstacle_y,"
        << "obstacle_z,"
        << "planner,"
        << "search_time_s,"
        << "path_length_rad,"
        << "stretch,"
        << "execution_time_s,"
        << "joint_error_rad,"
        << "success\n";


    std::vector<ExperimentResult>
        results;


    // ========================================================
    // PHASE 4:
    // 三组正式实验
    // ========================================================

    for (std::size_t scene_index = 0;
         scene_index <
            calibrated_scenes.size();
         ++scene_index)
    {
        const ObstacleScene &obstacle =
            calibrated_scenes[
                scene_index
            ];


        std::cout
            << "\n\n============================================================\n";


        std::cout
            << "EXPERIMENT "
            << scene_index + 1
            << "/3 : "
            << obstacle.name
            << "\n";


        std::cout
            << "============================================================\n";


        removeObstacle(
            scene,
            object_id
        );


        if (!returnHome(
                move_group,
                joint_group,
                home,
                logger))
        {
            RCLCPP_ERROR(
                logger,
                "Cannot return Home."
            );


            break;
        }


        if (!addObstacle(
                scene,
                model_frame,
                object_id,
                obstacle.pose))
        {
            RCLCPP_ERROR(
                logger,
                "Cannot add calibrated obstacle."
            );


            continue;
        }


        // ====================================================
        // 正式实验前再次验证
        // ========================================================

        const bool start_valid =
            stateValid(
                validity_client,
                home_state,
                joint_group,
                home,
                group_name
            );


        const bool goal_valid =
            stateValid(
                validity_client,
                home_state,
                joint_group,
                target,
                group_name
            );


        const bool original_blocked =
            baselineBlocked(
                validity_client,
                home_state,
                joint_group,
                baseline.plan,
                group_name
            );


        std::cout
            << "\nScene validation:\n";


        std::cout
            << "Start             : "
            << (
                start_valid
                ?
                "VALID"
                :
                "INVALID"
            )
            << "\n";


        std::cout
            << "Goal              : "
            << (
                goal_valid
                ?
                "VALID"
                :
                "INVALID"
            )
            << "\n";


        std::cout
            << "Original Baseline : "
            << (
                original_blocked
                ?
                "BLOCKED"
                :
                "CLEAR"
            )
            << "\n";


        std::cout
            << "Obstacle size     : "
            << BOX_X * 1000.0
            << " x "
            << BOX_Y * 1000.0
            << " x "
            << BOX_Z * 1000.0
            << " mm\n";


        std::cout
            << "Nominal region    : "
            << obstacle.nominal_ratio
                * 100.0
            << "%\n";


        std::cout
            << "Actual position   : "
            << obstacle.actual_ratio
                * 100.0
            << "%\n";


        std::cout
            << "Local offset      : "
            << obstacle.local_offset
            << " m\n";


        if (!start_valid ||
            !goal_valid ||
            !original_blocked)
        {
            RCLCPP_ERROR(
                logger,
                "Scene changed and is no longer valid."
            );


            continue;
        }


        std::cout
            << "\nObserve the obstacle for 3 seconds...\n";


        std::this_thread::sleep_for(
            std::chrono::seconds(
                3
            )
        );


        ExperimentResult result;


        result.name =
            obstacle.name;


        result.nominal_ratio =
            obstacle.nominal_ratio;


        result.actual_ratio =
            obstacle.actual_ratio;


        result.local_offset =
            obstacle.local_offset;


        result.direction =
            obstacle.direction;


        result.obstacle_x =
            obstacle.pose.position.x;


        result.obstacle_y =
            obstacle.pose.position.y;


        result.obstacle_z =
            obstacle.pose.position.z;


        // ====================================================
        // 最多3次完整执行尝试
        // ========================================================

        for (int execution_attempt = 1;
             execution_attempt <= 3;
             ++execution_attempt)
        {
            if (execution_attempt >
                1)
            {
                removeObstacle(
                    scene,
                    object_id
                );


                if (!returnHome(
                        move_group,
                        joint_group,
                        home,
                        logger))
                {
                    break;
                }


                if (!addObstacle(
                        scene,
                        model_frame,
                        object_id,
                        obstacle.pose))
                {
                    continue;
                }


                std::this_thread::sleep_for(
                    std::chrono::
                        milliseconds(
                            500
                        )
                );
            }


            std::cout
                << "\nOptimized planning attempt "
                << execution_attempt
                << "/3\n";


            // =================================================
            // 多规划器优化
            // =================================================

            PlanResult optimized =
                optimizedDetourPlanning(
                    move_group,
                    validity_client,
                    home_state,
                    joint_group,
                    target,
                    group_name
                );


            if (!optimized.success)
            {
                std::cout
                    << "No valid candidate."
                    << " Retrying...\n";


                continue;
            }


            // 最终验证
            if (!validateTrajectory(
                    validity_client,
                    home_state,
                    joint_group,
                    optimized.plan,
                    group_name))
            {
                continue;
            }


            result.planner =
                optimized.planner;


            result.search_time =
                optimized.planning_time;


            result.path_length =
                optimized.path_length;


            result.stretch =
                baseline.path_length >
                    0.0
                ?
                optimized.path_length /
                    baseline.path_length
                :
                0.0;


            publishTrajectory(
                display_publisher,
                optimized.plan
            );


            std::cout
                << "\n========================================\n";


            std::cout
                << "OPTIMIZED COLLISION-FREE PATH READY\n";


            std::cout
                << "========================================\n";


            std::cout
                << "Planner       : "
                << optimized.planner
                << "\n";


            std::cout
                << "Search time   : "
                << optimized.planning_time
                << " s\n";


            std::cout
                << "Baseline path : "
                << baseline.path_length
                << " rad\n";


            std::cout
                << "Detour path   : "
                << optimized.path_length
                << " rad\n";


            std::cout
                << "Stretch       : "
                << result.stretch
                << "\n";


            std::cout
                << "\nRobot starts at only "
                << EXEC_VELOCITY * 100.0
                << "% velocity in 4 seconds.\n";


            std::cout
                << "Watch the actual avoidance motion carefully.\n";


            std::this_thread::sleep_for(
                std::chrono::seconds(
                    4
                )
            );


            result.success =
                executePlanAndVerify(
                    move_group,
                    joint_group,
                    target,
                    optimized.plan,
                    result.execution_time,
                    result.final_error
                );


            if (result.success)
            {
                std::cout
                    << "\n";


                std::cout
                    << "############################################\n";


                std::cout
                    << "#       OBSTACLE AVOIDANCE SUCCESS        #\n";


                std::cout
                    << "############################################\n";


                std::cout
                    << "Same original Target reached.\n";


                std::cout
                    << "Final joint error = "
                    << result.final_error
                    << " rad\n";


                std::cout
                    << "Holding Target for 3 seconds...\n";


                std::this_thread::sleep_for(
                    std::chrono::seconds(
                        3
                    )
                );


                break;
            }


            std::cout
                << "\nExecution failed."
                << " Returning Home and retrying...\n";
        }


        results.push_back(
            result
        );


        csv
            << result.name
            << ","
            << result.nominal_ratio
            << ","
            << result.actual_ratio
            << ","
            << result.local_offset
            << ","
            << result.direction
            << ","
            << result.obstacle_x
            << ","
            << result.obstacle_y
            << ","
            << result.obstacle_z
            << ","
            << result.planner
            << ","
            << result.search_time
            << ","
            << result.path_length
            << ","
            << result.stretch
            << ","
            << result.execution_time
            << ","
            << result.final_error
            << ","
            << (
                result.success
                ?
                1
                :
                0
            )
            << "\n";


        csv.flush();


        removeObstacle(
            scene,
            object_id
        );


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    500
                )
        );


        if (!returnHome(
                move_group,
                joint_group,
                home,
                logger))
        {
            RCLCPP_ERROR(
                logger,
                "Home recovery failed."
            );


            break;
        }


        std::cout
            << "\n"
            << obstacle.name
            << " completed."
            << "\nNext scene starts in 3 seconds...\n";


        std::this_thread::sleep_for(
            std::chrono::seconds(
                3
            )
        );
    }


    // ========================================================
    // FINAL
    // ========================================================

    removeObstacle(
        scene,
        object_id
    );


    csv.close();


    printResults(
        baseline,
        target_displacement,
        results
    );


    int successful_scenes =
        0;


    for (const auto &result :
         results)
    {
        if (result.success)
        {
            ++successful_scenes;
        }
    }


    std::cout
        << "\nFINAL SUMMARY\n";


    std::cout
        << "--------------------------------------------------\n";


    std::cout
        << "EE point-to-point travel : "
        << target_displacement
        << " m\n";


    std::cout
        << "Obstacle size            : "
        << BOX_X * 1000.0
        << " x "
        << BOX_Y * 1000.0
        << " x "
        << BOX_Z * 1000.0
        << " mm\n";


    std::cout
        << "Target regions           : "
        << "40%, 60%, 80%\n";


    std::cout
        << "Stable calibration       : "
        << calibrated_scenes.size()
        << " / 3\n";


    std::cout
        << "Reached same Target      : "
        << successful_scenes
        << " / "
        << results.size()
        << "\n";


    std::cout
        << "CSV                      : "
        << csv_path
        << "\n";


    std::cout
        << "--------------------------------------------------\n";


    executor.cancel();


    if (spin_thread.joinable())
    {
        spin_thread.join();
    }


    rclcpp::shutdown();


    return 0;
}
