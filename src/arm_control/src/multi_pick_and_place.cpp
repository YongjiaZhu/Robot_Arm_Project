#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>


// ================================================================
//                         全局参数
// ================================================================

constexpr double ARM_VELOCITY = 0.10;
constexpr double ARM_ACCELERATION = 0.05;

constexpr double CART_VELOCITY = 0.07;
constexpr double CART_ACCELERATION = 0.03;

constexpr double EEF_STEP = 0.005;
constexpr double MIN_CART_FRACTION = 0.98;

// 抓取前下降
constexpr double APPROACH_DISTANCE = 0.11;

// 抓取后抬升
constexpr double LIFT_DISTANCE = 0.15;

// 放置后撤离
constexpr double RETREAT_DISTANCE = 0.12;

constexpr double POSITION_TOLERANCE = 0.020;
constexpr double HOME_TOLERANCE = 0.050;

constexpr double GRIPPER_OPEN = 0.040;
constexpr double GRIPPER_FORCE = 20.0;


// ================================================================
// TABLE
// ================================================================

constexpr double TABLE_SIZE_X = 0.78;
constexpr double TABLE_SIZE_Y = 0.78;
constexpr double TABLE_SIZE_Z = 0.10;

constexpr double TABLE_CENTER_Z = 0.15;

constexpr double TABLE_TOP_Z =
    TABLE_CENTER_Z + TABLE_SIZE_Z / 2.0;


// 物体与桌面保持5mm余量
constexpr double OBJECT_GAP = 0.005;


// panda_link8 与物体中心之间抓取高度偏置
constexpr double GRASP_EE_Z_OFFSET = 0.130;


// ================================================================
// 数据结构
// ================================================================

struct ObjectTask
{
    std::string id;
    std::string name;
    std::string color_name;

    double sx;
    double sy;
    double sz;

    std_msgs::msg::ColorRGBA color;

    double pick_x;
    double pick_y;

    double place_x;
    double place_y;

    double gripper_close;

    bool use_obstacle;

    std::string obstacle_id;
    std::string obstacle_color_name;

    std_msgs::msg::ColorRGBA obstacle_color;

    double obstacle_base_x;
    double obstacle_base_y;
};


struct TaskMetrics
{
    std::string name;
    std::string color_name;

    bool pregrasp_success = false;
    bool approach_success = false;
    bool gripper_success = false;
    bool attach_success = false;
    bool lift_success = false;

    bool obstacle_required = false;
    bool obstacle_calibrated = false;

    double direct_fraction = 100.0;

    std::string transfer_planner = "-";
    bool transfer_success = false;

    bool place_success = false;
    bool detach_success = false;
    bool retreat_success = false;

    double pregrasp_error = 999.0;
    double approach_fraction = 0.0;
    double lift_fraction = 0.0;
    double place_fraction = 0.0;

    double transfer_error = 999.0;

    double placement_error_m = 999.0;

    double total_time = 0.0;

    bool success = false;
};


struct ObstacleResult
{
    bool success = false;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;

    double direct_fraction = 1.0;

    std::string feasible_planner = "-";
};


// ================================================================
// Color
// ================================================================

std_msgs::msg::ColorRGBA makeColor(
    double r,
    double g,
    double b,
    double a = 1.0)
{
    std_msgs::msg::ColorRGBA c;

    c.r = static_cast<float>(r);
    c.g = static_cast<float>(g);
    c.b = static_cast<float>(b);
    c.a = static_cast<float>(a);

    return c;
}


// ================================================================
// Pose工具
// ================================================================

double positionDistance(
    const geometry_msgs::msg::Pose &a,
    const geometry_msgs::msg::Pose &b)
{
    const double dx =
        a.position.x - b.position.x;

    const double dy =
        a.position.y - b.position.y;

    const double dz =
        a.position.z - b.position.z;

    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );
}


double objectCenterZ(
    const ObjectTask &task)
{
    return
        TABLE_TOP_Z
        +
        task.sz / 2.0
        +
        OBJECT_GAP;
}


double graspZ(
    const ObjectTask &task)
{
    return
        objectCenterZ(task)
        +
        GRASP_EE_Z_OFFSET;
}


// ================================================================
// 当前关节误差
// ================================================================

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
         i < current.size();
         ++i)
    {
        error =
            std::max(
                error,
                std::fabs(
                    current[i] -
                    target[i]
                )
            );
    }

    return error;
}


// ================================================================
// Collision Object
// ================================================================

moveit_msgs::msg::CollisionObject createBox(
    const std::string &frame,
    const std::string &id,
    double x,
    double y,
    double z,
    double sx,
    double sy,
    double sz)
{
    moveit_msgs::msg::CollisionObject object;

    object.header.frame_id = frame;
    object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        shape_msgs::msg::SolidPrimitive::BOX;

    primitive.dimensions.resize(3);

    primitive.dimensions[0] = sx;
    primitive.dimensions[1] = sy;
    primitive.dimensions[2] = sz;

    geometry_msgs::msg::Pose pose;

    pose.orientation.w = 1.0;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    object.primitives.push_back(
        primitive
    );

    object.primitive_poses.push_back(
        pose
    );

    object.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    return object;
}


bool addColoredBox(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &frame,
    const std::string &id,
    double x,
    double y,
    double z,
    double sx,
    double sy,
    double sz,
    const std_msgs::msg::ColorRGBA &color)
{
    auto object =
        createBox(
            frame,
            id,
            x,
            y,
            z,
            sx,
            sy,
            sz
        );

    return
        scene.applyCollisionObject(
            object,
            color
        );
}


bool worldObjectExists(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &id)
{
    const auto names =
        scene.getKnownObjectNames();

    return
        std::find(
            names.begin(),
            names.end(),
            id
        )
        !=
        names.end();
}


bool attachedObjectExists(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &id)
{
    const auto objects =
        scene.getAttachedObjects(
            {id}
        );

    return !objects.empty();
}


bool waitWorld(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &id,
    bool expected,
    double timeout = 3.0)
{
    const auto begin =
        std::chrono::steady_clock::now();

    while (true)
    {
        if (worldObjectExists(scene, id) ==
            expected)
        {
            return true;
        }

        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                begin
            ).count();

        if (elapsed > timeout)
        {
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }
}


bool waitAttached(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &id,
    bool expected,
    double timeout = 3.0)
{
    const auto begin =
        std::chrono::steady_clock::now();

    while (true)
    {
        if (attachedObjectExists(scene, id) ==
            expected)
        {
            return true;
        }

        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                begin
            ).count();

        if (elapsed > timeout)
        {
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }
}


void removeObject(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &id)
{
    scene.removeCollisionObjects(
        {id}
    );

    waitWorld(
        scene,
        id,
        false,
        2.0
    );
}


// ================================================================
// Gripper
// ================================================================

bool commandGripper(
    const rclcpp::Node::SharedPtr &node,
    double position)
{
    using GripperCommand =
        control_msgs::action::GripperCommand;

    auto client =
        rclcpp_action::create_client<
            GripperCommand
        >(
            node,
            "/panda_hand_controller/gripper_cmd"
        );

    if (!client->wait_for_action_server(
            std::chrono::seconds(4)))
    {
        return false;
    }

    GripperCommand::Goal goal;

    goal.command.position = position;
    goal.command.max_effort = GRIPPER_FORCE;

    auto send_future =
        client->async_send_goal(goal);

    if (send_future.wait_for(
            std::chrono::seconds(4))
        !=
        std::future_status::ready)
    {
        return false;
    }

    auto handle =
        send_future.get();

    if (!handle)
    {
        return false;
    }

    auto result_future =
        client->async_get_result(
            handle
        );

    if (result_future.wait_for(
            std::chrono::seconds(8))
        !=
        std::future_status::ready)
    {
        return false;
    }

    const auto result =
        result_future.get();

    return
        result.code ==
        rclcpp_action::ResultCode::SUCCEEDED;
}


// ================================================================
// 时间参数化
// ================================================================

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


// ================================================================
// RViz轨迹
// ================================================================

void publishTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::core::RobotStatePtr &state,

    const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    moveit_msgs::msg::DisplayTrajectory display;

    display.model_id = "panda";

    moveit::core::robotStateToRobotStateMsg(
        *state,
        display.trajectory_start
    );

    display.trajectory.push_back(
        trajectory
    );

    publisher->publish(
        display
    );
}


// ================================================================
// Home
// ================================================================

bool moveHome(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const moveit::core::JointModelGroup *joint_group,
    const std::vector<double> &home)
{
    if (maxJointError(
            move_group,
            joint_group,
            home)
        <
        HOME_TOLERANCE)
    {
        return true;
    }

    for (int attempt = 0;
         attempt < 4;
         ++attempt)
    {
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
            6.0
        );

        move_group.setNumPlanningAttempts(
            10
        );

        move_group.setMaxVelocityScalingFactor(
            ARM_VELOCITY
        );

        move_group.setMaxAccelerationScalingFactor(
            ARM_ACCELERATION
        );

        move_group.setStartStateToCurrentState();

        if (!move_group.setJointValueTarget(
                home))
        {
            continue;
        }

        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        if (!static_cast<bool>(
                move_group.plan(
                    plan
                )))
        {
            continue;
        }

        const bool raw =
            static_cast<bool>(
                move_group.execute(
                    plan
                )
            );

        std::this_thread::sleep_for(
            std::chrono::milliseconds(500)
        );

        const double error =
            maxJointError(
                move_group,
                joint_group,
                home
            );

        if (raw ||
            error <
            HOME_TOLERANCE)
        {
            return true;
        }
    }

    return false;
}


// ================================================================
// 普通Pose规划
// ================================================================

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target,
    const std::string &ee_link,
    const std::string &planner,
    double planning_time,
    double &error)
{
    for (int attempt = 0;
         attempt < 3;
         ++attempt)
    {
        move_group.stop();

        move_group.clearPoseTargets();
        move_group.clearPathConstraints();

        move_group.setPlanningPipelineId(
            "ompl"
        );

        move_group.setPlannerId(
            planner
        );

        move_group.setPlanningTime(
            planning_time
        );

        move_group.setNumPlanningAttempts(
            8
        );

        move_group.setMaxVelocityScalingFactor(
            ARM_VELOCITY
        );

        move_group.setMaxAccelerationScalingFactor(
            ARM_ACCELERATION
        );

        move_group.setStartStateToCurrentState();

        if (!move_group.setPoseTarget(
                target,
                ee_link))
        {
            continue;
        }

        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        const bool planned =
            static_cast<bool>(
                move_group.plan(
                    plan
                )
            );

        if (!planned)
        {
            move_group.clearPoseTargets();
            continue;
        }

        const bool raw =
            static_cast<bool>(
                move_group.execute(
                    plan
                )
            );

        move_group.clearPoseTargets();

        std::this_thread::sleep_for(
            std::chrono::milliseconds(600)
        );

        const auto actual =
            move_group.getCurrentPose(
                ee_link
            ).pose;

        error =
            positionDistance(
                actual,
                target
            );

        if ((raw ||
             error <
             POSITION_TOLERANCE)
            &&
            error <
            POSITION_TOLERANCE)
        {
            return true;
        }
    }

    return false;
}


// ================================================================
// 只测试规划可行性，不执行
// ================================================================

bool planExists(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target,
    const std::string &ee_link,
    const std::string &planner,
    double planning_time)
{
    move_group.stop();

    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

    move_group.setPlanningPipelineId(
        "ompl"
    );

    move_group.setPlannerId(
        planner
    );

    move_group.setPlanningTime(
        planning_time
    );

    move_group.setNumPlanningAttempts(
        6
    );

    move_group.setStartStateToCurrentState();

    if (!move_group.setPoseTarget(
            target,
            ee_link))
    {
        return false;
    }

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;

    const bool ok =
        static_cast<bool>(
            move_group.plan(
                plan
            )
        );

    move_group.clearPoseTargets();

    return ok;
}


// ================================================================
// Cartesian相对运动
// ================================================================

bool cartesianMove(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::string &group_name,
    const std::string &ee_link,

    const rclcpp::Publisher<
        moveit_msgs::msg::DisplayTrajectory
    >::SharedPtr &display_publisher,

    double dx,
    double dy,
    double dz,

    double &fraction,
    double &position_error)
{
    const auto start =
        move_group.getCurrentPose(
            ee_link
        ).pose;

    auto target =
        start;

    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;

    std::vector<
        geometry_msgs::msg::Pose
    > waypoints;

    waypoints.push_back(
        target
    );

    moveit_msgs::msg::
        RobotTrajectory trajectory;

    move_group.setStartStateToCurrentState();

    fraction =
        move_group.computeCartesianPath(
            waypoints,
            EEF_STEP,
            0.0,
            trajectory,
            true
        );

    if (fraction <
        MIN_CART_FRACTION)
    {
        return false;
    }

    auto state =
        move_group.getCurrentState(
            5.0
        );

    if (!state)
    {
        return false;
    }

    if (!retimeTrajectory(
            state,
            group_name,
            trajectory))
    {
        return false;
    }

    publishTrajectory(
        display_publisher,
        state,
        trajectory
    );

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;

    plan.trajectory_ =
        trajectory;

    const bool raw =
        static_cast<bool>(
            move_group.execute(
                plan
            )
        );

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500)
    );

    const auto actual =
        move_group.getCurrentPose(
            ee_link
        ).pose;

    position_error =
        positionDistance(
            actual,
            target
        );

    return
        fraction >=
            MIN_CART_FRACTION
        &&
        position_error <
            POSITION_TOLERANCE
        &&
        (
            raw ||
            position_error <
            POSITION_TOLERANCE
        );
}


// ================================================================
// 计算当前Pose到目标Pose的直接Cartesian fraction
//
// 不执行
//
// 用于证明：
// “原来的直接搬运路线被障碍物阻断”
// ================================================================

double directCartesianFraction(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target)
{
    std::vector<
        geometry_msgs::msg::Pose
    > waypoints;

    waypoints.push_back(
        target
    );

    moveit_msgs::msg::
        RobotTrajectory trajectory;

    move_group.setStartStateToCurrentState();

    return
        move_group.computeCartesianPath(
            waypoints,
            EEF_STEP,
            0.0,
            trajectory,
            true
        );
}


// ================================================================
// 障碍物自动校准
//
// 条件：
//
// 1. 直接Cartesian路线 fraction < 95%
//    证明直线路径被挡住
//
// 2. RRTConnect或RRT*仍然能找到路径
//
// 只有同时满足才采用这个障碍物。
// ================================================================

ObstacleResult calibrateObstacle(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::PlanningSceneInterface &scene,

    const std::string &planning_frame,
    const std::string &ee_link,

    const ObjectTask &task,

    const geometry_msgs::msg::Pose &target_pose)
{
    ObstacleResult result;

    const auto start_pose =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    // 障碍物沿直接搬运路线的不同百分比尝试
    const std::vector<double> ratios =
    {
        0.50,
        0.58,
        0.42
    };


    // 从较小墙体开始
    const std::vector<double> scale =
    {
        1.00,
        1.15,
        1.30
    };


    for (double s : scale)
    {
        for (double ratio : ratios)
        {
            removeObject(
                scene,
                task.obstacle_id
            );


            const double ox =
                start_pose.position.x
                +
                ratio
                *
                (
                    target_pose.position.x
                    -
                    start_pose.position.x
                );


            const double oy =
                start_pose.position.y
                +
                ratio
                *
                (
                    target_pose.position.y
                    -
                    start_pose.position.y
                );


            const double sx =
                task.obstacle_base_x * s;


            const double sy =
                task.obstacle_base_y * s;


            // 障碍墙顶端略高于当前搬运高度，
            // 确保直接路线受到阻挡。
            const double desired_top =
                start_pose.position.z
                +
                0.035;


            const double sz =
                std::max(
                    0.26,
                    desired_top -
                    TABLE_TOP_Z
                );


            const double oz =
                TABLE_TOP_Z
                +
                sz / 2.0;


            if (!addColoredBox(
                    scene,
                    planning_frame,
                    task.obstacle_id,

                    ox,
                    oy,
                    oz,

                    sx,
                    sy,
                    sz,

                    task.obstacle_color))
            {
                continue;
            }


            std::this_thread::sleep_for(
                std::chrono::milliseconds(400)
            );


            const double fraction =
                directCartesianFraction(
                    move_group,
                    target_pose
                );


            std::cout
                << "  Obstacle test: "
                << "size="
                << sx * 1000.0
                << "x"
                << sy * 1000.0
                << "x"
                << sz * 1000.0
                << " mm"
                << "  direct_fraction="
                << fraction * 100.0
                << "%"
                << "\n";


            // 直接路线必须真的被阻断
            if (fraction < 0.0 ||
                fraction >= 0.95)
            {
                continue;
            }


            // 再确认绕开障碍物仍然可行
            const bool rrt_connect_ok =
                planExists(
                    move_group,
                    target_pose,
                    ee_link,
                    "RRTConnectkConfigDefault",
                    5.0
                );


            bool rrt_star_ok =
                false;


            if (!rrt_connect_ok)
            {
                rrt_star_ok =
                    planExists(
                        move_group,
                        target_pose,
                        ee_link,
                        "RRTstarkConfigDefault",
                        7.0
                    );
            }


            if (rrt_connect_ok ||
                rrt_star_ok)
            {
                result.success = true;

                result.x = ox;
                result.y = oy;
                result.z = oz;

                result.sx = sx;
                result.sy = sy;
                result.sz = sz;

                result.direct_fraction =
                    fraction;

                result.feasible_planner =
                    rrt_connect_ok
                    ?
                    "RRTConnect"
                    :
                    "RRT*";

                return result;
            }
        }
    }


    removeObject(
        scene,
        task.obstacle_id
    );


    return result;
}


// ================================================================
// RRTConnect优先，RRT*作为备用
// ================================================================

bool transferWithPlannerFallback(
    moveit::planning_interface::MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,
    const std::string &ee_link,

    std::string &planner_used,
    double &error)
{
    if (moveToPose(
            move_group,
            target,
            ee_link,
            "RRTConnectkConfigDefault",
            7.0,
            error))
    {
        planner_used =
            "RRTConnect";

        return true;
    }


    std::cout
        << "RRTConnect failed. Trying RRT*...\n";


    if (moveToPose(
            move_group,
            target,
            ee_link,
            "RRTstarkConfigDefault",
            10.0,
            error))
    {
        planner_used =
            "RRT*";

        return true;
    }


    planner_used =
        "FAILED";

    return false;
}


// ================================================================
// 参数表
// ================================================================

void printTaskParameterTable(
    const std::vector<ObjectTask> &tasks)
{
    std::cout
        << "\n\n";


    std::cout
        << "========================================================================================================================\n";


    std::cout
        << "                                      MULTI-TASK OBJECT CONFIGURATION\n";


    std::cout
        << "========================================================================================================================\n";


    std::cout
        << std::left
        << std::setw(12)
        << "Task"

        << std::setw(12)
        << "Color"

        << std::setw(24)
        << "Size(mm)"

        << std::setw(24)
        << "Pick XY(m)"

        << std::setw(24)
        << "Place XY(m)"

        << "Obstacle\n";


    std::cout
        << "------------------------------------------------------------------------------------------------------------------------\n";


    for (std::size_t i = 0;
         i < tasks.size();
         ++i)
    {
        const auto &t =
            tasks[i];


        const std::string size =
            std::to_string(
                static_cast<int>(
                    std::round(
                        t.sx * 1000.0
                    )
                )
            )
            +
            "x"
            +
            std::to_string(
                static_cast<int>(
                    std::round(
                        t.sy * 1000.0
                    )
                )
            )
            +
            "x"
            +
            std::to_string(
                static_cast<int>(
                    std::round(
                        t.sz * 1000.0
                    )
                )
            );


        std::ostringstream pick;

        pick
            << std::fixed
            << std::setprecision(3)
            << "("
            << t.pick_x
            << ", "
            << t.pick_y
            << ")";


        std::ostringstream place;

        place
            << std::fixed
            << std::setprecision(3)
            << "("
            << t.place_x
            << ", "
            << t.place_y
            << ")";


        std::cout
            << std::setw(12)
            << (
                "Task "
                +
                std::to_string(
                    i + 1
                )
            )

            << std::setw(12)
            << t.color_name

            << std::setw(24)
            << size

            << std::setw(24)
            << pick.str()

            << std::setw(24)
            << place.str()

            << (
                t.use_obstacle
                ?
                t.obstacle_color_name
                :
                "NONE"
            )

            << "\n";
    }


    std::cout
        << "========================================================================================================================\n";


    std::cout
        << "RRT velocity           : "
        << ARM_VELOCITY * 100.0
        << "%\n";


    std::cout
        << "Cartesian velocity     : "
        << CART_VELOCITY * 100.0
        << "%\n";


    std::cout
        << "Cartesian EEF step     : "
        << EEF_STEP
        << " m\n";


    std::cout
        << "Minimum fraction       : "
        << MIN_CART_FRACTION * 100.0
        << "%\n";


    std::cout
        << "Approach distance      : "
        << APPROACH_DISTANCE
        << " m\n";


    std::cout
        << "Lift distance          : "
        << LIFT_DISTANCE
        << " m\n";


    std::cout
        << "========================================================================================================================\n";
}


// ================================================================
// 最终结果表
// ================================================================

void printResultTable(
    const std::vector<TaskMetrics> &metrics)
{
    std::cout
        << "\n\n";


    std::cout
        << "==========================================================================================================================================\n";


    std::cout
        << "                                                   MULTI-TASK RESULTS\n";


    std::cout
        << "==========================================================================================================================================\n";


    std::cout
        << std::left

        << std::setw(10)
        << "Task"

        << std::setw(10)
        << "Color"

        << std::setw(14)
        << "Approach"

        << std::setw(14)
        << "Lift"

        << std::setw(18)
        << "DirectPath"

        << std::setw(16)
        << "Planner"

        << std::setw(14)
        << "Place"

        << std::setw(18)
        << "PlaceErr(mm)"

        << "Result\n";


    std::cout
        << "------------------------------------------------------------------------------------------------------------------------------------------\n";


    for (const auto &m :
         metrics)
    {
        std::ostringstream approach;
        approach
            << std::fixed
            << std::setprecision(1)
            << m.approach_fraction
            << "%";


        std::ostringstream lift;
        lift
            << std::fixed
            << std::setprecision(1)
            << m.lift_fraction
            << "%";


        std::ostringstream direct;

        if (m.obstacle_required)
        {
            direct
                << std::fixed
                << std::setprecision(1)
                << m.direct_fraction
                << "% BLOCKED";
        }
        else
        {
            direct
                << "No obstacle";
        }


        std::ostringstream place;

        place
            << std::fixed
            << std::setprecision(1)
            << m.place_fraction
            << "%";


        std::ostringstream error;

        error
            << std::fixed
            << std::setprecision(3)
            << m.placement_error_m
                   * 1000.0;


        std::cout
            << std::setw(10)
            << m.name

            << std::setw(10)
            << m.color_name

            << std::setw(14)
            << approach.str()

            << std::setw(14)
            << lift.str()

            << std::setw(18)
            << direct.str()

            << std::setw(16)
            << m.transfer_planner

            << std::setw(14)
            << place.str()

            << std::setw(18)
            << error.str()

            << (
                m.success
                ?
                "SUCCESS"
                :
                "FAIL"
            )

            << "\n";
    }


    std::cout
        << "==========================================================================================================================================\n";
}


// ================================================================
// MAIN
// ================================================================

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
            "multi_pick_and_place",

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


    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );


    std::string ee_link =
        move_group.getEndEffectorLink();


    if (ee_link.empty())
    {
        ee_link =
            "panda_link8";
    }


    const std::string planning_frame =
        move_group.getPlanningFrame();


    auto display_publisher =
        node->create_publisher<
            moveit_msgs::msg::DisplayTrajectory
        >(
            "/display_planned_path",
            10
        );


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


    // ============================================================
    // Safe Home
    // ============================================================

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
        << "================================================================\n";


    std::cout
        << "          PANDA MULTI-OBJECT PICK & PLACE SYSTEM\n";


    std::cout
        << "================================================================\n";


    std::cout
        << "Planning frame : "
        << planning_frame
        << "\n";


    std::cout
        << "End effector   : "
        << ee_link
        << "\n";


    std::cout
        << "Objects        : 3\n";


    std::cout
        << "Obstacle tasks : Task 2 + Task 3\n";


    // ============================================================
    // 清理旧场景
    // ============================================================

    const std::vector<std::string>
        cleanup_ids =
    {
        "multi_table",
        "red_object",
        "green_object",
        "blue_object",
        "task2_obstacle",
        "task3_obstacle",
        "pick_place_table",
        "pick_place_box",
        "benchmark_box",
        "tiny_benchmark_box"
    };


    for (const auto &id :
         cleanup_ids)
    {
        move_group.detachObject(
            id
        );
    }


    scene.removeCollisionObjects(
        cleanup_ids
    );


    std::this_thread::sleep_for(
        std::chrono::milliseconds(700)
    );


    // ============================================================
    // Home
    // ============================================================

    if (!moveHome(
            move_group,
            joint_group,
            home))
    {
        RCLCPP_ERROR(
            logger,
            "Cannot reach Safe Home."
        );

        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();

        return 1;
    }


    if (!commandGripper(
            node,
            GRIPPER_OPEN))
    {
        RCLCPP_ERROR(
            logger,
            "Cannot open gripper."
        );

        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();

        return 1;
    }


    const auto home_pose =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    // ============================================================
    // Table
    // ============================================================

    const double table_x =
        home_pose.position.x
        +
        0.15;


    if (!addColoredBox(
            scene,
            planning_frame,

            "multi_table",

            table_x,
            0.0,
            TABLE_CENTER_Z,

            TABLE_SIZE_X,
            TABLE_SIZE_Y,
            TABLE_SIZE_Z,

            makeColor(
                0.42,
                0.42,
                0.46,
                1.0
            )))
    {
        RCLCPP_ERROR(
            logger,
            "Cannot add table."
        );

        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();

        return 1;
    }


    // ============================================================
    // 三个物体
    //
    // x位置相互错开
    // 左侧Pick，右侧Place
    // ============================================================

    std::vector<ObjectTask> tasks;


    // ------------------------------------------------------------
    // Task 1 - RED
    // 35 x 35 x 55 mm
    // 无障碍
    // ------------------------------------------------------------

    tasks.push_back(
        {
            "red_object",
            "Task-1",
            "RED",

            0.035,
            0.035,
            0.055,

            makeColor(
                0.92,
                0.12,
                0.12,
                1.0
            ),

            home_pose.position.x + 0.07,
            -0.18,

            home_pose.position.x + 0.07,
             0.18,

            0.016,

            false,

            "",
            "NONE",

            makeColor(
                1.0,
                1.0,
                1.0,
                1.0
            ),

            0.0,
            0.0
        }
    );


    // ------------------------------------------------------------
    // Task 2 - GREEN
    // 45 x 50 x 70 mm
    // 黄色障碍墙
    // ------------------------------------------------------------

    tasks.push_back(
        {
            "green_object",
            "Task-2",
            "GREEN",

            0.045,
            0.050,
            0.070,

            makeColor(
                0.10,
                0.82,
                0.22,
                1.0
            ),

            home_pose.position.x + 0.15,
            -0.18,

            home_pose.position.x + 0.15,
             0.18,

            0.022,

            true,

            "task2_obstacle",
            "YELLOW",

            makeColor(
                0.95,
                0.67,
                0.06,
                1.0
            ),

            0.100,
            0.070
        }
    );


    // ------------------------------------------------------------
    // Task 3 - BLUE
    // 60 x 40 x 50 mm
    // 紫色障碍墙
    // ------------------------------------------------------------

    tasks.push_back(
        {
            "blue_object",
            "Task-3",
            "BLUE",

            0.060,
            0.040,
            0.050,

            makeColor(
                0.08,
                0.32,
                0.95,
                1.0
            ),

            home_pose.position.x + 0.23,
            -0.18,

            home_pose.position.x + 0.23,
             0.18,

            0.018,

            true,

            "task3_obstacle",
            "PURPLE",

            makeColor(
                0.66,
                0.16,
                0.88,
                1.0
            ),

            0.125,
            0.080
        }
    );


    // ============================================================
    // 一次性加入三个不同颜色、不同尺寸的物体
    // ============================================================

    for (const auto &task :
         tasks)
    {
        const bool added =
            addColoredBox(
                scene,
                planning_frame,

                task.id,

                task.pick_x,
                task.pick_y,
                objectCenterZ(
                    task
                ),

                task.sx,
                task.sy,
                task.sz,

                task.color
            );


        if (!added)
        {
            RCLCPP_ERROR(
                logger,
                "Unable to add object %s.",
                task.id.c_str()
            );

            executor.cancel();
            spin_thread.join();
            rclcpp::shutdown();

            return 1;
        }
    }


    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );


    printTaskParameterTable(
        tasks
    );


    std::vector<TaskMetrics>
        all_metrics;


    const auto full_begin =
        std::chrono::
            steady_clock::now();


    // ============================================================
    //                  多任务循环
    // ============================================================

    for (std::size_t task_index = 0;
         task_index < tasks.size();
         ++task_index)
    {
        const auto &task =
            tasks[task_index];


        TaskMetrics metrics;

        metrics.name =
            task.name;

        metrics.color_name =
            task.color_name;

        metrics.obstacle_required =
            task.use_obstacle;


        const auto task_begin =
            std::chrono::
                steady_clock::now();


        std::cout
            << "\n\n";


        std::cout
            << "================================================================\n";


        std::cout
            << "                    "
            << task.name
            << " / "
            << task.color_name
            << "\n";


        std::cout
            << "================================================================\n";


        std::cout
            << "Object size : "
            << task.sx * 1000.0
            << " x "
            << task.sy * 1000.0
            << " x "
            << task.sz * 1000.0
            << " mm\n";


        std::cout
            << "Pick        : ("
            << task.pick_x
            << ", "
            << task.pick_y
            << ")\n";


        std::cout
            << "Place       : ("
            << task.place_x
            << ", "
            << task.place_y
            << ")\n";


        if (task.use_obstacle)
        {
            std::cout
                << "Obstacle    : "
                << task.obstacle_color_name
                << " wall - REQUIRED\n";
        }
        else
        {
            std::cout
                << "Obstacle    : NONE\n";
        }


        // ========================================================
        // STATE 1
        // Open Gripper
        // ========================================================

        std::cout
            << "\n[STATE 1] OPEN GRIPPER\n";


        if (!commandGripper(
                node,
                GRIPPER_OPEN))
        {
            std::cout
                << "FAILED\n";

            all_metrics.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "SUCCESS\n";


        // ========================================================
        // Pre-grasp pose
        // ========================================================

        geometry_msgs::msg::Pose
            pregrasp =
                home_pose;


        pregrasp.position.x =
            task.pick_x;


        pregrasp.position.y =
            task.pick_y;


        pregrasp.position.z =
            graspZ(task)
            +
            APPROACH_DISTANCE;


        // ========================================================
        // STATE 2
        // RRTConnect到Pre-grasp
        // ========================================================

        std::cout
            << "\n[STATE 2] MOVE TO PRE-GRASP - RRTConnect\n";


        double pre_error =
            999.0;


        metrics.pregrasp_success =
            moveToPose(
                move_group,
                pregrasp,
                ee_link,
                "RRTConnectkConfigDefault",
                7.0,
                pre_error
            );


        metrics.pregrasp_error =
            pre_error;


        std::cout
            << "Position error = "
            << pre_error * 1000.0
            << " mm\n";


        if (!metrics.pregrasp_success)
        {
            std::cout
                << "FAILED\n";

            all_metrics.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "SUCCESS\n";


        // ========================================================
        // STATE 3
        // Cartesian下降
        // ========================================================

        std::cout
            << "\n[STATE 3] CARTESIAN APPROACH DOWN\n";


        double approach_fraction =
            0.0;


        double approach_error =
            999.0;


        metrics.approach_success =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                -APPROACH_DISTANCE,

                approach_fraction,
                approach_error
            );


        metrics.approach_fraction =
            approach_fraction
            *
            100.0;


        std::cout
            << "Fraction = "
            << metrics.approach_fraction
            << "%\n";


        if (!metrics.approach_success)
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ========================================================
        // STATE 4
        // Close Gripper
        // ========================================================

        std::cout
            << "\n[STATE 4] CLOSE GRIPPER\n";


        std::cout
            << "Target finger position = "
            << task.gripper_close
            << " m\n";


        metrics.gripper_success =
            commandGripper(
                node,
                task.gripper_close
            );


        if (!metrics.gripper_success)
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ========================================================
        // STATE 5
        // Attach
        // ========================================================

        std::cout
            << "\n[STATE 5] ATTACH OBJECT\n";


        const std::vector<
            std::string
        > touch_links =
        {
            "panda_link8",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger"
        };


        const bool raw_attach =
            move_group.attachObject(
                task.id,
                ee_link,
                touch_links
            );


        metrics.attach_success =
            raw_attach
            &&
            waitAttached(
                scene,
                task.id,
                true,
                3.0
            );


        if (!metrics.attach_success)
        {
            std::cout
                << "FAILED\n";

            all_metrics.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "SUCCESS - "
            << task.color_name
            << " object attached\n";


        // ========================================================
        // STATE 6
        // Lift
        // ========================================================

        std::cout
            << "\n[STATE 6] CARTESIAN LIFT\n";


        double lift_fraction =
            0.0;


        double lift_error =
            999.0;


        metrics.lift_success =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                LIFT_DISTANCE,

                lift_fraction,
                lift_error
            );


        metrics.lift_fraction =
            lift_fraction * 100.0;


        std::cout
            << "Fraction = "
            << metrics.lift_fraction
            << "%\n";


        if (!metrics.lift_success)
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ========================================================
        // Pre-place pose
        // ========================================================

        geometry_msgs::msg::Pose
            preplace =
                home_pose;


        preplace.position.x =
            task.place_x;


        preplace.position.y =
            task.place_y;


        preplace.position.z =
            graspZ(task)
            +
            LIFT_DISTANCE;


        // ========================================================
        // STATE 7
        // Task2/Task3 创建并自动校准障碍物
        // ========================================================

        if (task.use_obstacle)
        {
            std::cout
                << "\n[STATE 7] CALIBRATE OBSTACLE\n";


            std::cout
                << "Requirement:\n";


            std::cout
                << "  1) Direct Cartesian path must be BLOCKED\n";


            std::cout
                << "  2) Sampling planner must still find detour\n";


            const ObstacleResult obstacle =
                calibrateObstacle(
                    move_group,
                    scene,

                    planning_frame,
                    ee_link,

                    task,

                    preplace
                );


            metrics.obstacle_calibrated =
                obstacle.success;


            metrics.direct_fraction =
                obstacle.direct_fraction
                *
                100.0;


            if (!obstacle.success)
            {
                std::cout
                    << "Obstacle calibration FAILED\n";


                all_metrics.push_back(
                    metrics
                );

                break;
            }


            std::cout
                << "\nObstacle accepted.\n";


            std::cout
                << "Color           : "
                << task.obstacle_color_name
                << "\n";


            std::cout
                << "Size            : "
                << obstacle.sx * 1000.0
                << " x "
                << obstacle.sy * 1000.0
                << " x "
                << obstacle.sz * 1000.0
                << " mm\n";


            std::cout
                << "Direct fraction : "
                << metrics.direct_fraction
                << "%\n";


            std::cout
                << "Direct path     : BLOCKED\n";


            std::cout
                << "Detour planner  : "
                << obstacle.feasible_planner
                << " feasible\n";


            std::cout
                << "\nObserve obstacle in RViz for 2 seconds...\n";


            std::this_thread::sleep_for(
                std::chrono::seconds(2)
            );
        }
        else
        {
            metrics.obstacle_calibrated =
                true;


            metrics.direct_fraction =
                100.0;
        }


        // ========================================================
        // STATE 8
        // 携物搬运
        // ========================================================

        std::cout
            << "\n[STATE 8] TRANSFER OBJECT\n";


        if (task.use_obstacle)
        {
            std::cout
                << "Attached object + robot must avoid obstacle.\n";
        }


        double transfer_error =
            999.0;


        metrics.transfer_success =
            transferWithPlannerFallback(
                move_group,
                preplace,
                ee_link,
                metrics.transfer_planner,
                transfer_error
            );


        metrics.transfer_error =
            transfer_error;


        std::cout
            << "Planner = "
            << metrics.transfer_planner
            << "\n";


        std::cout
            << "Final EE error = "
            << transfer_error * 1000.0
            << " mm\n";


        if (!metrics.transfer_success)
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ========================================================
        // STATE 9
        // 放置下降
        // ========================================================

        std::cout
            << "\n[STATE 9] CARTESIAN PLACE DESCEND\n";


        double place_fraction =
            0.0;


        double place_error =
            999.0;


        metrics.place_success =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                -LIFT_DISTANCE,

                place_fraction,
                place_error
            );


        metrics.place_fraction =
            place_fraction
            *
            100.0;


        std::cout
            << "Fraction = "
            << metrics.place_fraction
            << "%\n";


        if (!metrics.place_success)
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ========================================================
        // STATE 10
        // Open Gripper
        // ========================================================

        std::cout
            << "\n[STATE 10] OPEN GRIPPER\n";


        if (!commandGripper(
                node,
                GRIPPER_OPEN))
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ========================================================
        // STATE 11
        // Detach
        // ========================================================

        std::cout
            << "\n[STATE 11] DETACH OBJECT\n";


        const bool raw_detach =
            move_group.detachObject(
                task.id
            );


        metrics.detach_success =
            raw_detach
            &&
            waitAttached(
                scene,
                task.id,
                false,
                3.0
            );


        waitWorld(
            scene,
            task.id,
            true,
            3.0
        );


        if (!metrics.detach_success)
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "SUCCESS\n";


        // ========================================================
        // Placement Error
        // ========================================================

        std::this_thread::sleep_for(
            std::chrono::milliseconds(400)
        );


        const auto poses =
            scene.getObjectPoses(
                {task.id}
            );


        const auto it =
            poses.find(
                task.id
            );


        if (it !=
            poses.end())
        {
            geometry_msgs::msg::Pose
                expected;


            expected.orientation.w =
                1.0;


            expected.position.x =
                task.place_x;


            expected.position.y =
                task.place_y;


            expected.position.z =
                objectCenterZ(task);


            metrics.placement_error_m =
                positionDistance(
                    it->second,
                    expected
                );
        }


        std::cout
            << "Placement error = "
            << metrics.placement_error_m
                   *
                   1000.0
            << " mm\n";


        // ========================================================
        // STATE 12
        // Retreat
        // ========================================================

        std::cout
            << "\n[STATE 12] CARTESIAN RETREAT\n";


        double retreat_fraction =
            0.0;


        double retreat_error =
            999.0;


        metrics.retreat_success =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                RETREAT_DISTANCE,

                retreat_fraction,
                retreat_error
            );


        std::cout
            << "Fraction = "
            << retreat_fraction
                   *
                   100.0
            << "%\n";


        // 当前任务的障碍物任务结束后删除
        if (task.use_obstacle)
        {
            removeObject(
                scene,
                task.obstacle_id
            );
        }


        metrics.total_time =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    task_begin
                ).count();


        metrics.success =
            metrics.pregrasp_success
            &&
            metrics.approach_success
            &&
            metrics.gripper_success
            &&
            metrics.attach_success
            &&
            metrics.lift_success
            &&
            metrics.obstacle_calibrated
            &&
            metrics.transfer_success
            &&
            metrics.place_success
            &&
            metrics.detach_success
            &&
            metrics.placement_error_m
                <
                0.03;


        all_metrics.push_back(
            metrics
        );


        std::cout
            << "\n";


        std::cout
            << "------------------------------------------------------------\n";


        std::cout
            << task.name
            << " RESULT : "
            << (
                metrics.success
                ?
                "SUCCESS"
                :
                "FAILED"
            )
            << "\n";


        std::cout
            << "------------------------------------------------------------\n";


        if (!metrics.success)
        {
            break;
        }


        std::this_thread::sleep_for(
            std::chrono::seconds(2)
        );
    }


    // ============================================================
    // 最后返回Home
    // ============================================================

    std::cout
        << "\n\n";


    std::cout
        << "================================================================\n";


    std::cout
        << "                    RETURN SAFE HOME\n";


    std::cout
        << "================================================================\n";


    const bool home_success =
        moveHome(
            move_group,
            joint_group,
            home
        );


    // ============================================================
    // 总统计
    // ============================================================

    int success_count =
        0;


    double mean_error_mm =
        0.0;


    double max_error_mm =
        0.0;


    for (const auto &m :
         all_metrics)
    {
        if (m.success)
        {
            ++success_count;
        }


        if (m.placement_error_m <
            10.0)
        {
            const double mm =
                m.placement_error_m
                *
                1000.0;


            mean_error_mm +=
                mm;


            max_error_mm =
                std::max(
                    max_error_mm,
                    mm
                );
        }
    }


    if (!all_metrics.empty())
    {
        mean_error_mm /=
            static_cast<double>(
                all_metrics.size()
            );
    }


    const double total_time =
        std::chrono::
            duration<double>(
                std::chrono::
                    steady_clock::now()
                -
                full_begin
            ).count();


    printTaskParameterTable(
        tasks
    );


    printResultTable(
        all_metrics
    );


    const bool final_success =
        success_count
            ==
            static_cast<int>(
                tasks.size()
            )
        &&
        home_success;


    std::cout
        << "\n\n";


    std::cout
        << "================================================================\n";


    std::cout
        << "                 MULTI-OBJECT FINAL SUMMARY\n";


    std::cout
        << "================================================================\n";


    std::cout
        << "Total objects           : "
        << tasks.size()
        << "\n";


    std::cout
        << "Successful tasks        : "
        << success_count
        << " / "
        << tasks.size()
        << "\n";


    std::cout
        << "Task-1 obstacle         : NONE\n";


    std::cout
        << "Task-2 obstacle         : YELLOW wall\n";


    std::cout
        << "Task-3 obstacle         : PURPLE wall\n";


    std::cout
        << "Mean placement error    : "
        << std::fixed
        << std::setprecision(3)
        << mean_error_mm
        << " mm\n";


    std::cout
        << "Max placement error     : "
        << max_error_mm
        << " mm\n";


    std::cout
        << "Total execution time    : "
        << total_time
        << " s\n";


    std::cout
        << "Return Home             : "
        << (
            home_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "OVERALL RESULT          : "
        << (
            final_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "================================================================\n";


    // ============================================================
    // CSV
    // ============================================================

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
        "multi_pick_and_place_result.csv";


    std::ofstream csv(
        csv_path
    );


    csv
        << "task,"
        << "color,"
        << "obstacle_required,"
        << "obstacle_calibrated,"
        << "direct_fraction_percent,"
        << "approach_fraction_percent,"
        << "lift_fraction_percent,"
        << "transfer_planner,"
        << "transfer_error_m,"
        << "place_fraction_percent,"
        << "placement_error_m,"
        << "total_time_s,"
        << "success\n";


    for (const auto &m :
         all_metrics)
    {
        csv
            << m.name
            << ","
            << m.color_name
            << ","
            << (
                m.obstacle_required
                ?
                1
                :
                0
            )
            << ","
            << (
                m.obstacle_calibrated
                ?
                1
                :
                0
            )
            << ","
            << m.direct_fraction
            << ","
            << m.approach_fraction
            << ","
            << m.lift_fraction
            << ","
            << m.transfer_planner
            << ","
            << m.transfer_error
            << ","
            << m.place_fraction
            << ","
            << m.placement_error_m
            << ","
            << m.total_time
            << ","
            << (
                m.success
                ?
                1
                :
                0
            )
            << "\n";
    }


    csv.close();


    std::cout
        << "\nResult CSV:\n"
        << csv_path
        << "\n";


    std::cout
        << "\nFinal scene remains visible for 8 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(8)
    );


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
