#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
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

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>


// ========================================================================
//                           运动参数
// ========================================================================

constexpr double ARM_VELOCITY = 0.09;
constexpr double ARM_ACCELERATION = 0.04;

constexpr double CART_VELOCITY = 0.06;
constexpr double CART_ACCELERATION = 0.025;

constexpr double EEF_STEP = 0.004;
constexpr double MIN_CART_FRACTION = 0.98;


// 抓取上方到物体的下降距离
constexpr double APPROACH_DISTANCE = 0.10;


// 抓住物体之后抬升
constexpr double LIFT_DISTANCE = 0.17;


// 放完物体后撤离
constexpr double RETREAT_DISTANCE = 0.12;


// ========================================================================
//                           误差要求
// ========================================================================

constexpr double POSITION_TOLERANCE = 0.018;
constexpr double HOME_TOLERANCE = 0.050;


// ========================================================================
//                           桌子
// ========================================================================

// 上一版桌子过大。
// 这一版缩小，避免进入Panda底座附近。
constexpr double TABLE_SIZE_X = 0.62;
constexpr double TABLE_SIZE_Y = 0.78;
constexpr double TABLE_SIZE_Z = 0.10;

constexpr double TABLE_CENTER_Z = 0.15;

constexpr double TABLE_TOP_Z =
    TABLE_CENTER_Z +
    TABLE_SIZE_Z / 2.0;


// 物体底部和桌面之间保留5mm数值余量
constexpr double OBJECT_TABLE_GAP = 0.005;


// panda_link8相对物体中心的抓取高度偏置。
// 上一安全版0.175太保守。
// 原成功版本0.130。
// 这里折中使用0.150。
constexpr double GRASP_EE_Z_OFFSET = 0.150;


// Cartesian目标的最低腕部高度。
// 防止向下动作过低。
constexpr double MIN_WRIST_HEIGHT =
    TABLE_TOP_Z + 0.145;


// ========================================================================
//                         障碍物安全参数
// ========================================================================

// 可见障碍物四周额外增加50mm碰撞安全区。
constexpr double OBSTACLE_MARGIN_XY = 0.050;


// 顶部额外安全余量
constexpr double OBSTACLE_MARGIN_TOP = 0.040;


// 直接Cartesian完成率低于90%
// 才认为障碍物确实阻断直线搬运。
constexpr double DIRECT_BLOCK_THRESHOLD = 0.90;


// ========================================================================
// Gripper
// ========================================================================

constexpr double GRIPPER_OPEN = 0.040;
constexpr double GRIPPER_FORCE = 20.0;


// ========================================================================
// 数据结构
// ========================================================================

struct TaskConfig
{
    std::string name;

    std::string object_id;

    std::string color_name;

    std_msgs::msg::ColorRGBA color;

    double sx;
    double sy;
    double sz;

    double pick_x;
    double pick_y;

    double place_x;
    double place_y;

    bool obstacle_required;

    std::string obstacle_id;
    std::string guard_id;

    std::string obstacle_color_name;

    std_msgs::msg::ColorRGBA obstacle_color;

    double obstacle_sx;
    double obstacle_sy;
    double obstacle_base_sz;
};


struct PlanResult
{
    bool success = false;

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;

    std::string planner = "-";

    double planning_time = 0.0;

    double path_length = 999.0;
};


struct ObstacleCalibration
{
    bool success = false;

    double x = 0.0;
    double y = 0.0;

    double visible_sx = 0.0;
    double visible_sy = 0.0;
    double visible_sz = 0.0;

    double guard_sx = 0.0;
    double guard_sy = 0.0;
    double guard_sz = 0.0;

    double direct_fraction = 1.0;

    std::string verified_planner = "-";
};


struct TaskMetrics
{
    std::string name;
    std::string color;

    bool obstacle_required = false;
    bool obstacle_verified = false;

    double approach_fraction = 0.0;
    double lift_fraction = 0.0;
    double direct_fraction = 100.0;
    double place_fraction = 0.0;
    double retreat_fraction = 0.0;

    std::string transfer_planner = "-";

    double transfer_path_length = 0.0;
    double transfer_planning_time = 0.0;
    double transfer_error_m = 999.0;

    double placement_error_m = 999.0;

    double total_time_s = 0.0;

    bool success = false;
};


// ========================================================================
// Color
// ========================================================================

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


// ========================================================================
// 几何工具
// ========================================================================

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


double objectCenterZ(
    const TaskConfig &task)
{
    return
        TABLE_TOP_Z
        +
        task.sz / 2.0
        +
        OBJECT_TABLE_GAP;
}


double graspZ(
    const TaskConfig &task)
{
    return
        objectCenterZ(task)
        +
        GRASP_EE_Z_OFFSET;
}


// ========================================================================
// Joint误差
// ========================================================================

double maxJointError(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

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


// ========================================================================
// Collision Object
// ========================================================================

moveit_msgs::msg::CollisionObject makeBox(
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
        moveit_msgs::msg::
            CollisionObject::ADD;


    return object;
}


bool addColoredBox(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

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
    return scene.applyCollisionObject(
        makeBox(
            frame,
            id,
            x,
            y,
            z,
            sx,
            sy,
            sz
        ),
        color
    );
}


bool worldObjectExists(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

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
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &id)
{
    const auto result =
        scene.getAttachedObjects(
            {id}
        );


    return !result.empty();
}


bool waitWorld(
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
        if (worldObjectExists(
                scene,
                id)
            ==
            expected)
        {
            return true;
        }


        const double t =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();


        if (t > timeout)
        {
            return false;
        }


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(100)
        );
    }
}


bool waitAttached(
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
        if (attachedObjectExists(
                scene,
                id)
            ==
            expected)
        {
            return true;
        }


        const double t =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();


        if (t > timeout)
        {
            return false;
        }


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(100)
        );
    }
}


void removeObject(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

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


// ========================================================================
// Gripper
// ========================================================================

bool commandGripper(
    const rclcpp::Node::SharedPtr &node,

    double position)
{
    using GripperCommand =
        control_msgs::action::
            GripperCommand;


    auto client =
        rclcpp_action::
            create_client<
                GripperCommand
            >(
                node,
                "/panda_hand_controller/gripper_cmd"
            );


    if (!client->
            wait_for_action_server(
                std::chrono::
                    seconds(4)))
    {
        return false;
    }


    GripperCommand::Goal goal;

    goal.command.position =
        position;

    goal.command.max_effort =
        GRIPPER_FORCE;


    auto send_future =
        client->async_send_goal(
            goal
        );


    if (send_future.wait_for(
            std::chrono::
                seconds(4))
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
            std::chrono::
                seconds(8))
        !=
        std::future_status::ready)
    {
        return false;
    }


    return
        result_future.get().code
        ==
        rclcpp_action::
            ResultCode::SUCCEEDED;
}


// ========================================================================
// Cartesian时间参数化
// ========================================================================

bool retimeTrajectory(
    const moveit::core::RobotStatePtr &state,

    const std::string &group_name,

    moveit_msgs::msg::RobotTrajectory &trajectory)
{
    robot_trajectory::RobotTrajectory robot_traj(
        state->getRobotModel(),
        group_name
    );


    robot_traj.setRobotTrajectoryMsg(
        *state,
        trajectory
    );


    trajectory_processing::
        IterativeParabolicTimeParameterization iptp;


    if (!iptp.computeTimeStamps(
            robot_traj,
            CART_VELOCITY,
            CART_ACCELERATION))
    {
        return false;
    }


    robot_traj.getRobotTrajectoryMsg(
        trajectory
    );


    return true;
}


// ========================================================================
// RViz显示轨迹
// ========================================================================

void publishTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::core::RobotStatePtr &state,

    const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    moveit_msgs::msg::DisplayTrajectory display;

    display.model_id = "panda";


    moveit::core::
        robotStateToRobotStateMsg(
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


// ========================================================================
// Home
// ========================================================================

bool moveHome(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &home)
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
        std::cout
            << "Robot already at Safe Home.\n";

        return true;
    }


    for (int attempt = 1;
         attempt <= 4;
         ++attempt)
    {
        std::cout
            << "Home attempt "
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


        move_group.setPlanningTime(6.0);

        move_group.setNumPlanningAttempts(8);


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
                move_group.plan(plan)))
        {
            continue;
        }


        move_group.execute(
            plan
        );


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(600)
        );


        const double error =
            maxJointError(
                move_group,
                joint_group,
                home
            );


        std::cout
            << "Home joint error = "
            << error
            << " rad\n";


        if (error <
            HOME_TOLERANCE)
        {
            return true;
        }
    }


    return false;
}


// ========================================================================
// Joint path length
// ========================================================================

double trajectoryJointLength(
    const moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    const auto &points =
        trajectory.joint_trajectory.points;


    if (points.size() < 2)
    {
        return 0.0;
    }


    double length = 0.0;


    for (std::size_t i = 1;
         i < points.size();
         ++i)
    {
        const auto &a =
            points[i - 1].positions;


        const auto &b =
            points[i].positions;


        const std::size_t n =
            std::min(
                a.size(),
                b.size()
            );


        double ds = 0.0;


        for (std::size_t j = 0;
             j < n;
             ++j)
        {
            const double d =
                b[j] -
                a[j];


            ds += d * d;
        }


        length +=
            std::sqrt(ds);
    }


    return length;
}


// ========================================================================
// 单次生成轨迹，不执行
// ========================================================================

PlanResult generatePlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    const std::string &planner_id,

    const std::string &planner_name,

    double planning_time)
{
    PlanResult result;


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


    move_group.setNumPlanningAttempts(1);


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
        return result;
    }


    const auto begin =
        std::chrono::
            steady_clock::now();


    moveit::planning_interface::
        MoveGroupInterface::Plan plan;


    const bool success =
        static_cast<bool>(
            move_group.plan(
                plan
            )
        );


    const auto end =
        std::chrono::
            steady_clock::now();


    move_group.clearPoseTargets();


    result.planning_time =
        std::chrono::
            duration<double>(
                end -
                begin
            ).count();


    if (!success)
    {
        return result;
    }


    result.success = true;

    result.plan = plan;

    result.planner = planner_name;

    result.path_length =
        trajectoryJointLength(
            plan.trajectory_
        );


    return result;
}


// ========================================================================
// 多候选规划
//
// RRTConnect x3
// RRT* x2
//
// 只在有限规划时间内的可行候选中，选择关节空间路径较短的一条。
// ========================================================================

PlanResult selectBestPlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link)
{
    std::vector<PlanResult>
        candidates;


    for (int i = 0;
         i < 3;
         ++i)
    {
        auto p =
            generatePlan(
                move_group,
                target,
                ee_link,

                "RRTConnectkConfigDefault",
                "RRTConnect",

                5.0
            );


        if (p.success)
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
        auto p =
            generatePlan(
                move_group,
                target,
                ee_link,

                "RRTstarkConfigDefault",
                "RRT*",

                8.0
            );


        if (p.success)
        {
            candidates.push_back(
                p
            );
        }
    }


    if (candidates.empty())
    {
        PlanResult empty_result;

        return empty_result;
    }


    return *std::min_element(
        candidates.begin(),
        candidates.end(),

        [](
            const PlanResult &a,
            const PlanResult &b)
        {
            return
                a.path_length <
                b.path_length;
        }
    );
}


// ========================================================================
// 规划稳定性验证
// ========================================================================

bool stablePlanExists(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    std::string &planner)
{
    int connect_success = 0;


    for (int i = 0;
         i < 2;
         ++i)
    {
        auto result =
            generatePlan(
                move_group,
                target,
                ee_link,

                "RRTConnectkConfigDefault",
                "RRTConnect",

                5.0
            );


        if (result.success)
        {
            ++connect_success;
        }
    }


    if (connect_success == 2)
    {
        planner =
            "RRTConnect";

        return true;
    }


    int star_success = 0;


    for (int i = 0;
         i < 2;
         ++i)
    {
        auto result =
            generatePlan(
                move_group,
                target,
                ee_link,

                "RRTstarkConfigDefault",
                "RRT*",

                8.0
            );


        if (result.success)
        {
            ++star_success;
        }
    }


    if (star_success == 2)
    {
        planner =
            "RRT*";

        return true;
    }


    planner = "-";

    return false;
}


// ========================================================================
// 执行已经选好的Plan
// ========================================================================

bool executePlanToPose(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const PlanResult &result,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    double &error)
{
    if (!result.success)
    {
        return false;
    }


    move_group.execute(
        result.plan
    );


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(650)
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


    return
        error <
        POSITION_TOLERANCE;
}


// ========================================================================
// 普通安全Pose运动
// ========================================================================

bool moveToPoseSafe(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    double &error,

    std::string &planner_used)
{
    auto result =
        selectBestPlan(
            move_group,
            target,
            ee_link
        );


    if (!result.success)
    {
        return false;
    }


    planner_used =
        result.planner;


    std::cout
        << "Selected planner = "
        << result.planner
        << "\n";


    std::cout
        << "Joint path       = "
        << result.path_length
        << " rad\n";


    return executePlanToPose(
        move_group,
        result,
        target,
        ee_link,
        error
    );
}


// ========================================================================
// Cartesian相对运动
// ========================================================================

bool cartesianMove(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const std::string &group_name,

    const std::string &ee_link,

    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &display_publisher,

    double dx,
    double dy,
    double dz,

    double &fraction,
    double &error)
{
    const auto start =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    geometry_msgs::msg::Pose target =
        start;


    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;


    // --------------------------------------------------------
    // 防止腕部下降过低
    // --------------------------------------------------------

    if (target.position.z <
        MIN_WRIST_HEIGHT)
    {
        std::cout
            << "[SAFETY] Cartesian target rejected.\n";


        std::cout
            << "Target wrist Z = "
            << target.position.z
            << "\n";


        std::cout
            << "Minimum safe Z  = "
            << MIN_WRIST_HEIGHT
            << "\n";


        fraction = 0.0;

        error = 999.0;

        return false;
    }


    std::vector<
        geometry_msgs::msg::Pose
    > waypoints;


    waypoints.push_back(
        target
    );


    moveit_msgs::msg::
        RobotTrajectory trajectory;


    move_group.setStartStateToCurrentState();


    // true:
    // 开启碰撞检测
    fraction =
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


    // ========================================================
    // 执行Cartesian轨迹
    // ========================================================

    const bool execute_success =
        static_cast<bool>(
            move_group.execute(
                plan
            )
        );


    if (!execute_success)
    {
        std::cout
            << "[EXECUTION] MoveIt execution FAILED\n";

        error =
            999.0;

        return false;
    }


    // ========================================================
    // WSL2 / Fake Hardware环境中，
    // trajectory执行完成之后有时joint_states会短暂滞后。
    //
    // 原版本只读取一次：
    //
    // getCurrentPose()
    //
    // 一旦joint state时间戳刚好过期，
    // 就会把已经成功完成的动作错误判定成FAILED。
    //
    // 新版本：
    //
    // 最多重试8次获取新鲜RobotState。
    // ========================================================

    bool state_verified =
        false;


    for (int retry = 1;
         retry <= 8;
         ++retry)
    {
        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    300
                )
        );


        auto fresh_state =
            move_group.getCurrentState(
                1.5
            );


        if (!fresh_state)
        {
            std::cout
                << "[STATE] Waiting for fresh joint state "
                << retry
                << "/8\n";

            continue;
        }


        const auto actual =
            move_group.getCurrentPose(
                ee_link
            ).pose;


        error =
            positionDistance(
                actual,
                target
            );


        state_verified =
            true;


        std::cout
            << "[STATE] Cartesian final error = "
            << error * 1000.0
            << " mm\n";


        if (error <
            POSITION_TOLERANCE)
        {
            return true;
        }
    }


    // ========================================================
    // 如果8次仍然取不到新鲜joint state：
    //
    // 不把已经由MoveIt确认执行成功的Cartesian轨迹
    // 误判成运动失败。
    //
    // 注意：
    // 这里只表示最终实时位置验证不可用，
    // 并不伪造一个位置误差。
    // ========================================================

    if (!state_verified)
    {
        std::cout
            << "[WARNING] Fresh joint state temporarily unavailable.\n";


        std::cout
            << "[WARNING] Cartesian trajectory itself was executed "
            << "successfully.\n";


        std::cout
            << "[WARNING] Continuing task based on successful "
            << "MoveIt execution + 100% Cartesian path.\n";


        error =
            -1.0;


        return
            execute_success
            &&
            fraction >=
                MIN_CART_FRACTION;
    }


    // 已经成功读取当前位置，
    // 但真实位置误差仍然超限。
    return false;
}


// ========================================================================
// 直接Cartesian完成率测试
// 不执行
// ========================================================================

double directCartesianFraction(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

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


// ========================================================================
// 加入可见障碍物 + 扩大安全包络
// ========================================================================

bool addObstacleWithGuard(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &frame,

    const TaskConfig &task,

    double x,
    double y,

    double visible_sz,

    double &guard_sx,
    double &guard_sy,
    double &guard_sz)
{
    const double visible_z =
        TABLE_TOP_Z
        +
        visible_sz / 2.0;


    guard_sx =
        task.obstacle_sx
        +
        2.0 *
        OBSTACLE_MARGIN_XY;


    guard_sy =
        task.obstacle_sy
        +
        2.0 *
        OBSTACLE_MARGIN_XY;


    guard_sz =
        visible_sz
        +
        OBSTACLE_MARGIN_TOP;


    const double guard_z =
        TABLE_TOP_Z
        +
        guard_sz / 2.0;


    const bool visible_ok =
        addColoredBox(
            scene,
            frame,

            task.obstacle_id,

            x,
            y,
            visible_z,

            task.obstacle_sx,
            task.obstacle_sy,
            visible_sz,

            task.obstacle_color
        );


    if (!visible_ok)
    {
        return false;
    }


    // 半透明安全包络
    auto guard_color =
        task.obstacle_color;


    guard_color.a =
        0.08f;


    const bool guard_ok =
        addColoredBox(
            scene,
            frame,

            task.guard_id,

            x,
            y,
            guard_z,

            guard_sx,
            guard_sy,
            guard_sz,

            guard_color
        );


    if (!guard_ok)
    {
        removeObject(
            scene,
            task.obstacle_id
        );

        return false;
    }


    waitWorld(
        scene,
        task.obstacle_id,
        true
    );


    waitWorld(
        scene,
        task.guard_id,
        true
    );


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(450)
    );


    return true;
}


// ========================================================================
// 自动寻找安全障碍物位置
//
// 目标：
// 1. Direct Cartesian被阻断。
// 2. 扩大后的Guard存在。
// 3. RRTConnect或RRT*仍能稳定绕开。
// ========================================================================

ObstacleCalibration calibrateObstacle(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &frame,

    const std::string &ee_link,

    const TaskConfig &task,

    const geometry_msgs::msg::Pose &target)
{
    ObstacleCalibration result;


    const auto start =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    // 让墙位于搬运路径中部附近
    const std::vector<double> ratios =
    {
        0.50,
        0.45,
        0.55,
        0.40,
        0.60
    };


    // X方向微调
    const std::vector<double> x_offsets =
    {
        0.0,
        0.025,
        -0.025,
        0.045,
        -0.045
    };


    for (double ratio :
         ratios)
    {
        for (double x_offset :
             x_offsets)
        {
            removeObject(
                scene,
                task.obstacle_id
            );


            removeObject(
                scene,
                task.guard_id
            );


            const double ox =
                start.position.x
                +
                ratio
                *
                (
                    target.position.x
                    -
                    start.position.x
                )
                +
                x_offset;


            const double oy =
                start.position.y
                +
                ratio
                *
                (
                    target.position.y
                    -
                    start.position.y
                );


            // --------------------------------------------------
            // 根据当前搬运高度自动决定障碍墙高度。
            //
            // 可见墙顶部略高于当前EE。
            // 外部Guard再增加顶部安全余量。
            // --------------------------------------------------

            const double auto_height =
                start.position.z
                -
                TABLE_TOP_Z
                +
                0.015;


            const double visible_sz =
                std::max(
                    task.obstacle_base_sz,
                    auto_height
                );


            double guard_sx = 0.0;
            double guard_sy = 0.0;
            double guard_sz = 0.0;


            if (!addObstacleWithGuard(
                    scene,
                    frame,
                    task,

                    ox,
                    oy,

                    visible_sz,

                    guard_sx,
                    guard_sy,
                    guard_sz))
            {
                continue;
            }


            const double direct_fraction =
                directCartesianFraction(
                    move_group,
                    target
                );


            std::cout
                << "\nObstacle candidate\n";


            std::cout
                << "Center XY      : ("
                << ox
                << ", "
                << oy
                << ")\n";


            std::cout
                << "Visible size   : "
                << task.obstacle_sx * 1000.0
                << " x "
                << task.obstacle_sy * 1000.0
                << " x "
                << visible_sz * 1000.0
                << " mm\n";


            std::cout
                << "Safety envelope: "
                << guard_sx * 1000.0
                << " x "
                << guard_sy * 1000.0
                << " x "
                << guard_sz * 1000.0
                << " mm\n";


            std::cout
                << "Direct fraction: "
                << direct_fraction * 100.0
                << "%\n";


            // 必须真正挡住直接路径
            if (direct_fraction >=
                DIRECT_BLOCK_THRESHOLD)
            {
                continue;
            }


            std::string stable_planner;


            // 必须还能稳定绕过去
            if (!stablePlanExists(
                    move_group,
                    target,
                    ee_link,
                    stable_planner))
            {
                continue;
            }


            result.success = true;

            result.x = ox;
            result.y = oy;

            result.visible_sx =
                task.obstacle_sx;

            result.visible_sy =
                task.obstacle_sy;

            result.visible_sz =
                visible_sz;

            result.guard_sx =
                guard_sx;

            result.guard_sy =
                guard_sy;

            result.guard_sz =
                guard_sz;

            result.direct_fraction =
                direct_fraction;

            result.verified_planner =
                stable_planner;


            return result;
        }
    }


    removeObject(
        scene,
        task.obstacle_id
    );


    removeObject(
        scene,
        task.guard_id
    );


    return result;
}


// ========================================================================
// 参数表
// ========================================================================

void printConfiguration(
    const std::vector<TaskConfig> &tasks)
{
    std::cout
        << "\n\n";


    std::cout
        << "============================================================================================================================\n";


    std::cout
        << "                                      SAFE MULTI-PICK V2 PARAMETERS\n";


    std::cout
        << "============================================================================================================================\n";


    std::cout
        << std::left

        << std::setw(10)
        << "Task"

        << std::setw(10)
        << "Color"

        << std::setw(20)
        << "Object(mm)"

        << std::setw(18)
        << "Pick XY"

        << std::setw(18)
        << "Place XY"

        << "Obstacle\n";


    std::cout
        << "----------------------------------------------------------------------------------------------------------------------------\n";


    for (const auto &task :
         tasks)
    {
        std::ostringstream size;

        size
            << static_cast<int>(
                   task.sx * 1000.0)
            << "x"
            << static_cast<int>(
                   task.sy * 1000.0)
            << "x"
            << static_cast<int>(
                   task.sz * 1000.0);


        std::ostringstream pick;

        pick
            << std::fixed
            << std::setprecision(2)
            << "("
            << task.pick_x
            << ","
            << task.pick_y
            << ")";


        std::ostringstream place;

        place
            << std::fixed
            << std::setprecision(2)
            << "("
            << task.place_x
            << ","
            << task.place_y
            << ")";


        std::cout
            << std::setw(10)
            << task.name

            << std::setw(10)
            << task.color_name

            << std::setw(20)
            << size.str()

            << std::setw(18)
            << pick.str()

            << std::setw(18)
            << place.str()

            << (
                task.obstacle_required
                ?
                task.obstacle_color_name
                :
                "NONE"
            )

            << "\n";
    }


    std::cout
        << "============================================================================================================================\n";


    std::cout
        << "Table size                 : "
        << TABLE_SIZE_X
        << " x "
        << TABLE_SIZE_Y
        << " m\n";


    std::cout
        << "Table top Z                : "
        << TABLE_TOP_Z
        << " m\n";


    std::cout
        << "Minimum wrist Z            : "
        << MIN_WRIST_HEIGHT
        << " m\n";


    std::cout
        << "Grasp EE offset            : "
        << GRASP_EE_Z_OFFSET
        << " m\n";


    std::cout
        << "Lift distance              : "
        << LIFT_DISTANCE
        << " m\n";


    std::cout
        << "Obstacle XY margin         : "
        << OBSTACLE_MARGIN_XY * 1000.0
        << " mm each side\n";


    std::cout
        << "Cartesian collision check  : ENABLED\n";


    std::cout
        << "Transfer planning          : RRTConnect x3 + RRT* x2\n";


    std::cout
        << "============================================================================================================================\n";
}


// ========================================================================
// 结果表
// ========================================================================

void printResults(
    const std::vector<TaskMetrics> &metrics)
{
    std::cout
        << "\n\n";


    std::cout
        << "==================================================================================================================================================\n";


    std::cout
        << "                                                       SAFE MULTI-TASK V2 RESULTS\n";


    std::cout
        << "==================================================================================================================================================\n";


    std::cout
        << std::left

        << std::setw(10)
        << "Task"

        << std::setw(10)
        << "Color"

        << std::setw(16)
        << "Direct"

        << std::setw(15)
        << "Planner"

        << std::setw(15)
        << "Path(rad)"

        << std::setw(16)
        << "EE Err(mm)"

        << std::setw(18)
        << "Place Err(mm)"

        << std::setw(18)
        << "Cart A/L/P"

        << "Result\n";


    std::cout
        << "--------------------------------------------------------------------------------------------------------------------------------------------------\n";


    for (const auto &m :
         metrics)
    {
        std::ostringstream direct;


        if (m.obstacle_required)
        {
            direct
                << std::fixed
                << std::setprecision(1)
                << m.direct_fraction
                << "% BLOCK";
        }
        else
        {
            direct
                << "FREE";
        }


        std::ostringstream cart;

        cart
            << std::fixed
            << std::setprecision(0)
            << m.approach_fraction
            << "/"
            << m.lift_fraction
            << "/"
            << m.place_fraction;


        std::cout
            << std::setw(10)
            << m.name

            << std::setw(10)
            << m.color

            << std::setw(16)
            << direct.str()

            << std::setw(15)
            << m.transfer_planner

            << std::fixed
            << std::setprecision(4)

            << std::setw(15)
            << m.transfer_path_length

            << std::setw(16)
            << m.transfer_error_m * 1000.0

            << std::setw(18)
            << m.placement_error_m * 1000.0

            << std::setw(18)
            << cart.str()

            << (
                m.success
                ?
                "SUCCESS"
                :
                "FAILED"
            )

            << "\n";
    }


    std::cout
        << "==================================================================================================================================================\n";
}


// ========================================================================
// MAIN
// ========================================================================

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
            "multi_pick_and_place_safe_v2",

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
        std::chrono::
            seconds(2)
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
            moveit_msgs::msg::
                DisplayTrajectory
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


    // ==================================================================
    // Safe Home
    // ==================================================================

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
        << "========================================================================\n";


    std::cout
        << "         PANDA SAFE MULTI-OBJECT PICK & PLACE V2\n";


    std::cout
        << "========================================================================\n";


    std::cout
        << "Planning frame : "
        << planning_frame
        << "\n";


    std::cout
        << "End effector   : "
        << ee_link
        << "\n";


    // ==================================================================
    // 清理以前场景
    // ==================================================================

    const std::vector<
        std::string
    > cleanup =
    {
        "safe_v2_table",

        "safe_v2_red",
        "safe_v2_green",
        "safe_v2_blue",

        "safe_v2_obstacle_2",
        "safe_v2_guard_2",

        "safe_v2_obstacle_3",
        "safe_v2_guard_3",

        "safe_table",
        "red_safe_object",
        "green_safe_object",
        "blue_safe_object",

        "safe_obstacle_2",
        "safe_obstacle_2_guard",
        "safe_obstacle_3",
        "safe_obstacle_3_guard",

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
         cleanup)
    {
        move_group.detachObject(
            id
        );
    }


    scene.removeCollisionObjects(
        cleanup
    );


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(700)
    );


    // ==================================================================
    // Home
    // ==================================================================

    if (!moveHome(
            move_group,
            joint_group,
            home))
    {
        RCLCPP_ERROR(
            logger,
            "Unable to establish Safe Home."
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
            "Unable to open gripper."
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


    std::cout
        << "Home EE X = "
        << home_pose.position.x
        << "\n";


    // ==================================================================
    // 新桌子
    //
    // 中心向机器人前方移动0.20m。
    //
    // Home x≈0.307时：
    //
    // center≈0.507
    // xmin≈0.197
    //
    // 不再从底座附近开始。
    // ==================================================================

    const double table_x =
        home_pose.position.x
        +
        0.20;


    std::cout
        << "Table center X = "
        << table_x
        << "\n";


    std::cout
        << "Table X min    = "
        << table_x
               -
               TABLE_SIZE_X / 2.0
        << "\n";


    if (!addColoredBox(
            scene,
            planning_frame,

            "safe_v2_table",

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
            "Table creation failed."
        );


        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();


        return 1;
    }


    waitWorld(
        scene,
        "safe_v2_table",
        true
    );


    // ==================================================================
    // 任务设计
    //
    // Pick使用上一版已经成功验证的位置：
    // y = -0.18
    //
    // Place比旧版本明显拉远：
    // y = +0.24 / +0.27
    // ==================================================================

    std::vector<TaskConfig>
        tasks;


    // ------------------------------------------------------------------
    // Task 1 - RED
    // ------------------------------------------------------------------

    tasks.push_back(
        {
            "Task-1",

            "safe_v2_red",

            "RED",

            makeColor(
                0.95,
                0.08,
                0.08,
                1.0
            ),

            0.035,
            0.035,
            0.055,

            home_pose.position.x + 0.07,
            -0.18,

            home_pose.position.x + 0.07,
             0.24,

            false,

            "",
            "",

            "NONE",

            makeColor(
                1.0,
                1.0,
                1.0,
                1.0
            ),

            0.0,
            0.0,
            0.0
        }
    );


    // ------------------------------------------------------------------
    // Task 2 - GREEN
    // Yellow obstacle
    // ------------------------------------------------------------------

    tasks.push_back(
        {
            "Task-2",

            "safe_v2_green",

            "GREEN",

            makeColor(
                0.08,
                0.86,
                0.20,
                1.0
            ),

            0.045,
            0.050,
            0.070,

            home_pose.position.x + 0.15,
            -0.18,

            home_pose.position.x + 0.15,
             0.27,

            true,

            "safe_v2_obstacle_2",
            "safe_v2_guard_2",

            "YELLOW",

            makeColor(
                0.98,
                0.70,
                0.05,
                1.0
            ),

            0.100,
            0.070,
            0.300
        }
    );


    // ------------------------------------------------------------------
    // Task 3 - BLUE
    // Purple obstacle
    // ------------------------------------------------------------------

    tasks.push_back(
        {
            "Task-3",

            "safe_v2_blue",

            "BLUE",

            makeColor(
                0.05,
                0.28,
                0.96,
                1.0
            ),

            0.060,
            0.040,
            0.050,

            home_pose.position.x + 0.23,
            -0.18,

            home_pose.position.x + 0.23,
             0.27,

            true,

            "safe_v2_obstacle_3",
            "safe_v2_guard_3",

            "PURPLE",

            makeColor(
                0.66,
                0.12,
                0.90,
                1.0
            ),

            0.120,
            0.080,
            0.320
        }
    );


    // ==================================================================
    // 加入三个不同尺寸不同颜色的物体
    // ==================================================================

    for (const auto &task :
         tasks)
    {
        if (!addColoredBox(
                scene,
                planning_frame,

                task.object_id,

                task.pick_x,
                task.pick_y,
                objectCenterZ(
                    task
                ),

                task.sx,
                task.sy,
                task.sz,

                task.color))
        {
            RCLCPP_ERROR(
                logger,
                "Unable to create %s.",
                task.object_id.c_str()
            );


            executor.cancel();
            spin_thread.join();
            rclcpp::shutdown();


            return 1;
        }


        waitWorld(
            scene,
            task.object_id,
            true
        );
    }


    std::this_thread::sleep_for(
        std::chrono::
            seconds(2)
    );


    printConfiguration(
        tasks
    );


    std::vector<TaskMetrics>
        all_metrics;


    const auto system_begin =
        std::chrono::
            steady_clock::now();


    // ==================================================================
    // 多任务循环
    // ==================================================================

    for (std::size_t index = 0;
         index < tasks.size();
         ++index)
    {
        const auto &task =
            tasks[index];


        TaskMetrics metrics;


        metrics.name =
            task.name;


        metrics.color =
            task.color_name;


        metrics.obstacle_required =
            task.obstacle_required;


        const auto task_begin =
            std::chrono::
                steady_clock::now();


        std::cout
            << "\n\n";


        std::cout
            << "========================================================================\n";


        std::cout
            << "                 "
            << task.name
            << " / "
            << task.color_name
            << "\n";


        std::cout
            << "========================================================================\n";


        std::cout
            << "Object size : "
            << task.sx * 1000.0
            << " x "
            << task.sy * 1000.0
            << " x "
            << task.sz * 1000.0
            << " mm\n";


        std::cout
            << "Pick XY     : ("
            << task.pick_x
            << ", "
            << task.pick_y
            << ")\n";


        std::cout
            << "Place XY    : ("
            << task.place_x
            << ", "
            << task.place_y
            << ")\n";


        // ==============================================================
        // 1. Open
        // ==============================================================

        std::cout
            << "\n[STATE 1] OPEN GRIPPER\n";


        if (!commandGripper(
                node,
                GRIPPER_OPEN))
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ==============================================================
        // 2. Pre-grasp
        // ==============================================================

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


        std::cout
            << "\n[STATE 2] PRE-GRASP\n";


        std::cout
            << "Target EE = ("
            << pregrasp.position.x
            << ", "
            << pregrasp.position.y
            << ", "
            << pregrasp.position.z
            << ")\n";


        double pre_error =
            999.0;


        std::string pre_planner;


        if (!moveToPoseSafe(
                move_group,
                pregrasp,
                ee_link,
                pre_error,
                pre_planner))
        {
            std::cout
                << "PRE-GRASP FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "Pre-grasp SUCCESS\n";


        std::cout
            << "Planner = "
            << pre_planner
            << "\n";


        std::cout
            << "EE error = "
            << pre_error * 1000.0
            << " mm\n";


        // ==============================================================
        // 3. Approach
        // ==============================================================

        std::cout
            << "\n[STATE 3] CARTESIAN APPROACH DOWN\n";


        double approach_fraction =
            0.0;


        double approach_error =
            999.0;


        if (!cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                -APPROACH_DISTANCE,

                approach_fraction,
                approach_error))
        {
            std::cout
                << "APPROACH FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        metrics.approach_fraction =
            approach_fraction *
            100.0;


        // ==============================================================
        // 4. Gripper close
        // ==============================================================

        std::cout
            << "\n[STATE 4] ADAPTIVE GRIPPER CLOSE\n";


        // 根据Y方向尺寸自动设置夹爪目标
        const double gripper_target =
            std::max(
                0.006,
                std::min(
                    0.034,
                    task.sy / 2.0
                    -
                    0.003
                )
            );


        std::cout
            << "Finger target = "
            << gripper_target
            << " m\n";


        if (!commandGripper(
                node,
                gripper_target))
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ==============================================================
        // 5. Attach
        // ==============================================================

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


        const bool attach_raw =
            move_group.attachObject(
                task.object_id,
                ee_link,
                touch_links
            );


        if (!attach_raw ||
            !waitAttached(
                scene,
                task.object_id,
                true,
                3.0))
        {
            std::cout
                << "ATTACH FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "Attach SUCCESS\n";


        // ==============================================================
        // 6. Lift
        // ==============================================================

        std::cout
            << "\n[STATE 6] HIGH CARTESIAN LIFT\n";


        double lift_fraction =
            0.0;


        double lift_error =
            999.0;


        if (!cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                LIFT_DISTANCE,

                lift_fraction,
                lift_error))
        {
            std::cout
                << "LIFT FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        metrics.lift_fraction =
            lift_fraction *
            100.0;


        const auto lifted_pose =
            move_group.getCurrentPose(
                ee_link
            ).pose;


        std::cout
            << "Lift EE Z = "
            << lifted_pose.position.z
            << " m\n";


        std::cout
            << "Clearance above table = "
            << lifted_pose.position.z
                   -
                   TABLE_TOP_Z
            << " m\n";


        // ==============================================================
        // Pre-place
        // ==============================================================

        geometry_msgs::msg::Pose
            preplace =
                home_pose;


        preplace.position.x =
            task.place_x;


        preplace.position.y =
            task.place_y;


        // 搬运保持当前高位
        preplace.position.z =
            lifted_pose.position.z;


        ObstacleCalibration
            calibrated_obstacle;


        // ==============================================================
        // 7. Obstacle
        // ==============================================================

        if (task.obstacle_required)
        {
            std::cout
                << "\n[STATE 7] OBSTACLE SAFETY CALIBRATION\n";


            calibrated_obstacle =
                calibrateObstacle(
                    move_group,
                    scene,

                    planning_frame,
                    ee_link,

                    task,

                    preplace
                );


            if (!calibrated_obstacle.success)
            {
                std::cout
                    << "SAFE OBSTACLE CALIBRATION FAILED\n";


                all_metrics.push_back(
                    metrics
                );

                break;
            }


            metrics.obstacle_verified =
                true;


            metrics.direct_fraction =
                calibrated_obstacle.direct_fraction
                *
                100.0;


            std::cout
                << "\nSAFE OBSTACLE ACCEPTED\n";


            std::cout
                << "Color = "
                << task.obstacle_color_name
                << "\n";


            std::cout
                << "Direct path = "
                << metrics.direct_fraction
                << "% BLOCKED\n";


            std::cout
                << "Safety guard = +"
                << OBSTACLE_MARGIN_XY *
                       1000.0
                << " mm per XY side\n";


            std::cout
                << "Stable planner = "
                << calibrated_obstacle.verified_planner
                << "\n";


            std::cout
                << "\nObserve the safety obstacle for 2 seconds...\n";


            std::this_thread::sleep_for(
                std::chrono::
                    seconds(2)
            );
        }
        else
        {
            metrics.obstacle_verified =
                true;


            metrics.direct_fraction =
                100.0;
        }


        // ==============================================================
        // 8. Formal transfer
        // ==============================================================

        std::cout
            << "\n[STATE 8] FORMAL COLLISION-FREE TRANSFER\n";


        const PlanResult transfer =
            selectBestPlan(
                move_group,
                preplace,
                ee_link
            );


        if (!transfer.success)
        {
            std::cout
                << "TRANSFER PLANNING FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        metrics.transfer_planner =
            transfer.planner;


        metrics.transfer_path_length =
            transfer.path_length;


        metrics.transfer_planning_time =
            transfer.planning_time;


        std::cout
            << "Selected planner = "
            << transfer.planner
            << "\n";


        std::cout
            << "Joint path = "
            << transfer.path_length
            << " rad\n";


        double transfer_error =
            999.0;


        if (!executePlanToPose(
                move_group,
                transfer,
                preplace,
                ee_link,
                transfer_error))
        {
            std::cout
                << "TRANSFER EXECUTION FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        metrics.transfer_error_m =
            transfer_error;


        std::cout
            << "Transfer EE error = "
            << transfer_error *
                   1000.0
            << " mm\n";


        // ==============================================================
        // 9. Place descend
        // ==============================================================

        std::cout
            << "\n[STATE 9] CARTESIAN PLACE DESCEND\n";


        const double current_z =
            move_group.getCurrentPose(
                ee_link
            ).pose.position.z;


        const double target_z =
            graspZ(task);


        const double descend =
            current_z -
            target_z;


        std::cout
            << "Current EE Z = "
            << current_z
            << "\n";


        std::cout
            << "Target EE Z  = "
            << target_z
            << "\n";


        std::cout
            << "Descend      = "
            << descend
            << " m\n";


        double place_fraction =
            0.0;


        double place_error =
            999.0;


        if (descend <= 0.0 ||
            !cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                -descend,

                place_fraction,
                place_error))
        {
            std::cout
                << "PLACE DESCEND FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        metrics.place_fraction =
            place_fraction *
            100.0;


        // ==============================================================
        // 10. Release
        // ==============================================================

        std::cout
            << "\n[STATE 10] OPEN GRIPPER / RELEASE\n";


        if (!commandGripper(
                node,
                GRIPPER_OPEN))
        {
            all_metrics.push_back(
                metrics
            );

            break;
        }


        // ==============================================================
        // 11. Detach
        // ==============================================================

        std::cout
            << "\n[STATE 11] DETACH OBJECT\n";


        const bool detach_raw =
            move_group.detachObject(
                task.object_id
            );


        if (!detach_raw ||
            !waitAttached(
                scene,
                task.object_id,
                false,
                3.0))
        {
            std::cout
                << "DETACH FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        waitWorld(
            scene,
            task.object_id,
            true,
            3.0
        );


        // ==============================================================
        // Placement error
        // ==============================================================

        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(400)
        );


        const auto object_poses =
            scene.getObjectPoses(
                {task.object_id}
            );


        const auto found =
            object_poses.find(
                task.object_id
            );


        if (found !=
            object_poses.end())
        {
            geometry_msgs::msg::Pose expected;


            expected.orientation.w =
                1.0;


            expected.position.x =
                task.place_x;


            expected.position.y =
                task.place_y;


            expected.position.z =
                objectCenterZ(
                    task
                );


            metrics.placement_error_m =
                positionDistance(
                    found->second,
                    expected
                );
        }


        std::cout
            << "Placement error = "
            << metrics.placement_error_m *
                   1000.0
            << " mm\n";


        // ==============================================================
        // 12. Retreat
        // ==============================================================

        std::cout
            << "\n[STATE 12] VERTICAL SAFE RETREAT\n";


        double retreat_fraction =
            0.0;


        double retreat_error =
            999.0;


        if (!cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                RETREAT_DISTANCE,

                retreat_fraction,
                retreat_error))
        {
            std::cout
                << "RETREAT FAILED\n";


            all_metrics.push_back(
                metrics
            );

            break;
        }


        metrics.retreat_fraction =
            retreat_fraction *
            100.0;


        // ==============================================================
        // 当前任务结束后移除当前障碍物。
        // 已放置的物体继续保留在Planning Scene。
        // ==============================================================

        if (task.obstacle_required)
        {
            removeObject(
                scene,
                task.obstacle_id
            );


            removeObject(
                scene,
                task.guard_id
            );
        }


        metrics.total_time_s =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    task_begin
                ).count();


        metrics.success =
            metrics.approach_fraction >= 98.0
            &&
            metrics.lift_fraction >= 98.0
            &&
            metrics.place_fraction >= 98.0
            &&
            metrics.retreat_fraction >= 98.0
            &&
            metrics.obstacle_verified
            &&
            metrics.transfer_error_m <
                POSITION_TOLERANCE
            &&
            metrics.placement_error_m <
                0.025;


        all_metrics.push_back(
            metrics
        );


        std::cout
            << "\n------------------------------------------------------------\n";


        std::cout
            << task.name
            << " RESULT = "
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
            std::chrono::
                seconds(2)
        );
    }


    // ==================================================================
    // Return Home
    // ==================================================================

    std::cout
        << "\n\n";


    std::cout
        << "========================================================================\n";


    std::cout
        << "                           RETURN HOME\n";


    std::cout
        << "========================================================================\n";


    const bool home_success =
        moveHome(
            move_group,
            joint_group,
            home
        );


    // ==================================================================
    // Statistics
    // ==================================================================

    int success_count = 0;


    double error_sum_mm = 0.0;


    double max_error_mm = 0.0;


    int valid_error_count = 0;


    for (const auto &m :
         all_metrics)
    {
        if (m.success)
        {
            ++success_count;
        }


        if (m.placement_error_m <
            1.0)
        {
            const double mm =
                m.placement_error_m *
                1000.0;


            error_sum_mm += mm;


            max_error_mm =
                std::max(
                    max_error_mm,
                    mm
                );


            ++valid_error_count;
        }
    }


    const double mean_error_mm =
        valid_error_count > 0
        ?
        error_sum_mm /
            valid_error_count
        :
        999.0;


    const double system_time =
        std::chrono::
            duration<double>(
                std::chrono::
                    steady_clock::now()
                -
                system_begin
            ).count();


    printConfiguration(
        tasks
    );


    printResults(
        all_metrics
    );


    const bool final_success =
        success_count ==
            static_cast<int>(
                tasks.size()
            )
        &&
        home_success;


    std::cout
        << "\n\n";


    std::cout
        << "========================================================================\n";


    std::cout
        << "                       FINAL SAFETY SUMMARY\n";


    std::cout
        << "========================================================================\n";


    std::cout
        << "Successful tasks        : "
        << success_count
        << " / "
        << tasks.size()
        << "\n";


    std::cout
        << "Table size              : "
        << TABLE_SIZE_X
        << " x "
        << TABLE_SIZE_Y
        << " m\n";


    std::cout
        << "Table collision         : ENABLED\n";


    std::cout
        << "Cartesian collision     : ENABLED\n";


    std::cout
        << "Minimum wrist Z         : "
        << MIN_WRIST_HEIGHT
        << " m\n";


    std::cout
        << "Obstacle safety margin  : "
        << OBSTACLE_MARGIN_XY *
               1000.0
        << " mm each side\n";


    std::cout
        << "Task-2 Place Y          : "
        << tasks[1].place_y
        << " m\n";


    std::cout
        << "Task-3 Place Y          : "
        << tasks[2].place_y
        << " m\n";


    std::cout
        << "Mean placement error    : "
        << mean_error_mm
        << " mm\n";


    std::cout
        << "Max placement error     : "
        << max_error_mm
        << " mm\n";


    std::cout
        << "Total execution time    : "
        << system_time
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
        << "========================================================================\n";


    // ==================================================================
    // CSV
    // ==================================================================

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
        "multi_pick_and_place_safe_v2_result.csv";


    std::ofstream csv(
        csv_path
    );


    csv
        << "task,"
        << "color,"
        << "obstacle_required,"
        << "direct_fraction_percent,"
        << "approach_fraction_percent,"
        << "lift_fraction_percent,"
        << "transfer_planner,"
        << "transfer_path_rad,"
        << "transfer_planning_time_s,"
        << "transfer_error_m,"
        << "place_fraction_percent,"
        << "retreat_fraction_percent,"
        << "placement_error_m,"
        << "total_time_s,"
        << "success\n";


    for (const auto &m :
         all_metrics)
    {
        csv
            << m.name
            << ","
            << m.color
            << ","
            << (
                m.obstacle_required
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
            << m.transfer_path_length
            << ","
            << m.transfer_planning_time
            << ","
            << m.transfer_error_m
            << ","
            << m.place_fraction
            << ","
            << m.retreat_fraction
            << ","
            << m.placement_error_m
            << ","
            << m.total_time_s
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
        << "\nCSV saved:\n"
        << csv_path
        << "\n";


    std::cout
        << "\nFinal scene remains visible for 8 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::
            seconds(8)
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
