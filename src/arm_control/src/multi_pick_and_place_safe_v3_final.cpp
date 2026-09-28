#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>


// ======================================================================
//                        全局运动参数
// ======================================================================

constexpr double ARM_VEL = 0.09;
constexpr double ARM_ACC = 0.04;

constexpr double CART_VEL = 0.045;
constexpr double CART_ACC = 0.020;

constexpr double EEF_STEP = 0.004;
constexpr double MIN_CART_FRACTION = 0.98;

constexpr double POSITION_TOL = 0.018;
constexpr double HOME_TOL = 0.050;


// ======================================================================
//                         抓取参数
// ======================================================================

// 抓取前末端垂直下降
constexpr double APPROACH_DISTANCE = 0.070;

// 抓取完成后的总抬升高度
constexpr double LIFT_DISTANCE = 0.160;

// 放置后撤离
constexpr double RETREAT_DISTANCE = 0.110;

// 分段Cartesian最大单段距离
constexpr double CART_SEGMENT = 0.025;

// 每段失败最多重试
constexpr int CART_RETRY = 3;


// ======================================================================
//                         桌面
// ======================================================================

constexpr double TABLE_X = 0.62;
constexpr double TABLE_Y = 0.78;
constexpr double TABLE_Z = 0.10;

constexpr double TABLE_CENTER_Z = 0.15;

constexpr double TABLE_TOP_Z =
    TABLE_CENTER_Z + TABLE_Z / 2.0;


// 只留2mm数值间隙。
// RViz里看起来基本就是放在桌面上。
constexpr double OBJECT_TABLE_GAP = 0.002;


// ======================================================================
// Panda抓取几何
// ======================================================================

// panda_link8到抓取中心约使用105mm。
// 相比上一版150mm明显降低。
constexpr double GRASP_EE_Z_OFFSET = 0.105;


// 腕部硬安全高度
constexpr double MIN_WRIST_Z =
    TABLE_TOP_Z + 0.120;


// ======================================================================
// 障碍安全
// ======================================================================

constexpr double OBSTACLE_MARGIN_XY = 0.050;
constexpr double OBSTACLE_MARGIN_TOP = 0.040;

constexpr double BLOCK_THRESHOLD = 0.90;


// ======================================================================
// Gripper
// ======================================================================

constexpr double GRIPPER_OPEN = 0.040;
constexpr double GRIPPER_FORCE = 20.0;


// ======================================================================
// 数据结构
// ======================================================================

struct Task
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

    bool obstacle;

    std::string obstacle_id;
    std::string guard_id;
    std::string obstacle_color_name;

    std_msgs::msg::ColorRGBA obstacle_color;

    double obstacle_sx;
    double obstacle_sy;
    double obstacle_sz;
};


struct PlanCandidate
{
    bool success = false;

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;

    std::string planner = "-";

    double path_length = 999.0;
    double planning_time = 0.0;
};


struct ObstacleInfo
{
    bool success = false;

    double direct_fraction = 1.0;

    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;

    double guard_sx = 0.0;
    double guard_sy = 0.0;
    double guard_sz = 0.0;

    std::string planner = "-";
};


struct Result
{
    std::string task;
    std::string color;

    bool obstacle = false;

    double approach_fraction = 0.0;
    double lift_fraction = 0.0;
    double direct_fraction = -1.0;
    double place_fraction = 0.0;
    double retreat_fraction = 0.0;

    std::string transfer_planner = "-";

    double transfer_path = 0.0;
    double transfer_error = 999.0;

    double placement_error = 999.0;

    double task_time = 0.0;

    bool success = false;
};


// ======================================================================
// 工具
// ======================================================================

std_msgs::msg::ColorRGBA color(
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


double poseDistance(
    const geometry_msgs::msg::Pose &a,
    const geometry_msgs::msg::Pose &b)
{
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;

    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );
}


double objectCenterZ(
    const Task &t)
{
    return
        TABLE_TOP_Z
        +
        t.sz / 2.0
        +
        OBJECT_TABLE_GAP;
}


double graspEEZ(
    const Task &t)
{
    return
        objectCenterZ(t)
        +
        GRASP_EE_Z_OFFSET;
}


// ======================================================================
// Planning Scene
// ======================================================================

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
    moveit_msgs::msg::CollisionObject obj;

    obj.header.frame_id = frame;
    obj.id = id;

    shape_msgs::msg::SolidPrimitive box;

    box.type =
        shape_msgs::msg::SolidPrimitive::BOX;

    box.dimensions =
    {
        sx,
        sy,
        sz
    };

    geometry_msgs::msg::Pose pose;

    pose.orientation.w = 1.0;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);

    obj.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    return obj;
}


bool addBox(
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

    const std_msgs::msg::ColorRGBA &c)
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
        c
    );
}


bool worldExists(
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


bool attachedExists(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &id)
{
    return
        !scene.getAttachedObjects(
            {id}
        ).empty();
}


bool waitWorld(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &id,

    bool expected,

    double timeout = 3.0)
{
    const auto start =
        std::chrono::
            steady_clock::now();

    while (true)
    {
        if (worldExists(
                scene,
                id)
            ==
            expected)
        {
            return true;
        }

        if (std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    start
                ).count()
            >
            timeout)
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
    const auto start =
        std::chrono::
            steady_clock::now();

    while (true)
    {
        if (attachedExists(
                scene,
                id)
            ==
            expected)
        {
            return true;
        }

        if (std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    start
                ).count()
            >
            timeout)
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


// ======================================================================
// Gripper
// ======================================================================

bool gripper(
    const rclcpp::Node::SharedPtr &node,

    double position)
{
    using Action =
        control_msgs::action::
            GripperCommand;

    auto client =
        rclcpp_action::
            create_client<Action>(
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

    Action::Goal goal;

    goal.command.position =
        position;

    goal.command.max_effort =
        GRIPPER_FORCE;

    auto send =
        client->async_send_goal(goal);

    if (send.wait_for(
            std::chrono::
                seconds(4))
        !=
        std::future_status::ready)
    {
        return false;
    }

    auto handle =
        send.get();

    if (!handle)
    {
        return false;
    }

    auto result =
        client->async_get_result(
            handle
        );

    if (result.wait_for(
            std::chrono::
                seconds(8))
        !=
        std::future_status::ready)
    {
        return false;
    }

    return
        result.get().code
        ==
        rclcpp_action::
            ResultCode::SUCCEEDED;
}


// ======================================================================
// 获取可靠Current Pose
// ======================================================================

bool freshPose(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const std::string &ee_link,

    geometry_msgs::msg::Pose &pose)
{
    for (int i = 0;
         i < 8;
         ++i)
    {
        auto state =
            move_group.getCurrentState(
                1.0
            );

        if (state)
        {
            pose =
                move_group.getCurrentPose(
                    ee_link
                ).pose;

            return true;
        }

        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(250)
        );
    }

    return false;
}


// ======================================================================
// Trajectory 时间参数化
// ======================================================================

bool retime(
    const moveit::core::RobotStatePtr &state,

    const std::string &group,

    moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    robot_trajectory::
        RobotTrajectory robot_trajectory(
            state->getRobotModel(),
            group
        );

    robot_trajectory.setRobotTrajectoryMsg(
        *state,
        trajectory
    );

    trajectory_processing::
        IterativeParabolicTimeParameterization
            iptp;

    if (!iptp.computeTimeStamps(
            robot_trajectory,
            CART_VEL,
            CART_ACC))
    {
        return false;
    }

    robot_trajectory.getRobotTrajectoryMsg(
        trajectory
    );

    return true;
}


// ======================================================================
// 显示轨迹
// ======================================================================

void displayTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::core::RobotStatePtr &state,

    const moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    moveit_msgs::msg::
        DisplayTrajectory display;

    display.model_id =
        "panda";

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


// ======================================================================
// Joint path length
// ======================================================================

double jointPathLength(
    const moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    const auto &points =
        trajectory.joint_trajectory.points;

    if (points.size() < 2)
    {
        return 0.0;
    }

    double total = 0.0;

    for (std::size_t i = 1;
         i < points.size();
         ++i)
    {
        const auto &a =
            points[i - 1].positions;

        const auto &b =
            points[i].positions;

        double s = 0.0;

        const std::size_t n =
            std::min(
                a.size(),
                b.size()
            );

        for (std::size_t j = 0;
             j < n;
             ++j)
        {
            const double d =
                b[j] - a[j];

            s += d * d;
        }

        total +=
            std::sqrt(s);
    }

    return total;
}


// ======================================================================
// 生成规划
// ======================================================================

PlanCandidate makePlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    const std::string &planner_id,

    const std::string &planner_name,

    double planning_time)
{
    PlanCandidate result;

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
        1
    );

    move_group.setMaxVelocityScalingFactor(
        ARM_VEL
    );

    move_group.setMaxAccelerationScalingFactor(
        ARM_ACC
    );

    move_group.setStartStateToCurrentState();

    if (!move_group.setPoseTarget(
            target,
            ee_link))
    {
        return result;
    }

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;

    const auto start =
        std::chrono::
            steady_clock::now();

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

    if (!success)
    {
        return result;
    }

    result.success =
        true;

    result.plan =
        plan;

    result.planner =
        planner_name;

    result.path_length =
        jointPathLength(
            plan.trajectory_
        );

    result.planning_time =
        std::chrono::
            duration<double>(
                end - start
            ).count();

    return result;
}


// ======================================================================
// RRTConnect x3 + RRT* x2
// ======================================================================

PlanCandidate bestPlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link)
{
    std::vector<PlanCandidate>
        candidates;

    for (int i = 0;
         i < 3;
         ++i)
    {
        auto p =
            makePlan(
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
            makePlan(
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
        PlanCandidate empty;

        return empty;
    }

    return
        *std::min_element(
            candidates.begin(),
            candidates.end(),

            [](
                const PlanCandidate &a,
                const PlanCandidate &b)
            {
                return
                    a.path_length <
                    b.path_length;
            }
        );
}


// ======================================================================
// 执行普通规划
// ======================================================================

bool executePosePlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const PlanCandidate &plan,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    double &error)
{
    if (!plan.success)
    {
        return false;
    }

    const bool raw =
        static_cast<bool>(
            move_group.execute(
                plan.plan
            )
        );

    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(500)
    );

    geometry_msgs::msg::Pose current;

    if (freshPose(
            move_group,
            ee_link,
            current))
    {
        error =
            poseDistance(
                current,
                target
            );

        if (error <
            POSITION_TOL)
        {
            return true;
        }
    }

    return raw;
}


// ======================================================================
// Home
// ======================================================================

bool moveHome(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const std::vector<double> &home)
{
    for (int attempt = 1;
         attempt <= 4;
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
            8
        );

        move_group.setMaxVelocityScalingFactor(
            ARM_VEL
        );

        move_group.setMaxAccelerationScalingFactor(
            ARM_ACC
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

        if (static_cast<bool>(
                move_group.execute(
                    plan
                )))
        {
            std::this_thread::sleep_for(
                std::chrono::
                    milliseconds(500)
            );

            return true;
        }
    }

    return false;
}


// ======================================================================
// 单个Cartesian绝对目标
//
// 失败时：
// 1. 检查是否其实已经到达
// 2. 等待joint state
// 3. 自动重新计算
// ======================================================================

bool cartesianToPose(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const std::string &group,

    const std::string &ee_link,

    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    const geometry_msgs::msg::Pose &target,

    double &fraction,

    double &error)
{
    if (target.position.z <
        MIN_WRIST_Z)
    {
        std::cout
            << "[SAFETY] Target rejected: wrist Z too low\n";

        return false;
    }


    for (int retry = 1;
         retry <= CART_RETRY;
         ++retry)
    {
        std::cout
            << "Cartesian attempt "
            << retry
            << "/"
            << CART_RETRY
            << "\n";


        geometry_msgs::msg::Pose current;

        if (!freshPose(
                move_group,
                ee_link,
                current))
        {
            std::this_thread::sleep_for(
                std::chrono::
                    milliseconds(500)
            );

            continue;
        }


        const double before_error =
            poseDistance(
                current,
                target
            );


        if (before_error <
            0.002)
        {
            fraction = 1.0;
            error = before_error;

            return true;
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


        fraction =
            move_group.computeCartesianPath(
                waypoints,
                EEF_STEP,
                0.0,
                trajectory,
                true
            );


        std::cout
            << "  fraction = "
            << fraction * 100.0
            << "%\n";


        if (fraction <
            MIN_CART_FRACTION)
        {
            std::this_thread::sleep_for(
                std::chrono::
                    milliseconds(400)
            );

            continue;
        }


        auto state =
            move_group.getCurrentState(
                2.0
            );


        if (!state)
        {
            continue;
        }


        if (!retime(
                state,
                group,
                trajectory))
        {
            continue;
        }


        displayTrajectory(
            publisher,
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
            std::chrono::
                milliseconds(500)
        );


        geometry_msgs::msg::Pose after;


        if (freshPose(
                move_group,
                ee_link,
                after))
        {
            error =
                poseDistance(
                    after,
                    target
                );


            std::cout
                << "  final error = "
                << error * 1000.0
                << " mm\n";


            // 即使controller反馈abort，
            // 实际已经到位也判成功。
            if (error <
                POSITION_TOL)
            {
                return true;
            }
        }


        if (!raw)
        {
            std::cout
                << "  execution aborted -> automatic retry\n";
        }


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(600)
        );
    }


    return false;
}


// ======================================================================
// 分段Cartesian
//
// 不再一次性Lift 160mm。
// 每段最多25mm。
// ======================================================================

bool segmentedCartesian(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const std::string &group,

    const std::string &ee_link,

    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    double dx,
    double dy,
    double dz,

    double &minimum_fraction)
{
    geometry_msgs::msg::Pose start;

    if (!freshPose(
            move_group,
            ee_link,
            start))
    {
        return false;
    }


    geometry_msgs::msg::Pose final_target =
        start;


    final_target.position.x += dx;
    final_target.position.y += dy;
    final_target.position.z += dz;


    const double distance =
        std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        );


    const int segments =
        std::max(
            1,
            static_cast<int>(
                std::ceil(
                    distance /
                    CART_SEGMENT
                )
            )
        );


    minimum_fraction =
        1.0;


    std::cout
        << "Segmented Cartesian: "
        << segments
        << " segments\n";


    for (int i = 1;
         i <= segments;
         ++i)
    {
        const double alpha =
            static_cast<double>(i)
            /
            static_cast<double>(
                segments
            );


        geometry_msgs::msg::Pose target =
            start;


        target.position.x =
            start.position.x +
            alpha * dx;


        target.position.y =
            start.position.y +
            alpha * dy;


        target.position.z =
            start.position.z +
            alpha * dz;


        double fraction =
            0.0;


        double error =
            999.0;


        std::cout
            << "  Segment "
            << i
            << "/"
            << segments
            << "\n";


        if (!cartesianToPose(
                move_group,
                group,
                ee_link,
                publisher,
                target,
                fraction,
                error))
        {
            // --------------------------------------------------
            // Cartesian连续失败：
            // 自动使用碰撞安全采样规划到该段目标。
            // --------------------------------------------------

            std::cout
                << "  Cartesian segment failed.\n";


            std::cout
                << "  Trying collision-free planner fallback...\n";


            auto fallback =
                bestPlan(
                    move_group,
                    target,
                    ee_link
                );


            double fallback_error =
                999.0;


            if (!executePosePlan(
                    move_group,
                    fallback,
                    target,
                    ee_link,
                    fallback_error))
            {
                return false;
            }


            std::cout
                << "  Fallback planner = "
                << fallback.planner
                << "\n";


            std::cout
                << "  Fallback error   = "
                << fallback_error *
                       1000.0
                << " mm\n";


            fraction =
                1.0;
        }


        minimum_fraction =
            std::min(
                minimum_fraction,
                fraction
            );
    }


    geometry_msgs::msg::Pose final_pose;


    if (freshPose(
            move_group,
            ee_link,
            final_pose))
    {
        const double final_error =
            poseDistance(
                final_pose,
                final_target
            );


        std::cout
            << "Segmented motion final error = "
            << final_error * 1000.0
            << " mm\n";


        return
            final_error <
            POSITION_TOL;
    }


    return false;
}


// ======================================================================
// Direct Cartesian Test
// ======================================================================

double directFraction(
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


// ======================================================================
// 添加实际障碍 + 安全碰撞包络
// ======================================================================

bool addObstacleGuard(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &frame,

    const Task &task,

    double x,
    double y,

    double obstacle_height,

    double &guard_sx,
    double &guard_sy,
    double &guard_sz)
{
    const double obstacle_z =
        TABLE_TOP_Z +
        obstacle_height / 2.0;


    if (!addBox(
            scene,
            frame,

            task.obstacle_id,

            x,
            y,
            obstacle_z,

            task.obstacle_sx,
            task.obstacle_sy,
            obstacle_height,

            task.obstacle_color))
    {
        return false;
    }


    guard_sx =
        task.obstacle_sx +
        2.0 * OBSTACLE_MARGIN_XY;


    guard_sy =
        task.obstacle_sy +
        2.0 * OBSTACLE_MARGIN_XY;


    guard_sz =
        obstacle_height +
        OBSTACLE_MARGIN_TOP;


    const double guard_z =
        TABLE_TOP_Z +
        guard_sz / 2.0;


    auto guard_color =
        task.obstacle_color;


    guard_color.a =
        0.06f;


    if (!addBox(
            scene,
            frame,

            task.guard_id,

            x,
            y,
            guard_z,

            guard_sx,
            guard_sy,
            guard_sz,

            guard_color))
    {
        return false;
    }


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(500)
    );


    return true;
}


// ======================================================================
// 障碍自动校准
// ======================================================================

ObstacleInfo calibrateObstacle(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &frame,

    const std::string &ee_link,

    const Task &task,

    const geometry_msgs::msg::Pose &target)
{
    ObstacleInfo info;


    geometry_msgs::msg::Pose start;


    if (!freshPose(
            move_group,
            ee_link,
            start))
    {
        return info;
    }


    const std::vector<double> ratios =
    {
        0.48,
        0.52,
        0.44,
        0.56
    };


    const std::vector<double> xoffset =
    {
        0.0,
        0.025,
        -0.025
    };


    for (double ratio :
         ratios)
    {
        for (double xo :
             xoffset)
        {
            removeObject(
                scene,
                task.obstacle_id
            );


            removeObject(
                scene,
                task.guard_id
            );


            const double x =
                start.position.x
                +
                ratio *
                (
                    target.position.x -
                    start.position.x
                )
                +
                xo;


            const double y =
                start.position.y
                +
                ratio *
                (
                    target.position.y -
                    start.position.y
                );


            // 障碍顶部略高于当前搬运末端，
            // 迫使机械臂侧向绕行。
            const double required_height =
                std::max(
                    task.obstacle_sz,
                    start.position.z
                    -
                    TABLE_TOP_Z
                    +
                    0.015
                );


            double gsx = 0.0;
            double gsy = 0.0;
            double gsz = 0.0;


            if (!addObstacleGuard(
                    scene,
                    frame,
                    task,
                    x,
                    y,
                    required_height,
                    gsx,
                    gsy,
                    gsz))
            {
                continue;
            }


            const double direct =
                directFraction(
                    move_group,
                    target
                );


            std::cout
                << "\nObstacle test\n";


            std::cout
                << "  Direct fraction = "
                << direct * 100.0
                << "%\n";


            std::cout
                << "  Visible obstacle = "
                << task.obstacle_sx *
                       1000.0
                << " x "
                << task.obstacle_sy *
                       1000.0
                << " x "
                << required_height *
                       1000.0
                << " mm\n";


            std::cout
                << "  Safety envelope  = "
                << gsx * 1000.0
                << " x "
                << gsy * 1000.0
                << " x "
                << gsz * 1000.0
                << " mm\n";


            if (direct >=
                BLOCK_THRESHOLD)
            {
                continue;
            }


            // 要求至少存在一条有效安全绕障轨迹
            auto plan =
                bestPlan(
                    move_group,
                    target,
                    ee_link
                );


            if (!plan.success)
            {
                continue;
            }


            info.success =
                true;


            info.direct_fraction =
                direct;


            info.sx =
                task.obstacle_sx;

            info.sy =
                task.obstacle_sy;

            info.sz =
                required_height;


            info.guard_sx =
                gsx;

            info.guard_sy =
                gsy;

            info.guard_sz =
                gsz;


            info.planner =
                plan.planner;


            return info;
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


    return info;
}


// ======================================================================
// 打印结果
// ======================================================================

void printResults(
    const std::vector<Result> &results)
{
    std::cout
        << "\n\n";


    std::cout
        << "====================================================================================================================================\n";


    std::cout
        << "                                             SAFE V3 MULTI-TASK RESULTS\n";


    std::cout
        << "====================================================================================================================================\n";


    std::cout
        << std::left
        << std::setw(10)
        << "Task"

        << std::setw(10)
        << "Color"

        << std::setw(16)
        << "Direct"

        << std::setw(16)
        << "Planner"

        << std::setw(15)
        << "Path(rad)"

        << std::setw(18)
        << "EE Error(mm)"

        << std::setw(20)
        << "Place Error(mm)"

        << "Result\n";


    std::cout
        << "------------------------------------------------------------------------------------------------------------------------------------\n";


    for (const auto &r :
         results)
    {
        std::ostringstream direct;


        if (!r.obstacle)
        {
            direct
                << "FREE";
        }
        else if (r.direct_fraction <
                 0.0)
        {
            direct
                << "N/A";
        }
        else
        {
            direct
                << std::fixed
                << std::setprecision(1)
                << r.direct_fraction
                << "% BLOCK";
        }


        std::cout
            << std::setw(10)
            << r.task

            << std::setw(10)
            << r.color

            << std::setw(16)
            << direct.str()

            << std::setw(16)
            << r.transfer_planner

            << std::fixed
            << std::setprecision(4)

            << std::setw(15)
            << r.transfer_path

            << std::setw(18)
            << r.transfer_error *
                   1000.0

            << std::setw(20)
            << r.placement_error *
                   1000.0

            << (
                r.success
                ?
                "SUCCESS"
                :
                "FAILED"
            )

            << "\n";
    }


    std::cout
        << "====================================================================================================================================\n";
}


// ======================================================================
// MAIN
// ======================================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<
            rclcpp::Node
        >(
            "multi_pick_and_place_safe_v3",

            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true
                )
        );


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


    const std::string group =
        "panda_arm";


    moveit::planning_interface::
        MoveGroupInterface move_group(
            node,
            group
        );


    moveit::planning_interface::
        PlanningSceneInterface scene;


    move_group.allowReplanning(
        true
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


    const std::string frame =
        move_group.getPlanningFrame();


    auto display_pub =
        node->create_publisher<
            moveit_msgs::msg::
                DisplayTrajectory
        >(
            "/display_planned_path",
            10
        );


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


    // ==================================================================
    // 清场
    // ==================================================================

    const std::vector<std::string>
        cleanup =
    {
        "v3_table",
        "v3_red",
        "v3_green",
        "v3_blue",

        "v3_obstacle2",
        "v3_guard2",

        "v3_obstacle3",
        "v3_guard3",

        "safe_v2_table",
        "safe_v2_red",
        "safe_v2_green",
        "safe_v2_blue",

        "safe_v2_obstacle_2",
        "safe_v2_guard_2",

        "safe_v2_obstacle_3",
        "safe_v2_guard_3",

        "multi_table",
        "red_object",
        "green_object",
        "blue_object",

        "task2_obstacle",
        "task3_obstacle"
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
            milliseconds(600)
    );


    // ==================================================================
    // Home
    // ==================================================================

    std::cout
        << "\n============================================================\n";


    std::cout
        << "SAFE V3 : RETURN / ESTABLISH HOME\n";


    std::cout
        << "============================================================\n";


    if (!moveHome(
            move_group,
            home))
    {
        std::cerr
            << "Home failed.\n";

        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();

        return 1;
    }


    gripper(
        node,
        GRIPPER_OPEN
    );


    geometry_msgs::msg::Pose
        home_pose;


    if (!freshPose(
            move_group,
            ee_link,
            home_pose))
    {
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();

        return 1;
    }


    // ==================================================================
    // Table
    // ==================================================================

    const double table_x =
        home_pose.position.x +
        0.20;


    if (!addBox(
            scene,
            frame,

            "v3_table",

            table_x,
            0.0,
            TABLE_CENTER_Z,

            TABLE_X,
            TABLE_Y,
            TABLE_Z,

            color(
                0.40,
                0.42,
                0.46,
                1.0
            )))
    {
        return 1;
    }


    waitWorld(
        scene,
        "v3_table",
        true
    );


    // ==================================================================
    // 三任务
    // ==================================================================

    std::vector<Task> tasks;


    // RED
    tasks.push_back(
    {
        "Task-1",
        "v3_red",
        "RED",

        color(
            0.95,
            0.05,
            0.05,
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

        color(
            1.0,
            1.0,
            1.0,
            1.0
        ),

        0.0,
        0.0,
        0.0
    });


    // GREEN
    tasks.push_back(
    {
        "Task-2",
        "v3_green",
        "GREEN",

        color(
            0.05,
            0.88,
            0.18,
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

        "v3_obstacle2",
        "v3_guard2",
        "YELLOW",

        color(
            0.98,
            0.70,
            0.05,
            1.0
        ),

        0.095,
        0.065,
        0.280
    });


    // BLUE
    tasks.push_back(
    {
        "Task-3",
        "v3_blue",
        "BLUE",

        color(
            0.05,
            0.28,
            0.98,
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

        "v3_obstacle3",
        "v3_guard3",
        "PURPLE",

        color(
            0.65,
            0.10,
            0.90,
            1.0
        ),

        0.110,
        0.075,
        0.300
    });


    // ==================================================================
    // 加入物体
    // ==================================================================

    for (const auto &t :
         tasks)
    {
        addBox(
            scene,
            frame,
            t.object_id,

            t.pick_x,
            t.pick_y,
            objectCenterZ(t),

            t.sx,
            t.sy,
            t.sz,

            t.color
        );


        waitWorld(
            scene,
            t.object_id,
            true
        );
    }


    std::cout
        << "\n============================================================\n";


    std::cout
        << "SAFE V3 KEY PARAMETERS\n";


    std::cout
        << "============================================================\n";


    std::cout
        << "Table               : 620 x 780 mm\n";


    std::cout
        << "Object-table gap    : "
        << OBJECT_TABLE_GAP *
               1000.0
        << " mm\n";


    std::cout
        << "Grasp EE offset     : "
        << GRASP_EE_Z_OFFSET *
               1000.0
        << " mm\n";


    std::cout
        << "Approach            : "
        << APPROACH_DISTANCE *
               1000.0
        << " mm\n";


    std::cout
        << "Lift                : "
        << LIFT_DISTANCE *
               1000.0
        << " mm\n";


    std::cout
        << "Cartesian segment   : <= "
        << CART_SEGMENT *
               1000.0
        << " mm\n";


    std::cout
        << "Obstacle margin     : "
        << OBSTACLE_MARGIN_XY *
               1000.0
        << " mm each side\n";


    std::cout
        << "============================================================\n";


    std::vector<Result>
        results;


    // ==================================================================
    // 多任务循环
    // ==================================================================

    for (const auto &task :
         tasks)
    {
        Result result;

        result.task =
            task.name;

        result.color =
            task.color_name;

        result.obstacle =
            task.obstacle;


        const auto task_start =
            std::chrono::
                steady_clock::now();


        std::cout
            << "\n\n============================================================\n";


        std::cout
            << task.name
            << " / "
            << task.color_name
            << "\n";


        std::cout
            << "============================================================\n";


        std::cout
            << "Object size = "
            << task.sx * 1000.0
            << " x "
            << task.sy * 1000.0
            << " x "
            << task.sz * 1000.0
            << " mm\n";


        // ==============================================================
        // OPEN
        // ==============================================================

        std::cout
            << "\n[1] OPEN GRIPPER\n";


        if (!gripper(
                node,
                GRIPPER_OPEN))
        {
            results.push_back(
                result
            );

            break;
        }


        // ==============================================================
        // PRE-GRASP
        // ==============================================================

        geometry_msgs::msg::Pose
            pregrasp =
                home_pose;


        pregrasp.position.x =
            task.pick_x;


        pregrasp.position.y =
            task.pick_y;


        pregrasp.position.z =
            graspEEZ(task)
            +
            APPROACH_DISTANCE;


        std::cout
            << "\n[2] MOVE TO PRE-GRASP\n";


        auto pre_plan =
            bestPlan(
                move_group,
                pregrasp,
                ee_link
            );


        double pre_error =
            999.0;


        if (!executePosePlan(
                move_group,
                pre_plan,
                pregrasp,
                ee_link,
                pre_error))
        {
            std::cout
                << "PRE-GRASP FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        std::cout
            << "Planner = "
            << pre_plan.planner
            << "\n";


        std::cout
            << "EE error = "
            << pre_error * 1000.0
            << " mm\n";


        // ==============================================================
        // APPROACH
        // ==============================================================

        std::cout
            << "\n[3] SLOW SEGMENTED APPROACH\n";


        double approach_fraction =
            0.0;


        if (!segmentedCartesian(
                move_group,
                group,
                ee_link,
                display_pub,

                0.0,
                0.0,
                -APPROACH_DISTANCE,

                approach_fraction))
        {
            std::cout
                << "APPROACH FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        result.approach_fraction =
            approach_fraction *
            100.0;


        // ==============================================================
        // 真正贴近式Gripper Close
        // ==============================================================

        std::cout
            << "\n[4] REALISTIC GRIPPER CLOSE\n";


        // Panda两根手指对称。
        // 每根手指位置约为物体宽度/2。
        const double contact_position =
            std::min(
                0.039,
                task.sy / 2.0
            );


        std::cout
            << "Object width        = "
            << task.sy * 1000.0
            << " mm\n";


        std::cout
            << "Finger target each  = "
            << contact_position *
                   1000.0
            << " mm\n";


        if (!gripper(
                node,
                contact_position))
        {
            results.push_back(
                result
            );

            break;
        }


        // 让RViz中夹爪闭合动作停顿一下，
        // 可以清楚看到两根手指夹到物体两侧。
        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    1000
                )
        );


        // ==============================================================
        // ATTACH
        // ==============================================================

        std::cout
            << "\n[5] ATTACH AFTER CONTACT\n";


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


        const bool attached =
            attach_raw
            &&
            waitAttached(
                scene,
                task.object_id,
                true,
                3.0
            );


        // Attach以后世界中的原物体应该消失，
        // 避免World Object与Attached Object重叠。
        waitWorld(
            scene,
            task.object_id,
            false,
            3.0
        );


        if (!attached)
        {
            std::cout
                << "ATTACH FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        std::cout
            << "CONTACT + ATTACH SUCCESS\n";


        // 给Planning Scene和joint state充分同步时间
        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    800
                )
        );


        // ==============================================================
        // SEGMENTED LIFT
        // ==============================================================

        std::cout
            << "\n[6] SEGMENTED SAFE LIFT\n";


        std::cout
            << "Instead of one "
            << LIFT_DISTANCE *
                   1000.0
            << " mm motion,\n";


        std::cout
            << "lift is divided into <= "
            << CART_SEGMENT *
                   1000.0
            << " mm segments.\n";


        double lift_fraction =
            0.0;


        if (!segmentedCartesian(
                move_group,
                group,
                ee_link,
                display_pub,

                0.0,
                0.0,
                LIFT_DISTANCE,

                lift_fraction))
        {
            std::cout
                << "LIFT FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        result.lift_fraction =
            lift_fraction *
            100.0;


        geometry_msgs::msg::Pose
            lifted;


        freshPose(
            move_group,
            ee_link,
            lifted
        );


        // ==============================================================
        // PRE-PLACE
        // ==============================================================

        geometry_msgs::msg::Pose
            preplace =
                home_pose;


        preplace.position.x =
            task.place_x;


        preplace.position.y =
            task.place_y;


        preplace.position.z =
            lifted.position.z;


        // ==============================================================
        // OBSTACLE
        // ==============================================================

        if (task.obstacle)
        {
            std::cout
                << "\n[7] CREATE SAFE OBSTACLE\n";


            const auto obstacle =
                calibrateObstacle(
                    move_group,
                    scene,
                    frame,
                    ee_link,
                    task,
                    preplace
                );


            if (!obstacle.success)
            {
                std::cout
                    << "OBSTACLE CALIBRATION FAILED\n";

                results.push_back(
                    result
                );

                break;
            }


            result.direct_fraction =
                obstacle.direct_fraction *
                100.0;


            std::cout
                << "Direct path = "
                << result.direct_fraction
                << "% -> BLOCKED\n";


            std::cout
                << "Visible obstacle: "
                << obstacle.sx *
                       1000.0
                << " x "
                << obstacle.sy *
                       1000.0
                << " x "
                << obstacle.sz *
                       1000.0
                << " mm\n";


            std::cout
                << "Safety envelope: "
                << obstacle.guard_sx *
                       1000.0
                << " x "
                << obstacle.guard_sy *
                       1000.0
                << " x "
                << obstacle.guard_sz *
                       1000.0
                << " mm\n";


            std::this_thread::sleep_for(
                std::chrono::
                    seconds(2)
            );
        }


        // ==============================================================
        // COLLISION-FREE TRANSFER
        // ==============================================================

        std::cout
            << "\n[8] COLLISION-FREE OBJECT TRANSFER\n";


        auto transfer =
            bestPlan(
                move_group,
                preplace,
                ee_link
            );


        if (!transfer.success)
        {
            std::cout
                << "TRANSFER PLAN FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        result.transfer_planner =
            transfer.planner;


        result.transfer_path =
            transfer.path_length;


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


        if (!executePosePlan(
                move_group,
                transfer,
                preplace,
                ee_link,
                transfer_error))
        {
            std::cout
                << "TRANSFER EXECUTION FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        result.transfer_error =
            transfer_error;


        // ==============================================================
        // PLACE DESCEND
        // ==============================================================

        std::cout
            << "\n[9] SEGMENTED PLACE DESCEND\n";


        geometry_msgs::msg::Pose
            before_place;


        freshPose(
            move_group,
            ee_link,
            before_place
        );


        const double descend =
            before_place.position.z
            -
            graspEEZ(task);


        double place_fraction =
            0.0;


        if (descend <= 0.0
            ||
            !segmentedCartesian(
                move_group,
                group,
                ee_link,
                display_pub,

                0.0,
                0.0,
                -descend,

                place_fraction))
        {
            std::cout
                << "PLACE DESCEND FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        result.place_fraction =
            place_fraction *
            100.0;


        // ==============================================================
        // RELEASE
        // ==============================================================

        std::cout
            << "\n[10] OPEN GRIPPER\n";


        if (!gripper(
                node,
                GRIPPER_OPEN))
        {
            results.push_back(
                result
            );

            break;
        }


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(700)
        );


        // ==============================================================
        // DETACH
        // ==============================================================

        std::cout
            << "\n[11] DETACH OBJECT\n";


        const bool detach_raw =
            move_group.detachObject(
                task.object_id
            );


        const bool detached =
            detach_raw
            &&
            waitAttached(
                scene,
                task.object_id,
                false,
                3.0
            );


        waitWorld(
            scene,
            task.object_id,
            true,
            3.0
        );


        if (!detached)
        {
            results.push_back(
                result
            );

            break;
        }


        // ==============================================================
        // Placement error
        // ==============================================================

        const auto poses =
            scene.getObjectPoses(
                {task.object_id}
            );


        const auto it =
            poses.find(
                task.object_id
            );


        if (it !=
            poses.end())
        {
            geometry_msgs::msg::Pose expected;

            expected.orientation.w = 1.0;

            expected.position.x =
                task.place_x;

            expected.position.y =
                task.place_y;

            expected.position.z =
                objectCenterZ(task);


            result.placement_error =
                poseDistance(
                    it->second,
                    expected
                );
        }


        std::cout
            << "Placement error = "
            << result.placement_error *
                   1000.0
            << " mm\n";


        // ==============================================================
        // RETREAT
        // ==============================================================

        std::cout
            << "\n[12] SEGMENTED RETREAT\n";


        double retreat_fraction =
            0.0;


        if (!segmentedCartesian(
                move_group,
                group,
                ee_link,
                display_pub,

                0.0,
                0.0,
                RETREAT_DISTANCE,

                retreat_fraction))
        {
            std::cout
                << "RETREAT FAILED\n";

            results.push_back(
                result
            );

            break;
        }


        result.retreat_fraction =
            retreat_fraction *
            100.0;


        // 当前障碍任务结束后删除障碍及安全包络。
        if (task.obstacle)
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


        result.task_time =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    task_start
                ).count();


        result.success =
            result.approach_fraction >=
                98.0
            &&
            result.lift_fraction >=
                98.0
            &&
            result.place_fraction >=
                98.0
            &&
            result.retreat_fraction >=
                98.0
            &&
            result.transfer_error <
                POSITION_TOL
            &&
            result.placement_error <
                0.025;


        results.push_back(
            result
        );


        std::cout
            << "\n------------------------------------------------------------\n";


        std::cout
            << task.name
            << " RESULT : "
            << (
                result.success
                ?
                "SUCCESS"
                :
                "FAILED"
            )
            << "\n";


        std::cout
            << "------------------------------------------------------------\n";


        if (!result.success)
        {
            break;
        }


        std::this_thread::sleep_for(
            std::chrono::
                seconds(2)
        );
    }


    // ==================================================================
    // HOME
    // ==================================================================

    std::cout
        << "\n============================================================\n";


    std::cout
        << "RETURN SAFE HOME\n";


    std::cout
        << "============================================================\n";


    const bool home_success =
        moveHome(
            move_group,
            home
        );


    // ==================================================================
    // SUMMARY
    // ==================================================================

    printResults(
        results
    );


    int successes = 0;

    double mean_error = 0.0;

    double max_error = 0.0;

    int valid_errors = 0;


    for (const auto &r :
         results)
    {
        if (r.success)
        {
            ++successes;
        }


        if (r.placement_error <
            1.0)
        {
            const double mm =
                r.placement_error *
                1000.0;


            mean_error += mm;

            max_error =
                std::max(
                    max_error,
                    mm
                );

            ++valid_errors;
        }
    }


    if (valid_errors > 0)
    {
        mean_error /=
            valid_errors;
    }


    const bool overall =
        successes == 3
        &&
        home_success;


    std::cout
        << "\n============================================================\n";


    std::cout
        << "FINAL SAFE V3 SUMMARY\n";


    std::cout
        << "============================================================\n";


    std::cout
        << "Successful tasks       : "
        << successes
        << " / 3\n";


    std::cout
        << "Object-table gap       : "
        << OBJECT_TABLE_GAP *
               1000.0
        << " mm\n";


    std::cout
        << "Grasp EE offset        : "
        << GRASP_EE_Z_OFFSET *
               1000.0
        << " mm\n";


    std::cout
        << "Cartesian segmentation : "
        << CART_SEGMENT *
               1000.0
        << " mm max/segment\n";


    std::cout
        << "Obstacle safety margin : "
        << OBSTACLE_MARGIN_XY *
               1000.0
        << " mm each side\n";


    std::cout
        << "Mean placement error   : "
        << mean_error
        << " mm\n";


    std::cout
        << "Max placement error    : "
        << max_error
        << " mm\n";


    std::cout
        << "Return Home            : "
        << (
            home_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "OVERALL RESULT         : "
        << (
            overall
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "============================================================\n";


    // ==================================================================
    // CSV
    // ==================================================================

    const char *home_env =
        std::getenv(
            "HOME"
        );


    std::string root =
        home_env
        ?
        home_env
        :
        "/home/yj";


    const std::string csv_path =
        root
        +
        "/Robot_Arm_Project/data/"
        "multi_pick_and_place_safe_v3_result.csv";


    std::ofstream csv(
        csv_path
    );


    csv
        << "task,"
        << "color,"
        << "obstacle,"
        << "direct_fraction_percent,"
        << "approach_fraction_percent,"
        << "lift_fraction_percent,"
        << "transfer_planner,"
        << "transfer_path_rad,"
        << "transfer_error_m,"
        << "place_fraction_percent,"
        << "retreat_fraction_percent,"
        << "placement_error_m,"
        << "task_time_s,"
        << "success\n";


    for (const auto &r :
         results)
    {
        csv
            << r.task
            << ","
            << r.color
            << ","
            << (
                r.obstacle
                ?
                1
                :
                0
            )
            << ","
            << r.direct_fraction
            << ","
            << r.approach_fraction
            << ","
            << r.lift_fraction
            << ","
            << r.transfer_planner
            << ","
            << r.transfer_path
            << ","
            << r.transfer_error
            << ","
            << r.place_fraction
            << ","
            << r.retreat_fraction
            << ","
            << r.placement_error
            << ","
            << r.task_time
            << ","
            << (
                r.success
                ?
                1
                :
                0
            )
            << "\n";
    }


    csv.close();


    std::cout
        << "\nCSV: "
        << csv_path
        << "\n";


    std::cout
        << "\nHolding final scene for 8 seconds...\n";


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
        overall
        ?
        0
        :
        1;
}
