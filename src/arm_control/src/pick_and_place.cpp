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
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>

#include <geometry_msgs/msg/pose.hpp>

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


// ============================================================
//                 Pick & Place 核心参数
// ============================================================


// ----------------------
// ARM
// ----------------------

constexpr double ARM_VELOCITY =
    0.12;

constexpr double ARM_ACCELERATION =
    0.05;


// ----------------------
// Cartesian
// ----------------------

constexpr double CART_VELOCITY =
    0.08;

constexpr double CART_ACCELERATION =
    0.03;

constexpr double EEF_STEP =
    0.005;

constexpr double MIN_CART_FRACTION =
    0.98;


// ----------------------
// 距离
// ----------------------

// Pre-grasp -> grasp
constexpr double APPROACH_DISTANCE =
    0.12;

// 抓取之后向上抬
constexpr double LIFT_DISTANCE =
    0.14;


// ----------------------
// 误差阈值
// ----------------------

constexpr double POSITION_TOLERANCE =
    0.020;

constexpr double HOME_TOLERANCE =
    0.050;


// ----------------------
// Gripper
// ----------------------

// Panda单指打开位置
constexpr double GRIPPER_OPEN =
    0.040;

// 闭合
constexpr double GRIPPER_CLOSE =
    0.000;

constexpr double GRIPPER_FORCE =
    20.0;


// ----------------------
// Table
// ----------------------

constexpr double TABLE_SIZE_X =
    0.65;

constexpr double TABLE_SIZE_Y =
    0.70;

constexpr double TABLE_SIZE_Z =
    0.10;

constexpr double TABLE_CENTER_Z =
    0.15;

// 桌面高度
constexpr double TABLE_TOP_Z =
    TABLE_CENTER_Z
    +
    TABLE_SIZE_Z / 2.0;


// ----------------------
// Object
// ----------------------

constexpr double OBJECT_SIZE_X =
    0.050;

constexpr double OBJECT_SIZE_Y =
    0.050;

constexpr double OBJECT_SIZE_Z =
    0.070;

// 物体底部留5mm安全间隙
constexpr double OBJECT_CENTER_Z =
    TABLE_TOP_Z
    +
    OBJECT_SIZE_Z / 2.0
    +
    0.005;


// panda_link8 到物体中心的近似抓取偏置
constexpr double GRASP_EE_Z_OFFSET =
    0.130;


// ============================================================
// 数据
// ============================================================

struct PhaseResult
{
    std::string phase;

    std::string method;

    bool success = false;

    double time = 0.0;

    double metric = 0.0;

    std::string metric_name;
};


struct Layout
{
    double pick_x = 0.0;
    double pick_y = 0.0;

    double place_x = 0.0;
    double place_y = 0.0;
};


// ============================================================
// 工具
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
        return 999.0;
    }


    std::vector<double> current;


    state->copyJointGroupPositions(
        joint_group,
        current
    );


    if (current.size() !=
        target.size())
    {
        return 999.0;
    }


    double error =
        0.0;


    for (std::size_t i = 0;
         i < current.size();
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
// Planning Scene
// ============================================================

bool waitForWorldObject(
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
        const auto names =
            scene.getKnownObjectNames();


        const bool exists =
            std::find(
                names.begin(),
                names.end(),
                id
            )
            != names.end();


        if (exists ==
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
                milliseconds(
                    100
                )
        );
    }
}


bool waitForAttachedObject(
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
        const auto attached =
            scene.getAttachedObjects(
                {id}
            );


        const bool exists =
            !attached.empty();


        if (exists ==
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
                milliseconds(
                    100
                )
        );
    }
}


void removeWorldObject(
    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &id)
{
    scene.removeCollisionObjects(
        {id}
    );


    waitForWorldObject(
        scene,
        id,
        false,
        2.0
    );
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
    double sz)
{
    moveit_msgs::msg::
        CollisionObject object;


    object.header.frame_id =
        frame;


    object.id =
        id;


    shape_msgs::msg::
        SolidPrimitive primitive;


    primitive.type =
        shape_msgs::msg::
            SolidPrimitive::BOX;


    primitive.dimensions.resize(
        3
    );


    primitive.dimensions[0] =
        sx;

    primitive.dimensions[1] =
        sy;

    primitive.dimensions[2] =
        sz;


    geometry_msgs::msg::
        Pose pose;


    pose.orientation.w =
        1.0;


    pose.position.x =
        x;

    pose.position.y =
        y;

    pose.position.z =
        z;


    object.primitives.push_back(
        primitive
    );


    object.primitive_poses.push_back(
        pose
    );


    object.operation =
        moveit_msgs::msg::
            CollisionObject::ADD;


    if (!scene.applyCollisionObject(
            object))
    {
        return false;
    }


    return
        waitForWorldObject(
            scene,
            id,
            true
        );
}


// ============================================================
// 时间参数化
// ============================================================

bool retimeTrajectory(
    const moveit::core::
        RobotStatePtr &start_state,

    const std::string &group_name,

    moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    robot_trajectory::
        RobotTrajectory robot_traj(
            start_state->getRobotModel(),
            group_name
        );


    robot_traj.setRobotTrajectoryMsg(
        *start_state,
        trajectory
    );


    trajectory_processing::
        IterativeParabolicTimeParameterization
            iptp;


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
// RViz
// ============================================================

void publishTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::core::
        RobotStatePtr &start_state,

    const moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    moveit_msgs::msg::
        DisplayTrajectory display;


    display.model_id =
        "panda";


    moveit::core::
        robotStateToRobotStateMsg(
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
// Gripper
// ============================================================

bool commandGripper(
    const rclcpp::Node::SharedPtr &node,

    double position,

    double effort,

    double &elapsed)
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


    const auto begin =
        std::chrono::
            steady_clock::now();


    if (!client->
            wait_for_action_server(
                std::chrono::
                    seconds(
                        4
                    )
            ))
    {
        elapsed =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();

        return false;
    }


    GripperCommand::Goal goal;


    goal.command.position =
        position;


    goal.command.max_effort =
        effort;


    auto send_future =
        client->async_send_goal(
            goal
        );


    if (send_future.wait_for(
            std::chrono::
                seconds(
                    4
                ))
        !=
        std::future_status::ready)
    {
        elapsed =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();

        return false;
    }


    auto goal_handle =
        send_future.get();


    if (!goal_handle)
    {
        elapsed =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();

        return false;
    }


    auto result_future =
        client->async_get_result(
            goal_handle
        );


    if (result_future.wait_for(
            std::chrono::
                seconds(
                    8
                ))
        !=
        std::future_status::ready)
    {
        elapsed =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();

        return false;
    }


    const auto wrapped =
        result_future.get();


    elapsed =
        std::chrono::
            duration<double>(
                std::chrono::
                    steady_clock::now()
                -
                begin
            ).count();


    return
        wrapped.code
        ==
        rclcpp_action::
            ResultCode::SUCCEEDED;
}


// ============================================================
// Home
// ============================================================

bool moveHome(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

    const std::vector<double> &home,

    double &elapsed,

    double &final_error)
{
    const auto begin =
        std::chrono::
            steady_clock::now();


    if (maxJointError(
            move_group,
            joint_group,
            home)
        <
        HOME_TOLERANCE)
    {
        elapsed =
            0.0;

        final_error =
            maxJointError(
                move_group,
                joint_group,
                home
            );

        return true;
    }


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


        const bool planned =
            static_cast<bool>(
                move_group.plan(
                    plan
                )
            );


        if (!planned)
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
            std::chrono::
                milliseconds(
                    600
                )
        );


        final_error =
            maxJointError(
                move_group,
                joint_group,
                home
            );


        if (raw ||
            final_error <
            HOME_TOLERANCE)
        {
            elapsed =
                std::chrono::
                    duration<double>(
                        std::chrono::
                            steady_clock::now()
                        -
                        begin
                    ).count();

            return true;
        }
    }


    elapsed =
        std::chrono::
            duration<double>(
                std::chrono::
                    steady_clock::now()
                -
                begin
            ).count();


    final_error =
        maxJointError(
            move_group,
            joint_group,
            home
        );


    return false;
}


// ============================================================
// Pose规划测试
// 不执行，仅用于自动寻找合理Pick/Place位置
// ============================================================

bool posePlanExists(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link)
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
        4.0
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


    const bool success =
        static_cast<bool>(
            move_group.plan(
                plan
            )
        );


    move_group.clearPoseTargets();


    return success;
}


// ============================================================
// RRTConnect移动到Pose
// ============================================================

bool moveToPose(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::Pose &target,

    const std::string &ee_link,

    double &elapsed,

    double &position_error)
{
    const auto begin =
        std::chrono::
            steady_clock::now();


    for (int attempt = 1;
         attempt <= 3;
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


        if (!move_group.setPoseTarget(
                target,
                ee_link))
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


        move_group.clearPoseTargets();


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    700
                )
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


        if (raw ||
            position_error <
            POSITION_TOLERANCE)
        {
            elapsed =
                std::chrono::
                    duration<double>(
                        std::chrono::
                            steady_clock::now()
                        -
                        begin
                    ).count();

            return
                position_error
                <
                POSITION_TOLERANCE;
        }
    }


    elapsed =
        std::chrono::
            duration<double>(
                std::chrono::
                    steady_clock::now()
                -
                begin
            ).count();


    const auto actual =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    position_error =
        positionDistance(
            actual,
            target
        );


    return false;
}


// ============================================================
// Cartesian相对运动
// ============================================================

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

    double &elapsed,

    double &position_error)
{
    const auto begin =
        std::chrono::
            steady_clock::now();


    const auto start_pose =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    geometry_msgs::msg::
        Pose target =
            start_pose;


    target.position.x +=
        dx;

    target.position.y +=
        dy;

    target.position.z +=
        dz;


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
        elapsed =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();

        position_error =
            999.0;

        return false;
    }


    auto current_state =
        move_group.getCurrentState(
            5.0
        );


    if (!current_state)
    {
        return false;
    }


    if (!retimeTrajectory(
            current_state,
            group_name,
            trajectory))
    {
        return false;
    }


    publishTrajectory(
        display_publisher,
        current_state,
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
            milliseconds(
                600
            )
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


    elapsed =
        std::chrono::
            duration<double>(
                std::chrono::
                    steady_clock::now()
                -
                begin
            ).count();


    return
        fraction >=
            MIN_CART_FRACTION
        &&
        position_error <
            POSITION_TOLERANCE
        &&
        (
            raw
            ||
            position_error <
                POSITION_TOLERANCE
        );
}


// ============================================================
// 终端结果表
// ============================================================

void printParameterTable(
    const Layout &layout)
{
    std::cout
        << "\n\n";


    std::cout
        << "================================================================================================================\n";


    std::cout
        << "                                      PICK & PLACE KEY PARAMETERS\n";


    std::cout
        << "================================================================================================================\n";


    std::cout
        << std::left
        << std::setw(36)
        << "Parameter"
        << std::setw(30)
        << "Value"
        << "Meaning\n";


    std::cout
        << "----------------------------------------------------------------------------------------------------------------\n";


    std::cout
        << std::setw(36)
        << "Arm planner"
        << std::setw(30)
        << "RRTConnect"
        << "Free-space motion planning\n";


    std::cout
        << std::setw(36)
        << "Cartesian EEF step"
        << std::setw(30)
        << "0.005 m"
        << "5 mm interpolation\n";


    std::cout
        << std::setw(36)
        << "Minimum Cartesian fraction"
        << std::setw(30)
        << "98%"
        << "Incomplete paths are rejected\n";


    std::cout
        << std::setw(36)
        << "Arm velocity scale"
        << std::setw(30)
        << "12%"
        << "RRT motion\n";


    std::cout
        << std::setw(36)
        << "Cartesian velocity scale"
        << std::setw(30)
        << "8%"
        << "Approach / lift / descend\n";


    std::cout
        << std::setw(36)
        << "Approach distance"
        << std::setw(30)
        << "0.120 m"
        << "Vertical grasp approach\n";


    std::cout
        << std::setw(36)
        << "Lift distance"
        << std::setw(30)
        << "0.140 m"
        << "Object lifting height\n";


    std::cout
        << std::setw(36)
        << "Object size"
        << std::setw(30)
        << "50 x 50 x 70 mm"
        << "Collision object\n";


    std::cout
        << std::setw(36)
        << "Table top Z"
        << std::setw(30)
        << "0.200 m"
        << "Workspace table\n";


    std::cout
        << std::setw(36)
        << "Gripper open"
        << std::setw(30)
        << "0.040 m"
        << "Panda finger command\n";


    std::cout
        << std::setw(36)
        << "Gripper close"
        << std::setw(30)
        << "0.000 m"
        << "Panda finger command\n";


    std::cout
        << std::setw(36)
        << "Pick XY"
        << std::setw(30)
        << (
            "("
            +
            std::to_string(
                layout.pick_x
            )
            +
            ", "
            +
            std::to_string(
                layout.pick_y
            )
            +
            ") m"
        )
        << "Auto-selected reachable point\n";


    std::cout
        << std::setw(36)
        << "Place XY"
        << std::setw(30)
        << (
            "("
            +
            std::to_string(
                layout.place_x
            )
            +
            ", "
            +
            std::to_string(
                layout.place_y
            )
            +
            ") m"
        )
        << "Auto-selected reachable point\n";


    std::cout
        << "================================================================================================================\n";
}


void printResultTable(
    const std::vector<
        PhaseResult
    > &results)
{
    std::cout
        << "\n\n";


    std::cout
        << "================================================================================================================\n";


    std::cout
        << "                                      PICK & PLACE RESULTS\n";


    std::cout
        << "================================================================================================================\n";


    std::cout
        << std::left

        << std::setw(22)
        << "Phase"

        << std::setw(22)
        << "Method"

        << std::setw(14)
        << "Time(s)"

        << std::setw(24)
        << "Metric"

        << std::setw(15)
        << "Value"

        << "Result\n";


    std::cout
        << "----------------------------------------------------------------------------------------------------------------\n";


    for (const auto &r :
         results)
    {
        std::cout
            << std::left

            << std::setw(22)
            << r.phase

            << std::setw(22)
            << r.method

            << std::fixed
            << std::setprecision(
                4
            )

            << std::setw(14)
            << r.time

            << std::setw(24)
            << r.metric_name

            << std::setw(15)
            << r.metric

            << (
                r.success
                ?
                "SUCCESS"
                :
                "FAIL"
            )

            << "\n";
    }


    std::cout
        << "================================================================================================================\n";
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
            "pick_and_place",

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


    // 等待TF稳定
    std::this_thread::sleep_for(
        std::chrono::seconds(
            2
        )
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


    const std::string table_id =
        "pick_place_table";


    const std::string object_id =
        "pick_place_box";


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


    // ========================================================
    // Safe Home
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


    std::vector<
        PhaseResult
    > results;


    std::cout
        << "\n\n============================================================\n";


    std::cout
        << "          PANDA PICK & PLACE EXPERIMENT\n";


    std::cout
        << "============================================================\n";


    std::cout
        << "Planning frame : "
        << planning_frame
        << "\n";


    std::cout
        << "Planning group : "
        << group_name
        << "\n";


    std::cout
        << "End effector   : "
        << ee_link
        << "\n";


    // ========================================================
    // 清理旧场景
    // ========================================================

    move_group.detachObject(
        object_id
    );


    scene.removeCollisionObjects(
        {
            table_id,
            object_id,
            "benchmark_box",
            "tiny_benchmark_box"
        }
    );


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(
                500
            )
    );


    // ========================================================
    // Phase 0: 打开夹爪
    // ========================================================

    {
        double t =
            0.0;


        const bool ok =
            commandGripper(
                node,
                GRIPPER_OPEN,
                GRIPPER_FORCE,
                t
            );


        results.push_back(
            {
                "Gripper Open",
                "GripperAction",
                ok,
                t,
                GRIPPER_OPEN,
                "position(m)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Cannot open Panda gripper."
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    // ========================================================
    // Phase 1: Home
    // ========================================================

    {
        double t =
            0.0;

        double err =
            999.0;


        const bool ok =
            moveHome(
                move_group,
                joint_group,
                home,
                t,
                err
            );


        results.push_back(
            {
                "Safe Home",
                "RRTConnect",
                ok,
                t,
                err,
                "joint_error(rad)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Cannot establish Safe Home."
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    const auto home_pose =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    // ========================================================
    // 添加桌子
    // ========================================================

    const double table_center_x =
        home_pose.position.x
        +
        0.15;


    if (!addBox(
            scene,
            planning_frame,
            table_id,

            table_center_x,
            0.0,
            TABLE_CENTER_Z,

            TABLE_SIZE_X,
            TABLE_SIZE_Y,
            TABLE_SIZE_Z))
    {
        RCLCPP_ERROR(
            logger,
            "Unable to create table."
        );

        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    // ========================================================
    // 自动寻找合理 Pick / Place 位置
    // ========================================================

    const std::vector<
        Layout
    > candidates =
    {
        {
            home_pose.position.x + 0.15,
            -0.15,
            home_pose.position.x + 0.15,
             0.15
        },

        {
            home_pose.position.x + 0.11,
            -0.14,
            home_pose.position.x + 0.11,
             0.14
        },

        {
            home_pose.position.x + 0.18,
            -0.12,
            home_pose.position.x + 0.18,
             0.12
        },

        {
            home_pose.position.x + 0.13,
             0.15,
            home_pose.position.x + 0.13,
            -0.15
        }
    };


    const double grasp_z =
        OBJECT_CENTER_Z
        +
        GRASP_EE_Z_OFFSET;


    const double pregrasp_z =
        grasp_z
        +
        APPROACH_DISTANCE;


    const double preplace_z =
        grasp_z
        +
        LIFT_DISTANCE;


    Layout selected_layout;


    bool layout_found =
        false;


    geometry_msgs::msg::Pose
        selected_pregrasp;


    geometry_msgs::msg::Pose
        selected_preplace;


    std::cout
        << "\n============================================================\n";


    std::cout
        << "PHASE 2 : AUTO-CALIBRATE PICK / PLACE POSITIONS\n";


    std::cout
        << "============================================================\n";


    for (std::size_t i = 0;
         i < candidates.size();
         ++i)
    {
        removeWorldObject(
            scene,
            object_id
        );


        const auto &layout =
            candidates[i];


        if (!addBox(
                scene,
                planning_frame,
                object_id,

                layout.pick_x,
                layout.pick_y,
                OBJECT_CENTER_Z,

                OBJECT_SIZE_X,
                OBJECT_SIZE_Y,
                OBJECT_SIZE_Z))
        {
            continue;
        }


        geometry_msgs::msg::Pose
            pregrasp =
                home_pose;


        pregrasp.position.x =
            layout.pick_x;

        pregrasp.position.y =
            layout.pick_y;

        pregrasp.position.z =
            pregrasp_z;


        geometry_msgs::msg::Pose
            preplace =
                home_pose;


        preplace.position.x =
            layout.place_x;

        preplace.position.y =
            layout.place_y;

        preplace.position.z =
            preplace_z;


        std::cout
            << "Candidate "
            << i + 1
            << " : Pick("
            << layout.pick_x
            << ", "
            << layout.pick_y
            << ")  Place("
            << layout.place_x
            << ", "
            << layout.place_y
            << ")\n";


        const bool pick_ok =
            posePlanExists(
                move_group,
                pregrasp,
                ee_link
            );


        const bool place_ok =
            posePlanExists(
                move_group,
                preplace,
                ee_link
            );


        std::cout
            << "  Pre-grasp : "
            << (
                pick_ok
                ?
                "VALID"
                :
                "FAIL"
            )
            << "\n";


        std::cout
            << "  Pre-place : "
            << (
                place_ok
                ?
                "VALID"
                :
                "FAIL"
            )
            << "\n";


        if (pick_ok &&
            place_ok)
        {
            selected_layout =
                layout;


            selected_pregrasp =
                pregrasp;


            selected_preplace =
                preplace;


            layout_found =
                true;


            break;
        }
    }


    if (!layout_found)
    {
        RCLCPP_ERROR(
            logger,
            "Unable to find reachable Pick / Place layout."
        );


        executor.cancel();

        spin_thread.join();

        rclcpp::shutdown();

        return 1;
    }


    printParameterTable(
        selected_layout
    );


    std::cout
        << "\nSelected reachable workspace.\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            2
        )
    );


    // ========================================================
    // Phase 3: RRTConnect -> Pre-grasp
    // ========================================================

    {
        double t =
            0.0;

        double err =
            999.0;


        const bool ok =
            moveToPose(
                move_group,
                selected_pregrasp,
                ee_link,
                t,
                err
            );


        results.push_back(
            {
                "Pre-Grasp",
                "RRTConnect",
                ok,
                t,
                err,
                "position_error(m)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Pre-grasp failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    std::cout
        << "\nPre-grasp reached. Cartesian approach begins...\n";


    // ========================================================
    // Phase 4: Cartesian下降
    // ========================================================

    {
        double fraction =
            0.0;

        double t =
            0.0;

        double err =
            999.0;


        const bool ok =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                -APPROACH_DISTANCE,

                fraction,
                t,
                err
            );


        results.push_back(
            {
                "Approach Down",
                "Cartesian",
                ok,
                t,
                fraction * 100.0,
                "fraction(%)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Cartesian grasp approach failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    // ========================================================
    // Phase 5: Close gripper
    // ========================================================

    {
        double t =
            0.0;


        const bool ok =
            commandGripper(
                node,
                GRIPPER_CLOSE,
                GRIPPER_FORCE,
                t
            );


        results.push_back(
            {
                "Gripper Close",
                "GripperAction",
                ok,
                t,
                GRIPPER_CLOSE,
                "position(m)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Gripper close failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    std::this_thread::sleep_for(
        std::chrono::seconds(
            1
        )
    );


    // ========================================================
    // Phase 6: Attach object
    // ========================================================

    {
        const auto begin =
            std::chrono::
                steady_clock::now();


        const std::vector<
            std::string
        > touch_links =
        {
            "panda_link8",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger"
        };


        const bool raw =
            move_group.attachObject(
                object_id,
                ee_link,
                touch_links
            );


        const bool attached =
            raw
            &&
            waitForAttachedObject(
                scene,
                object_id,
                true
            );


        const double t =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();


        results.push_back(
            {
                "Attach Object",
                "PlanningScene",
                attached,
                t,
                attached ? 1.0 : 0.0,
                "attached"
            }
        );


        if (!attached)
        {
            RCLCPP_ERROR(
                logger,
                "Object attachment failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    std::cout
        << "\nObject attached to Panda.\n";


    // ========================================================
    // Phase 7: Cartesian Lift
    // ========================================================

    {
        double fraction =
            0.0;

        double t =
            0.0;

        double err =
            999.0;


        const bool ok =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                LIFT_DISTANCE,

                fraction,
                t,
                err
            );


        results.push_back(
            {
                "Lift Object",
                "Cartesian",
                ok,
                t,
                fraction * 100.0,
                "fraction(%)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Object lift failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    std::cout
        << "\nObject lifted successfully.\n";


    // ========================================================
    // Phase 8: Transfer
    // RRTConnect优先
    // 若失败自动Cartesian水平搬运
    // ========================================================

    bool transfer_ok =
        false;


    {
        double t =
            0.0;

        double err =
            999.0;


        transfer_ok =
            moveToPose(
                move_group,
                selected_preplace,
                ee_link,
                t,
                err
            );


        if (transfer_ok)
        {
            results.push_back(
                {
                    "Transfer",
                    "RRTConnect",
                    true,
                    t,
                    err,
                    "position_error(m)"
                }
            );
        }
        else
        {
            std::cout
                << "\nRRTConnect transfer failed."
                << " Trying Cartesian fallback...\n";


            const auto current =
                move_group.getCurrentPose(
                    ee_link
                ).pose;


            const double dx =
                selected_preplace.position.x
                -
                current.position.x;


            const double dy =
                selected_preplace.position.y
                -
                current.position.y;


            const double dz =
                selected_preplace.position.z
                -
                current.position.z;


            double fraction =
                0.0;

            double cart_t =
                0.0;

            double cart_err =
                999.0;


            transfer_ok =
                cartesianMove(
                    move_group,
                    group_name,
                    ee_link,
                    display_publisher,

                    dx,
                    dy,
                    dz,

                    fraction,
                    cart_t,
                    cart_err
                );


            results.push_back(
                {
                    "Transfer",
                    "CartesianFallback",
                    transfer_ok,
                    cart_t,
                    fraction * 100.0,
                    "fraction(%)"
                }
            );
        }


        if (!transfer_ok)
        {
            RCLCPP_ERROR(
                logger,
                "Transfer failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    // ========================================================
    // Phase 9: Cartesian Place Descend
    // ========================================================

    {
        double fraction =
            0.0;

        double t =
            0.0;

        double err =
            999.0;


        const bool ok =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                -LIFT_DISTANCE,

                fraction,
                t,
                err
            );


        results.push_back(
            {
                "Place Descend",
                "Cartesian",
                ok,
                t,
                fraction * 100.0,
                "fraction(%)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Place descend failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    // ========================================================
    // Phase 10: Open gripper
    // ========================================================

    {
        double t =
            0.0;


        const bool ok =
            commandGripper(
                node,
                GRIPPER_OPEN,
                GRIPPER_FORCE,
                t
            );


        results.push_back(
            {
                "Gripper Open",
                "GripperAction",
                ok,
                t,
                GRIPPER_OPEN,
                "position(m)"
            }
        );


        if (!ok)
        {
            RCLCPP_ERROR(
                logger,
                "Gripper open failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    // ========================================================
    // Phase 11: Detach
    // ========================================================

    bool detached =
        false;


    {
        const auto begin =
            std::chrono::
                steady_clock::now();


        const bool raw =
            move_group.detachObject(
                object_id
            );


        detached =
            raw
            &&
            waitForAttachedObject(
                scene,
                object_id,
                false
            );


        waitForWorldObject(
            scene,
            object_id,
            true,
            3.0
        );


        const double t =
            std::chrono::
                duration<double>(
                    std::chrono::
                        steady_clock::now()
                    -
                    begin
                ).count();


        results.push_back(
            {
                "Detach Object",
                "PlanningScene",
                detached,
                t,
                detached ? 1.0 : 0.0,
                "detached"
            }
        );


        if (!detached)
        {
            RCLCPP_ERROR(
                logger,
                "Object detach failed."
            );

            printResultTable(
                results
            );

            executor.cancel();

            spin_thread.join();

            rclcpp::shutdown();

            return 1;
        }
    }


    // ========================================================
    // Placement error
    // ========================================================

    double placement_error =
        999.0;


    {
        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    500
                )
        );


        const auto poses =
            scene.getObjectPoses(
                {object_id}
            );


        const auto it =
            poses.find(
                object_id
            );


        if (it !=
            poses.end())
        {
            geometry_msgs::msg::Pose
                expected;


            expected.position.x =
                selected_layout.place_x;


            expected.position.y =
                selected_layout.place_y;


            expected.position.z =
                OBJECT_CENTER_Z;


            expected.orientation.w =
                1.0;


            placement_error =
                positionDistance(
                    it->second,
                    expected
                );
        }
    }


    // ========================================================
    // Phase 12: Retreat
    // ========================================================

    {
        double fraction =
            0.0;

        double t =
            0.0;

        double err =
            999.0;


        const bool ok =
            cartesianMove(
                move_group,
                group_name,
                ee_link,
                display_publisher,

                0.0,
                0.0,
                APPROACH_DISTANCE,

                fraction,
                t,
                err
            );


        results.push_back(
            {
                "Retreat",
                "Cartesian",
                ok,
                t,
                fraction * 100.0,
                "fraction(%)"
            }
        );


        if (!ok)
        {
            RCLCPP_WARN(
                logger,
                "Retreat failed, trying Home anyway."
            );
        }
    }


    // ========================================================
    // Phase 13: Return Home
    // ========================================================

    bool home_success =
        false;


    {
        double t =
            0.0;

        double err =
            999.0;


        home_success =
            moveHome(
                move_group,
                joint_group,
                home,
                t,
                err
            );


        results.push_back(
            {
                "Return Home",
                "RRTConnect",
                home_success,
                t,
                err,
                "joint_error(rad)"
            }
        );
    }


    // ========================================================
    // 最终成功判定
    // ========================================================

    bool all_phases_ok =
        true;


    for (const auto &r :
         results)
    {
        if (!r.success)
        {
            // Retreat失败不影响抓取放置是否完成
            if (r.phase !=
                "Retreat")
            {
                all_phases_ok =
                    false;
            }
        }
    }


    const bool placement_ok =
        placement_error
        <
        0.03;


    const bool final_success =
        all_phases_ok
        &&
        placement_ok
        &&
        home_success;


    // ========================================================
    // 打印表
    // ========================================================

    printParameterTable(
        selected_layout
    );


    printResultTable(
        results
    );


    std::cout
        << "\n\n";


    std::cout
        << "============================================================\n";


    std::cout
        << "                    FINAL SUMMARY\n";


    std::cout
        << "============================================================\n";


    std::cout
        << std::fixed
        << std::setprecision(
            4
        );


    std::cout
        << "Pick position      : ("
        << selected_layout.pick_x
        << ", "
        << selected_layout.pick_y
        << ", "
        << OBJECT_CENTER_Z
        << ") m\n";


    std::cout
        << "Place position     : ("
        << selected_layout.place_x
        << ", "
        << selected_layout.place_y
        << ", "
        << OBJECT_CENTER_Z
        << ") m\n";


    std::cout
        << "Approach distance  : "
        << APPROACH_DISTANCE
        << " m\n";


    std::cout
        << "Lift distance      : "
        << LIFT_DISTANCE
        << " m\n";


    std::cout
        << "Placement error    : "
        << placement_error
        << " m\n";


    std::cout
        << "Object attached    : SUCCESS\n";


    std::cout
        << "Object detached    : "
        << (
            detached
            ?
            "SUCCESS"
            :
            "FAIL"
        )
        << "\n";


    std::cout
        << "Return Home        : "
        << (
            home_success
            ?
            "SUCCESS"
            :
            "FAIL"
        )
        << "\n";


    std::cout
        << "Pick & Place       : "
        << (
            final_success
            ?
            "SUCCESS"
            :
            "FAIL"
        )
        << "\n";


    std::cout
        << "============================================================\n";


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
        "pick_and_place_result.csv";


    std::ofstream csv(
        csv_path
    );


    csv
        << "phase,"
        << "method,"
        << "time_s,"
        << "metric_name,"
        << "metric,"
        << "success\n";


    for (const auto &r :
         results)
    {
        csv
            << r.phase
            << ","
            << r.method
            << ","
            << r.time
            << ","
            << r.metric_name
            << ","
            << r.metric
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


    csv
        << "FINAL,"
        << "PickPlace,"
        << "0,"
        << "placement_error_m,"
        << placement_error
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
        << "\nCSV saved to:\n"
        << csv_path
        << "\n";


    std::cout
        << "\nHolding final scene for 5 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::seconds(
            5
        )
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
