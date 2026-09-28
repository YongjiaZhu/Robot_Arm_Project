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


// ======================================================================
//                         系统安全参数
// ======================================================================


// ---------------------------
// 普通RRT运动
// ---------------------------

constexpr double ARM_VELOCITY = 0.09;

constexpr double ARM_ACCELERATION = 0.04;


// ---------------------------
// Cartesian精细动作
// ---------------------------

constexpr double CART_VELOCITY = 0.06;

constexpr double CART_ACCELERATION = 0.025;

constexpr double EEF_STEP = 0.004;

constexpr double MIN_CART_FRACTION = 0.98;


// ---------------------------
// 抓取动作
// ---------------------------

// Pre-Grasp -> Grasp
constexpr double APPROACH_DISTANCE = 0.10;

// 抓起后明显升高
constexpr double LIFT_DISTANCE = 0.20;

// 放完后撤离
constexpr double RETREAT_DISTANCE = 0.13;


// ---------------------------
// 允许误差
// ---------------------------

constexpr double POSITION_TOLERANCE = 0.018;

constexpr double HOME_TOLERANCE = 0.050;


// ======================================================================
//                        桌面安全参数
// ======================================================================

constexpr double TABLE_SIZE_X = 0.82;

constexpr double TABLE_SIZE_Y = 0.96;

constexpr double TABLE_SIZE_Z = 0.10;

constexpr double TABLE_CENTER_Z = 0.15;

constexpr double TABLE_TOP_Z =
    TABLE_CENTER_Z +
    TABLE_SIZE_Z / 2.0;


// 物体不与桌面模型完全贴合，保留5 mm数值余量
constexpr double OBJECT_TABLE_GAP = 0.005;


// panda_link8保持在较高位置。
// 相比上一版明显加高，避免手指/手掌视觉上扎入桌面。
constexpr double GRASP_EE_Z_OFFSET = 0.175;


// 任何Cartesian目标的腕部最低高度。
// 如果目标低于这个高度，直接拒绝执行。
constexpr double MIN_WRIST_HEIGHT =
    TABLE_TOP_Z + 0.175;


// ======================================================================
//                         障碍物安全参数
// ======================================================================

// 真实可见障碍物外侧再扩大5 cm。
// 规划器避让的是扩大后的Safety Envelope。
// 因此机器人不会贴着真实障碍物走。
constexpr double OBSTACLE_MARGIN_XY = 0.050;


// 顶部再留4 cm
constexpr double OBSTACLE_MARGIN_TOP = 0.040;


// 直接Cartesian小于90%才算真正被挡住
constexpr double DIRECT_BLOCK_THRESHOLD = 0.90;


// ======================================================================
// Gripper
// ======================================================================

constexpr double GRIPPER_OPEN = 0.040;

constexpr double GRIPPER_FORCE = 20.0;


// ======================================================================
// 数据结构
// ======================================================================

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

    double obstacle_sz;
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

    double direct_fraction = 100.0;

    double approach_fraction = 0.0;

    double lift_fraction = 0.0;

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


// ======================================================================
// Color
// ======================================================================

std_msgs::msg::ColorRGBA makeColor(
    double r,
    double g,
    double b,
    double a = 1.0)
{
    std_msgs::msg::ColorRGBA color;

    color.r =
        static_cast<float>(r);

    color.g =
        static_cast<float>(g);

    color.b =
        static_cast<float>(b);

    color.a =
        static_cast<float>(a);

    return color;
}


// ======================================================================
// Geometry
// ======================================================================

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

    return
        std::sqrt(
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


    double result =
        0.0;


    for (std::size_t i = 0;
         i < current.size();
         ++i)
    {
        result =
            std::max(
                result,
                std::fabs(
                    current[i] -
                    target[i]
                )
            );
    }


    return result;
}


// ======================================================================
// Collision object
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
        sx;

    box.dimensions[1] =
        sy;

    box.dimensions[2] =
        sz;


    geometry_msgs::msg::Pose pose;

    pose.orientation.w =
        1.0;

    pose.position.x =
        x;

    pose.position.y =
        y;

    pose.position.z =
        z;


    object.primitives.push_back(
        box
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

    const std_msgs::msg::
        ColorRGBA &color)
{
    return
        scene.applyCollisionObject(
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


bool objectExists(
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
    const auto result =
        scene.getAttachedObjects(
            {id}
        );


    return
        !result.empty();
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
        if (objectExists(
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
                    begin
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
                    begin
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

bool commandGripper(
    const rclcpp::Node::SharedPtr &node,

    double position)
{
    using GripperAction =
        control_msgs::action::
            GripperCommand;


    auto client =
        rclcpp_action::
            create_client<
                GripperAction
            >(
                node,
                "/panda_hand_controller/gripper_cmd"
            );


    if (!client->
            wait_for_action_server(
                std::chrono::
                    seconds(
                        4
                    )
            ))
    {
        return false;
    }


    GripperAction::Goal goal;

    goal.command.position =
        position;

    goal.command.max_effort =
        GRIPPER_FORCE;


    auto future =
        client->async_send_goal(
            goal
        );


    if (future.wait_for(
            std::chrono::
                seconds(
                    4
                ))
        !=
        std::future_status::ready)
    {
        return false;
    }


    auto handle =
        future.get();


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
                seconds(
                    8
                ))
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
// Cartesian time parameterization
// ======================================================================

bool retimeTrajectory(
    const moveit::core::
        RobotStatePtr &state,

    const std::string &group_name,

    moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    robot_trajectory::
        RobotTrajectory robot_trajectory(
            state->getRobotModel(),
            group_name
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
            CART_VELOCITY,
            CART_ACCELERATION))
    {
        return false;
    }


    robot_trajectory.getRobotTrajectoryMsg(
        trajectory
    );


    return true;
}


// ======================================================================
// RViz
// ======================================================================

void publishTrajectory(
    const rclcpp::Publisher<
        moveit_msgs::msg::
            DisplayTrajectory
    >::SharedPtr &publisher,

    const moveit::core::
        RobotStatePtr &state,

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
// Home
// ======================================================================

bool moveHome(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const moveit::core::
        JointModelGroup *joint_group,

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


    for (int i = 0;
         i < 4;
         ++i)
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


        move_group.execute(
            plan
        );


        std::this_thread::sleep_for(
            std::chrono::
                milliseconds(
                    600
                )
        );


        if (maxJointError(
                move_group,
                joint_group,
                home)
            <
            HOME_TOLERANCE)
        {
            return true;
        }
    }


    return false;
}


// ======================================================================
// Joint-space path length
// ======================================================================

double trajectoryJointLength(
    const moveit_msgs::msg::
        RobotTrajectory &trajectory)
{
    const auto &points =
        trajectory.joint_trajectory.points;


    if (points.size() <
        2)
    {
        return 0.0;
    }


    double result =
        0.0;


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


        double step =
            0.0;


        for (std::size_t j = 0;
             j < n;
             ++j)
        {
            const double d =
                b[j] -
                a[j];

            step +=
                d * d;
        }


        result +=
            std::sqrt(
                step
            );
    }


    return result;
}


// ======================================================================
// 只规划，不执行
// ======================================================================

PlanResult generatePlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::
        Pose &target,

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


    move_group.setNumPlanningAttempts(
        1
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


    result.success =
        true;


    result.plan =
        plan;


    result.planner =
        planner_name;


    result.path_length =
        trajectoryJointLength(
            plan.trajectory_
        );


    return result;
}


// ======================================================================
// 多候选规划
//
// RRTConnect x 3
// RRT*       x 2
//
// 从有限时间生成的无碰撞候选中，选关节空间路径较短者。
// ======================================================================

PlanResult selectBestPlan(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::
        Pose &target,

    const std::string &ee_link)
{
    std::vector<PlanResult>
        candidates;


    for (int i = 0;
         i < 3;
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
            candidates.push_back(
                result
            );
        }
    }


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
            candidates.push_back(
                result
            );
        }
    }


    if (candidates.empty())
    {
        return PlanResult{};
    }


    return
        *std::min_element(
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
}


// ======================================================================
// 快速稳定可行性检查
//
// 同一个障碍位置要求RRTConnect连续成功2次。
// ======================================================================

bool stablePlanExists(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::
        Pose &target,

    const std::string &ee_link,

    std::string &planner_name)
{
    int connect_success =
        0;


    for (int i = 0;
         i < 2;
         ++i)
    {
        if (generatePlan(
                move_group,
                target,
                ee_link,

                "RRTConnectkConfigDefault",
                "RRTConnect",

                5.0
            ).success)
        {
            ++connect_success;
        }
    }


    if (connect_success ==
        2)
    {
        planner_name =
            "RRTConnect";

        return true;
    }


    int star_success =
        0;


    for (int i = 0;
         i < 2;
         ++i)
    {
        if (generatePlan(
                move_group,
                target,
                ee_link,

                "RRTstarkConfigDefault",
                "RRT*",

                8.0
            ).success)
        {
            ++star_success;
        }
    }


    if (star_success ==
        2)
    {
        planner_name =
            "RRT*";

        return true;
    }


    planner_name =
        "-";


    return false;
}


// ======================================================================
// 执行普通规划并检查终点
// ======================================================================

bool executePlanToPose(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const PlanResult &result,

    const geometry_msgs::msg::
        Pose &target,

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
            milliseconds(
                650
            )
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
        error
        <
        POSITION_TOLERANCE;
}


// ======================================================================
// 普通到达Pose
// ======================================================================

bool moveToPoseSafe(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::
        Pose &target,

    const std::string &ee_link,

    double &error)
{
    auto result =
        selectBestPlan(
            move_group,
            target,
            ee_link
        );


    return
        executePlanToPose(
            move_group,
            result,
            target,
            ee_link,
            error
        );
}


// ======================================================================
// Cartesian
// ======================================================================

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


    auto target =
        start;


    target.position.x +=
        dx;

    target.position.y +=
        dy;

    target.position.z +=
        dz;


    // ----------------------------------------------------------
    // 防止腕部接近桌面
    // ----------------------------------------------------------

    if (target.position.z <
        MIN_WRIST_HEIGHT)
    {
        std::cout
            << "[SAFETY] Cartesian target rejected.\n";


        std::cout
            << "Target wrist Z = "
            << target.position.z
            << " m\n";


        std::cout
            << "Minimum safe Z  = "
            << MIN_WRIST_HEIGHT
            << " m\n";


        fraction =
            0.0;


        error =
            999.0;


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


    // true = 开启碰撞检查
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


    move_group.execute(
        plan
    );


    std::this_thread::sleep_for(
        std::chrono::
            milliseconds(
                550
            )
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
        fraction >=
            MIN_CART_FRACTION
        &&
        error <
            POSITION_TOLERANCE;
}


// ======================================================================
// 只检查直接Cartesian能走多少
// ======================================================================

double directCartesianFraction(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    const geometry_msgs::msg::
        Pose &target)
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
// 添加障碍物 + Safety Envelope
//
// 可见障碍物：正常颜色。
// Guard：半透明，尺寸更大。
// 真正规划需要避让Guard。
// ======================================================================

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


    // 非常淡的安全包络。
    // 它参与碰撞规划，但视觉上只是一层透明保护区。
    const auto guard_color =
        makeColor(
            task.obstacle_color.r,
            task.obstacle_color.g,
            task.obstacle_color.b,
            0.08
        );


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
            milliseconds(
                500
            )
    );


    return true;
}


// ======================================================================
// 障碍物校准
//
// 条件：
// 1. Direct Cartesian < 90%
// 2. RRTConnect 或 RRT* 连续规划2次成功
//
// 只有两个条件同时满足才正式使用。
// ======================================================================

ObstacleCalibration calibrateSafeObstacle(
    moveit::planning_interface::
        MoveGroupInterface &move_group,

    moveit::planning_interface::
        PlanningSceneInterface &scene,

    const std::string &frame,

    const std::string &ee_link,

    const TaskConfig &task,

    const geometry_msgs::msg::
        Pose &target)
{
    ObstacleCalibration result;


    const auto start =
        move_group.getCurrentPose(
            ee_link
        ).pose;


    const std::vector<double>
        ratios =
    {
        0.46,
        0.50,
        0.54
    };


    const std::vector<double>
        x_offsets =
    {
        0.0,
        0.025,
        -0.025
    };


    const std::vector<double>
        height_additions =
    {
        0.00,
        0.04
    };


    for (double h_add :
         height_additions)
    {
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


                const double visible_sz =
                    task.obstacle_sz
                    +
                    h_add;


                double guard_sx =
                    0.0;

                double guard_sy =
                    0.0;

                double guard_sz =
                    0.0;


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
                    << "Visible size : "
                    << task.obstacle_sx * 1000.0
                    << " x "
                    << task.obstacle_sy * 1000.0
                    << " x "
                    << visible_sz * 1000.0
                    << " mm\n";


                std::cout
                    << "Safety size  : "
                    << guard_sx * 1000.0
                    << " x "
                    << guard_sy * 1000.0
                    << " x "
                    << guard_sz * 1000.0
                    << " mm\n";


                std::cout
                    << "Direct path  : "
                    << direct_fraction * 100.0
                    << "%\n";


                if (direct_fraction >=
                    DIRECT_BLOCK_THRESHOLD)
                {
                    continue;
                }


                std::string verified_planner;


                if (!stablePlanExists(
                        move_group,
                        target,
                        ee_link,
                        verified_planner))
                {
                    continue;
                }


                result.success =
                    true;


                result.x =
                    ox;

                result.y =
                    oy;


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
                    verified_planner;


                return result;
            }
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


// ======================================================================
// 终端任务参数
// ======================================================================

void printConfiguration(
    const std::vector<TaskConfig> &tasks)
{
    std::cout
        << "\n\n";


    std::cout
        << "==========================================================================================================================\n";


    std::cout
        << "                                      SAFE MULTI-PICK CONFIGURATION\n";


    std::cout
        << "==========================================================================================================================\n";


    std::cout
        << std::left

        << std::setw(10)
        << "Task"

        << std::setw(10)
        << "Color"

        << std::setw(21)
        << "Object(mm)"

        << std::setw(20)
        << "Pick Y(m)"

        << std::setw(20)
        << "Place Y(m)"

        << "Obstacle\n";


    std::cout
        << "--------------------------------------------------------------------------------------------------------------------------\n";


    for (const auto &task :
         tasks)
    {
        std::ostringstream size;


        size
            << static_cast<int>(
                   std::round(
                       task.sx * 1000.0
                   ))
            << "x"
            << static_cast<int>(
                   std::round(
                       task.sy * 1000.0
                   ))
            << "x"
            << static_cast<int>(
                   std::round(
                       task.sz * 1000.0
                   ));


        std::cout
            << std::setw(10)
            << task.name

            << std::setw(10)
            << task.color_name

            << std::setw(21)
            << size.str()

            << std::setw(20)
            << task.pick_y

            << std::setw(20)
            << task.place_y

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
        << "==========================================================================================================================\n";


    std::cout
        << "Table top Z                 : "
        << TABLE_TOP_Z
        << " m\n";


    std::cout
        << "Minimum wrist Z             : "
        << MIN_WRIST_HEIGHT
        << " m\n";


    std::cout
        << "Grasp EE offset             : "
        << GRASP_EE_Z_OFFSET
        << " m\n";


    std::cout
        << "Lift distance               : "
        << LIFT_DISTANCE
        << " m\n";


    std::cout
        << "Obstacle XY safety margin   : "
        << OBSTACLE_MARGIN_XY * 1000.0
        << " mm per side\n";


    std::cout
        << "Obstacle top safety margin  : "
        << OBSTACLE_MARGIN_TOP * 1000.0
        << " mm\n";


    std::cout
        << "Cartesian collision checking: ENABLED\n";


    std::cout
        << "Transfer candidates         : RRTConnect x3 + RRT* x2\n";


    std::cout
        << "==========================================================================================================================\n";
}


// ======================================================================
// 结果表
// ======================================================================

void printResults(
    const std::vector<TaskMetrics> &metrics)
{
    std::cout
        << "\n\n";


    std::cout
        << "================================================================================================================================================\n";


    std::cout
        << "                                                        SAFE MULTI-TASK RESULTS\n";


    std::cout
        << "================================================================================================================================================\n";


    std::cout
        << std::left

        << std::setw(9)
        << "Task"

        << std::setw(9)
        << "Color"

        << std::setw(15)
        << "Direct"

        << std::setw(14)
        << "Planner"

        << std::setw(16)
        << "Path(rad)"

        << std::setw(16)
        << "EE Err(mm)"

        << std::setw(18)
        << "Place Err(mm)"

        << std::setw(14)
        << "Cartesian"

        << "Result\n";


    std::cout
        << "------------------------------------------------------------------------------------------------------------------------------------------------\n";


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
            << m.place_fraction
            << "%";


        std::cout
            << std::setw(9)
            << m.name

            << std::setw(9)
            << m.color

            << std::setw(15)
            << direct.str()

            << std::setw(14)
            << m.transfer_planner

            << std::fixed
            << std::setprecision(4)

            << std::setw(16)
            << m.transfer_path_length

            << std::setw(16)
            << m.transfer_error_m * 1000.0

            << std::setw(18)
            << m.placement_error_m * 1000.0

            << std::setw(14)
            << cart.str()

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
        << "================================================================================================================================================\n";
}


// ======================================================================
// MAIN
// ======================================================================

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
            "multi_pick_and_place_safe",

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


    // 等待TF、joint state、planning scene稳定
    std::this_thread::sleep_for(
        std::chrono::
            seconds(
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
            "Cannot obtain robot state."
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
            "panda_arm not found."
        );


        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();


        return 1;
    }


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
        << "======================================================================\n";


    std::cout
        << "       PANDA SAFE MULTI-OBJECT PICK & PLACE SYSTEM\n";


    std::cout
        << "======================================================================\n";


    std::cout
        << "Planning frame : "
        << planning_frame
        << "\n";


    std::cout
        << "End effector   : "
        << ee_link
        << "\n";


    std::cout
        << "Collision      : ENABLED\n";


    std::cout
        << "Safety margin  : "
        << OBSTACLE_MARGIN_XY * 1000.0
        << " mm around visible obstacle\n";


    // ==================================================================
    // 清理上次场景
    // ==================================================================

    const std::vector<
        std::string
    > cleanup =
    {
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
            milliseconds(
                700
            )
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
            "Safe Home failed."
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
            "Gripper open failed."
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


    // ==================================================================
    // 桌子
    // ==================================================================

    const double table_x =
        home_pose.position.x
        +
        0.13;


    if (!addColoredBox(
            scene,
            planning_frame,

            "safe_table",

            table_x,
            0.0,
            TABLE_CENTER_Z,

            TABLE_SIZE_X,
            TABLE_SIZE_Y,
            TABLE_SIZE_Z,

            makeColor(
                0.40,
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
        "safe_table",
        true
    );


    // ==================================================================
    // 三个不同物体
    //
    // 重点：
    // place_y = +0.27 ~ +0.30
    // 明显远离y=0附近的障碍墙
    // ==================================================================

    std::vector<TaskConfig>
        tasks;


    // ------------------------------------------------------------------
    // RED
    // 35 x 35 x 55 mm
    // 无障碍
    // ------------------------------------------------------------------

    tasks.push_back(
        {
            "Task-1",

            "red_safe_object",

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

            home_pose.position.x + 0.03,
            -0.24,

            home_pose.position.x + 0.03,
             0.27,

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
    // GREEN
    // 45 x 50 x 70 mm
    // 黄色墙
    // ------------------------------------------------------------------

    tasks.push_back(
        {
            "Task-2",

            "green_safe_object",

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

            home_pose.position.x + 0.13,
            -0.24,

            home_pose.position.x + 0.13,
             0.30,

            true,

            "safe_obstacle_2",

            "safe_obstacle_2_guard",

            "YELLOW",

            makeColor(
                0.98,
                0.70,
                0.05,
                1.0
            ),

            0.100,
            0.070,
            0.470
        }
    );


    // ------------------------------------------------------------------
    // BLUE
    // 60 x 40 x 50 mm
    // 紫色墙
    // ------------------------------------------------------------------

    tasks.push_back(
        {
            "Task-3",

            "blue_safe_object",

            "BLUE",

            makeColor(
                0.05,
                0.26,
                0.96,
                1.0
            ),

            0.060,
            0.040,
            0.050,

            home_pose.position.x + 0.23,
            -0.24,

            home_pose.position.x + 0.23,
             0.30,

            true,

            "safe_obstacle_3",

            "safe_obstacle_3_guard",

            "PURPLE",

            makeColor(
                0.66,
                0.12,
                0.90,
                1.0
            ),

            0.120,
            0.080,
            0.500
        }
    );


    // ==================================================================
    // 加入三个物体
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
                "Failed to add %s",
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
            seconds(
                2
            )
    );


    printConfiguration(
        tasks
    );


    std::vector<TaskMetrics>
        metrics_list;


    const auto system_begin =
        std::chrono::
            steady_clock::now();


    // ==================================================================
    // 多任务循环
    // ==================================================================

    for (std::size_t task_number = 0;
         task_number < tasks.size();
         ++task_number)
    {
        const auto &task =
            tasks[task_number];


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
            << "======================================================================\n";


        std::cout
            << "                     "
            << task.name
            << "  "
            << task.color_name
            << "\n";


        std::cout
            << "======================================================================\n";


        std::cout
            << "Object size : "
            << task.sx * 1000.0
            << " x "
            << task.sy * 1000.0
            << " x "
            << task.sz * 1000.0
            << " mm\n";


        std::cout
            << "Pick Y      : "
            << task.pick_y
            << " m\n";


        std::cout
            << "Place Y     : "
            << task.place_y
            << " m\n";


        // ==============================================================
        // 1. Open
        // ==============================================================

        std::cout
            << "\n[1] OPEN GRIPPER\n";


        if (!commandGripper(
                node,
                GRIPPER_OPEN))
        {
            metrics_list.push_back(
                metrics
            );

            break;
        }


        // ==============================================================
        // 2. Pre-Grasp
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
            << "\n[2] PRE-GRASP\n";


        std::cout
            << "EE Z = "
            << pregrasp.position.z
            << " m\n";


        double pregrasp_error =
            999.0;


        if (!moveToPoseSafe(
                move_group,
                pregrasp,
                ee_link,
                pregrasp_error))
        {
            std::cout
                << "PRE-GRASP FAILED\n";


            metrics_list.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "Pre-grasp error = "
            << pregrasp_error * 1000.0
            << " mm\n";


        // ==============================================================
        // 3. Approach Down
        // ==============================================================

        std::cout
            << "\n[3] CARTESIAN APPROACH\n";


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


            metrics_list.push_back(
                metrics
            );

            break;
        }


        metrics.approach_fraction =
            approach_fraction *
            100.0;


        std::cout
            << "Fraction = "
            << metrics.approach_fraction
            << "%\n";


        // ==============================================================
        // 4. Close
        // ==============================================================

        std::cout
            << "\n[4] CLOSE GRIPPER\n";


        // Panda每侧手指最大约40mm。
        // 根据物体Y方向尺寸自动设置。
        const double auto_close =
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
            << "Adaptive finger target = "
            << auto_close
            << " m\n";


        if (!commandGripper(
                node,
                auto_close))
        {
            metrics_list.push_back(
                metrics
            );

            break;
        }


        // ==============================================================
        // 5. Attach
        // ==============================================================

        std::cout
            << "\n[5] ATTACH OBJECT\n";


        const std::vector<
            std::string
        > touch_links =
        {
            "panda_link8",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger"
        };


        if (!move_group.attachObject(
                task.object_id,
                ee_link,
                touch_links)
            ||
            !waitAttached(
                scene,
                task.object_id,
                true,
                3.0))
        {
            std::cout
                << "ATTACH FAILED\n";


            metrics_list.push_back(
                metrics
            );

            break;
        }


        std::cout
            << "Attached successfully.\n";


        // ==============================================================
        // 6. High Lift
        // ==============================================================

        std::cout
            << "\n[6] SAFE HIGH LIFT\n";


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
            metrics_list.push_back(
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
            << "Lift fraction  = "
            << metrics.lift_fraction
            << "%\n";


        std::cout
            << "Transfer EE Z  = "
            << lifted_pose.position.z
            << " m\n";


        std::cout
            << "Above table by = "
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


        // 搬运过程中始终保持高位
        preplace.position.z =
            lifted_pose.position.z;


        // ==============================================================
        // 7. Obstacle verification
        // ==============================================================

        if (task.obstacle_required)
        {
            std::cout
                << "\n[7] SAFE OBSTACLE CALIBRATION\n";


            std::cout
                << "Visible obstacle + enlarged collision envelope\n";


            std::cout
                << "Target is placed far beyond obstacle.\n";


            const auto obstacle =
                calibrateSafeObstacle(
                    move_group,
                    scene,
                    planning_frame,
                    ee_link,
                    task,
                    preplace
                );


            if (!obstacle.success)
            {
                std::cout
                    << "NO STABLE SAFE OBSTACLE FOUND\n";


                metrics_list.push_back(
                    metrics
                );

                break;
            }


            metrics.obstacle_verified =
                true;


            metrics.direct_fraction =
                obstacle.direct_fraction *
                100.0;


            std::cout
                << "\nSAFE OBSTACLE ACCEPTED\n";


            std::cout
                << "Visible obstacle = "
                << obstacle.visible_sx * 1000.0
                << " x "
                << obstacle.visible_sy * 1000.0
                << " x "
                << obstacle.visible_sz * 1000.0
                << " mm\n";


            std::cout
                << "Collision guard = "
                << obstacle.guard_sx * 1000.0
                << " x "
                << obstacle.guard_sy * 1000.0
                << " x "
                << obstacle.guard_sz * 1000.0
                << " mm\n";


            std::cout
                << "Direct Cartesian = "
                << metrics.direct_fraction
                << "% BLOCKED\n";


            std::cout
                << "Stable planner   = "
                << obstacle.verified_planner
                << "\n";


            std::cout
                << "\nRobot will avoid the enlarged guard,"
                << " not merely the visible wall.\n";


            std::this_thread::sleep_for(
                std::chrono::
                    seconds(
                        2
                    )
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
            << "\n[8] FORMAL COLLISION-FREE TRANSFER\n";


        std::cout
            << "Generate RRTConnect/RRT* candidate trajectories...\n";


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


            metrics_list.push_back(
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
            << "Joint path       = "
            << transfer.path_length
            << " rad\n";


        std::cout
            << "Planning time    = "
            << transfer.planning_time
            << " s\n";


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


            metrics_list.push_back(
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
            << "\n[9] CARTESIAN PLACE DESCEND\n";


        // 从当前高位下降到与抓取时相同的腕部高度
        const double target_grasp_z =
            graspZ(task);


        const double current_z =
            move_group.getCurrentPose(
                ee_link
            ).pose.position.z;


        const double descend =
            current_z -
            target_grasp_z;


        std::cout
            << "Descend distance = "
            << descend
            << " m\n";


        double place_fraction =
            0.0;


        double place_error =
            999.0;


        if (descend <= 0.0
            ||
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


            metrics_list.push_back(
                metrics
            );

            break;
        }


        metrics.place_fraction =
            place_fraction *
            100.0;


        std::cout
            << "Fraction = "
            << metrics.place_fraction
            << "%\n";


        // ==============================================================
        // 10. Open
        // ==============================================================

        std::cout
            << "\n[10] RELEASE OBJECT\n";


        if (!commandGripper(
                node,
                GRIPPER_OPEN))
        {
            metrics_list.push_back(
                metrics
            );

            break;
        }


        // ==============================================================
        // 11. Detach
        // ==============================================================

        std::cout
            << "\n[11] DETACH OBJECT\n";


        if (!move_group.detachObject(
                task.object_id)
            ||
            !waitAttached(
                scene,
                task.object_id,
                false,
                3.0))
        {
            std::cout
                << "DETACH FAILED\n";


            metrics_list.push_back(
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
                milliseconds(
                    400
                )
        );


        const auto poses =
            scene.getObjectPoses(
                {task.object_id}
            );


        const auto found =
            poses.find(
                task.object_id
            );


        if (found !=
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
        // 12. Vertical retreat
        // ==============================================================

        std::cout
            << "\n[12] VERTICAL SAFE RETREAT\n";


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


            metrics_list.push_back(
                metrics
            );

            break;
        }


        metrics.retreat_fraction =
            retreat_fraction *
            100.0;


        // 当前任务完成后才移除障碍物。
        // 目标物体保持在桌面上，下一任务仍把它视为碰撞物。
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
            metrics.approach_fraction >=
                98.0
            &&
            metrics.lift_fraction >=
                98.0
            &&
            metrics.place_fraction >=
                98.0
            &&
            metrics.retreat_fraction >=
                98.0
            &&
            metrics.obstacle_verified
            &&
            metrics.transfer_error_m <
                POSITION_TOLERANCE
            &&
            metrics.placement_error_m <
                0.025;


        metrics_list.push_back(
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
                seconds(
                    2
                )
        );
    }


    // ==================================================================
    // Return Home
    // ==================================================================

    std::cout
        << "\n\n";


    std::cout
        << "======================================================================\n";


    std::cout
        << "                         RETURN HOME\n";


    std::cout
        << "======================================================================\n";


    const bool home_success =
        moveHome(
            move_group,
            joint_group,
            home
        );


    // ==================================================================
    // Statistics
    // ==================================================================

    int success_count =
        0;


    double error_sum =
        0.0;


    double max_error =
        0.0;


    int error_count =
        0;


    for (const auto &m :
         metrics_list)
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


            error_sum +=
                mm;


            ++error_count;


            max_error =
                std::max(
                    max_error,
                    mm
                );
        }
    }


    const double mean_error =
        error_count > 0
        ?
        error_sum /
            static_cast<double>(
                error_count
            )
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
        metrics_list
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
        << "======================================================================\n";


    std::cout
        << "                       FINAL SAFETY SUMMARY\n";


    std::cout
        << "======================================================================\n";


    std::cout
        << "Successful tasks       : "
        << success_count
        << " / "
        << tasks.size()
        << "\n";


    std::cout
        << "Table collision model  : ENABLED\n";


    std::cout
        << "Cartesian collision    : ENABLED\n";


    std::cout
        << "Minimum wrist height   : "
        << MIN_WRIST_HEIGHT
        << " m\n";


    std::cout
        << "Visible obstacle guard : +"
        << OBSTACLE_MARGIN_XY *
               1000.0
        << " mm each XY side\n";


    std::cout
        << "Task-2 Place Y         : "
        << tasks[1].place_y
        << " m\n";


    std::cout
        << "Task-3 Place Y         : "
        << tasks[2].place_y
        << " m\n";


    std::cout
        << "Mean placement error   : "
        << mean_error
        << " mm\n";


    std::cout
        << "Max placement error    : "
        << max_error
        << " mm\n";


    std::cout
        << "Total task time        : "
        << system_time
        << " s\n";


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
            final_success
            ?
            "SUCCESS"
            :
            "FAILED"
        )
        << "\n";


    std::cout
        << "======================================================================\n";


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
        "multi_pick_and_place_safe_result.csv";


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
         metrics_list)
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
        << "\nCSV:\n"
        << csv_path
        << "\n";


    std::cout
        << "\nFinal scene stays for 8 seconds...\n";


    std::this_thread::sleep_for(
        std::chrono::
            seconds(
                8
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
