cmake_minimum_required(VERSION 3.8)

project(arm_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
    )
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(control_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(shape_msgs REQUIRED)


function(add_arm_target TARGET SOURCE)

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}")

        add_executable(
            ${TARGET}
            ${SOURCE}
        )

        ament_target_dependencies(
            ${TARGET}
            ${ARGN}
        )

        install(
            TARGETS
            ${TARGET}
            DESTINATION lib/${PROJECT_NAME}
        )

    endif()

endfunction()


# ========================================================================
# 基础运动
# ========================================================================

add_arm_target(
    move_to_pose
    src/move_to_pose.cpp

    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)


add_arm_target(
    waypoint_control
    src/waypoint_control.cpp

    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)


# ========================================================================
# 避障
# ========================================================================

add_arm_target(
    obstacle_demo
    src/obstacle_demo.cpp

    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)


add_arm_target(
    obstacle_benchmark
    src/obstacle_benchmark.cpp

    rclcpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)


add_arm_target(
    obstacle_visual_demo
    src/obstacle_visual_demo.cpp

    rclcpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)


# ========================================================================
# Cartesian
# ========================================================================

add_arm_target(
    cartesian_path
    src/cartesian_path.cpp

    rclcpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
)


# ========================================================================
# 单目标 Pick & Place
# ========================================================================

add_arm_target(
    pick_and_place
    src/pick_and_place.cpp

    rclcpp
    rclcpp_action
    control_msgs
    geometry_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)


# ========================================================================
# 原多目标
# ========================================================================

add_arm_target(
    multi_pick_and_place
    src/multi_pick_and_place.cpp

    rclcpp
    rclcpp_action
    control_msgs
    geometry_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)


# ========================================================================
# Safe V1
# ========================================================================

add_arm_target(
    multi_pick_and_place_safe
    src/multi_pick_and_place_safe.cpp

    rclcpp
    rclcpp_action
    control_msgs
    geometry_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)


# ========================================================================
# Safe V2
# ========================================================================

add_arm_target(
    multi_pick_and_place_safe_v2
    src/multi_pick_and_place_safe_v2.cpp

    rclcpp
    rclcpp_action
    control_msgs
    geometry_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)



add_arm_target(
    multi_pick_and_place_safe_v3
    src/multi_pick_and_place_safe_v3.cpp

    rclcpp
    rclcpp_action
    control_msgs
    geometry_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    shape_msgs
)

install(
    DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
