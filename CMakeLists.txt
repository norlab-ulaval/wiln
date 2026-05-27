cmake_minimum_required(VERSION 3.8)
project(wiln)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(norlab_icp_mapper_ros REQUIRED)
find_package(norlab_controllers_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/WilnState.msg"
    "msg/ReplayDiagnostics.msg"
    DEPENDENCIES builtin_interfaces
)

# -----------------------------------------------------------------------
# Common include dirs and Eigen target (reused across executables)
# -----------------------------------------------------------------------
set(WILN_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
)

set(WILN_COMMON_DEPS
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    sensor_msgs
    visualization_msgs
    norlab_controllers_msgs
)

# -----------------------------------------------------------------------
# wiln_obstacle_node -- Phase 1: standalone obstacle aggregation node
# -----------------------------------------------------------------------
add_executable(wiln_obstacle_node
    src/wiln_obstacle_node.cpp
    src/ObstacleManager.cpp
)

target_include_directories(wiln_obstacle_node PUBLIC ${WILN_INCLUDE_DIRS})

ament_target_dependencies(wiln_obstacle_node
    ${WILN_COMMON_DEPS}
)

# -----------------------------------------------------------------------
# wiln_teach_node -- Phase 2: teach / recording node
# -----------------------------------------------------------------------
add_executable(wiln_teach_node
    src/wiln_teach_node.cpp
    src/TeachRecorder.cpp
)

target_include_directories(wiln_teach_node PUBLIC ${WILN_INCLUDE_DIRS})

ament_target_dependencies(wiln_teach_node
    ${WILN_COMMON_DEPS}
)

# -----------------------------------------------------------------------
# wiln_route_node -- Phase 2: route save/load node
# -----------------------------------------------------------------------
add_executable(wiln_route_node
    src/wiln_route_node.cpp
    src/TrajectoryManager.cpp
)

target_include_directories(wiln_route_node PUBLIC ${WILN_INCLUDE_DIRS})

ament_target_dependencies(wiln_route_node
    ${WILN_COMMON_DEPS}
    norlab_icp_mapper_ros
    std_srvs
)

# -----------------------------------------------------------------------
# wiln_replay_node -- Phase 3: hot-path 10 Hz replay node
# -----------------------------------------------------------------------
add_executable(wiln_replay_node
    src/wiln_replay_node.cpp
    src/TrajectoryStreamer.cpp
    src/PathDeformer.cpp
)

target_include_directories(wiln_replay_node PUBLIC ${WILN_INCLUDE_DIRS})

ament_target_dependencies(wiln_replay_node
    ${WILN_COMMON_DEPS}
    norlab_icp_mapper_ros
    std_srvs
)

# -----------------------------------------------------------------------
# wiln_path_follower -- Phase 4: 20 Hz curvature-based steering controller
# -----------------------------------------------------------------------
add_executable(wiln_path_follower
    src/PathFollower.cpp
)

target_include_directories(wiln_path_follower PUBLIC ${WILN_INCLUDE_DIRS})

ament_target_dependencies(wiln_path_follower
    ${WILN_COMMON_DEPS}
)

# -----------------------------------------------------------------------
# Link generated message typesupport to all executables
# -----------------------------------------------------------------------
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wiln_obstacle_node  "${cpp_typesupport_target}")
target_link_libraries(wiln_teach_node     "${cpp_typesupport_target}")
target_link_libraries(wiln_route_node     "${cpp_typesupport_target}")
target_link_libraries(wiln_replay_node    "${cpp_typesupport_target}")
target_link_libraries(wiln_path_follower  "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------------------------------------------------
# Python scripts (safety monitor)
# -----------------------------------------------------------------------
install(PROGRAMS
    scripts/wiln_safety_monitor.py
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    wiln_obstacle_node
    wiln_teach_node
    wiln_route_node
    wiln_replay_node
    wiln_path_follower
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
