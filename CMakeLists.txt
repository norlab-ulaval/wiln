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
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(norlab_icp_mapper_ros REQUIRED)
find_package(norlab_controllers_msgs REQUIRED)
find_package(service_caller REQUIRED)

find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/LoadMapTraj.srv"
    "srv/PlayLoop.srv"
    "srv/SaveMapTraj.srv"
    DEPENDENCIES std_msgs std_srvs
)

add_executable(wiln_node 
    src/WilnNode.cpp
    src/TeachRecorder.cpp
    src/TrajectoryStreamer.cpp
    src/PathDeformer.cpp
    src/TrajectoryManager.cpp
)

target_include_directories(wiln_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(wiln_node
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    tf2_ros
    tf2
    rclcpp_action
    norlab_icp_mapper_ros
    norlab_controllers_msgs
    service_caller
    tf2_geometry_msgs
    sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wiln_node "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS wiln_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
