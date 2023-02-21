cmake_minimum_required(VERSION 3.5)
project(ron_tf)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ron_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(ron_node_tf_broadcaster_i SHARED src/ron_node_tf_broadcaster_i.cpp)
ament_target_dependencies(
    ron_node_tf_broadcaster_i
    rclcpp
    ron_common
    tf2
    tf2_ros
)

add_executable(bridge_node_tf_broadcaster src/bridge_node_tf_broadcaster.cpp)
target_link_libraries(bridge_node_tf_broadcaster ron_node_tf_broadcaster_i)
ament_target_dependencies(
    bridge_node_tf_broadcaster
    rclcpp
    ron_common
    tf2
    tf2_ros
)

add_executable(robot_node_tf_broadcaster src/robot_node_tf_broadcaster.cpp)
target_link_libraries(robot_node_tf_broadcaster ron_node_tf_broadcaster_i)
ament_target_dependencies(
    robot_node_tf_broadcaster
    geometry_msgs
    rclcpp
    ron_common
    tf2
    tf2_ros
)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS ron_node_tf_broadcaster_i
    EXPORT ron_node_tf_broadcaster_iTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(
    TARGETS bridge_node_tf_broadcaster robot_node_tf_broadcaster
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)
install(
    DIRECTORY bin
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ament_export_include_directories(include)
ament_export_targets(ron_node_tf_broadcaster_iTargets HAS_LIBRARY_TARGET)
ament_export_libraries(bridge_node_tf_broadcaster robot_node_tf_broadcaster)
ament_export_dependencies(
    ament_cmake
    geometry_msgs
    rclcpp
    ron_common
    tf2
    tf2_ros
)
ament_package()
