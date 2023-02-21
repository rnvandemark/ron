cmake_minimum_required(VERSION 3.5)
project(ron_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(ron_node_node SHARED src/ron_node_node.cpp)
ament_target_dependencies(
    ron_node_node
    rclcpp
)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS ron_node_node
    EXPORT ron_node_nodeTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(ron_node_nodeTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    rclcpp
)
ament_package()
