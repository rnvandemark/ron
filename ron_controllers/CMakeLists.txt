cmake_minimum_required(VERSION 3.5)
project(ron_controllers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ron_common REQUIRED)
find_package(ron_interfaces REQUIRED)

include_directories(include)

add_executable(bridge_node_controller src/bridge_node_controller.cpp)
ament_target_dependencies(
    bridge_node_controller
    rclcpp
    ron_common
    ron_interfaces
)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS bridge_node_controller
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(bridge_node_controller)
ament_export_dependencies(
    ament_cmake
    rclcpp
    ron_common
    ron_interfaces
)
ament_package()
