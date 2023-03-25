cmake_minimum_required(VERSION 3.5)
project(ron_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(ron_configuration src/ron_configuration.cpp)
ament_target_dependencies(
    ron_configuration
    rclcpp
)

install(
    DIRECTORY config launch urdf
    DESTINATION share/${PROJECT_NAME}
)
install(
    TARGETS ron_configuration
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(ron_configuration)
ament_export_dependencies(
    ament_cmake
    rclcpp
)
ament_package()
