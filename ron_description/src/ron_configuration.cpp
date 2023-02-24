#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
    // Create node to host configuration data
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ron_configuration");

    // Bridge node configuration data

    node->declare_parameter("bridge_node_radius", 0.0);

    // Robot node configuration data

    node->declare_parameter("robot_node_bottom_face_translation", 0.0);
    node->declare_parameter("robot_node_top_face_translation", 0.0);

    // Nothing else for this node to ever do, spin until shutdown
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
