#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ron_configuration");
    node->declare_parameter("robot_node_bottom_face_translation", 0.0);
    node->declare_parameter("robot_node_top_face_translation", 0.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
