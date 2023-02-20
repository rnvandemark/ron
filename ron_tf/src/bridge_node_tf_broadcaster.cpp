#include "ron_tf/bridge_node_tf_broadcaster.hpp"

namespace ron_tf {

BridgeNodeTfBroadcaster::BridgeNodeTfBroadcaster() :
    RonNodeTfBroadcasterI("bridge_node")
{
}

}   // namespaces

int main(int argc, char* argv[])
{
    // Init ROS network, create the node, and spin it
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<ron_tf::BridgeNodeTfBroadcaster>();
    if (!node->isNodeIdValid())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node failed to initialize.");
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
