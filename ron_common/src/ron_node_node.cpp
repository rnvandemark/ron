#include "ron_common/ron_node_node.hpp"

namespace ron_common {

RonNodeNode::RonNodeNode(const std::string& name) :
    rclcpp::Node(name),
    node_id(declare_parameter("node_id", -1)),
    node_id_valid(node_id >= 0)
{
    if (node_id_valid)
    {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Started with node_id=" << static_cast<int>(node_id)
        );
    }
    else
    {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Invalid node_id=" << static_cast<int>(node_id)
        );
    }
}

int8_t RonNodeNode::get_node_id() const
{
    return node_id;
}
bool RonNodeNode::is_node_id_valid() const
{
    return node_id_valid;
}

}   // namespaces
