#ifndef RON_CONTROLLERS__BRIDGE_NODE_CONTROLLER_HPP
#define RON_CONTROLLERS__BRIDGE_NODE_CONTROLLER_HPP

#include <rclcpp/publisher.hpp>
#include <ron_common/ron_node_node.hpp>
#include <ron_interfaces/msg/bridge_node_telemetry.hpp>

namespace ron_controllers {

class BridgeNodeController : public ron_common::RonNodeNode
{
protected:
    bool curr_host_id_valid;
    bool curr_parent_ids_valid;

    ron_interfaces::msg::BridgeNodeTelemetry curr_telemetry;
    rclcpp::Publisher<ron_interfaces::msg::BridgeNodeTelemetry>::SharedPtr telemetry_pub;

public:
    explicit BridgeNodeController();

    bool are_curr_parent_and_host_ids_valid() const;
};

}   // namespaces

#endif  // #ifndef RON_CONTROLLERS__BRIDGE_NODE_CONTROLLER_HPP
