#ifndef RON_CONTROLLERS__BRIDGE_NODE_CONTROLLER_HPP
#define RON_CONTROLLERS__BRIDGE_NODE_CONTROLLER_HPP

#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <ron_common/ron_node_node.hpp>
#include <ron_interfaces/msg/bridge_node_telemetry.hpp>
#include <ron_interfaces/srv/add_or_remove_bridge_node_parent.hpp>
#include <ron_interfaces/srv/set_bridge_node_host.hpp>

namespace ron_controllers {

class BridgeNodeController : public ron_common::RonNodeNode
{
protected:
    bool curr_host_id_valid;
    bool curr_parent_ids_valid;

    rclcpp::Service<ron_interfaces::srv::AddOrRemoveBridgeNodeParent>::SharedPtr add_or_remove_parent_ssv;

    rclcpp::Service<ron_interfaces::srv::SetBridgeNodeHost>::SharedPtr set_host_id_ssv;

    ron_interfaces::msg::BridgeNodeTelemetry curr_telemetry;
    rclcpp::Publisher<ron_interfaces::msg::BridgeNodeTelemetry>::SharedPtr telemetry_pub;

public:
    explicit BridgeNodeController();

    bool are_curr_parent_and_host_ids_valid() const;

    void finish_initializing_telemetry(const double radius);

protected:
    void handle_add_or_remove_parent_requests(
        const std::shared_ptr<ron_interfaces::srv::AddOrRemoveBridgeNodeParent::Request> req,
        std::shared_ptr<ron_interfaces::srv::AddOrRemoveBridgeNodeParent::Response> res
    );

    void handle_set_host_id_requests(
        const std::shared_ptr<ron_interfaces::srv::SetBridgeNodeHost::Request> req,
        std::shared_ptr<ron_interfaces::srv::SetBridgeNodeHost::Response> res
    );
};

}   // namespaces

#endif  // #ifndef RON_CONTROLLERS__BRIDGE_NODE_CONTROLLER_HPP
