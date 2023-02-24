#include "ron_controllers/bridge_node_controller.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

namespace {
    template <typename T>
    std::string vector_to_string(const std::vector<T>& v)
    {
        if (v.empty())
        {
            return "[]";
        }

        auto iter = v.cbegin();
        std::ostringstream ss;
        ss << "[" << *iter;
        ++iter;
        do
        {
            ss << ", " << *iter;
            ++iter;
        }
        while (iter != v.cend());
        ss << "]";

        return ss.str();
    }

    template <>
    std::string vector_to_string(const std::vector<int8_t>& v)
    {
        if (v.empty())
        {
            return "[]";
        }

        auto iter = v.cbegin();
        std::ostringstream ss;
        ss << "[" << static_cast<int>(*iter);
        ++iter;
        do
        {
            ss << ", " << static_cast<int>(*iter);
            ++iter;
        }
        while (iter != v.cend());
        ss << "]";

        return ss.str();
    }

    template <typename T>
    bool vector_contains(const std::vector<T>& v, const T& e)
    {
        return v.cend() != std::find(v.cbegin(), v.cend(), e);
    }

    template <typename T>
    bool vector_contains_unique(const std::vector<T>& v)
    {
        return v.size() == std::set(v.cbegin(), v.cend()).size();
    }
}

namespace ron_controllers {

BridgeNodeController::BridgeNodeController() :
    ron_common::RonNodeNode("bridge_node_controller")
{
    // Only bother continuing if base and this class met requirements
    if (node_id_valid)
    {
        const auto initial_parent_ids_parameter_value = declare_parameter(
            "initial_parent_ids",
            rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY
        ).get<std::vector<int64_t>>();
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Initial unmanipulated parent_ids=" << vector_to_string(initial_parent_ids_parameter_value)
        );
        curr_telemetry.parent_ids.resize(initial_parent_ids_parameter_value.size());
        std::transform(
            initial_parent_ids_parameter_value.cbegin(),
            initial_parent_ids_parameter_value.cend(),
            curr_telemetry.parent_ids.begin(),
            [](const int64_t i){ return static_cast<int8_t>(i); }
        );
        curr_parent_ids_valid = (curr_telemetry.parent_ids.size() > 0)
            && vector_contains_unique(curr_telemetry.parent_ids)
        ;

        if (curr_parent_ids_valid)
        {
            RCLCPP_INFO_STREAM(
                get_logger(),
                "Accepted initial parent_ids=" << vector_to_string(curr_telemetry.parent_ids)
            );

            curr_telemetry.host_id = declare_parameter("initial_host_id", -1);
            curr_host_id_valid = (curr_telemetry.host_id >= 0)
                && vector_contains(curr_telemetry.parent_ids, curr_telemetry.host_id)
            ;

            if (curr_host_id_valid)
            {
                RCLCPP_INFO_STREAM(
                    get_logger(),
                    "Accepted initial host_id=" << static_cast<int>(curr_telemetry.host_id)
                );

                telemetry_pub = create_publisher<ron_interfaces::msg::BridgeNodeTelemetry>(
                    std::string(get_fully_qualified_name()) + "/telemetry",
                    1
                );
                telemetry_pub->publish(curr_telemetry);
            }
            else
            {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Invalid host_id=" << static_cast<int>(curr_telemetry.host_id)
                );
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Invalid parent_ids=" << vector_to_string(curr_telemetry.parent_ids)
            );
        }
    }
    // else base class already logged an error
}

bool BridgeNodeController::are_curr_parent_and_host_ids_valid() const
{
    return curr_parent_ids_valid && curr_host_id_valid;
}

}   // namespaces

int main(int argc, char** argv)
{
    // Init ROS network, create the node, and spin it
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<ron_controllers::BridgeNodeController>();
    if (!(node->is_node_id_valid() && node->are_curr_parent_and_host_ids_valid()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node failed to initialize.");
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
