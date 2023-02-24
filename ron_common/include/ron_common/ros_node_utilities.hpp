#ifndef RON_COMMON__ROS_NODE_UTILITIES_HPP
#define RON_COMMON__ROS_NODE_UTILITIES_HPP

#include <chrono>

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/utilities.hpp>

namespace ron_common {

template <typename NodeT, size_t N>
int place_get_parameter_request(
    std::shared_ptr<NodeT>& node,
    const std::string& get_parameters_topic,
    const std::array<std::string, N>& parameter_names,
    std::array<rclcpp::ParameterValue, N>& parameter_values)
{
    using namespace std::chrono_literals;

    auto logger = rclcpp::get_logger("rclcpp");

    auto get_parameters_scl = node->template create_client<rcl_interfaces::srv::GetParameters>(get_parameters_topic);
    while (!get_parameters_scl->wait_for_service(1s))
    {
        if (rclcpp::ok())
        {
            RCLCPP_INFO(logger, "'get_parameters' service not available, waiting again...");
        }
        else
        {
            RCLCPP_ERROR(logger, "Interrupted while waiting for the service.");
            return 1;
        }
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = std::vector<std::string>(parameter_names.cbegin(), parameter_names.cend());

    auto result = get_parameters_scl->async_send_request(request);
    if (rclcpp::FutureReturnCode::SUCCESS != rclcpp::spin_until_future_complete(node, result))
    {
        RCLCPP_ERROR(logger, "Failed to get configuration data");
        return 2;
    }

    const auto& parameter_values_msg = result.get()->values;
    const size_t expected_num_values = N;
    const size_t actual_num_values = parameter_values_msg.size();
    if (actual_num_values != expected_num_values)
    {
        RCLCPP_ERROR_STREAM(
            logger,
            "Did not receive expected number of parameter values ("
                << expected_num_values << " != " << actual_num_values << ")"
        );
        return 3;
    }

    std::transform(
        parameter_values_msg.cbegin(),
        parameter_values_msg.cend(),
        parameter_values.begin(),
        [](const rcl_interfaces::msg::ParameterValue& pv){ return rclcpp::ParameterValue(pv); }
    );
    return 0;
}

}   // namespaces

#endif  // #ifndef RON_COMMON__ROS_NODE_UTILITIES_HPP
