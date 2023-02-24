#include "ron_controllers/bridge_node_controller.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <ron_common/ros_node_utilities.hpp>

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
        curr_telemetry.configuration.id = node_id;

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

                add_or_remove_parent_ssv = create_service<ron_interfaces::srv::AddOrRemoveBridgeNodeParent>(
                    std::string(get_fully_qualified_name()) + "/add_or_remove_parent",
                    std::bind(
                        &ron_controllers::BridgeNodeController::handle_add_or_remove_parent_requests,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );

                set_host_id_ssv = create_service<ron_interfaces::srv::SetBridgeNodeHost>(
                    std::string(get_fully_qualified_name()) + "/set_host_id",
                    std::bind(
                        &ron_controllers::BridgeNodeController::handle_set_host_id_requests,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );

                telemetry_pub = create_publisher<ron_interfaces::msg::BridgeNodeTelemetry>(
                    std::string(get_fully_qualified_name()) + "/telemetry",
                    1
                );
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

void BridgeNodeController::finish_initializing_telemetry(const double radius)
{
    curr_telemetry.configuration.radius = radius;
    telemetry_pub->publish(curr_telemetry);
}

void BridgeNodeController::handle_add_or_remove_parent_requests(
    const std::shared_ptr<ron_interfaces::srv::AddOrRemoveBridgeNodeParent::Request> req,
    std::shared_ptr<ron_interfaces::srv::AddOrRemoveBridgeNodeParent::Response> res)
{
    // Prove these otherwise
    res->success = false;
    bool need_new_host = false;

    auto& curr_parent_ids = curr_telemetry.parent_ids;
    auto& curr_host_id = curr_telemetry.host_id;

    const bool has_parent = vector_contains(curr_parent_ids, req->parent_id);

    // If we are removing, jump to there
    if (!req->add)
    {
        goto REMOVE;
    }

    // If we are here, then we are attempting to add a parent

    // We cannot add a parent we already have
    if (has_parent)
    {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Tried to add parent_id=" << static_cast<int>(req->parent_id)
                << ", but already have it"
        );
        goto END;
    }

    // All conditions have been met
    curr_parent_ids.push_back(req->parent_id);
    RCLCPP_INFO_STREAM(
        get_logger(),
        "Accepted new parent_id=" << static_cast<int>(req->parent_id)
    );
    goto SUCCESS;

REMOVE:

    // If we are here, then we are attempting to remove a parent

    // We cannot remove a parent that we do not have
    if (!has_parent)
    {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Tried to remove parent_id=" << static_cast<int>(req->parent_id)
                << ", but do not have it"
        );
        goto END;
    }

    // We cannot remove a parent node if we only have one, there would not be
    // a new host node available
    if (curr_parent_ids.size() <= 1)
    {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Refused to remove parent_id=" << static_cast<int>(req->parent_id)
                << " because it is the only current parent"
        );
        goto END;
    }

    // If the caller is attempting to remove the current host node, then more
    // conditions have to be met to be successful because we have to find a new
    // host
    if (req->parent_id == curr_host_id)
    {
        // If the caller specified a required new host robot node, check
        // additional conditions required for success
        if (req->new_host_id >= 0)
        {
            // Ensure the required new host is not the current host (the parent
            // being removed)
            if (req->new_host_id == curr_host_id)
            {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Refused to remove parent_id=" << static_cast<int>(req->parent_id)
                        << " and set new host_id=" << static_cast<int>(req->new_host_id)
                        << " because this is already the current host"
                );
                goto END;
            }
            // Ensure the current parents contains the required new host
            else if (!vector_contains(curr_parent_ids, req->new_host_id))
            {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Refused to remove parent_id=" << static_cast<int>(req->parent_id)
                        << " and set new host_id=" << static_cast<int>(req->new_host_id)
                        << " because this host was not found in the current list of hosts"
                );
                goto END;
            }
            // else conditions have been met
        }
        // else conditions have been met

        need_new_host = true;
    }
    // else conditions have been met

    // All conditions have been met

    // Remove parent
    curr_parent_ids.erase(std::remove_if(
        curr_parent_ids.begin(),
        curr_parent_ids.end(),
        [&req](const int8_t i){ return i == req->parent_id; }
    ));

    // Set new host and log
    if (need_new_host)
    {
        curr_host_id = (req->new_host_id >= 0) ? req->new_host_id : curr_parent_ids[0];
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Removed parent_id=" << static_cast<int>(req->parent_id)
                << " and set new host_id=" << static_cast<int>(curr_host_id)
        );
    }
    // Just log
    else
    {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Removed parent_id=" << static_cast<int>(req->parent_id)
        );
    }
    goto SUCCESS;

SUCCESS:
    res->success = true;
    telemetry_pub->publish(curr_telemetry);

END:
    return;
}

void BridgeNodeController::handle_set_host_id_requests(
    const std::shared_ptr<ron_interfaces::srv::SetBridgeNodeHost::Request> req,
    std::shared_ptr<ron_interfaces::srv::SetBridgeNodeHost::Response> res)
{
    if (req->new_host_id >= 0)
    {
        if (vector_contains(curr_telemetry.parent_ids, req->new_host_id))
        {
            res->success = true;
            curr_telemetry.host_id = req->new_host_id;
            telemetry_pub->publish(curr_telemetry);
            curr_host_id_valid = true;
            RCLCPP_INFO_STREAM(
                get_logger(),
                "Accepted new host_id=" << static_cast<int>(curr_telemetry.host_id)
            );
        }
        else
        {
            res->success = false;
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Denied new host_id=" << static_cast<int>(req->new_host_id)
                    << " because no current parent has this ID"
            );
        }
    }
    else
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Denied new host_id=" << static_cast<int>(req->new_host_id)
                << " because it is an illegal value"
        );
    }
}

}   // namespaces

int main(int argc, char** argv)
{
    // Init ROS network and create the node
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<ron_controllers::BridgeNodeController>();
    if (!(node->is_node_id_valid() && node->are_curr_parent_and_host_ids_valid()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node failed to initialize.");
        return 1;
    }

    // Get RON node static configuration data
    const std::array<std::string, 1> parameter_names({"bridge_node_radius"});
    std::array<rclcpp::ParameterValue, 1> parameter_values;
    const int get_params_rc = ron_common::place_get_parameter_request(
        node,
        "ron_configuration/get_parameters",
        parameter_names,
        parameter_values
    );
    if (get_params_rc != 0)
    {
        return get_params_rc+1;
    }

    // We got the required configuration data, publish initial telem and spin
    node->finish_initializing_telemetry(parameter_values[0].get<double>());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
