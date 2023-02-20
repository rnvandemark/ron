#include "ron_tf/robot_node_tf_broadcaster.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/utilities.hpp>

namespace ron_tf {

RobotNodeTfBroadcaster::RobotNodeTfBroadcaster() :
    RonNodeTfBroadcasterI("robot_node")
{
    // Initialize the static transform broadcaster
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void RobotNodeTfBroadcaster::broadcast_static_transforms(
    const double bottom_face_translation,
    const double top_face_translation)
{
    const rclcpp::Time now = get_clock()->now();
    static_tf_broadcaster->sendTransform(build_stamped_tf(
        tf_root,
        tf_root + "_bottom_face",
        0.0,
        0.0,
        bottom_face_translation,
        0.0,
        0.0,
        0.0,
        now
    ));
    static_tf_broadcaster->sendTransform(build_stamped_tf(
        tf_root,
        tf_root + "_top_face",
        0.0,
        0.0,
        top_face_translation,
        0.0,
        0.0,
        0.0,
        now
    ));
}

}   // namespaces

int main(int argc, char* argv[])
{
    using namespace std::chrono_literals;

    auto logger = rclcpp::get_logger("rclcpp");

    // Init ROS network and create the node
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<ron_tf::RobotNodeTfBroadcaster>();
    if (!node->isNodeIdValid())
    {
        RCLCPP_ERROR(logger, "Node failed to initialize.");
        return 1;
    }

    // Create service client to get RON node static configuration data
    auto get_parameters_scl = node->create_client<rcl_interfaces::srv::GetParameters>(
        "ron_configuration/get_parameters"
    );
    while (!get_parameters_scl->wait_for_service(1s))
    {
        if (rclcpp::ok())
        {
            RCLCPP_INFO(logger, "'get_parameters' service not available, waiting again...");
        }
        else
        {
            RCLCPP_ERROR(logger, "Interrupted while waiting for the service.");
            return 2;
        }
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {
        "robot_node_bottom_face_translation",
        "robot_node_top_face_translation"
    };

    // We got the requird configuration data, publish static TF's and spin
    auto result = get_parameters_scl->async_send_request(request);
    if (rclcpp::FutureReturnCode::SUCCESS == rclcpp::spin_until_future_complete(node, result))
    {
        node->broadcast_static_transforms(
            result.get()->values[0].double_value,
            result.get()->values[1].double_value
        );
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to get configuration data");
        return 3;
    }

    return 0;
}
