#include "ron_tf/robot_node_tf_broadcaster.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/utilities.hpp>
#include <ron_common/ros_node_utilities.hpp>

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
    if (!node->is_node_id_valid())
    {
        RCLCPP_ERROR(logger, "Node failed to initialize.");
        return 1;
    }

    // Get RON node static configuration data
    const std::array<std::string, 2> parameter_names({
        "robot_node_bottom_face_translation",
        "robot_node_top_face_translation"
    });
    std::array<rclcpp::ParameterValue, 2> parameter_values;
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

    // We got the required configuration data, publish static TF's and spin
    node->broadcast_static_transforms(
        parameter_values[0].get<double>(),
        parameter_values[1].get<double>()
    );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
