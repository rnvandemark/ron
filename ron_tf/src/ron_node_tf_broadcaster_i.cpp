#include "ron_tf/ron_node_tf_broadcaster_i.hpp"

#include <tf2/LinearMath/Quaternion.h>

namespace ron_tf {

RonNodeTfBroadcasterI::RonNodeTfBroadcasterI(const char* name_prefix) :
        rclcpp::Node(std::string(name_prefix) + "_tf_broadcaster")
{
    // Declare and get required node ID, ensuring it's a proper value
    node_id = declare_parameter("node_id", -1);
    node_id_valid = (node_id >= 0);
    if (node_id_valid)
    {
        // Set the optionally configurable TF name root
        tf_root = declare_parameter(
            "tf_root",
            std::string(name_prefix) + "_" + std::to_string(node_id)
        );

        // Initialize the transform broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO_STREAM(
            get_logger(),
            "Started with node_id=" << node_id << ", tf_root=\""
                << tf_root << "\""
        );
    }
}

bool RonNodeTfBroadcasterI::isNodeIdValid() const
{
    return node_id_valid;
}
int RonNodeTfBroadcasterI::getNodeId() const
{
    return node_id;
}
std::string RonNodeTfBroadcasterI::getTfRoot() const
{
    return tf_root;
}

geometry_msgs::msg::TransformStamped RonNodeTfBroadcasterI::build_stamped_tf(
    const std::string& parent_frame,
    const std::string& child_frame,
    const double tx,
    const double ty,
    const double tz,
    const double r,
    const double p,
    const double y,
    const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = stamp;
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;

    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.translation.z = tz;

    tf2::Quaternion q;
    q.setRPY(r,p,y);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
}

geometry_msgs::msg::TransformStamped RonNodeTfBroadcasterI::build_stamped_tf_from_now(
    const std::string& parent_frame,
    const std::string& child_frame,
    const double tx,
    const double ty,
    const double tz,
    const double r,
    const double p,
    const double y)
{
    return build_stamped_tf(
        parent_frame,
        child_frame,
        tx,
        ty,
        tz,
        r,
        p,
        y,
        get_clock()->now()
    );
}

}   // namespaces
