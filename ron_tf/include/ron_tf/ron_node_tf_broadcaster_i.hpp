#ifndef RON_TF__RON_NODE_TF_BROADCASTER_I_HPP
#define RON_TF__RON_NODE_TF_BROADCASTER_I_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace ron_tf {

class RonNodeTfBroadcasterI : public rclcpp::Node
{
protected:
    int node_id;
    bool node_id_valid;

    std::string tf_root;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

public:
    explicit RonNodeTfBroadcasterI(const char* name_prefix);

    bool isNodeIdValid() const;
    int getNodeId() const;
    std::string getTfRoot() const;

protected:
    static geometry_msgs::msg::TransformStamped build_stamped_tf(
        const std::string& parent_frame,
        const std::string& child_frame,
        const double tx,
        const double ty,
        const double tz,
        const double r,
        const double p,
        const double y,
        const rclcpp::Time& stamp
    );
    
    geometry_msgs::msg::TransformStamped build_stamped_tf_from_now(
        const std::string& parent_frame,
        const std::string& child_frame,
        const double tx,
        const double ty,
        const double tz,
        const double r,
        const double p,
        const double y
    );
};

}   // namespaces

#endif  // #ifndef RON_TF__RON_NODE_TF_BROADCASTER_I_HPP
