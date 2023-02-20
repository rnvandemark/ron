#ifndef RON_TF__ROBOT_NODE_TF_BROADCASTER_HPP
#define RON_TF__ROBOT_NODE_TF_BROADCASTER_HPP

#include <tf2_ros/static_transform_broadcaster.h>

#include "ron_tf/ron_node_tf_broadcaster_i.hpp"

namespace ron_tf {

class RobotNodeTfBroadcaster : public RonNodeTfBroadcasterI
{
protected:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

public:
    explicit RobotNodeTfBroadcaster();

    void broadcast_static_transforms(
        const double bottom_face_translation,
        const double top_face_translation
    );
};

}   // namespaces

#endif  // #ifndef RON_TF__ROBOT_NODE_TF_BROADCASTER_HPP
