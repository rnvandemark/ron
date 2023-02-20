#ifndef RON_TF__BRIDGE_NODE_TF_BROADCASTER_HPP
#define RON_TF__BRIDGE_NODE_TF_BROADCASTER_HPP

#include "ron_tf/ron_node_tf_broadcaster_i.hpp"

namespace ron_tf {

class BridgeNodeTfBroadcaster : public RonNodeTfBroadcasterI
{
public:
    explicit BridgeNodeTfBroadcaster();
};

}   // namespaces

#endif  // #ifndef RON_TF__BRIDGE_NODE_TF_BROADCASTER_HPP
