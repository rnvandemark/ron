#ifndef RON_COMMON__RON_NODE_NODE_HPP
#define RON_COMMON__RON_NODE_NODE_HPP

#include <rclcpp/node.hpp>

namespace ron_common {

class RonNodeNode : public rclcpp::Node
{
protected:
    const int node_id;
    const bool node_id_valid;

public:
    explicit RonNodeNode(const std::string& name);

    int get_node_id() const;
    bool is_node_id_valid() const;
};

}   // namespaces

#endif  // #ifndef RON_COMMON__RON_NODE_NODE_HPP
