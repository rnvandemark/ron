# The overall telemetry of a single bridge node. A bridge node has 1 or more
# parent robot nodes at all times, which is just a robot node that is currently
# attached to this bridge node. Any one of these at a time is its host.

# The static configuration data of this bridge node
BridgeNodeConfiguration configuration

# The unique ID of each current parent robot node
uint8[] parent_ids

# The unique ID of the current host robot node, which is used by this bridge
# node to resolve its pose relative to the world
uint8 host_id
