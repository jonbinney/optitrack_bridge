# optitrack_bridge

ROS node optitrack_bridge_node, listen to pose multicasted via UDP by optitrack system, and republish it as ROS messages.

- parameter `address`, multicast address for optitrack system, default to `239.255.42.99`
- parameter `port`, multicast address for optitrack system, default to `1511`
- parameter `body_name`, name of rigid body published by optitrack, default to `base_link`
- parameter `body_frame_id`, frame_id of rigid body published to ROS, default to the name from optitrack
- parameter `map_frame_id`, frame_id of map, default to `map`
- parameter `fps`, desired frame rate for pose messages, if fall below 80% than this, would generate a diagnostic error, default to `120`


- publish topic `tf`, the topic for map to body transformations
- publish topic `pose`, for body pose relative to map
