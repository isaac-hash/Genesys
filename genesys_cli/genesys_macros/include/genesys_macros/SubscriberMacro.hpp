#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// -------------------------------------------------------------
// UNIVERSAL ROS NODE MACRO
// 
// NODE_NAME: Name of the class and the node
// DECLARATIONS: Put your member variables and callback definitions here
// INITIALIZERS: Put your create_subscription/publisher logic here
// -------------------------------------------------------------
#define ROS_UNIVERSAL_NODE(NODE_NAME, DECLARATIONS, INITIALIZERS) \
class NODE_NAME : public rclcpp::Node \
{ \
public: \
    NODE_NAME() : rclcpp::Node(#NODE_NAME) \
    { \
        INITIALIZERS \
        RCLCPP_INFO(this->get_logger(), #NODE_NAME " initialized."); \
    } \
private: \
    DECLARATIONS \
};
