#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// -------------------------------------------------------------------
// Base macro that generates a full ROS2 node class
// -------------------------------------------------------------------
#define ROS_NODE_CLASS(NODE_NAME)                                            \
class NODE_NAME : public rclcpp::Node                                       \
{                                                                            \
public:                                                                      \
    NODE_NAME() : rclcpp::Node(#NODE_NAME) {                                 \
        RCLCPP_INFO(this->get_logger(), #NODE_NAME " started");              \
    }                                                                        \
};

// -------------------------------------------------------------------
// Macro to add a timer + callback
// -------------------------------------------------------------------
#define ROS_NODE_WITH_TIMER(NODE_NAME, TIMER_MS, CALLBACK_FN)                \
class NODE_NAME : public rclcpp::Node                                       \
{                                                                            \
public:                                                                      \
    NODE_NAME() : rclcpp::Node(#NODE_NAME)                                   \
    {                                                                        \
        timer_ = this->create_wall_timer(                                    \
            std::chrono::milliseconds(TIMER_MS),                             \
            std::bind(&NODE_NAME::CALLBACK_FN, this));                       \
        RCLCPP_INFO(this->get_logger(), #NODE_NAME " started with timer");   \
    }                                                                        \
private:                                                                     \
    void CALLBACK_FN();                                                      \
    rclcpp::TimerBase::SharedPtr timer_;                                     \
};

// -------------------------------------------------------------------
// Macro to add a publisher+timer node
// -------------------------------------------------------------------
#define ROS_PUBLISHING_NODE(NODE_NAME, TOPIC, MSG_TYPE, TIMER_MS, CALLBACK_FN)\
class NODE_NAME : public rclcpp::Node                                       \
{                                                                            \
public:                                                                      \
    NODE_NAME() : rclcpp::Node(#NODE_NAME)                                   \
    {                                                                        \
        publisher_ = this->create_publisher<MSG_TYPE>(TOPIC, 10);            \
        timer_ = this->create_wall_timer(                                    \
            std::chrono::milliseconds(TIMER_MS),                             \
            std::bind(&NODE_NAME::CALLBACK_FN, this));                       \
        RCLCPP_INFO(this->get_logger(), #NODE_NAME " publisher started");    \
    }                                                                        \
protected:                                                                     \
    void CALLBACK_FN();                                                      \
    rclcpp::Publisher<MSG_TYPE>::SharedPtr publisher_;                       \
    rclcpp::TimerBase::SharedPtr timer_;                                     \
};

// -------------------------------------------------------------------
// Macro to generate the main() entrypoint
// -------------------------------------------------------------------
#define ROS_NODE_MAIN(NODE_NAME)                                             \
int main(int argc, char * argv[])                                            \
{                                                                            \
    rclcpp::init(argc, argv);                                                \
    rclcpp::spin(std::make_shared<NODE_NAME>());                             \
    rclcpp::shutdown();                                                      \
    return 0;                                                                \
}
