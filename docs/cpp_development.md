# C++ Development with Genesys Macros

The Genesys framework provides a powerful, macro-based declarative system for C++ that mirrors the convenience of the Python decorator API. It is designed to significantly reduce boilerplate code when creating ROS 2 nodes and components.

This guide explains how the system works and how to use the macros provided in `genesys/macros.hpp`.

## How It Works: Static Registration

For those interested in the underlying mechanism, the system is built on a C++ pattern called **static registration**.

1.  **`NodeRegistry`**: A global, templated `NodeRegistry<YourClass>` struct exists for each node class you define. It contains static vectors to store metadata about your desired parameters, publishers, subscribers, and timers.
2.  **Macros and Static Objects**: When you use a macro like `GEN_PARAM` or `GEN_PUB`, it defines a private `static` object inside your class. The constructor of this static object runs when the program is first loaded, before `main()` is even called. This constructor's job is to add the metadata for that parameter or publisher into the class's `NodeRegistry`.
3.  **`GenesysNodeMixin`**: When your node is created, it inherits from `GenesysNodeMixin`. This mixin contains the "engine." During construction (or in the `on_configure` step for lifecycle nodes), it calls an `_init_genesys()` function.
4.  **Wiring**: The `_init_genesys()` function iterates through the now-populated static vectors in the `NodeRegistry`. For each piece of metadata, it calls the appropriate ROS 2 function (`create_publisher`, `create_subscription`, etc.) to build the ROS entities and connect them to your class methods.

This approach allows you to declare the structure of your node, and the framework handles the imperative setup code for you.

---

## Usage Guide

The following macros are intended to be used inside a C++ class definition. The `genesys make node` and `genesys make component` commands will generate files that already use this structure.

### 1. Creating a Node

First, you must define your class using one of the node-creation macros.

#### `GEN_NODE(ClassName, node_name)`
Use this for a standard ROS 2 node.

- **`ClassName`**: The name of your C++ class.
- **`node_name`**: The string name of the ROS 2 node.

```cpp
// In your_node.hpp
#include "genesys/macros.hpp"

class MyNode
{
    GEN_NODE(MyNode, "my_node_name") // No semicolon needed

    // Other macros go here
};
```

#### `GEN_LIFECYCLE_NODE(ClassName, node_name)`
Use this for a lifecycle node. The mixin will automatically handle calling the initialization functions in `on_configure` and managing resource cleanup.

```cpp
// In your_lifecycle_node.hpp
#include "genesys/macros.hpp"

class MyLifecycleNode
{
    GEN_LIFECYCLE_NODE(MyLifecycleNode, "my_lifecycle_node") // No semicolon

    // You can override on_configure, on_activate, etc. if you need custom logic,
    // but be sure to call the base implementation.
};
```
---
### 2. Defining ROS Entities

#### `GEN_PARAM(attr_name, param_name, default_value)`
Declares a ROS 2 parameter and a corresponding public member variable.

- `attr_name`: The name of the C++ member variable.
- `param_name`: The string name of the ROS 2 parameter.
- `default_value`: The default value. The type of the member variable is inferred from this.

```cpp
class MyNode
{
    GEN_NODE(MyNode, "my_node")
    GEN_PARAM(threshold_, "threshold", 100.0) // Creates public member `double threshold_`
};
```
The framework automatically creates a parameter callback. When you change the parameter externally (`ros2 param set`), the `threshold_` member variable in your class instance will be updated.

#### `GEN_PUB(topic_name, msg_type, qos)`
Creates a publisher and a public method for publishing messages.

- `topic_name`: The name of the topic. This also becomes the name of the publish method.
- `msg_type`: The message type (e.g., `std_msgs::msg::String`).
- `qos`: A QoS profile object (e.g., `rclcpp::SensorDataQoS()`).

```cpp
// In your .hpp file
#include <std_msgs/msg/string.hpp>

class MyNode
{
    GEN_NODE(MyNode, "my_node")
    GEN_PUB(chatter, std_msgs::msg::String, rclcpp::SystemDefaultsQoS())
};

// In your .cpp file
// You can now call the 'chatter' method to publish
void some_method() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello";
    this->chatter(msg); // Publishes the message
}
```

#### `GEN_SUB(topic_name, msg_type, qos)`
Creates a subscription and declares the callback method you must implement.

- `topic_name`: The name of the topic to subscribe to. This also becomes the name of the callback method.
- `msg_type`: The message type.
- `qos`: A QoS profile object.

```cpp
// In your .hpp file
#include <std_msgs/msg/string.hpp>

class MyNode
{
    GEN_NODE(MyNode, "my_node")
    GEN_SUB(chatter, std_msgs::msg::String, rclcpp::SystemDefaultsQoS())
};

// In your .cpp file - YOU MUST IMPLEMENT THE METHOD
void MyNode::chatter(std::shared_ptr<const std_msgs::msg::String> msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}
```

#### `GEN_TIMER(method_name, period_ms)`
Creates a wall timer and declares the callback method you must implement.

- `method_name`: The name of the timer callback method.
- `period_ms`: The timer period in **milliseconds**.

```cpp
// In your .hpp file
class MyNode
{
    GEN_NODE(MyNode, "my_node")
    GEN_TIMER(timer_callback, 500) // 500ms = 2Hz
};

// In your .cpp file - YOU MUST IMPLEMENT THE METHOD
void MyNode::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer fired!");
}
```

---
## Generated Code from `genesys make`

When you run `genesys make node --pkg my_cpp_pkg` or `genesys make component --pkg my_cpp_pkg`, the tool generates header (`.hpp`) and source (`.cpp`) files that use this macro system. Your job is simply to fill in the business logic inside the generated method bodies.

For example, a generated C++ publisher component might look like this:

```cpp
// my_component.hpp
#pragma once

#include "genesys/macros.hpp"
#include <std_msgs/msg/string.hpp>

namespace my_cpp_pkg
{

class MyComponent
{
    GEN_LIFECYCLE_NODE(MyComponent, "my_component")

    GEN_PARAM(frequency_, "frequency", 2.0)
    GEN_PUB(output_topic, std_msgs::msg::String, rclcpp::SystemDefaultsQoS())
    GEN_TIMER(timer_callback, 500) // period is calculated from frequency later

private:
    int count_ = 0;
};

} // namespace my_cpp_pkg
```

You would then open the corresponding `my_component.cpp` file and implement the `timer_callback` method to do the work and publish the message.
