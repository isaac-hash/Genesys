# C++ Development with the `genesys_macros` Package

The `genesys_macros` package is an auxiliary ROS 2 package provided with the Genesys toolkit. It offers a straightforward, header-only macro system designed to rapidly scaffold simple C++ nodes. This provides an alternative to more complex, feature-rich node-authoring systems and is ideal for quick prototyping or for developers who prefer a more direct, file-based approach to node creation.

## Package Structure

The `genesys_macros` package is a standard ROS 2 `ament_cmake` package with the following key files:

-   **`package.xml`**: Defines the package name (`genesys_macros`), dependencies, and build information. It depends on `ament_cmake`.
-   **`CMakeLists.txt`**: Contains the build logic. It finds necessary dependencies and ensures the `include` directory is correctly exported, making the macros available to other packages that depend on it.
-   **`include/genesys_macros/`**: This directory contains the header files where the macros are defined.
    -   `PublisherMacro.hpp`: Contains macros for creating basic nodes, nodes with timers, and publishing nodes.
    -   `SubscriberMacro.hpp`: Contains a "universal" macro for creating flexible nodes, often used for subscribers.

## Macro Usage Guide

The following macros are designed to generate entire C++ class definitions and, in some cases, the `main()` entrypoint.

---

### `ROS_NODE_CLASS(NODE_NAME)`

-   **Header**: `PublisherMacro.hpp`
-   **Purpose**: Creates a minimal `rclcpp::Node` class. It generates a constructor that logs a startup message.
-   **Parameters**:
    -   `NODE_NAME`: The name for the C++ class and the ROS node.
-   **Example**:

    ```cpp
    #include "genesys_macros/PublisherMacro.hpp"

    // This expands into a class named 'MySimpleNode'
    ROS_NODE_CLASS(MySimpleNode)

    // You still need a main function to run it
    int main(int argc, char* argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MySimpleNode>());
        rclcpp::shutdown();
        return 0;
    }
    ```

---

### `ROS_NODE_WITH_TIMER(NODE_NAME, TIMER_MS, CALLBACK_FN)`

-   **Header**: `PublisherMacro.hpp`
-   **Purpose**: Generates a node containing a recurring wall timer that triggers a callback function.
-   **Parameters**:
    -   `NODE_NAME`: The name for the C++ class and the ROS node.
    -   `TIMER_MS`: The period of the timer in milliseconds.
    -   `CALLBACK_FN`: The name of the member function to be called by the timer. **You must provide the implementation for this function.**
-   **Example**:

    ```cpp
    #include "genesys_macros/PublisherMacro.hpp"

    // Generates class 'MyTimerNode' with a 500ms timer
    ROS_NODE_WITH_TIMER(MyTimerNode, 500, timer_callback)

    // You must implement the callback
    void MyTimerNode::timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Timer fired!");
    }

    // Main function needed to run
    ROS_NODE_MAIN(MyTimerNode)
    ```

---

### `ROS_PUBLISHING_NODE(NODE_NAME, TOPIC, MSG_TYPE, TIMER_MS, CALLBACK_FN)`

-   **Header**: `PublisherMacro.hpp`
-   **Purpose**: Generates a node that includes a publisher and a timer. The timer's callback is intended to be used for publishing messages.
-   **Parameters**:
    -   `NODE_NAME`: The name for the C++ class and the ROS node.
    -   `TOPIC`: A string literal for the topic name (e.g., `"chatter"`).
    -   `MSG_TYPE`: The message type (e.g., `std_msgs::msg::String`).
    -   `TIMER_MS`: The period of the timer in milliseconds.
    -   `CALLBACK_FN`: The name of the callback function where you can access the `publisher_` member to send messages. **You must implement this function.**
-   **Example**:

    ```cpp
    #include "genesys_macros/PublisherMacro.hpp"
    #include "std_msgs/msg/string.hpp"

    ROS_PUBLISHING_NODE(MyPublisher, "hello_world", std_msgs::msg::String, 1000, publish_message)

    void MyPublisher::publish_message()
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello from the macro!";
        publisher_->publish(std::move(msg)); // 'publisher_' is a member of the generated class
    }

    ROS_NODE_MAIN(MyPublisher)
    ```

---

### `ROS_UNIVERSAL_NODE(NODE_NAME, DECLARATIONS, INITIALIZERS)`

-   **Header**: `SubscriberMacro.hpp`
-   **Purpose**: A highly flexible macro for defining a node with custom member variables and constructor logic. It is particularly useful for creating subscribers or other complex nodes.
-   **Parameters**:
    -   `NODE_NAME`: The name for the C++ class and the ROS node.
    -   `DECLARATIONS`: A block of C++ code for declaring member variables (like publishers, subscribers, timers) and member function prototypes.
    -   `INITIALIZERS`: A block of C++ code that will be placed inside the node's constructor to initialize the members defined in `DECLARATIONS`.
-   **Example (Subscriber)**:

    ```cpp
    #include "genesys_macros/SubscriberMacro.hpp"
    #include "std_msgs/msg/string.hpp"

    ROS_UNIVERSAL_NODE(
        MySubscriber,
        // --- Declarations ---
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        void topic_callback(const std_msgs::msg::String::SharedPtr msg);
        , // Note the comma separating the blocks
        // --- Initializers ---
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "hello_world", 10, std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1));
    )

    // You must implement the callback function
    void MySubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    ROS_NODE_MAIN(MySubscriber)
    ```

---

### `ROS_NODE_MAIN(NODE_NAME)`

-   **Header**: `PublisherMacro.hpp`
-   **Purpose**: Generates the standard `main()` function entrypoint needed to run a ROS 2 node. It handles `rclcpp::init`, `rclcpp::spin`, and `rclcpp::shutdown`.
-   **Parameters**:
    -   `NODE_NAME`: The name of the node class to instantiate and spin.
-   **Example**:

    ```cpp
    #include "genesys_macros/PublisherMacro.hpp"

    // Define your node class first
    ROS_NODE_CLASS(MyAwesomeNode)

    // This expands into the main() function for MyAwesomeNode
    ROS_NODE_MAIN(MyAwesomeNode)
    ```
