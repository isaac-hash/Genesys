# Genesys Python API: Decorators and Declarative Nodes

The Genesys framework provides a powerful, declarative Python API for creating ROS 2 nodes. Instead of writing boilerplate `__init__` methods to manually create publishers, subscribers, and other ROS entities, you can simply decorate your classes and methods to define their behavior.

This document provides a comprehensive guide to the entire Python API, from the core decorators to advanced features like lifecycle nodes and dynamic configuration.

## How It Works: The `NodeBase` Engine

The magic behind the declarative API is the `genesys.node_base.NodeBase` class. When you apply the top-level `@node` decorator to your class, Genesys wraps your class in a `NodeBase` instance. Here’s a high-level overview of the process:

1.  **Instantiation:** The `NodeBase` wrapper is instantiated, creating an instance of your user-defined class within it.
2.  **Injection:** `NodeBase` injects useful ROS 2 utilities into your class instance, making `self.logger` (for logging) and `self.get_clock` available.
3.  **Inspection:** `NodeBase` uses Python's `inspect` module to scan all of your class's methods and attributes.
4.  **Discovery:** It looks for special metadata attached by the Genesys decorators (e.g., a method decorated with `@subscriber` will have a `_ros_subscribers` attribute). It also finds declarative attributes like `parameter()` and `action_client()`.
5.  **Wiring:** Based on the discovered metadata, `NodeBase` automatically:
    *   Creates all the necessary ROS 2 publishers, subscribers, timers, services, and action servers.
    *   Connects them to the correct callback methods in your class.
    *   Initializes ROS 2 parameters and sets up automatic updates.
    *   Creates action clients.
    *   Wraps methods where necessary (e.g., for publishers) to handle return values.

This automated setup significantly reduces boilerplate code and lets you focus purely on the logic of your node.

---

## The Top-Level `@node` Decorator

This is the main entry point for creating a Genesys node. It must be applied to the class you intend to run as a node.

```python
from genesys.decorators import node

@node('my_first_node')
class MyNode:
    # ... your logic here ...
```

### Parameters
- **`node_name`** (str): The name of the ROS 2 node.

### How It Works
The `@node` decorator is responsible for creating the `NodeBase` wrapper around your class. It determines whether to create a standard `rclpy.node.Node` or a `rclpy.lifecycle.LifecycleNode` based on whether the `@lifecycle_node` decorator is also present.

---

## Communication Decorators

These decorators are used on methods within your `@node`-decorated class to set up ROS 2 communication channels.

### `@publisher(topic, msg_type, qos="default")`

Decorates a method that produces messages. The decorated method's return value will be automatically published to the specified topic.

- **`topic`** (str | `Parameter`): The name of the topic to publish to. Can be a string or a `Parameter` for dynamic configuration.
- **`msg_type`**: The ROS 2 message type (e.g., `std_msgs.msg.String`).
- **`qos`**: A QoS profile. Can be a `QoSProfile` object or a string alias (see QoS section).

**Example:**
```python
from std_msgs.msg import String
from genesys.decorators import publisher, timer

@timer(1.0)
@publisher('/my_topic', String)
def publish_message(self):
    msg = String()
    msg.data = f'Current time: {self.get_clock().now()}'
    return msg # This return value is published
```

#### How It Works
The `@publisher` decorator attaches metadata to the `publish_message` method. When `NodeBase` finds this, it creates a `rclpy.Publisher`. It then *wraps* the original `publish_message` method. The wrapper function calls your original method, and if the return value is not `None`, it calls `publish()` on it. The `@timer` decorator then uses this wrapped function as its callback.

### `@subscriber(topic, msg_type, qos="default", debug_log=False)`

Decorates a method that will be used as a callback for a topic subscription.

- **`topic`** (str | `Parameter`): The name of the topic to subscribe to.
- **`msg_type`**: The ROS 2 message type.
- **`qos`**: The QoS profile or string alias.
- **`debug_log`** (bool): If `True`, a `DEBUG` level log message will be printed every time a message is received.

**Example:**
```python
from std_msgs.msg import String
from genesys.decorators import subscriber

@subscriber('/my_topic', String)
def topic_callback(self, msg: String):
    self.logger.info(f'I heard: "{msg.data}"')
```

#### How It Works
`NodeBase` discovers the `_ros_subscribers` metadata on the `topic_callback` method and calls `self.create_subscription()`, passing the method itself as the callback. If `debug_log` is enabled, it first wraps the callback in a function that logs the event.

### `@timer(period_sec)`

Decorates a method to be called at a fixed interval.

- **`period_sec`** (float): The timer period in seconds.

**Example:**
```python
from genesys.decorators import timer

@timer(2.5)
def periodic_task(self):
    self.logger.info('This runs every 2.5 seconds.')
```

#### How It Works
`NodeBase` finds the `_ros_timers` metadata and calls `self.create_timer()`, passing the decorated method as the callback.

### `@service(service_name, service_type)`

Decorates a method to act as a callback for a ROS 2 service. The method signature must include `request` and `response` arguments.

- **`service_name`** (str | `Parameter`): The name of the service.
- **`service_type`**: The ROS 2 service type (e.g., `std_srvs.srv.SetBool`).

**Example:**
```python
from std_srvs.srv import SetBool
from genesys.decorators import service

@service('my_service', SetBool)
def service_callback(self, request: SetBool.Request, response: SetBool.Response):
    self.logger.info(f'Request received: {request.data}')
    response.success = True
    response.message = "Processed!"
    return response # Must return the response object
```

### `@action_server(action_name, action_type)`

Decorates a method to act as the `execute_callback` for a ROS 2 action server.

- **`action_name`** (str | `Parameter`): The name of the action.
- **`action_type`**: The ROS 2 action type (e.g., `nav2_msgs.action.NavigateToPose`).

**Example:**
```python
import time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from genesys.decorators import action_server

@action_server('navigate_to_pose', NavigateToPose)
def execute_nav(self, goal_handle):
    self.logger.info('Executing goal...')

    # Publish feedback
    feedback_msg = NavigateToPose.Feedback()
    feedback_msg.current_pose = PoseStamped() # Dummy feedback
    goal_handle.publish_feedback(feedback_msg)

    # Check for cancellation
    if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        self.logger.info('Goal canceled')
        return NavigateToPose.Result()

    time.sleep(2) # Simulate work

    goal_handle.succeed()
    self.logger.info('Goal succeeded!')
    
    result = NavigateToPose.Result()
    return result
```
---

## Component and Lifecycle Decorators

### `@component`

A **class decorator** that marks a Python class as a ROS 2 component, making it discoverable by component containers.

**Example:**
```python
from genesys.decorators import component, node

@node('my_component_node')
@component
class MyImageProcessor:
    # ... node logic using other decorators ...
```

#### How It Works
The `@component` decorator does two things:
1.  It sets a `__genesys_is_component__` flag on the class.
2.  It adds a static method `get_node_factory` to your class, which is required by the `rclpy_components` infrastructure to load the node.

When you use `genesys make component`, the scaffolding tool automatically adds the necessary `entry_point` to your `setup.py` file to register this component.

### `@lifecycle_node`

A **class decorator** that transforms a standard node into a [Lifecycle Node](https://design.ros2.org/articles/node_lifecycle.html).

**Example:**
```python
from genesys.decorators import lifecycle_node, node

@node('my_lifecycle_node')
@lifecycle_node
class MyLifecycleNode:
    # ... on_configure, on_activate, etc. callbacks ...
```

#### How It Works
This decorator simply adds an `_is_lifecycle_node` flag to the class. The main `@node` decorator checks for this flag and, if present, makes the `NodeBase` wrapper inherit from `rclpy.lifecycle.LifecycleNode` instead of the standard `rclpy.node.Node`. You are then responsible for implementing the standard lifecycle transition callbacks (e.g., `on_configure`, `on_activate`).

---

## Declarative Attributes

Besides decorators, Genesys provides functions to declaratively create attributes like parameters and action clients.

### `parameter(name, default_value=None)`

Declares a ROS 2 parameter as a class attribute.

- **`name`** (str): The ROS 2 parameter name.
- **`default_value`**: The default value for the parameter.

**Example:**
```python
from genesys.decorators import node, parameter, timer, publisher
from std_msgs.msg import String

@node('param_node')
class ParameterExample:
    # Declare a parameter. The type is inferred from the type hint.
    timer_period: float = parameter('timer_period_sec', 1.0)
    
    # Declare a parameter to be used as a topic name
    output_topic: str = parameter('output_topic_name', '/default_topic')

    def __init__(self):
        # The parameter is available at self.timer_period
        # We can now create a timer manually if its period needs to be dynamic
        self.create_timer(self.timer_period, self._do_work)

    @publisher(output_topic, String) # Use the parameter for the topic name
    def _do_work(self):
        return String(data="Hello from a dynamic topic!")
```

#### How It Works
The `parameter()` function returns a special `Parameter` object. `NodeBase` discovers these objects during initialization. It uses the attribute's name (`timer_period`) and type hint (`float`) to call `self.declare_parameter()`. It also sets up a callback so that if the ROS 2 parameter is changed externally (e.g., with `ros2 param set`), the attribute `self.timer_period` on your class instance is **automatically updated**.

### `action_client(name, action_type)`

Declares a ROS 2 action client as a class attribute.

- **`name`** (str | `Parameter`): The name of the action.
- **`action_type`**: The ROS 2 action type.

**Example:**
```python
from genesys.decorators import node, action_client
from nav2_msgs.action import NavigateToPose

@node('action_client_node')
class MyActionClient:
    nav_client = action_client('navigate_to_pose', NavigateToPose)

    def send_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.logger.error('Action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        # ... populate goal_msg ...

        self.logger.info('Sending goal request...')
        self.nav_client.send_goal_async(goal_msg)
```

#### How It Works
Similar to `parameter()`, `action_client()` returns a placeholder object. `NodeBase` finds this placeholder, creates a real `rclpy.action.ActionClient`, and replaces the placeholder attribute (`self.nav_client`) with the actual, ready-to-use client.

---

## Quality of Service (QoS) Aliases

For convenience, all decorators that accept a `qos` argument can take one of the following string aliases instead of a full `QoSProfile` object:

- **`"default"`**: `RELIABLE`, `KEEP_LAST`, `depth=10`. Good for most use cases.
- **`"sensor"`**: `BEST_EFFORT`, `KEEP_LAST`, `depth=1`. Ideal for high-frequency sensor data where it's okay to drop messages.
- **`"system"`**: The default ROS 2 system profile (`rclpy.qos.qos_profile_system_default`).
- **`"services"`**: The default ROS 2 services profile (`rclpy.qos.qos_profile_services_default`).
