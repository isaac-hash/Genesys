# Genesys Decorators

Genesys simplifies ROS 2 node development by using a decorator-based approach. Instead of manually creating publishers, subscribers, timers, etc., in your node's `__init__` method, you can decorate your callback functions.

The framework provides a `NodeBase` class that automatically handles the boilerplate of setting up these communication channels. When your node is initialized, `NodeBase` inspects all methods, finds the Genesys decorators, and wires everything up.

## Key Advantages
- **Declarative:** Your code clearly declares its intent. A method decorated with `@subscriber` is obviously a subscription callback.
- **Clean:** Keeps your `__init__` method free of repetitive setup code.
- **Efficient:** Reduces boilerplate and the chance of common errors.

---

## Available Decorators

### `@publisher(topic, msg_type, qos="default")`
Decorates a method that produces messages. The decorated method's return value will be automatically published.

- **`topic`**: The name of the topic to publish to.
- **`msg_type`**: The ROS 2 message type (e.g., `std_msgs.msg.String`).
- **`qos`**: A QoS profile. Can be a `QoSProfile` object or a string alias (`"default"`, `"sensor"`, `"system"`, `"services"`).

**Example:**
```python
from std_msgs.msg import String
from genesys.decorators import publisher, timer

@timer(1.0)
@publisher('/my_topic', String)
def publish_message(self):
    msg = String()
    msg.data = 'Hello, world!'
    return msg
```

### `@subscriber(topic, msg_type, qos="default")`
Decorates a method that will be used as a callback for a topic subscription.

- **`topic`**: The name of the topic to subscribe to.
- **`msg_type`**: The ROS 2 message type.
- **`qos`**: The QoS profile or string alias.

**Example:**
```python
from std_msgs.msg import String
from genesys.decorators import subscriber

@subscriber('/my_topic', String)
def topic_callback(self, msg):
    self.logger.info(f'I heard: "{msg.data}"')
```

### `@timer(period_sec)`
Decorates a method to be called at a fixed interval.

- **`period_sec`**: The timer period in seconds.

**Example:**
```python
from genesys.decorators import timer

@timer(2.5)
def periodic_task(self):
    self.logger.info('This runs every 2.5 seconds.')
```

### `@service(service_name, service_type)`
Decorates a method to act as a callback for a ROS 2 service.

- **`service_name`**: The name of the service.
- **`service_type`**: The ROS 2 service type (e.g., `std_srvs.srv.SetBool`).

**Example:**
```python
from std_srvs.srv import SetBool
from genesys.decorators import service

@service('my_service', SetBool)
def service_callback(self, request, response):
    self.logger.info(f'Request received: {request.data}')
    response.success = True
    response.message = "Processed!"
    return response
```

### `@action_server(action_name, action_type)`
Decorates a method to act as the `execute_callback` for a ROS 2 action server.

- **`action_name`**: The name of the action.
- **`action_type`**: The ROS 2 action type (e.g., `nav2_msgs.action.NavigateToPose`).

**Example:**
```python
from my_interfaces.action import Fibonacci
from genesys.decorators import action_server

@action_server('fibonacci', Fibonacci)
def execute_fibonacci(self, goal_handle):
    # ... action execution logic ...
    goal_handle.succeed()
    result = Fibonacci.Result()
    # ... set result fields ...
    return result
```

### `@component`
A class decorator that marks a Python class as a ROS 2 component, making it discoverable by component containers. It automatically adds the necessary entry points.

**Example:**
```python
from genesys.decorators import component

@component
class MyImageProcessor:
    # ... node logic using other decorators ...
```
