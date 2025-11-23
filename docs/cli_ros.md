# CLI: ROS 2 Wrappers

Genesys provides a set of convenient wrappers around the most common `ros2` CLI command groups. These wrappers automatically source your local workspace's `install/setup.bash` file before executing the command, so you don't have to remember to do it yourself.

## Why Use Them?

The main advantage is convenience. In a typical workflow, you might open a new terminal and forget to source your workspace overlay. If you then try to interact with a node or topic from your workspace using `ros2 topic echo`, it will likely fail.

By using the `genesys` wrappers (`genesys topic echo`), the command ensures the environment is correctly set up first, leading to fewer "package not found" or "topic not found" errors.

## Usage

You can use these commands exactly like their `ros2` counterparts, but with `genesys` as the prefix.

```bash
# Instead of 'ros2 topic list'
genesys topic list

# Instead of 'ros2 param get /my_node my_param'
genesys param get /my_node my_param
```

## Available Command Groups

### `genesys node`
- `list`: List all running nodes.
- `info <node_name>`: Get information about a specific node.

### `genesys topic`
- `list`: List all active topics.
- `info <topic_name>`: Get information about a specific topic.
- `echo <topic_name>`: Echo messages from a topic.
- `pub <topic_name> <msg_type> [args]`: Publish a message to a topic.
- `bw <topic_name>`: Display bandwidth used by a topic.
- `find <msg_type>`: Find topics by message type.
- `record [topics...]`: Record topics to a bag file. Records all topics if none are specified.
- `replay <bag_file>`: Play back a bag file.

### `genesys service`
- `list`: List all active services.
- `type <srv_name>`: Get the type of a service.
- `info <srv_name>`: Get information about a specific service.
- `find <srv_type>`: Find services by service type.
- `call <srv_name> <srv_type> [args]`: Call a service with the given arguments.

### `genesys action`
- `list`: List all active actions.
- `type <action_name>`: Get the type of an action.
- `info <action_name>`: Get information about a specific action.
- `send_goal <action_name> <action_type> [args]`: Send a goal to an action server.

### `genesys param`
- `list [node_name]`: List parameters for a specific node or all nodes.
- `get <node_name> <param_name>`: Get a parameter from a node.
- `set <node_name> <param_name> <value>`: Set a parameter on a node.
- `dump <node_name>`: Dump all parameters for a node to a YAML file.
- `load <node_name> <file_path>`: Load parameters for a node from a file.

---
## Notes on Specific Commands

Some commands have slightly different behavior or are not implemented.

- **`genesys service info`**: The `ros2 service info` command does not exist. This wrapper is a convenience alias for `genesys service type`.
- **`genesys action type`**: The `ros2 action type` command does not exist. This wrapper is a convenience alias for `genesys action info`.
- **`genesys service echo`**: This feature is not implemented, as it would require a custom node to intercept and print service requests.
- **`genesys action echo`**: This feature is not implemented, as it would require a custom node to monitor the action feedback and result topics.
