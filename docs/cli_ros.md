# CLI Command: ROS 2 Wrappers

Genesys provides a set of convenient wrappers around common `ros2` CLI commands. These wrappers automatically source your local workspace's `install/setup.bash` file before executing the command, so you don't have to remember to do it yourself.

## Usage

You can use these commands exactly like their `ros2` counterparts, but with `genesys` as the prefix.

```bash
# Instead of 'ros2 topic list'
genesys topic list

# Instead of 'ros2 param get /my_node my_param'
genesys param get /my_node my_param
```

## Available Command Groups

The following `ros2` command groups are wrapped:

- **`genesys node`**: Interact with running nodes.
  - `list`, `info`

- **`genesys topic`**: Interact with topics.
  - `list`, `info`, `echo`, `pub`, `bw`, `find`
  - Also includes wrappers for `ros2 bag`: `record`, `replay`

- **`genesys service`**: Interact with services.
  - `list`, `type`, `info`, `find`, `call`

- **`genesys action`**: Interact with actions.
  - `list`, `info`, `send_goal`

- **`genesys param`**: Interact with node parameters.
  - `list`, `get`, `set`, `dump`, `load`

## Why Use Them?

The main advantage is convenience. In a typical workflow, you might open a new terminal and forget to source your workspace overlay. If you then try to run a node from your workspace using `ros2 run`, it will fail.

By using `genesys run` or `genesys topic echo`, the command ensures the environment is correctly set up first, leading to fewer "package not found" or "topic not found" errors.
