# CLI Command: `genesys run`

The `genesys run` command executes a ROS 2 node by its name, automatically finding which package it belongs to.

## Usage

```bash
genesys run <node_name> [arguments]
```

- **`<node_name>`**: The name of the executable for the node you want to run.
- **`[arguments]`**: Any additional arguments to pass to the node, including ROS arguments like remapping rules.

## Description

This command simplifies running nodes by removing the need to specify the package name, which is required by `ros2 run`. It searches through all executables in the built workspace, finds the package that provides the specified `<node_name>`, and then executes it.

It also provides a simplified syntax for topic remapping.

### Example: Running a node

If you have a node named `talker_node` in `my_package`, you can run it with:

```bash
genesys run talker_node
```
This is equivalent to `ros2 run my_package talker_node`.

### Example: Running a node with remapping

`genesys run` supports a simplified `--remap` flag.

```bash
genesys run listener_node --remap /chatter:=/my_chatter_topic
```

This will start the `listener_node` and remap the `/chatter` topic to `/my_chatter_topic`.
