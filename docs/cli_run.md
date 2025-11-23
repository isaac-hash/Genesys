# CLI Command: `genesys run`

The `genesys run` command executes a ROS 2 node by its name, automatically finding which package it belongs to. This is a significant convenience over the standard `ros2 run <package_name> <executable_name>` command.

## Usage

```bash
genesys run <node_name> [arguments]
```

- **`<node_name>`**: The name of the executable for the node you want to run.
- **`[arguments]`**: Any additional arguments to pass to the node, including ROS arguments like remapping rules.

## How It Works

The command simplifies running nodes by removing the need to specify the package name. Here's the process:

1.  **Workspace Validation**: It first checks for an `install/` directory to ensure the workspace has been built.
2.  **Discover Nodes**: It runs the `ros2 pkg executables` command in a sourced sub-shell. This command returns a list of all packages and the executables they provide.
3.  **Find Package**: It parses the output from the previous step to create a map of every known executable to its parent package. It uses this map to find the package that owns the `<node_name>` you requested.
4.  **Error Handling**: If the node name is not found in any package, it prints an error and a list of all available node executables it discovered, helping you find the correct name.
5.  **Execute**: Once the package is found, it constructs and executes the full `ros2 run <package_name> <node_name>` command, passing along any extra arguments you provided.

## Remapping Arguments

The `run` command provides a simplified and more intuitive syntax for remapping topics.

- `--remap <old>:=<new>`
- `--remap=<old>:=<new>`

The command automatically converts this into the format that `ros2 run` expects. For example:

```bash
# This command...
genesys run listener_node --remap /chatter:=/my_chatter_topic

# ...is automatically converted and run as:
ros2 run <pkg> listener_node --ros-args -r /chatter:=/my_chatter_topic
```

## Example

If you have a node named `talker_node` in `my_package`, you can run it with:

```bash
genesys run talker_node
```
The command will discover that `talker_node` belongs to `my_package` and execute `ros2 run my_package talker_node`.
