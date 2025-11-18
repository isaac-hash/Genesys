# CLI Command: `genesys rqt`

The `genesys rqt` command group provides simple shortcuts for launching common RQT (ROS Qt) GUI tools.

## Usage

```bash
genesys rqt <tool_name>
```

## Available Tools

### `genesys rqt graph`

Launches the RQT Node Graph plugin.

**Usage:**
```bash
genesys rqt graph
```

This is one of the most useful debugging tools in ROS 2. It provides a graphical representation of your running system, showing all the active nodes and the topics connecting them.

### `genesys rqt console`

Launches the RQT Console plugin.

**Usage:**
```bash
genesys rqt console
```

This tool aggregates log messages from all running nodes into a single, filterable GUI. You can use it to view, sort, and search through log messages by severity, node name, or time. It is often more convenient than looking at the raw console output.
