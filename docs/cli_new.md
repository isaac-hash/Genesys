# CLI Command: `genesys new`

The `genesys new` command creates a new, structured ROS 2 workspace with the standard Genesys directory layout, it also provides a package which contains the c++ macros for c++ nodes. It is contained in the genesys_macro directory.

## Usage

```bash
genesys new <project_name>
```

- **`<project_name>`**: The name of the workspace directory to be created.

## Description

This command sets up a clean workspace, ready for you to start creating packages. It creates the following directory structure:

```
<project_name>/
├── src/         # For your ROS 2 packages
├── launch/      # For top-level launch files
├── config/      # For global configuration files
├── sim/         # For Gazebo simulation packages
├── tests/       # For workspace-level integration tests
├── scripts/     # For utility scripts
└── tools/       # For miscellaneous tools
```

### Example

```bash
genesys new my_robot_project
```

This will create a directory named `my_robot_project` with the structure shown above.

After running the command, you can navigate into the new directory and begin adding packages:

```bash
cd my_robot_project
genesys make pkg my_first_package
```
