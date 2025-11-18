# CLI Command: `genesys make`

The `genesys make` command is a group of sub-commands used for scaffolding common ROS 2 and Genesys entities like packages, nodes, and components.

## `genesys make pkg`

Creates a new ROS 2 package in the `src/` directory.

### Usage
```bash
genesys make pkg <package_name> [options]
```

- **`<package_name>`**: The name of the package to create.

### Options
- **`--dependencies <dep1> <dep2>...`**: A list of ROS 2 package dependencies.
- **`--with-node`**: Automatically create an initial "hello world" node inside the new package.

### Description
This command interactively prompts you to choose a language (`Python` or `C++`). It then runs `ros2 pkg create` with the correct build type (`ament_python` or `ament_cmake`) and places the new package in the `src/` directory of your workspace.

---

## `genesys make node`

Creates a new ROS 2 node file and registers it in an existing package.

### Usage
```bash
genesys make node <node_name> --pkg <package_name>
```

- **`<node_name>`**: The name for the new node (e.g., `image_processor`).
- **`--pkg <package_name>`**: The existing package to add the node to.

### Description
This command will:
1. Prompt you to select a node type (e.g., Publisher, Subscriber, Service).
2. Generate a boilerplate Python or C++ file for the node.
3. For Python, it automatically adds the required entry point to `setup.py`.
4. For C++, it updates `CMakeLists.txt` to build the new executable.
5. It intelligently adds the new node to a launch file within the package, creating one if necessary.

---

## `genesys make component`

Creates a new ROS 2 component (a composable node) and registers it.

### Usage
```bash
genesys make component <component_name> --pkg <package_name>
```

- **`<component_name>`**: The name for the new component (e.g., `sensor_driver`).
- **`--pkg <package_name>`**: The existing Python package to add the component to.

### Description
This command scaffolds a Python class decorated with `@component`. It automatically handles:
1.  Creating the component file (e.g., `sensor_driver.py`).
2.  Adding the `rclpy_components` entry point to `setup.py` so ROS 2 can discover it.
3.  Intelligently registering the component in a launch file, preferring `mixed_launch.py` or updating an existing launch file to support components.

---

## `genesys make launch`

Creates a new mixed launch file for managing both nodes and components.

### Usage
```bash
genesys make launch --pkg <package_name>
```

- **`--pkg <package_name>`**: The package where the launch file will be created.

### Description
This creates a `launch/mixed_launch.py` file with boilerplate for a `ComposableNodeContainer`, which is ready for you to add components to.
