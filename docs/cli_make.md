# CLI Command: `genesys make`

The `genesys make` command is a powerful scaffolding tool used to create and modify ROS 2 packages, nodes, and components. It automates the tedious process of creating files and editing build configurations.

This guide provides a detailed breakdown of what each `make` command does behind the scenes.

---

## `genesys make pkg`

Creates a new ROS 2 package in the `src/` directory.

### Usage
```bash
genesys make pkg <package_name> [options]
```
- **`<package_name>`**: The name of the package to create.

### Options
- **`--dependencies <dep1> <dep2>...`**: A list of ROS 2 package dependencies.
- **`--with-node`**: Automatically create an initial node inside the new package.

### How It Works

1.  **Language Choice**: The command interactively prompts you to choose a language (`Python` or `C++`).
2.  **Package Creation**: It runs `ros2 pkg create` with the correct build type (`ament_python` or `ament_cmake`).
3.  **Dependencies**: If dependencies are provided, they are passed to the `ros2 pkg create` command, which adds them to `package.xml`.
4.  **C++ Template**: For C++ packages, it overwrites the default `CMakeLists.txt` with a custom Genesys template that includes helper comments and insertion points for future scaffolding commands.

---

## `genesys make node`

Creates a new ROS 2 node file and registers it in an existing package.

### Usage
```bash
genesys make node <node_name> --pkg <package_name> [options]
```
- **`<node_name>`**: The name for the new node (e.g., `image_processor`).
- **`--pkg <package_name>`**: The existing package to add the node to.
- **`--component`**: A flag to create a component instead. This is a shortcut for `genesys make component`.

### How It Works (Python)

1.  **Node Type**: Prompts you to select a node type (Publisher, Subscriber, etc.).
2.  **File Creation**: Creates a Python file (e.g., `src/my_pkg/my_pkg/my_node.py`) based on a template for the selected node type.
3.  **`setup.py` Modification**: Automatically adds the node's entry point to `src/my_pkg/setup.py`. This makes the node an executable that ROS 2 can find.
    ```python
    # In setup.py
    'console_scripts': [
        'my_node = my_pkg.my_node:main', # This line is added
    ],
    ```
4.  **Launch File**:
    - Creates a `src/my_pkg/launch/my_pkg_launch.py` file if one doesn't exist.
    - Adds the new node to this launch file.
    - Adds an install rule to `setup.py` to ensure the launch directory is installed.
      ```python
      # In setup.py data_files
      (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
      ```

### How It Works (C++)

1.  **Node Type**: Prompts you to select a node type.
2.  **File Creation**: Creates a header (`.hpp`) and source (`.cpp`) file in `src/my_pkg/include/my_pkg/` and `src/my_pkg/src/` respectively, based on templates. The C++ macros are also available as a seperate package in the src of your general workspace, the templates generated for the publisher and subscriber nodes, make use of the macros, however, they can be altered to suit your coding choices!.
3.  **`CMakeLists.txt` Modification**: Adds a new executable target as well as the genesys macros dependencies.
    ```cmake
    # In CMakeLists.txt
    add_executable(my_node src/my_node.cpp)
    ament_target_dependencies(my_node rclcpp std_msgs) # Dependencies added here
    install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
    ```
4.  **`package.xml` Modification**: Adds any necessary `<depend>` tags for the node's dependencies as well as the macros.
5.  **Launch File**: The same launch file generation and installation logic as the Python version is applied, but the install rule is added to `CMakeLists.txt`.
    ```cmake
    # In CMakeLists.txt
    install(
      DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
    )
    ```
---

## `genesys make component`

Creates a new ROS 2 component (a composable node) and registers it.

### Usage
```bash
genesys make component <component_name> --pkg <package_name>
```
- **`<component_name>`**: The name for the new component.
- **`--pkg <package_name>`**: The existing package to add the component to.


### How It Works (C++)

This is the most complex scaffolding operation.

1.  **File Creation**: Creates `.hpp` and `.cpp` files for the component class.
2.  **Component Registration**: Creates or updates a `src/register_components.cpp` file. This file uses the `RCLCPP_COMPONENTS_REGISTER_NODE` macro to register your component with `pluginlib`, which is the underlying mechanism for C++ components.
    ```cpp
    // In src/register_components.cpp
    #include "my_pkg/my_component.hpp"
    RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MyComponent) // This line is added
    ```
3.  **`CMakeLists.txt` Modification**:
    - If this is the first component in the package, it adds a large block to create a **shared library** named `${PROJECT_NAME}_components`. All subsequent components in the package are added to this same library.
    - It links the component source files and `register_components.cpp` to this library.
    - It uses `rclcpp_components_register_nodes` to export the component plugins.
4.  **Plugin XML**: Creates or updates `resource/pkg_name_plugin.xml`. This XML file is what `pluginlib` uses at runtime to find and load your component.
    ```xml
    <!-- In resource/pkg_name_plugin.xml -->
    <class type="my_pkg::MyComponent" base_class_type="rclcpp::Node">
      <description>...</description>
    </class>
    ```
5.  **`package.xml` Modification**:
    - Adds dependencies like `rclcpp_components` and `pluginlib`.
    - Adds an `<export>` tag to point to the plugin XML file.

---

## `genesys make launch`

Creates a new launch file.

### Usage
```bash
genesys make launch --pkg <package_name> --name <launch_name>
```
- **`--pkg <package_name>`**: The package to create the launch file in.
- **`--name <launch_name>`**: The name of the launch file (defaults to `mixed_launch`).

### Description
By default, this creates a `mixed_launch.py` file. A mixed launch file is pre-configured with a `ComposableNodeContainer`, which allows you to efficiently run multiple components in a single process. It also has a section for regular, standalone nodes. This is the preferred launch file type when working with components.

---

## `genesys make interface`

This command is a placeholder and is **not yet implemented**. It will print manual instructions for how to create custom message, service, or action files.
