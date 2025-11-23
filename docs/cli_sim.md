# CLI Command: `genesys sim`

The `genesys sim` command group provides tools to create and manage Gazebo simulations within your workspace. It standardizes the process of setting up a simulation environment for a robot, which is often a complex and error-prone task.

---

## `genesys sim create`

Creates a new `*_gazebo` package in the `sim/` directory, fully configured for a specific robot.

### Usage
```bash
genesys sim create <package_name> --from-pkg <robot_description_package>
```
- **`<package_name>`**: The name of the simulation package. By convention, this **must** end in `_gazebo` (e.g., `my_robot_gazebo`).
- **`--from-pkg <robot_description_package>`**: The name of an existing package that contains the robot's URDF or XACRO files (e.g., `ur_description`).

### How It Works

This command automates the creation of a complete Gazebo simulation package.

1.  **Directory Creation**: It creates a new package inside the `sim/` directory (creating `sim/` if it doesn't exist). This keeps simulation packages separate from your robot's source code in `src/`.
2.  **Dependency Check**: It runs `rosdep check` and `rosdep install` to ensure that necessary simulation dependencies like `gazebo_ros_pkgs` are installed on your system.
3.  **Template Rendering**: It generates a standard set of subdirectories (`launch`, `worlds`, `models`, `config`, `urdf`) and populates them with boilerplate files rendered from internal templates. This includes:
    - `CMakeLists.txt` and `package.xml`
    - A main launch file (`<package_name>.launch.py`) to start Gazebo and the robot.
    - A secondary launch file to spawn the robot model (`spawn_<robot_name>.launch.py`).
    - A default `controllers.yaml` for a `ros2_control` setup.
    - `default.world` and `empty.world` files.
4.  **URDF Linking**: To ensure the simulation always uses the most up-to-date robot model, it tries to **symlink** the `urdf` directory from the source package (`--from-pkg`) into the new simulation package. If symlinking fails (e.g., due to permissions), it falls back to copying the directory.

**Example:**
```bash
genesys sim create my_robot_gazebo --from-pkg my_robot_description
```

---

## `genesys sim run`

Launches a Gazebo simulation from one of the `*_gazebo` packages.

### Usage
```bash
genesys sim run <package_name> [options]
```
- **`<package_name>`**: The name of the simulation package to run (e.g., `my_robot_gazebo`). It must be located in the `sim/` directory.

### Options
- **`--world <world_file.world>`**: Specify a world file to load from the package's `worlds/` directory. Defaults to `empty.world`.
- **`--headless`**: Run Gazebo in headless mode (no GUI). This is useful for running simulations on a server or as part of a CI/CD pipeline.

### How It Works

1.  **Workspace Validation**: Checks that the workspace has been built (`install/` exists) and the specified `<package_name>` exists in the `sim/` directory.
2.  **Environment Setup**: It modifies the environment for the launch subprocess. This is the key to making Gazebo find all the necessary files. It sets the following environment variables:
    - `GAZEBO_MODEL_PATH`: Points to the `models` directory in your sim package.
    - `GZ_SIM_RESOURCE_PATH`: Points to both the `worlds` and `models` directories.
    - `GAZEBO_WORLD`: Set to the value of the `--world` option.
    - `HEADLESS`: Set to `1` if the `--headless` flag is used.
3.  **Execution**: It sources the local workspace and executes the main launch file for the simulation package (`ros2 launch <package_name> <package_name>.launch.py`) using the modified environment.

**Example:**
```bash
# Run the simulation with a GUI and a specific world
genesys sim run my_robot_gazebo --world my_world.world

# Run the simulation without a GUI
genesys sim run my_robot_gazebo --headless
```
