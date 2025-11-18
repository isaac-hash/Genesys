# CLI Command: `genesys sim`

The `genesys sim` command group provides tools to create and manage Gazebo simulations within your workspace. It standardizes the process of setting up a simulation environment for a robot.

## `genesys sim create`

Creates a new `*_gazebo` package in the `sim/` directory, configured for a specific robot.

### Usage
```bash
genesys sim create <package_name> --from <robot_description_package>
```

- **`<package_name>`**: The name of the simulation package to create. By convention, this should end in `_gazebo` (e.g., `my_robot_gazebo`).
- **`--from <robot_description_package>`**: The name of an existing package that contains the robot's URDF or XACRO files (e.g., `ur_description`).

### Description
This command automates the creation of a Gazebo simulation package. It will:
1. Create a new package in the `sim/` directory.
2. Generate standard subdirectories (`launch`, `worlds`, `models`, `config`).
3. Create boilerplate launch files to spawn the robot and start Gazebo.
4. Create a default world file.
5. Attempt to symlink the `urdf` directory from the source robot package so that your simulation always uses the most up-to-date robot model.

**Example:**
```bash
genesys sim create my_robot_gazebo --from my_robot_description
```

---

## `genesys sim run`

Launches a Gazebo simulation from one of the `*_gazebo` packages.

### Usage
```bash
genesys sim run <package_name> [options]
```
- **`<package_name>`**: The name of the simulation package to run (e.g., `my_robot_gazebo`).

### Options
- **`--world <world_file.world>`**: Specify a world file to load from the package's `worlds/` directory. Defaults to `empty.world`.
- **`--headless`**: Run Gazebo in headless mode (no GUI). This is useful for running simulations on a server or as part of a CI/CD pipeline.

### Description
This command sources the workspace and then executes the main launch file for the simulation package, passing in the specified world and headless mode options as environment variables.

**Example:**
```bash
# Run the simulation with a GUI
genesys sim run my_robot_gazebo --world my_world.world

# Run the simulation without a GUI
genesys sim run my_robot_gazebo --headless
```
