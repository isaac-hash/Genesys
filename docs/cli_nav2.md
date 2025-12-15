# Genesys Nav2 CLI

The `nav2` command group provides a suite of tools to integrate autonomous navigation capabilities into your robot projects using the ROS 2 Navigation Stack (Nav2). This guide covers how to configure, generate, launch, and diagnose Nav2 setups.

## Overview

The workflow consists of four main steps:
1.  **Init**: Interactive configuration of your robot's geometry, sensors, and drive type.
2.  **Generate**: Auto-generation of ROS 2 packages (`_description`, `_navigation`, `_simulation`) with industry-standard Nav2 configs.
3.  **Launch**: One-line shortcuts to start Simulation, SLAM, or Navigation.
4.  **Doctor**: Diagnosing environment and runtime issues.

## Commands

### 1. Initialize Configuration (`init`)

start by creating a navigation configuration file.

```bash
genesys nav2 init
```

**Prompts:**
- **Robot Name**: Snake_case name (e.g., `my_bot`).
- **Drive Type**: Select from `Differential`, `Omni`, `Ackermann`, or `Legged`.
    - *Note*: This selection automatically picks the best planner/controller plugins (e.g., MPPI for Omni, SmacHybrid for Ackermann).
- **Geometry**: Robot dimensions (length, width, height) and wheel properties.
- **Sensors**: Toggle 2D/3D Lidar, Depth/RGB Cameras, and IMU.
- **Odometry**: Enable sensor fusion (EKF) via `robot_localization`.

**Output**: `config/navigation.yaml`

### 2. Generate Packages (`generate`)

Generates the ROS 2 packages required to run Nav2.

```bash
genesys nav2 generate [--config config/navigation.yaml]
```

**Generated Packages:**
- **`{robot}_description`**: URDF/Xacro files, Gazebo plugins, and `robot_state_publisher` launch.
- **`{robot}_navigation`**: Nav2 parameter files (`nav2_params.yaml`), specialized launch files (`bringup.launch.py`), and RViz config.
- **`{robot}_simulation`**: Gazebo world and simulation launch files (if enabled).

**Next Steps After Generation:**
```bash
genesys build
```

### 3. Launch Shortcuts (`launch`)

Simplified commands to start complex ROS 2 systems.

#### Simulation Mode
Starts Gazebo (with spawned robot) and the full Nav2 stack + RViz.
```bash
genesys nav2 launch sim
```

#### SLAM Mode
Mapping mode. Starts the robot (or simulation) and `slam_toolbox`.
```bash
genesys nav2 launch slam
```

#### Navigation Mode
Autonomous navigation mode. Starts AMCL localization and the Nav2 stack. Requires a map.
```bash
genesys nav2 launch navigate --map /path/to/map.yaml
```

### 4. Diagnostics (`doctor`)

Diagnoses the health of your Nav2 setup.

```bash
genesys nav2 doctor [--watch]
```

**Checks Performed:**
- **Topics**: Verifies `/scan`, `/odom`, `/tf`, and `/tf_static` are publishing.
- **TF Chain**: Checks validity of the `map` -> `odom` -> `base_link` transform tree.
- **Nodes**: Checks if critical nodes (`controller_server`, `bt_navigator`) are active.

**Output:**
```text
Checking Required Topics...     [PASS]
Checking TF Topics...           [PASS]
Checking Nav2 Nodes...          [FAIL]
  -> Check if Nav2 stack is launched. Run 'genesys nav2 launch sim' or 'navigate'.
```

### 5. Regenerate (`regen`)

To regenerate specific parts of the codebase without overwriting everything (Stub for future release).
```bash
genesys nav2 regen [urdf|params|launch]
```

## Customization

- **Parameters**: Edit `{robot}_navigation/config/nav2_params.yaml`. This file is pre-populated with optimal defaults based on your drive type and geometry.
- **URDF**: Modify `{robot}_description/urdf/base.xacro` to fine-tune your robot's physical model.

## Troubleshooting

- **"TF Chain Issue"**: Ensure `robot_state_publisher` is running (started automatically by `launch sim/slam/nav`).
- **"No /scan topic"**: Check if Gazebo is running or your physical Lidar driver is active.
- **"Navigation stuck"**: Verify the map is correct and the initial pose is set in RViz using "2D Pose Estimate".
